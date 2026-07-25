"""
Yaw-rate limiting and stability-based fallback for heading fusion.

Two problems this solves, both grounded in the real logs:

1. COG is noisy even at speed. Above 15 mph, ~12% of COG samples jump more than
   45 deg/s -- physically impossible for this boat. Feeding those to the
   autopilot makes it chase GPS jitter. So every candidate heading is checked
   against a realistic maximum yaw rate and discarded if it implies an
   impossible turn.

2. When sources disagree, prefer the one that is both stable and near the last
   accepted heading, so a single glitching sensor cannot yank the controller
   away from a good track.

Measured basis (2026-07-24/25 logs):
  * wall-compass heading yaw rate: p99 = 3.1 deg/s, max = 40.9 deg/s
  * so real turns stay under ~41 deg/s; 45 deg/s is just above that, clipping
    impossible jumps without touching legitimate sharp turns.

The limiter is deliberately simple and stateful: it remembers the last accepted
heading and time, and judges each new candidate against the elapsed time.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple


# Maximum plausible yaw rate for this hull, deg/s. Measured real turns top out
# near 41 deg/s (wall compass, all logs); 45 leaves a small margin so a genuine
# hard turn is never clipped while GPS-jitter spikes (hundreds of deg/s) are.
# For a slower/heavier boat, lower this; for an aggressive one, raise it.
YAW_RATE_MAX_DPS = 45.0

# If we have not had an accepted heading for this long, accept the next
# candidate unconditionally (we have no basis to rate-limit against a stale
# reference, and refusing forever would wedge the filter).
RATE_LIMIT_RESET_S = 3.0


def _angdiff(a: float, b: float) -> float:
    """Shortest signed difference a-b on the circle, in (-180, 180]."""
    d = a - b
    while d > 180:
        d -= 360
    while d <= -180:
        d += 360
    return d


@dataclass
class YawRateLimiter:
    """
    Rejects heading candidates that imply an impossible turn rate.

    Stateful: remembers the last accepted heading. A candidate is accepted if
    the implied rate (change / elapsed time) is within YAW_RATE_MAX_DPS, or if
    too much time has passed to judge (reset).
    """

    max_rate_dps: float = YAW_RATE_MAX_DPS
    reset_after_s: float = RATE_LIMIT_RESET_S
    _last_value: Optional[float] = None
    _last_time: float = 0.0
    rejected_count: int = 0
    accepted_count: int = 0
    _consecutive_rejects: int = 0
    # If this many candidates are rejected in a row, the reference is stale or
    # wrong (e.g. we locked onto an outlier, or timestamps are compressed in a
    # fast replay). Accept the next candidate to re-baseline rather than
    # rejecting forever.
    max_consecutive_rejects: int = 5

    def check(self, value: float, when: Optional[float] = None) -> bool:
        """
        Return True if `value` is a plausible next heading. Does not update
        state -- call accept() when you actually use the value, so rejected
        candidates do not move the reference.
        """
        t = when if when is not None else time.time()
        if self._last_value is None:
            return True
        if self._consecutive_rejects >= self.max_consecutive_rejects:
            return True  # re-baseline; the reference is no longer trustworthy
        dt = t - self._last_time
        if dt <= 0:
            return True
        if dt > self.reset_after_s:
            return True  # too stale to judge
        rate = abs(_angdiff(value, self._last_value)) / dt
        return rate <= self.max_rate_dps

    def accept(self, value: float, when: Optional[float] = None) -> None:
        """Record `value` as the new reference."""
        self._last_value = value
        self._last_time = when if when is not None else time.time()
        self.accepted_count += 1
        self._consecutive_rejects = 0

    def reject(self) -> None:
        """Record that a candidate was rejected (bridge calls this)."""
        self.rejected_count += 1
        self._consecutive_rejects += 1

    def filter(self, value: float, when: Optional[float] = None) -> bool:
        """
        Convenience: check, and if accepted, record. Returns True if accepted.
        Use this when there is a single candidate stream. For multi-source
        selection, use check()/accept() separately so a rejected source does
        not update the reference.
        """
        t = when if when is not None else time.time()
        if self.check(value, t):
            self.accept(value, t)
            return True
        self.reject()
        return False

    @property
    def last_value(self) -> Optional[float]:
        return self._last_value


@dataclass
class SourceStability:
    """
    Tracks recent variability of one source, for stability-based fallback.

    Keeps a short history of (value, time) and reports the source's recent
    scatter (max deviation across the window). A source that is jumping around
    has high scatter and should not be trusted over a steady one.
    """

    window_s: float = 2.0
    _history: list = field(default_factory=list)  # [(t, value)]

    def add(self, value: float, when: Optional[float] = None) -> None:
        t = when if when is not None else time.time()
        self._history.append((t, value))
        cutoff = t - self.window_s
        self._history = [(ht, hv) for ht, hv in self._history if ht >= cutoff]

    def scatter(self) -> float:
        """Max angular spread across the window, degrees. 0 if too few points."""
        if len(self._history) < 2:
            return 0.0
        vals = [v for _, v in self._history]
        # spread on the circle: max pairwise angular distance from the mean
        import math
        mx = sum(math.cos(math.radians(v)) for v in vals) / len(vals)
        my = sum(math.sin(math.radians(v)) for v in vals) / len(vals)
        mean = math.degrees(math.atan2(my, mx))
        return max(abs(_angdiff(v, mean)) for v in vals)

    def fresh(self, when: Optional[float] = None) -> bool:
        if not self._history:
            return False
        t = when if when is not None else time.time()
        return (t - self._history[-1][0]) <= self.window_s

    @property
    def last_value(self) -> Optional[float]:
        return self._history[-1][1] if self._history else None


def choose_stable_near_last(
    candidates: Dict[str, Tuple[float, float]],
    last_heading: Optional[float],
    stability: Dict[str, SourceStability],
    max_rate_dps: float = YAW_RATE_MAX_DPS,
    now: Optional[float] = None,
) -> Optional[Tuple[str, float]]:
    """
    Pick the best heading when sources disagree.

    candidates: {source_name: (value, timestamp)}
    last_heading: the previously accepted fused heading (or None)
    stability:   {source_name: SourceStability} for scatter lookup

    Strategy, in order:
      1. Drop candidates that imply an impossible turn from last_heading
         (yaw-rate limit) -- these are glitches.
      2. Among survivors, prefer the one closest to last_heading that is also
         stable (low scatter). This keeps the controller on its track rather
         than letting a single deviating sensor pull it away.
      3. If there is no last_heading (startup), pick the steadiest source.

    Returns (source_name, value) or None if nothing is usable.

    This is the "fall back to the most stable sensor near the last reading"
    behavior: erroneous data that deviates sharply is discarded, and among
    good data the least surprising, steadiest source wins.
    """
    t = now if now is not None else time.time()
    if not candidates:
        return None

    # Step 1: yaw-rate gate against the last accepted heading.
    survivors = {}
    for name, (value, ts) in candidates.items():
        if last_heading is None:
            survivors[name] = value
        else:
            dt = max(t - ts, 1e-3)
            # judge against how old this sample is relative to last accept;
            # use a nominal 0.1 s if timestamps are equal.
            rate = abs(_angdiff(value, last_heading)) / max(dt, 0.1)
            if rate <= max_rate_dps:
                survivors[name] = value

    if not survivors:
        # Everything looks like a glitch. Hold: return None so the caller keeps
        # the last heading rather than accepting a jump.
        return None

    # Step 3: startup -- no reference. Pick steadiest.
    if last_heading is None:
        best = min(
            survivors,
            key=lambda n: stability[n].scatter() if n in stability else 999.0,
        )
        return best, survivors[best]

    # Step 2: prefer close-to-last AND stable. Score = distance + scatter, so a
    # source that is both near the last heading and steady wins. Weighting them
    # equally is a reasonable default; distance dominates for big disagreements.
    def score(name):
        dist = abs(_angdiff(survivors[name], last_heading))
        scat = stability[name].scatter() if name in stability else 0.0
        return dist + scat

    best = min(survivors, key=score)
    return best, survivors[best]
