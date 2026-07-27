"""
hmi_heading.py -- Heading quality assessment and engage interlock.

Why this exists separately from the Kalman filter: the filter tells you its
own covariance, which is an estimate of how well it thinks it is doing. That
is necessary but not sufficient. A filter can be confidently wrong when the
compass is being shaken by pitch and roll in a way the process model does not
anticipate. So this module measures the *observed* dispersion of the raw
compass alongside whatever the filter reports, and gates engagement on both.

The pitch/roll coupling problem
-------------------------------
A strapdown magnetic compass measures the field in body axes. Heading is
recovered by projecting onto the local horizontal, which requires knowing
pitch and roll. If the tilt compensation is imperfect (or absent), heading
error scales roughly with the product of tilt angle and magnetic dip. At
Colorado's ~66 degree dip, 10 degrees of roll can throw heading by well over
10 degrees, and it oscillates at wave frequency rather than averaging out
cleanly over a short window.

Two consequences drive the code below:

  1. Averaging headings must be done circularly. A naive mean of 359 and 1
     gives 180, which is exactly backwards. Every statistic here goes through
     unit vectors.

  2. Dispersion should be attributed. If heading scatter is high *and* pitch
     or roll scatter is high at the same time, that is sea state, not a
     broken sensor, and the message on screen should say so. The operator
     needs to know whether to reseat a connector or slow down.
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Deque, Optional, Tuple

from hmi_state import F_HEADING_INVALID, F_HEADING_NOISE, SystemState

# Engagement thresholds
SIGMA_ENGAGE_MAX = 6.0      # deg, circular sigma above which engage is blocked
SIGMA_WARN = 4.0            # deg, above which we flag but still allow
TILT_SIGMA_NOISY = 3.0      # deg, pitch/roll dispersion that explains heading noise
MIN_SAMPLES = 8             # need this many before sigma means anything
WINDOW_SEC = 6.0            # rolling window length
MAX_PLAUSIBLE_RATE_DPS = 90.0  # above this, two-point differencing has aliased


def wrap180(deg: float) -> float:
    """Wrap to [-180, 180)."""
    return ((deg + 180.0) % 360.0) - 180.0


def wrap360(deg: float) -> float:
    """Wrap to [0, 360)."""
    return deg % 360.0


def circular_mean(degs) -> Optional[float]:
    """Mean direction via unit vector sum. None if the sum is degenerate."""
    if not degs:
        return None
    s = sum(math.sin(math.radians(d)) for d in degs)
    c = sum(math.cos(math.radians(d)) for d in degs)
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        return None
    return wrap360(math.degrees(math.atan2(s, c)))


def circular_sigma(degs) -> Optional[float]:
    """
    Circular standard deviation in degrees.

    Uses the standard estimator sigma = sqrt(-2 ln R) where R is the mean
    resultant length. For tight distributions this agrees with the linear
    standard deviation; unlike the linear version it does not blow up when
    the data straddles the 0/360 seam.
    """
    n = len(degs)
    if n < 2:
        return None
    s = sum(math.sin(math.radians(d)) for d in degs) / n
    c = sum(math.cos(math.radians(d)) for d in degs) / n
    R = math.hypot(s, c)
    if R <= 1e-9:
        return 180.0
    if R >= 1.0:
        return 0.0
    return math.degrees(math.sqrt(-2.0 * math.log(R)))


class HeadingMonitor:
    """
    Tracks recent heading and attitude samples, computes dispersion, and
    decides whether the heading is trustworthy enough to steer on.

    Feed it whatever you have. It works with compass alone; it works better
    with pitch and roll; it defers to the Kalman filter's sigma when the
    filter is running.
    """

    def __init__(self, state: SystemState, window_sec: float = WINDOW_SEC) -> None:
        self.state = state
        self.window_sec = window_sec
        self._heading: Deque[Tuple[float, float]] = deque()
        self._pitch: Deque[Tuple[float, float]] = deque()
        self._roll: Deque[Tuple[float, float]] = deque()
        self._filter_sigma: Optional[float] = None
        self._filter_sigma_at: float = 0.0

    # -- ingestion ---------------------------------------------------------

    def add_heading(self, deg: float, when: Optional[float] = None) -> None:
        self._push(self._heading, wrap360(deg), when)

    def add_pitch(self, deg: float, when: Optional[float] = None) -> None:
        self._push(self._pitch, wrap180(deg), when)

    def add_roll(self, deg: float, when: Optional[float] = None) -> None:
        self._push(self._roll, wrap180(deg), when)

    def set_filter_sigma(self, sigma: Optional[float], when: Optional[float] = None) -> None:
        """Accept the Kalman filter's own heading standard deviation."""
        self._filter_sigma = sigma
        self._filter_sigma_at = when or time.time()

    def _push(self, dq: Deque, value: float, when: Optional[float]) -> None:
        t = when or time.time()
        dq.append((t, value))
        cutoff = t - self.window_sec
        while dq and dq[0][0] < cutoff:
            dq.popleft()

    # -- statistics --------------------------------------------------------

    def observed_sigma(self) -> Optional[float]:
        """Dispersion of the raw heading over the window."""
        vals = [v for _, v in self._heading]
        if len(vals) < MIN_SAMPLES:
            return None
        return circular_sigma(vals)

    def tilt_sigma(self) -> Tuple[Optional[float], Optional[float]]:
        """(pitch_sigma, roll_sigma) as plain standard deviations."""
        def sd(dq):
            vals = [v for _, v in dq]
            if len(vals) < MIN_SAMPLES:
                return None
            m = sum(vals) / len(vals)
            return math.sqrt(sum((v - m) ** 2 for v in vals) / (len(vals) - 1))

        return sd(self._pitch), sd(self._roll)

    def effective_sigma(self) -> Optional[float]:
        """
        Best available heading uncertainty.

        Prefer the filter's estimate when it is fresh, but never report
        better than what the raw data actually shows -- if the filter claims
        1 degree while the compass is swinging 12, take the pessimistic view.
        A filter that is overconfident is exactly the failure that puts a
        boat into a turn it should not make.
        """
        obs = self.observed_sigma()
        filt = None
        if self._filter_sigma is not None and (time.time() - self._filter_sigma_at) < 2.0:
            filt = self._filter_sigma

        if filt is None:
            return obs
        if obs is None:
            return filt
        return max(filt, min(obs, filt * 3.0))

    def rate_dps(self) -> Optional[float]:
        """
        Crude turn rate from the window endpoints, for display only.

        The Kalman filter's rate state is the one to steer on; this exists so
        the TUI has something to show before the filter is wired in.
        """
        if len(self._heading) < 2:
            return None
        t0, h0 = self._heading[0]
        t1, h1 = self._heading[-1]
        dt = t1 - t0
        if dt < 0.5:
            return None
        rate = wrap180(h1 - h0) / dt

        # Two-endpoint differencing is only meaningful if the heading did not
        # wrap more than half a turn between samples. Above ~90 deg/s that
        # assumption fails for this window length, and the result is an
        # aliasing artifact rather than a turn rate -- most visibly when a
        # candump is replayed faster than real time. Report nothing rather
        # than something false; the Kalman rate state replaces this anyway.
        if abs(rate) > MAX_PLAUSIBLE_RATE_DPS:
            return None
        return rate

    # -- source selection --------------------------------------------------

    def best_heading_signal(self):
        """
        Return the freshest usable heading Signal, or None.

        Note the subtlety this exists to avoid: `get_signal("fused_heading")
        or get_signal("compass_heading")` looks like a fallback but is not.
        Both signals are registered at construction, and a Signal instance is
        always truthy, so the `or` never reaches the second operand. That
        silently pinned every consumer to fused_heading -- which is None
        until the Kalman filter is wired in -- and blocked engagement on a
        perfectly good compass. Select on is_valid(), not on the object.
        """
        for name in ("fused_heading", "compass_heading"):
            sig = self.state.get_signal(name)
            if sig is not None and sig.is_valid():
                return sig
        return None

    # -- assessment --------------------------------------------------------

    def diagnose(self) -> Tuple[str, str]:
        """
        Returns (quality, explanation) where quality is one of
        GOOD / NOISY / BAD / UNKNOWN.
        """
        sigma = self.effective_sigma()
        if sigma is None:
            if not self._heading:
                return "UNKNOWN", "no heading samples"
            return "UNKNOWN", f"only {len(self._heading)} samples"

        p_sd, r_sd = self.tilt_sigma()
        tilt_active = max(p_sd or 0.0, r_sd or 0.0) > TILT_SIGMA_NOISY

        if sigma > SIGMA_ENGAGE_MAX:
            if tilt_active:
                return "BAD", (
                    f"sigma {sigma:.1f}deg with pitch/roll "
                    f"{p_sd or 0:.1f}/{r_sd or 0:.1f}deg -- sea state, slow down "
                    f"or head off the swell"
                )
            return "BAD", f"sigma {sigma:.1f}deg with flat attitude -- suspect compass"

        if sigma > SIGMA_WARN:
            if tilt_active:
                return "NOISY", (
                    f"sigma {sigma:.1f}deg, tilt "
                    f"{p_sd or 0:.1f}/{r_sd or 0:.1f}deg"
                )
            return "NOISY", f"sigma {sigma:.1f}deg"

        return "GOOD", f"sigma {sigma:.1f}deg"

    def ok_to_engage(self) -> Tuple[bool, str]:
        """
        Interlock. Returns (allowed, reason).

        Blocking engagement on bad heading is the single most valuable guard
        here: the PID will faithfully chase a garbage setpoint error and put
        the rudder hard over based on noise.
        """
        sig = self.best_heading_signal()
        if sig is None:
            return False, "no valid heading"

        quality, why = self.diagnose()
        if quality == "BAD":
            return False, why
        if quality == "UNKNOWN":
            return False, why
        return True, why

    # -- periodic ----------------------------------------------------------

    def update_state(self) -> None:
        """Push derived values into SystemState and manage heading faults."""
        st = self.state
        sigma = self.effective_sigma()
        if sigma is not None:
            st.set_signal("heading_sigma", round(sigma, 2))

        rate = self.rate_dps()
        if rate is not None and st.get_signal("yaw_rate").value is None:
            st.set_signal("yaw_rate", round(rate, 2))

        quality, why = self.diagnose()
        if quality == "BAD":
            st.raise_fault(F_HEADING_NOISE, why)
        elif quality == "NOISY":
            st.raise_fault(F_HEADING_NOISE, why)
        else:
            st.clear_fault(F_HEADING_NOISE)

        if self.best_heading_signal() is None:
            st.raise_fault(F_HEADING_INVALID, "no fresh heading source")
        else:
            st.clear_fault(F_HEADING_INVALID)
