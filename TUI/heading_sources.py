"""
Heading source registry: bind heading sources by J1939 NAME, not address.

The boat has several devices that can report heading, and their J1939 source
addresses can change (address arbitration on power-up, a device replaced, the
24xd joining the bus and bumping someone). Binding by the stable NAME identity
instead of the momentary address means the fusion "just works" no matter which
address a device currently holds.

Each heading source has:
  - a stable identity (from its address claim NAME)
  - a priority (lower number = preferred)
  - an enabled flag (you can turn a source off entirely)
  - a live source address, resolved from claims as they arrive

The fusion asks this registry for the best currently-valid heading. To change
which source wins, edit the PROFILES below (priority) or toggle enabled at
runtime from the TUI / calibration tool. That is the ONE place to change source
selection -- derive_fused just consumes best_heading().

------------------------------------------------------------------------------
TO CHANGE SOURCE PRIORITY OR TURN A SOURCE OFF:
  - Edit DEFAULT_PROFILES below (priority number, or enabled=False), OR
  - Call registry.set_enabled(key, False) / registry.set_priority(key, n) at
    runtime (the calibration TUI and the main TUI both do this).
------------------------------------------------------------------------------
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


# A device is identified by (manufacturer_code, function, identity) from its
# NAME. That triple is unique and stable across power cycles and address
# changes. identity alone is actually unique, but including mfg+function makes
# the profiles self-documenting.
NameKey = Tuple[int, int, int]  # (mfg_code, function, identity)


@dataclass
class HeadingSourceProfile:
    """Configuration for one heading source, bound by NAME."""

    key: NameKey
    label: str                 # human name for the TUI
    priority: int              # lower = preferred
    enabled: bool = True
    # The PGN/signal this source's heading arrives on. For all current devices
    # that is PGN 127250 (vessel heading), decoded into the "compass_heading"
    # signal, but the field is here so a future source on a different PGN slots
    # in without special-casing.
    signal: str = "compass_heading"
    # Resolved at runtime from address claims. None until the device claims.
    source_address: Optional[int] = None


# ---------------------------------------------------------------------------
# Device profiles. Identities confirmed from the 2026-07-25 address claims.
#
# EDIT HERE to change which heading source is primary, or to disable one.
# ---------------------------------------------------------------------------

DEFAULT_PROFILES: List[HeadingSourceProfile] = [
    # The GPS 24xd: full attitude + magnetic variation + heading. Once
    # calibrated it is the best sensor, so it is priority 1. It is created
    # DISABLED until calibration completes -- the calibration tool enables it
    # (see calibrate_compass_tui.py). An uncalibrated 24xd sends the
    # data-not-available sentinel anyway, so even if enabled it would not
    # produce a heading until calibrated; starting disabled just makes the
    # intent explicit.
    HeadingSourceProfile(
        key=(229, 145, 1602535),
        label="GPS 24xd",
        priority=1,
        enabled=False,   # <- calibration flips this to True
    ),
    # The wall-mounted Garmin 3-axis compass. Proven, already reads ~south.
    # Priority 2: the fallback until the 24xd is calibrated, and the backup
    # after.
    HeadingSourceProfile(
        key=(229, 140, 1039212),
        label="Wall compass",
        priority=2,
        enabled=True,
    ),
    # The heading node (Teensy) from sensor_node/, if present. Bound by the
    # NAME it claims; leave disabled until that board is on the bus. See
    # sensor_node/PROTOCOL.md. Priority 3.
    # HeadingSourceProfile(
    #     key=(<mfg>, <func>, <identity>),
    #     label="Heading node",
    #     priority=3,
    #     enabled=False,
    # ),
]


class HeadingSourceRegistry:
    """
    Thread-safe registry of heading sources, bound by NAME.

    The CAN reader calls note_claim() as address claims arrive to keep the
    address<->identity mapping current. derive_fused() calls best_source() to
    pick which heading to use.
    """

    def __init__(self, profiles: Optional[List[HeadingSourceProfile]] = None):
        import copy
        self._lock = threading.RLock()
        # Deep-copy so each registry owns its profiles. DEFAULT_PROFILES is a
        # module-level list of dataclass instances; without copying, every
        # registry would mutate the same shared objects (enabling the 24xd in
        # one would enable it everywhere, and tests would leak into each other).
        self._profiles: List[HeadingSourceProfile] = copy.deepcopy(
            profiles if profiles is not None else DEFAULT_PROFILES
        )
        # identity -> profile, for fast claim resolution
        self._by_identity: Dict[int, HeadingSourceProfile] = {
            p.key[2]: p for p in self._profiles
        }

    # -- claim resolution --------------------------------------------------

    def note_claim(self, source_addr: int, mfg_code: int, function: int,
                   identity: int) -> None:
        """
        Called when an address claim is decoded. If the claiming device is a
        known heading source, record its current address. Binding on identity
        means an address change just updates the mapping.
        """
        with self._lock:
            prof = self._by_identity.get(identity)
            if prof is not None:
                prof.source_address = source_addr

    # -- selection ---------------------------------------------------------

    def best_source(self) -> Optional[HeadingSourceProfile]:
        """
        The highest-priority enabled source that has resolved an address.
        Returns None if nothing qualifies. Does NOT check heading validity or
        freshness -- the caller does that against the actual signal, because
        freshness lives in SystemState. This just answers "which source should
        I prefer right now."
        """
        with self._lock:
            candidates = [
                p for p in self._profiles
                if p.enabled and p.source_address is not None
            ]
            if not candidates:
                return None
            return min(candidates, key=lambda p: p.priority)

    def enabled_sources_by_priority(self) -> List[HeadingSourceProfile]:
        """All enabled sources, best first. For fallback iteration."""
        with self._lock:
            return sorted(
                (p for p in self._profiles if p.enabled),
                key=lambda p: p.priority,
            )

    def source_address_for(self, identity: int) -> Optional[int]:
        with self._lock:
            p = self._by_identity.get(identity)
            return p.source_address if p else None

    # -- runtime control (used by the TUIs) --------------------------------

    def set_enabled(self, identity: int, enabled: bool) -> None:
        with self._lock:
            p = self._by_identity.get(identity)
            if p is not None:
                p.enabled = enabled

    def set_priority(self, identity: int, priority: int) -> None:
        with self._lock:
            p = self._by_identity.get(identity)
            if p is not None:
                p.priority = priority

    def profiles(self) -> List[HeadingSourceProfile]:
        with self._lock:
            return list(self._profiles)

    def snapshot(self) -> List[dict]:
        """For the TUI / web to show source status."""
        with self._lock:
            return [
                {
                    "label": p.label,
                    "identity": p.key[2],
                    "priority": p.priority,
                    "enabled": p.enabled,
                    "address": p.source_address,
                    "bound": p.source_address is not None,
                }
                for p in sorted(self._profiles, key=lambda x: x.priority)
            ]
