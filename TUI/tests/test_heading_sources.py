"""
Tests for the NAME-bound heading source registry and multi-source heading.

The boat now has multiple heading sources (wall compass 0xF8, GPS 24xd 0x19),
and their addresses can change. The registry binds each by its stable J1939
NAME identity and picks the preferred one; the fusion consumes that choice.

Key behaviors:
  * a source is found by identity regardless of its current address
  * priority + enabled decide which source wins
  * the uncalibrated 24xd (sentinel heading) never wins until calibrated
  * a stale primary falls back to the next enabled source, then to COG
  * enabling the 24xd (post-calibration) makes it take over automatically
"""

from __future__ import annotations

import struct
import time

import can
import pytest

from heading_sources import HeadingSourceRegistry


# Confirmed identities from the 2026-07-25 address claims.
ID_24XD = 1602535       # mfg 229, function 145
ID_WALL = 1039212       # mfg 229, function 140


class TestRegistryBinding:
    def test_resolves_address_from_claim(self):
        r = HeadingSourceRegistry()
        r.note_claim(0x19, 229, 145, ID_24XD)
        assert r.source_address_for(ID_24XD) == 0x19

    def test_address_change_is_tracked(self):
        """NAME-binding's whole point: an address change just updates the map."""
        r = HeadingSourceRegistry()
        r.note_claim(0x19, 229, 145, ID_24XD)
        r.note_claim(0x1A, 229, 145, ID_24XD)  # 24xd moved
        assert r.source_address_for(ID_24XD) == 0x1A

    def test_unknown_device_ignored(self):
        r = HeadingSourceRegistry()
        r.note_claim(0x50, 999, 0, 424242)  # not a heading source
        # nothing blows up; no heading source resolves to it
        assert all(p.source_address != 0x50 for p in r.profiles())


class TestRegistryPriority:
    def test_wall_compass_wins_before_24xd_enabled(self):
        r = HeadingSourceRegistry()
        r.note_claim(0x19, 229, 145, ID_24XD)
        r.note_claim(0xF8, 229, 140, ID_WALL)
        best = r.best_source()
        assert best.label == "Wall compass"  # 24xd disabled by default

    def test_24xd_wins_once_enabled(self):
        r = HeadingSourceRegistry()
        r.note_claim(0x19, 229, 145, ID_24XD)
        r.note_claim(0xF8, 229, 140, ID_WALL)
        r.set_enabled(ID_24XD, True)
        assert r.best_source().label == "GPS 24xd"

    def test_disabled_source_never_wins(self):
        r = HeadingSourceRegistry()
        r.note_claim(0x19, 229, 145, ID_24XD)
        r.note_claim(0xF8, 229, 140, ID_WALL)
        r.set_enabled(ID_24XD, True)
        r.set_enabled(ID_WALL, False)
        r.set_enabled(ID_24XD, False)
        assert r.best_source() is None

    def test_unbound_source_not_selected(self):
        """A source that has not claimed yet cannot be chosen."""
        r = HeadingSourceRegistry()
        r.set_enabled(ID_24XD, True)
        # no note_claim -> no address
        assert r.best_source() is None

    def test_priority_change_reorders(self):
        r = HeadingSourceRegistry()
        r.note_claim(0x19, 229, 145, ID_24XD)
        r.note_claim(0xF8, 229, 140, ID_WALL)
        r.set_enabled(ID_24XD, True)
        r.set_priority(ID_24XD, 99)  # demote below wall
        assert r.best_source().label == "Wall compass"

    def test_enabled_by_priority_order(self):
        r = HeadingSourceRegistry()
        r.note_claim(0x19, 229, 145, ID_24XD)
        r.note_claim(0xF8, 229, 140, ID_WALL)
        r.set_enabled(ID_24XD, True)
        order = [p.label for p in r.enabled_sources_by_priority()]
        assert order == ["GPS 24xd", "Wall compass"]


# ---------------------------------------------------------------------------
# Decode: heading from any address, sentinel rejection
# ---------------------------------------------------------------------------


def heading_frame(sa, deg=None):
    """PGN 127250 from a given source address. deg=None -> not-available."""
    arb = (2 << 26) | (0x1F112 << 8) | sa
    if deg is None:
        raw = 0xFFFE
    else:
        raw = int(deg / 57.29578 / 0.0001)
    data = bytes([0xFF]) + struct.pack("<H", raw) + bytes([0, 0, 0xFF, 0x7F, 0xFD])
    return can.Message(arbitration_id=arb, data=data, is_extended_id=True)


class TestMultiSourceDecode:
    def test_heading_from_wall_compass_address(self, can_iface):
        can_iface.boat_speed = 0.0
        can_iface.heading_correction = 0.0  # isolate the decode from any offset
        out = can_iface.process_message(heading_frame(0xF8, 170.0))
        assert out is not None
        assert out["heading_sa"] == 0xF8
        assert out["compass_heading"] == pytest.approx(170.0, abs=0.5)

    def test_heading_from_24xd_address(self, can_iface):
        """The 24xd's heading arrives on a different arbitration ID (SA 0x19)."""
        can_iface.boat_speed = 0.0
        out = can_iface.process_message(heading_frame(0x19, 182.0))
        assert out is not None
        assert out["heading_sa"] == 0x19

    def test_sentinel_rejected(self, can_iface):
        """
        The uncalibrated 24xd sends 0xFFFE. It must not decode to ~375 deg --
        that was the bug in the 08:48 screenshot.
        """
        can_iface.boat_speed = 0.0
        out = can_iface.process_message(heading_frame(0x19, None))
        assert out is None

    def test_sentinel_ffff_rejected(self, can_iface):
        can_iface.boat_speed = 0.0
        arb = (2 << 26) | (0x1F112 << 8) | 0x19
        data = bytes([0xFF, 0xFF, 0xFF, 0, 0, 0xFF, 0x7F, 0xFD])
        msg = can.Message(arbitration_id=arb, data=data, is_extended_id=True)
        assert can_iface.process_message(msg) is None


# ---------------------------------------------------------------------------
# Fusion integration
# ---------------------------------------------------------------------------


class TestFusionSourceSelection:
    def _setup(self, bridge):
        bridge.heading_registry.note_claim(0x19, 229, 145, ID_24XD)
        bridge.heading_registry.note_claim(0xF8, 229, 140, ID_WALL)

    def test_wall_compass_feeds_fusion_before_cal(self, state, bridge):
        self._setup(bridge)
        bridge.on_decoded({"compass_heading": 170.0, "heading_sa": 0xF8,
                           "heading_value": 170.0})
        bridge.derive_fused()
        assert state.signal_value("fused_heading") == pytest.approx(170.0)
        assert state.heading_source == "WALL_COMPASS"

    def test_24xd_takes_over_when_enabled(self, state, bridge):
        self._setup(bridge)
        bridge.heading_registry.set_enabled(ID_24XD, True)
        bridge.on_decoded({"compass_heading": 182.5, "heading_sa": 0x19,
                           "heading_value": 182.5})
        bridge.on_decoded({"compass_heading": 170.0, "heading_sa": 0xF8,
                           "heading_value": 170.0})
        bridge.derive_fused()
        assert state.signal_value("fused_heading") == pytest.approx(182.5)
        assert state.heading_source == "GPS_24XD"

    def test_stale_primary_falls_back_to_wall(self, state, bridge):
        self._setup(bridge)
        bridge.heading_registry.set_enabled(ID_24XD, True)
        bridge.on_decoded({"compass_heading": 182.5, "heading_sa": 0x19,
                           "heading_value": 182.5})
        bridge.on_decoded({"compass_heading": 170.0, "heading_sa": 0xF8,
                           "heading_value": 170.0})
        # make the 24xd entry stale
        bridge._heading_by_sa[0x19] = (182.5, time.time() - 5)
        bridge.derive_fused()
        assert state.heading_source == "WALL_COMPASS"

    def test_heading_source_at_rest(self, state, bridge):
        """
        A magnetic heading source works at rest, unlike COG. With the wall
        compass fresh and the boat stopped, fusion still gives a heading and a
        lock (steering enabled at the dock).
        """
        self._setup(bridge)
        state.set_signal("sog", 0.0)
        bridge.on_decoded({"compass_heading": 170.0, "heading_sa": 0xF8,
                           "heading_value": 170.0})
        bridge.derive_fused()
        assert state.signal_value("fused_heading") == pytest.approx(170.0)
        assert state.cog_lock is True
