"""
Tests for the heading node: decode and fusion priority.

The node (Teensy + NEO-M9N + MPU-9250) runs edge fusion and broadcasts a
finished heading on three proprietary IDs. See sensor_node/PROTOCOL.md.

The point of the node is to give a trustworthy heading at ANY speed, lifting
the wait-for-motion restriction the stuck compass forced. So the key behaviors
to pin are:
  * the three messages decode with correct scaling and offsets
  * a valid node heading takes priority over COG, even at rest
  * the node's sigma feeds the engage gate
  * a dead or invalid node degrades cleanly to the existing COG-primary logic
"""

from __future__ import annotations

import struct
import time

import can
import pytest


def heading_frame(hdg, sigma, yaw, source=3, valid=True, mag_ok=True, gps_ok=True,
                  holding=False):
    """Build a 0x18FF80E1 frame per PROTOCOL.md."""
    h = int(round(hdg * 100))
    sg = int(round(sigma * 100))
    yr = int(round(yaw * 100)) + 32768
    status = ((1 if valid else 0) | (2 if mag_ok else 0)
              | (4 if gps_ok else 0) | (8 if holding else 0))
    data = struct.pack("<HHHBB", h, sg, yr & 0xFFFF, source, status)
    return can.Message(arbitration_id=0x18FF80E1, data=data, is_extended_id=True)


def attitude_frame(pitch, roll, accel_g, valid=True):
    p = int(round(pitch * 100)) + 18000
    r = int(round(roll * 100)) + 18000
    a = int(round(accel_g * 1000))
    data = struct.pack("<HHHBB", p, r, a, 0, 1 if valid else 0)
    return can.Message(arbitration_id=0x18FF81E1, data=data, is_extended_id=True)


def health_frame(fix=3, sats=12, hdop=0.9, mag_cal=3, temp=22):
    data = struct.pack("<BBHBbH", fix, sats, int(round(hdop * 100)), mag_cal,
                       temp, 0)
    return can.Message(arbitration_id=0x18FF82E1, data=data, is_extended_id=True)


# ---------------------------------------------------------------------------
# Decode
# ---------------------------------------------------------------------------


class TestHeadingDecode:
    def test_heading_scaling(self, can_iface):
        out = can_iface.process_message(heading_frame(137.5, 1.8, -2.3))
        assert out["node_heading"] == pytest.approx(137.5)
        assert out["node_heading_sigma"] == pytest.approx(1.8)
        assert out["node_yaw_rate"] == pytest.approx(-2.3, abs=0.01)

    def test_yaw_rate_offset_zero_is_straight(self, can_iface):
        out = can_iface.process_message(heading_frame(100.0, 1.0, 0.0))
        assert out["node_yaw_rate"] == pytest.approx(0.0, abs=0.01)

    def test_yaw_rate_positive_and_negative(self, can_iface):
        assert can_iface.process_message(
            heading_frame(100, 1, 50.0))["node_yaw_rate"] == pytest.approx(50.0, abs=0.01)
        assert can_iface.process_message(
            heading_frame(100, 1, -50.0))["node_yaw_rate"] == pytest.approx(-50.0, abs=0.01)

    def test_invalid_status_nulls_heading(self, can_iface):
        """status bit0 clear -> heading is None, so the HMI will not steer by it."""
        out = can_iface.process_message(heading_frame(137.5, 1.8, 0.0, valid=False))
        assert out["node_heading"] is None
        assert out["node_heading_valid"] is False

    def test_status_flags_decoded(self, can_iface):
        out = can_iface.process_message(
            heading_frame(100, 1, 0, mag_ok=True, gps_ok=False, holding=True))
        assert out["node_mag_cal_ok"] is True
        assert out["node_gps_ok"] is False
        assert out["node_holding"] is True

    def test_heading_at_north(self, can_iface):
        out = can_iface.process_message(heading_frame(0.0, 0.5, 0.0))
        assert out["node_heading"] == pytest.approx(0.0)

    def test_heading_near_full_circle(self, can_iface):
        out = can_iface.process_message(heading_frame(359.99, 0.5, 0.0))
        assert out["node_heading"] == pytest.approx(359.99, abs=0.01)


class TestAttitudeDecode:
    def test_pitch_roll_offset(self, can_iface):
        out = can_iface.process_message(attitude_frame(-1.3, 5.2, 1.0))
        assert out["node_pitch"] == pytest.approx(-1.3)
        assert out["node_roll"] == pytest.approx(5.2)

    def test_level_is_zero(self, can_iface):
        out = can_iface.process_message(attitude_frame(0.0, 0.0, 1.0))
        assert out["node_pitch"] == pytest.approx(0.0)
        assert out["node_roll"] == pytest.approx(0.0)

    def test_accel_magnitude(self, can_iface):
        out = can_iface.process_message(attitude_frame(0, 0, 1.5))
        assert out["node_accel_mag"] == pytest.approx(1.5)

    def test_invalid_nulls_attitude(self, can_iface):
        out = can_iface.process_message(attitude_frame(10, 10, 1.0, valid=False))
        assert out["node_pitch"] is None
        assert out["node_roll"] is None


class TestHealthDecode:
    def test_fields(self, can_iface):
        out = can_iface.process_message(health_frame(fix=3, sats=14, hdop=0.8,
                                                     mag_cal=3, temp=25))
        assert out["node_fix_type"] == 3
        assert out["node_num_sats"] == 14
        assert out["node_hdop"] == pytest.approx(0.8)
        assert out["node_mag_cal"] == 3
        assert out["node_temp_c"] == 25

    def test_negative_temp(self, can_iface):
        out = can_iface.process_message(health_frame(temp=-5))
        assert out["node_temp_c"] == -5


# ---------------------------------------------------------------------------
# Fusion priority
# ---------------------------------------------------------------------------


class TestNodePriority:
    def _feed(self, bridge, can_iface, frame):
        out = can_iface.process_message(frame)
        bridge.on_decoded(out)
        return out

    def test_node_beats_cog_at_rest(self, state, bridge, can_iface):
        """
        The headline capability: a valid node gives a heading at rest, which
        COG cannot. Steering is enabled (cog_lock True) even at zero speed.
        """
        state.set_signal("sog", 0.0)
        state.set_signal("cog", 200.0)
        state.set_signal("compass_heading", 264.0)
        self._feed(bridge, can_iface, heading_frame(137.5, 1.8, 0.0))
        bridge.derive_fused()
        assert state.signal_value("fused_heading") == pytest.approx(137.5)
        assert state.heading_source == "NODE"
        assert state.cog_lock is True

    def test_node_beats_cog_when_both_valid(self, state, bridge, can_iface):
        state.set_signal("sog", 10.0)
        state.set_signal("cog", 200.0)
        self._feed(bridge, can_iface, heading_frame(150.0, 1.0, 0.0))
        bridge.derive_fused()
        assert state.signal_value("fused_heading") == pytest.approx(150.0)
        assert state.heading_source == "NODE"

    def test_node_sigma_feeds_engage_gate(self, state, bridge, can_iface):
        """
        The node's own filter confidence drives the engage interlock, instead
        of the raw compass scatter the monitor would otherwise compute.
        """
        self._feed(bridge, can_iface, heading_frame(150.0, 2.5, 0.0))
        bridge.derive_fused()
        assert bridge.monitor.effective_sigma() == pytest.approx(2.5, abs=0.1)

    def test_invalid_node_falls_back_to_cog(self, state, bridge, can_iface):
        state.set_signal("sog", 10.0)
        state.set_signal("cog", 200.0)
        self._feed(bridge, can_iface, heading_frame(150.0, 1.0, 0.0, valid=False))
        bridge.derive_fused()
        assert state.signal_value("fused_heading") == pytest.approx(200.0)
        assert state.heading_source == "COG"

    def test_stale_node_falls_back_to_cog(self, state, bridge, can_iface):
        """A node that stopped talking degrades to COG-primary, no special case."""
        out = can_iface.process_message(heading_frame(150.0, 1.0, 0.0))
        # inject with an old timestamp so it reads stale
        state.set_signal("node_heading", out["node_heading"], time.time() - 60)
        state.set_signal("sog", 10.0)
        state.set_signal("cog", 200.0)
        bridge.derive_fused()
        assert state.heading_source == "COG"

    def test_stale_node_at_rest_gives_no_heading(self, state, bridge, can_iface):
        """
        Dead node + no steerage way = center and wait, exactly as before the
        node existed. The node adds capability without adding a failure mode.
        """
        state.set_signal("node_heading", 150.0, time.time() - 60)
        state.set_signal("sog", 0.0)
        state.set_signal("cog", 200.0)
        bridge.derive_fused()
        assert state.signal_value("fused_heading") is None
        assert state.heading_source == "NONE"


class TestNodeAttitudeFeedsMonitor:
    def test_roll_now_available(self, state, bridge, can_iface):
        """
        Roll was never published on the boat bus. The node finally provides it,
        so the heading monitor's sea-state attribution can use it.
        """
        out = can_iface.process_message(attitude_frame(-1.3, 8.5, 1.2))
        bridge.on_decoded(out)
        assert state.signal_value("roll") == pytest.approx(8.5)

    def test_pitch_from_node(self, state, bridge, can_iface):
        out = can_iface.process_message(attitude_frame(-2.0, 0.0, 1.0))
        bridge.on_decoded(out)
        assert state.signal_value("pitch") == pytest.approx(-2.0)


# ---------------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------------


class TestNodeSources:
    def test_three_node_sources_registered(self, state):
        from hmi_bridge import WATCHED

        assert 0x18FF80E1 in WATCHED
        assert 0x18FF81E1 in WATCHED
        assert 0x18FF82E1 in WATCHED

    def test_node_ids_do_not_collide(self):
        """The node IDs must not overlap the boat's existing PGNs."""
        from hmi_bridge import WATCHED

        boat_ids = {0x18F01D21, 0x19F10D13, 0x09F8021C, 0x09F8011C,
                    0x18FEE81C, 0x0CF00400, 0x09F112F8, 0x18FF50E0}
        node_ids = {0x18FF80E1, 0x18FF81E1, 0x18FF82E1}
        assert boat_ids.isdisjoint(node_ids)
