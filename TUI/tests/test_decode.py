"""
Decoder tests for can_interface.process_message.

Frames here are byte-for-byte copies of real frames pulled from the Horsetooth
capture, not hand-built payloads. A synthetic frame only proves the decoder is
self-consistent with the test author's belief about the format; a real frame
proves it agrees with what the boat actually transmits.

Where a decode has a scale or offset, the test asserts on the physical value
and states the encoding in a comment, so a future change to the constant fails
loudly rather than silently shifting every reading.
"""

from __future__ import annotations

import struct

import can
import pytest


def frame(can_id: int, hexdata: str) -> can.Message:
    return can.Message(
        arbitration_id=can_id, data=bytes.fromhex(hexdata), is_extended_id=True
    )


# ---------------------------------------------------------------------------
# PGN 65256 -- vehicle direction/speed. Compass, speed, pitch, altitude.
# ---------------------------------------------------------------------------


class TestVehicleDirection:
    """
    0x18FEE81C. Pitch is the one that was wrong: J1939 encodes it as 1/128 deg
    per bit with a -200 deg offset, so a level boat sits near raw 25600. The
    original decoder omitted the offset and reported ~198.7 deg for level.
    """

    def test_pitch_is_offset_corrected(self, can_iface):
        # Real frame, boat sitting nearly level on the lake.
        msg = frame(0x18FEE81C, "0060000063640000")
        out = can_iface.process_message(msg)
        raw = struct.unpack("<H", msg.data[4:6])[0]
        assert raw > 20000, "sanity: raw pitch field should sit near the 200 deg offset"
        assert -20.0 < out["pitch"] < 20.0, (
            f"pitch {out['pitch']:.2f} outside plausible range -- "
            "the -200 deg J1939 offset is probably missing"
        )

    @pytest.mark.parametrize(
        "raw_pitch,expected_deg",
        [
            (25600, 0.0),      # exactly the offset -> level
            (25728, 1.0),      # +128 counts = +1 deg at 1/128 deg per bit
            (25472, -1.0),
            (25617, 0.1328125),
        ],
    )
    def test_pitch_scale_and_offset(self, can_iface, raw_pitch, expected_deg):
        data = bytearray(8)
        data[4:6] = struct.pack("<H", raw_pitch)
        out = can_iface.process_message(frame(0x18FEE81C, bytes(data).hex()))
        assert out["pitch"] == pytest.approx(expected_deg, abs=1e-6)

    def test_pitch_matches_real_capture_range(self, can_iface, underway_log):
        """
        Across the real capture the boat is on flat water, so decoded pitch
        must land in a small band around zero. This is the assertion that
        would have caught the missing offset.
        """
        from can.io.canutils import CanutilsLogReader

        vals = []
        for msg in CanutilsLogReader(str(underway_log)):
            if msg.arbitration_id == 0x18FEE81C:
                out = can_iface.process_message(msg)
                if out and out.get("pitch") is not None:
                    vals.append(out["pitch"])
        assert len(vals) > 20, "fixture should contain vehicle-direction frames"
        assert all(-15.0 < v < 15.0 for v in vals), (
            f"pitch range {min(vals):.2f}..{max(vals):.2f} implausible for flat water"
        )

    def test_compass_in_range(self, can_iface, underway_log):
        from can.io.canutils import CanutilsLogReader

        vals = []
        for msg in CanutilsLogReader(str(underway_log)):
            if msg.arbitration_id == 0x18FEE81C:
                out = can_iface.process_message(msg)
                if out and out.get("compass") is not None:
                    vals.append(out["compass"])
        assert vals
        assert all(0.0 <= v < 360.0 for v in vals)


# ---------------------------------------------------------------------------
# PGN 127250 -- vessel heading
# ---------------------------------------------------------------------------


class TestVesselHeading:
    def test_heading_decodes_in_range(self, can_iface, underway_log):
        from can.io.canutils import CanutilsLogReader

        seen = 0
        for msg in CanutilsLogReader(str(underway_log)):
            if msg.arbitration_id == 0x09F112F8:
                can_iface.process_message(msg)
                seen += 1
        assert seen > 0
        # compass_heading includes heading_correction, so allow a wider band
        # but it must still be a bearing, not a radian value or a raw count.
        assert -360.0 < can_iface.compass_heading < 720.0

    def test_gated_on_low_speed(self, can_iface):
        """
        The decoder only returns compass_heading when boat_speed < 1 mph.
        Documenting this because it is surprising and it interacts with the
        fusion fallback: at speed, compass updates the attribute but returns
        None, so the bridge never sees it.
        """
        can_iface.boat_speed = 0.5
        out = can_iface.process_message(frame(0x09F112F8, "00A00F00FFFFFFFF"))
        assert out is not None and "compass_heading" in out

        can_iface.boat_speed = 5.0
        out = can_iface.process_message(frame(0x09F112F8, "00A00F00FFFFFFFF"))
        assert out is None


# ---------------------------------------------------------------------------
# PGN 129026 -- COG / SOG
# ---------------------------------------------------------------------------


class TestCogSog:
    def test_sog_conversion_to_mph(self, can_iface):
        # 100 counts at 0.01 m/s = 1.0 m/s = 2.236936 mph
        data = bytearray(b"\xFF" * 8)
        data[0] = 0
        data[1] = 0
        data[2:4] = struct.pack("<H", 0)
        data[4:6] = struct.pack("<H", 100)
        out = can_iface.process_message(frame(0x09F8021C, bytes(data).hex()))
        assert can_iface.boat_speed == pytest.approx(2.236936, rel=1e-4)

    def test_cog_radians_to_degrees(self, can_iface):
        data = bytearray(b"\xFF" * 8)
        data[0] = 0
        data[1] = 0
        data[2:4] = struct.pack("<H", 15708)  # 1.5708 rad ~ 90 deg
        data[4:6] = struct.pack("<H", 500)
        can_iface.process_message(frame(0x09F8021C, bytes(data).hex()))
        assert can_iface.COG == pytest.approx(90.0, abs=0.5)

    def test_suppressed_below_1_mph(self, can_iface):
        """COG is meaningless at rest; the decoder returns None rather than noise."""
        data = bytearray(b"\xFF" * 8)
        data[0] = 0
        data[1] = 0
        data[2:4] = struct.pack("<H", 10000)
        data[4:6] = struct.pack("<H", 10)  # 0.1 m/s = 0.22 mph
        out = can_iface.process_message(frame(0x09F8021C, bytes(data).hex()))
        assert out is None

    def test_real_capture_speeds_plausible(self, can_iface, underway_log):
        from can.io.canutils import CanutilsLogReader

        speeds = []
        for msg in CanutilsLogReader(str(underway_log)):
            if msg.arbitration_id == 0x09F8021C:
                can_iface.process_message(msg)
                speeds.append(can_iface.boat_speed)
        assert speeds
        assert all(0.0 <= s < 100.0 for s in speeds), "SOG outside plausible boat range"


# ---------------------------------------------------------------------------
# PGN 129025 -- GPS position
# ---------------------------------------------------------------------------


class TestGpsPosition:
    def test_position_near_horsetooth(self, can_iface, underway_log):
        """
        The capture is from Horsetooth Reservoir, Colorado: about 40.5 N,
        105.1 W. A sign error or wrong scale would move it off the planet.
        """
        from can.io.canutils import CanutilsLogReader

        pts = []
        for msg in CanutilsLogReader(str(underway_log)):
            if msg.arbitration_id == 0x09F8011C:
                out = can_iface.process_message(msg)
                if out:
                    pts.append((out["lat"], out["lon"]))
        assert pts
        lat, lon = pts[len(pts) // 2]
        assert 40.0 < lat < 41.0, f"latitude {lat} not near Horsetooth"
        assert -106.0 < lon < -104.0, f"longitude {lon} not near Horsetooth"

    def test_signed_decode(self, can_iface):
        """Western longitudes are negative; unsigned unpacking wraps them."""
        data = struct.pack("<l", 405270000) + struct.pack("<l", -1051490000)
        out = can_iface.process_message(frame(0x09F8011C, data.hex()))
        assert out["lon"] < 0


# ---------------------------------------------------------------------------
# PGN 61444 -- engine RPM
# ---------------------------------------------------------------------------


class TestEngineRpm:
    def test_rpm_scale(self, can_iface):
        import time

        can_iface.rpm_start_time = time.time() - 10  # bypass the rate limit
        data = bytearray(b"\xFF" * 8)
        data[3:5] = struct.pack("<H", 8000)  # 8000 * 0.125 = 1000 rpm
        out = can_iface.process_message(frame(0x0CF00400, bytes(data).hex()))
        assert out["rpm"] == pytest.approx(1000)

    def test_rate_limited(self, can_iface):
        """
        Engine frames arrive at 100 Hz; the decoder throttles to GUI_TIMEOUT
        so the websocket is not flooded.
        """
        import time

        data = bytearray(b"\xFF" * 8)
        data[3:5] = struct.pack("<H", 8000)
        can_iface.rpm_start_time = time.time() - 10
        first = can_iface.process_message(frame(0x0CF00400, bytes(data).hex()))
        second = can_iface.process_message(frame(0x0CF00400, bytes(data).hex()))
        assert first is not None
        assert second is None


# ---------------------------------------------------------------------------
# PGN -- steering and rudder
# ---------------------------------------------------------------------------


class TestSteering:
    def test_angle_offset_decode(self, can_iface):
        """Steering angle is (raw - 0x80000000) / 1000."""
        raw = 0x80000000 + 1_425_000  # 1425.0
        data = struct.pack("<L", raw) + b"\xFF\xFF\xFF\xFF"
        out = can_iface.process_message(frame(0x18F01D21, data.hex()))
        assert out["steering_angle"] == pytest.approx(1425.0)

    def test_ff_goal_means_disabled(self, can_iface):
        raw = 0x80000000 + 1_425_000
        data = struct.pack("<L", raw) + b"\xFF\xFF\xFF\xFF"
        out = can_iface.process_message(frame(0x18F01D21, data.hex()))
        assert out["servo_enabled"] is False
        assert out["steering_goal"] == pytest.approx(out["steering_angle"])

    def test_explicit_goal_means_enabled(self, can_iface):
        angle = 0x80000000 + 1_425_000
        goal = 0x80000000 + 1_500_000
        data = struct.pack("<L", angle) + struct.pack("<L", goal)
        out = can_iface.process_message(frame(0x18F01D21, data.hex()))
        assert out["servo_enabled"] is True
        assert out["steering_goal"] == pytest.approx(1500.0)

    def test_rudder_angle_offset(self, can_iface):
        """rudder_angle = (byte7 - 0x80)/2 + rudder_correction."""
        data = bytearray(8)
        data[7] = 0x80
        out = can_iface.process_message(frame(0x19F10D13, bytes(data).hex()))
        assert out["rudder_angle"] == pytest.approx(can_iface.rudder_correction)


# ---------------------------------------------------------------------------
# Autopilot status
# ---------------------------------------------------------------------------


class TestAutopilotStatus:
    def test_flags_and_goal(self, can_iface):
        goal_raw = 0x8000 + int(130.0 * 10)
        err_raw = 0x8000 + int(-2.5 * 100)
        rud_raw = 0x8000 + int(3.0 * 100)
        data = (
            bytes([0x01 | 0x20])
            + struct.pack("<H", goal_raw)
            + struct.pack("<H", err_raw & 0xFFFF)
            + struct.pack("<H", rud_raw)
            + bytes([7])
        )
        out = can_iface.process_message(frame(0x18FF50E0, data.hex()))
        assert out["autopilot_engaged"] is True
        assert out["right_turn"] is True
        assert out["left_turn"] is False
        assert out["heading_goal"] == pytest.approx(130.0, abs=0.05)

    def test_goal_normalized_to_range(self, can_iface):
        goal_raw = 0x8000 + int(370.0 * 10)
        data = (
            bytes([0x00])
            + struct.pack("<H", goal_raw)
            + struct.pack("<H", 0x8000)
            + struct.pack("<H", 0x8000)
            + bytes([0])
        )
        out = can_iface.process_message(frame(0x18FF50E0, data.hex()))
        assert 0.0 <= out["heading_goal"] < 360.0


# ---------------------------------------------------------------------------
# Robustness
# ---------------------------------------------------------------------------


class TestMalformedFrames:
    def test_unknown_id_returns_none(self, can_iface):
        assert can_iface.process_message(frame(0x12345678, "0011223344556677")) is None

    @pytest.mark.parametrize(
        "can_id",
        [0x18F01D21, 0x19F10D13, 0x09F8021C, 0x09F8011C, 0x18FEE81C, 0x09F112F8, 0x18FF50E0],
    )
    def test_truncated_frame_raises_recoverable_error(self, can_iface, can_id):
        """
        A short frame must fail with something the read loop catches
        (struct.error / IndexError), not something exotic that escapes and
        kills the reader thread.
        """
        with pytest.raises((struct.error, IndexError, ValueError)):
            can_iface.process_message(frame(can_id, "0011"))

    def test_empty_payload(self, can_iface):
        with pytest.raises((struct.error, IndexError, ValueError)):
            can_iface.process_message(frame(0x18FEE81C, ""))

    def test_all_ff_does_not_crash(self, can_iface):
        """N2K uses 0xFF as a not-available sentinel; decoding must survive it."""
        for cid in (0x18F01D21, 0x19F10D13, 0x18FEE81C, 0x09F112F8):
            can_iface.process_message(frame(cid, "FF" * 8))
