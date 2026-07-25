"""
Regression tests: replay real captures end-to-end through the real decode path.

These are the tests that catch what unit tests cannot. A unit test proves a
function does what its author intended; replaying an actual capture proves the
whole chain agrees with what the boat transmits.

Golden values are bounds, not exact numbers. Pinning an exact float would make
the suite fail on any legitimate improvement to the filter. Pinning a physical
range ("pitch on flat water is within 15 degrees of level") fails only when
something is genuinely wrong.

Two captures, deliberately chosen for opposite regimes:

  underway_fast.log   3-4 mph, all eight PGNs including autopilot status
  trolling_slow.log   98% of samples below the 1.6 mph COG cutoff, and no
                      autopilot status frames at all

The second is not a synthetic fault case. It is what the boat actually sends
while trolling, which is exactly when the COG-based heading falls apart.
"""

from __future__ import annotations

import time

import pytest

from hmi_state import F_HEADING_NOISE, F_STALE

pytestmark = pytest.mark.regression


# ---------------------------------------------------------------------------
# Decode integrity
# ---------------------------------------------------------------------------


class TestDecodeIntegrity:
    def test_no_decode_errors_underway(self, wired, underway_log):
        wired.feed(underway_log)
        assert wired.errors == [], f"decode errors: {wired.errors[:5]}"

    def test_no_decode_errors_trolling(self, wired, trolling_log):
        wired.feed(trolling_log)
        assert wired.errors == [], f"decode errors: {wired.errors[:5]}"

    def test_decode_yield_is_reasonable(self, wired, underway_log):
        """
        Most frames should produce something. A sudden drop means a decoder
        started silently returning None -- e.g. a gate condition inverted.
        """
        wired.feed(underway_log)
        ratio = wired.decoded / wired.frames
        assert ratio > 0.10, f"only {ratio:.1%} of frames decoded"

    def test_all_expected_sources_seen_underway(self, wired, underway_log):
        wired.feed(underway_log)
        seen = {s.name for s in wired.state.sources.values() if s.ever_seen}
        expected = {
            "steering",
            "rudder",
            "gps_cog_sog",
            "gps_position",
            "gps_vehicle_dir",
            "engine",
            "compass_F8",
            "autopilot_status",
        }
        assert expected <= seen, f"missing: {expected - seen}"

    def test_autopilot_status_absent_in_trolling_capture(self, wired, trolling_log):
        """
        This capture genuinely has no 0x18FF50E0 frames. It is the real-world
        version of 'a node stopped talking', and the HMI must handle it
        without blanking everything else.
        """
        wired.feed(trolling_log)
        src = [s for s in wired.state.sources.values() if s.name == "autopilot_status"][0]
        assert not src.ever_seen
        assert wired.state.signal_value("compass_heading") is not None


# ---------------------------------------------------------------------------
# Physical plausibility -- the assertions that catch scale and offset bugs
# ---------------------------------------------------------------------------


class TestPhysicalPlausibility:
    def test_pitch_near_level_on_flat_water(self, wired, underway_log):
        """
        Regression for the J1939 offset bug: pitch was decoded without its
        -200 degree offset and reported ~198.7 for a level boat. The scatter
        was right, so sea-state attribution still worked, which is precisely
        why it went unnoticed.
        """
        wired.feed(underway_log)
        pitch = wired.state.signal_value("pitch")
        assert pitch is not None
        assert -15.0 < pitch < 15.0, f"pitch {pitch:.1f} implausible for flat water"

    def test_headings_are_bearings(self, wired, underway_log):
        wired.feed(underway_log, tick_every=500)
        for name in ("compass_heading", "cog", "fused_heading"):
            v = wired.state.signal_value(name)
            if v is not None:
                assert -360.0 < v < 720.0, f"{name}={v} is not a bearing"

    def test_speed_plausible(self, wired, underway_log):
        wired.feed(underway_log)
        sog = wired.state.signal_value("sog")
        assert sog is not None
        assert 0.0 <= sog < 100.0

    def test_position_near_horsetooth(self, wired, underway_log):
        wired.feed(underway_log)
        lat = wired.state.signal_value("lat")
        lon = wired.state.signal_value("lon")
        assert lat is not None and lon is not None
        assert 40.0 < lat < 41.0
        assert -106.0 < lon < -104.0

    def test_rpm_plausible(self, wired, underway_log):
        wired.feed(underway_log)
        rpm = wired.state.signal_value("rpm")
        assert rpm is not None
        assert 0 <= rpm < 8000

    def test_rpm_throttle_suppresses_under_instant_replay(self, wired, underway_log):
        """
        The engine PGN arrives at 100 Hz and the decoder throttles it to
        GUI_TIMEOUT (0.35 s) so the websocket is not flooded. Replaying
        instantly means every frame after the first falls inside that window,
        so RPM never updates.

        This is worth pinning for two reasons: it documents that a blank RPM
        field during fast offline replay is expected rather than broken, and
        it fails if someone removes the throttle -- which would flood every
        connected phone at 100 Hz.
        """
        wired.feed(underway_log, defeat_rate_limits=False)
        assert wired.state.signal_value("rpm") is None

    def test_steering_within_shaft_range(self, wired, underway_log):
        wired.feed(underway_log)
        angle = wired.state.signal_value("steering_angle")
        assert angle is not None
        assert -500 < angle < 4000, f"shaft {angle} outside physical travel"


# ---------------------------------------------------------------------------
# Speed-dependent COG behaviour
# ---------------------------------------------------------------------------


class TestCogGating:
    def test_fast_capture_accepts_cog(self, wired, underway_log):
        wired.feed(underway_log)
        st = wired.state
        assert st.cog_accepted > 0
        assert st.cog_accepted > st.cog_rejected, (
            f"accepted={st.cog_accepted} rejected={st.cog_rejected} "
            "-- fast capture should mostly accept COG"
        )

    def test_slow_capture_rejects_cog(self, wired, trolling_log):
        """
        At trolling speed COG is dominated by GPS noise rather than actual
        course. The filter work found 503 of 1640 samples rejected on the
        full log; this slice is deliberately from the slowest section.
        """
        wired.feed(trolling_log)
        st = wired.state
        assert st.cog_rejected > 0, "trolling capture should reject some COG"
        assert st.cog_rejected > st.cog_accepted, (
            f"accepted={st.cog_accepted} rejected={st.cog_rejected} "
            "-- slow capture should mostly reject COG"
        )

    def test_rejection_ratio_differs_between_regimes(self, state, monitor, underway_log, trolling_log):
        """
        The discriminating test: same code, two speed regimes, opposite
        outcomes. If someone removes the speed gate, both ratios go to 1.0
        and this fails even though every other test still passes.
        """
        from app_tui import HMIApp
        from can_interface import CANinterface
        from hmi_bridge import Bridge
        from hmi_heading import HeadingMonitor
        from hmi_state import SystemState
        from can.io.canutils import CanutilsLogReader

        def ratio(path):
            st = SystemState()
            mon = HeadingMonitor(st)
            br = Bridge(st, mon)
            ci = CANinterface.__new__(CANinterface)
            HMIApp._init_can_fields(ci)
            ci.bus = None
            ci.add_listener(br.on_decoded)
            for msg in CanutilsLogReader(str(path)):
                d = ci.process_message(msg)
                if d:
                    for cb in ci.listeners:
                        cb(d)
            total = st.cog_accepted + st.cog_rejected
            return (st.cog_rejected / total) if total else 0.0

        fast = ratio(underway_log)
        slow = ratio(trolling_log)
        assert slow > fast + 0.5, (
            f"rejection ratio fast={fast:.2f} slow={slow:.2f} -- "
            "speed gate appears to be missing or ineffective"
        )


# ---------------------------------------------------------------------------
# Fusion and filter surface
# ---------------------------------------------------------------------------


class TestFusion:
    def test_fused_populates_on_every_tick(self, wired, underway_log):
        """
        Regression: freshness was stamped with the log's own timestamps, which
        are months old, so every signal read as instantly stale and the FUSED
        row stayed blank in offline replay -- the exact mode used for demos.
        """
        wired.feed(underway_log, tick_every=250)
        assert wired.state.signal_value("fused_heading") is not None

    def test_sigma_published(self, wired, underway_log):
        wired.feed(underway_log, tick_every=250)
        sigma = wired.state.signal_value("heading_sigma")
        assert sigma is not None
        assert 0.0 <= sigma < 180.0

    def test_snapshot_serializable_after_replay(self, wired, underway_log):
        import json

        wired.feed(underway_log, tick_every=500)
        json.dumps(wired.state.snapshot())

    def test_filter_surface_accepts_injected_values(self, wired, underway_log):
        """
        The bridge already routes fused_heading / heading_sigma / yaw_rate,
        so dropping the Kalman filter in should need no HMI changes. Prove
        the seam works before the filter exists.
        """
        wired.feed(underway_log, limit=500)
        wired.bridge.on_decoded(
            {"fused_heading": 123.4, "heading_sigma": 1.7, "yaw_rate": -2.5}
        )
        assert wired.state.signal_value("fused_heading") == pytest.approx(123.4)
        assert wired.state.signal_value("yaw_rate") == pytest.approx(-2.5)
        assert wired.monitor.effective_sigma() is not None


# ---------------------------------------------------------------------------
# Heading quality on real data
# ---------------------------------------------------------------------------


class TestHeadingQualityOnRealData:
    def test_sigma_reflects_real_compass_noise(self, wired, underway_log):
        """
        Observed sigma on this capture runs high with near-flat attitude,
        consistent with the hard-iron signature found during the filter work.
        Bounded loosely: this documents reality rather than asserting the
        boat behaves well.
        """
        wired.feed(underway_log, tick_every=250)
        sigma = wired.monitor.effective_sigma()
        assert sigma is not None
        assert 0.0 < sigma < 90.0

    def test_diagnosis_is_actionable(self, wired, underway_log):
        wired.feed(underway_log, tick_every=250)
        quality, why = wired.monitor.diagnose()
        assert quality in ("GOOD", "NOISY", "BAD", "UNKNOWN")
        assert len(why) > 5

    def test_engage_decision_has_a_reason(self, wired, underway_log):
        wired.feed(underway_log, tick_every=250)
        ok, why = wired.monitor.ok_to_engage()
        assert isinstance(ok, bool)
        assert why and why != "no valid heading", (
            "with a live compass the reason should be the diagnosis, "
            "not the no-heading fallback"
        )


# ---------------------------------------------------------------------------
# Performance guard
# ---------------------------------------------------------------------------


class TestThroughput:
    @pytest.mark.slow
    def test_decode_keeps_up_with_bus(self, wired, underway_log):
        """
        The bus carries a few thousand frames per second. If decode plus
        bridge cannot sustain well above that on a dev machine, the Pi will
        fall behind and the display will lag reality.
        """
        t0 = time.perf_counter()
        wired.feed(underway_log)
        elapsed = time.perf_counter() - t0
        rate = wired.frames / elapsed
        assert rate > 5000, f"only {rate:.0f} frames/s"

    @pytest.mark.slow
    def test_snapshot_is_cheap(self, wired, underway_log):
        """Snapshot runs at 5 Hz per client; it must not be expensive."""
        wired.feed(underway_log, limit=2000, tick_every=500)
        t0 = time.perf_counter()
        for _ in range(200):
            wired.state.snapshot()
        per = (time.perf_counter() - t0) / 200
        assert per < 0.01, f"snapshot takes {per*1000:.1f} ms"


# ---------------------------------------------------------------------------
# Golden summary -- one broad check that catches wholesale changes
# ---------------------------------------------------------------------------


@pytest.mark.regression
def test_golden_summary_underway(wired, underway_log):
    """
    A single consolidated assertion over the whole replay. If a refactor
    changes decoding in any material way, this fails and names the field.
    Bounds are intentionally wide; they encode physics, not current output.
    """
    wired.feed(underway_log, tick_every=250)
    st = wired.state

    checks = {
        "frames": (wired.frames, 3000, 5000),
        "decode_errors": (len(wired.errors), 0, 0),
        "sources_seen": (sum(1 for s in st.sources.values() if s.ever_seen), 8, 8),
        "compass_heading": (st.signal_value("compass_heading"), -360, 720),
        "sog_mph": (st.signal_value("sog"), 0, 100),
        "rpm": (st.signal_value("rpm"), 0, 8000),
        "lat": (st.signal_value("lat"), 40.0, 41.0),
        "lon": (st.signal_value("lon"), -106.0, -104.0),
        "pitch_deg": (st.signal_value("pitch"), -15, 15),
        "steering_angle": (st.signal_value("steering_angle"), -500, 4000),
        "heading_sigma": (st.signal_value("heading_sigma"), 0, 180),
    }

    failures = []
    for name, (value, lo, hi) in checks.items():
        if value is None:
            failures.append(f"{name} is None")
        elif not (lo <= value <= hi):
            failures.append(f"{name}={value} outside [{lo}, {hi}]")

    assert not failures, "golden summary drift: " + "; ".join(failures)
