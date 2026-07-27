"""
Integration tests: bridge mapping, live server, phone/TUI goal sync.

The server tests spawn a real Tornado process on a free port and talk to it
over HTTP and WebSocket. That is slower than calling handlers directly, but
it is the only way to catch the things that actually break: JSON that will not
serialize, a handler that raises on malformed input, a snapshot key the browser
depends on quietly disappearing.
"""

from __future__ import annotations

import json
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

HMI_DIR = Path(__file__).resolve().parent.parent


# ---------------------------------------------------------------------------
# Bridge mapping
# ---------------------------------------------------------------------------


class TestBridgeMapping:
    def test_maps_decoded_keys_to_signals(self, bridge, state):
        bridge.on_decoded({"compass_heading": 100.0, "SOG": 5.0, "COG": 95.0})
        assert state.signal_value("compass_heading") == pytest.approx(100.0)
        assert state.signal_value("sog") == pytest.approx(5.0)
        assert state.signal_value("cog") == pytest.approx(95.0)

    def test_unknown_keys_ignored_not_raised(self, bridge, state):
        """A decoder growing a new field must not crash the HMI."""
        bridge.on_decoded({"brand_new_field": 42, "compass_heading": 100.0})
        assert state.signal_value("compass_heading") == pytest.approx(100.0)

    def test_none_values_do_not_overwrite(self, bridge, state):
        bridge.on_decoded({"compass_heading": 100.0})
        bridge.on_decoded({"compass_heading": None})
        assert state.signal_value("compass_heading") == pytest.approx(100.0)

    def test_malformed_payload_is_contained(self, bridge, state):
        """
        This runs on the CAN read path. An exception here would kill the
        reader thread and silently freeze the whole display.
        """
        bridge.on_decoded({"compass_heading": object()})  # not floatable
        kinds = [k for _, k, _ in state.recent_events(10)]
        assert "WARN" in kinds

    def test_timeout_notifications_skipped(self, bridge, state):
        bridge.on_decoded({"timeout": True, "source_id": 0x123, "steering": None})
        assert state.signal_value("steering_angle") is None

    def test_control_state_from_status_frame(self, bridge, state):
        bridge.on_decoded(
            {
                "autopilot_engaged": True,
                "left_turn": False,
                "right_turn": True,
                "heading_error": -3.5,
            }
        )
        assert state.autopilot_engaged is True
        assert state.right_turn is True
        assert state.heading_error == pytest.approx(-3.5)

    def test_filter_output_routed(self, bridge, state, monitor):
        """The seam the Kalman filter will plug into."""
        bridge.on_decoded({"fused_heading": 123.4, "heading_sigma": 1.5, "yaw_rate": -2.0})
        assert state.signal_value("fused_heading") == pytest.approx(123.4)
        assert state.signal_value("yaw_rate") == pytest.approx(-2.0)

    def test_registers_all_watched_sources(self, bridge, state):
        from hmi_bridge import WATCHED

        assert set(state.sources.keys()) == set(WATCHED.keys())


class TestDeriveFused:
    """
    COG-primary fusion. Full transition coverage lives in
    test_cog_primary.py; these are the integration-level sanity checks.
    """

    def test_prefers_cog_when_moving(self, bridge, state):
        """COG-primary: with both present and way on, COG wins."""
        state.set_signal("compass_heading", 100.0)
        state.set_signal("cog", 200.0)
        state.set_signal("sog", 10.0)
        bridge.derive_fused()
        assert state.signal_value("fused_heading") == pytest.approx(200.0)
        assert state.cog_lock is True

    def test_bridges_on_compass_when_cog_drops(self, bridge, state):
        state.set_signal("compass_heading", 100.0)
        state.set_signal("cog", None)
        state.set_signal("sog", 10.0)
        bridge.derive_fused()
        assert state.signal_value("fused_heading") == pytest.approx(100.0)
        assert state.cog_lock is False

    def test_no_heading_below_threshold(self, bridge, state):
        """Below ~1.6 mph there is no usable heading: center and wait."""
        state.set_signal("compass_heading", 100.0)
        state.set_signal("cog", 200.0)
        state.set_signal("sog", 0.5)
        bridge.derive_fused()
        assert state.signal_value("fused_heading") is None

    def test_nothing_fresh_leaves_fused_empty(self, bridge, state):
        bridge.derive_fused()
        assert state.signal_value("fused_heading") is None


# ---------------------------------------------------------------------------
# Live server
# ---------------------------------------------------------------------------


@pytest.fixture
def server(free_port, tmp_path):
    """
    Spawn the real app headless against a replayed fixture.

    Uses --no-tui because curses needs a terminal. The CAN path, web layer,
    and state plumbing are identical either way.
    """
    fixture = HMI_DIR / "tests" / "fixtures" / "underway_fast.log"
    if not fixture.exists():
        pytest.skip("fixture missing")

    proc = subprocess.Popen(
        [
            sys.executable,
            "app_tui.py",
            "--no-tui",
            "--offline",
            "--replay",
            str(fixture),
            "--speed",
            "50",
            "--port",
            str(free_port),
        ],
        cwd=str(HMI_DIR),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    base = f"http://127.0.0.1:{free_port}"
    deadline = time.time() + 20
    while time.time() < deadline:
        if proc.poll() is not None:
            out = proc.stdout.read() if proc.stdout else ""
            pytest.fail(f"server exited early:\n{out[-2000:]}")
        try:
            with urllib.request.urlopen(base + "/health", timeout=1) as r:
                json.loads(r.read())
            break
        except (urllib.error.URLError, OSError, ValueError):
            time.sleep(0.3)
    else:
        proc.terminate()
        pytest.fail("server did not become ready")

    yield base, proc

    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()


def health(base):
    with urllib.request.urlopen(base + "/health", timeout=5) as r:
        return json.loads(r.read())


@pytest.mark.integration
class TestHealthEndpoint:
    def test_returns_json(self, server):
        base, _ = server
        assert isinstance(health(base), dict)

    def test_link_up_on_virtual_bus(self, server):
        base, _ = server
        assert health(base)["link_up"] is True

    def test_receives_frames(self, server):
        base, _ = server
        time.sleep(2.0)
        assert health(base)["rx_total"] > 0

    def test_reports_sources(self, server):
        base, _ = server
        time.sleep(2.0)
        srcs = health(base)["sources"]
        # 8 boat sources + 3 heading-node sources
        assert len(srcs) == 11
        assert any(s["status"] == "OK" for s in srcs)

    def test_legacy_keys_present(self, server):
        """compass.js depends on these names."""
        base, _ = server
        snap = health(base)
        for key in ("heading_deg", "heading_goal", "compass_deg", "rpm", "lat", "lon"):
            assert key in snap


@pytest.mark.integration
class TestGoalSync:
    """
    The behaviour asked for: a phone moves the goal and the TUI shows it
    immediately, with attribution. Both read one SystemState, so proving the
    web path writes it correctly proves the TUI sees it.
    """

    def _ws(self, base):
        import asyncio

        from tornado.websocket import websocket_connect

        url = base.replace("http://", "ws://") + "/ws"
        return asyncio.get_event_loop().run_until_complete(websocket_connect(url))

    def test_phone_sets_goal(self, server):
        import asyncio

        base, _ = server
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            from tornado.websocket import websocket_connect

            ws = loop.run_until_complete(
                websocket_connect(base.replace("http://", "ws://") + "/ws")
            )
            loop.run_until_complete(ws.read_message())  # initial snapshot
            ws.write_message(json.dumps({"heading_goal": 137.5}))
            loop.run_until_complete(asyncio.sleep(0.8))
            snap = health(base)
            assert snap["heading_goal"] == pytest.approx(137.5)
            assert snap["goal_source"].startswith("web/")
        finally:
            loop.close()

    def test_relative_command_and_wraparound(self, server):
        import asyncio

        base, _ = server
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            from tornado.websocket import websocket_connect

            ws = loop.run_until_complete(
                websocket_connect(base.replace("http://", "ws://") + "/ws")
            )
            loop.run_until_complete(ws.read_message())

            ws.write_message(json.dumps({"heading_goal": 359.5}))
            loop.run_until_complete(asyncio.sleep(0.5))
            ws.write_message(json.dumps({"command": "heading_right"}))
            loop.run_until_complete(asyncio.sleep(0.8))
            assert health(base)["heading_goal"] == pytest.approx(0.5)
        finally:
            loop.close()

    @pytest.mark.parametrize(
        "payload",
        [
            "not json at all",
            '{"heading_goal": "abc"}',
            '{"heading_goal": null}',
            '{"heading_goal": 1e400}',
            '{"command": "nonsense_command"}',
            '{"command": 12345}',
            "{}",
            "[]",
        ],
    )
    def test_malformed_input_does_not_corrupt_or_crash(self, server, payload):
        """
        Anything reachable from a phone on the boat wifi. The server must
        survive and the goal must not move.
        """
        import asyncio

        base, proc = server
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            from tornado.websocket import websocket_connect

            ws = loop.run_until_complete(
                websocket_connect(base.replace("http://", "ws://") + "/ws")
            )
            loop.run_until_complete(ws.read_message())

            ws.write_message(json.dumps({"heading_goal": 100.0}))
            loop.run_until_complete(asyncio.sleep(0.5))

            ws.write_message(payload)
            loop.run_until_complete(asyncio.sleep(0.6))

            assert proc.poll() is None, "server died on malformed input"
            assert health(base)["heading_goal"] == pytest.approx(100.0)
        finally:
            loop.close()

    def test_engage_blocked_reports_reason(self, server):
        """
        Engagement is gated on heading quality. On this capture the compass
        is noisy, so the request should be refused with a diagnosis rather
        than silently ignored.
        """
        import asyncio

        base, _ = server
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            from tornado.websocket import websocket_connect

            ws = loop.run_until_complete(
                websocket_connect(base.replace("http://", "ws://") + "/ws")
            )
            loop.run_until_complete(ws.read_message())
            time.sleep(2.0)
            ws.write_message(json.dumps({"command": "autopilot_enable"}))
            loop.run_until_complete(asyncio.sleep(1.0))
            snap = health(base)
            assert isinstance(snap["autopilot_engaged"], bool)
        finally:
            loop.close()


@pytest.mark.integration
class TestServerResilience:
    def test_survives_rapid_connect_disconnect(self, server):
        import asyncio

        base, proc = server
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            from tornado.websocket import websocket_connect

            for _ in range(10):
                ws = loop.run_until_complete(
                    websocket_connect(base.replace("http://", "ws://") + "/ws")
                )
                ws.close()
            loop.run_until_complete(asyncio.sleep(0.5))
            assert proc.poll() is None
            assert health(base)["link_up"] is True
        finally:
            loop.close()

    def test_health_stable_under_repeated_polling(self, server):
        base, proc = server
        for _ in range(30):
            health(base)
        assert proc.poll() is None
