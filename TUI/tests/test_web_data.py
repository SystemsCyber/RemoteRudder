"""
Tests that the web interface's data entries are properly shown.

We can't drive a browser here, but the web display is only correct if two
things hold, and both are testable:

  1. The telemetry snapshot the server broadcasts always CONTAINS the fields the
     page needs, with sane values -- so the page has something to show. The
     parked-boat bug was that Heading had no field to read (the page only read
     COG, which is absent when parked), so it showed '----'. A test that asserts
     heading is present whenever the compass is present would have caught it.

  2. compass.js READS the right fields. The goal-bounce/blank-heading bug was
     the page conflating heading with COG: it set `heading = data.COG` and then
     computed the goal as `ang + heading`, so with no COG both broke. We parse
     compass.js and assert it reads a real heading field (heading_deg /
     compass_deg), not only COG.

These are the "make sure the web data entries are properly shown" tests.
"""

from __future__ import annotations

import pathlib
import re

import pytest


# ---------------------------------------------------------------------------
# 1. Telemetry snapshot always carries the fields the page renders
# ---------------------------------------------------------------------------


@pytest.fixture
def telemetry():
    """
    Build the web telemetry the way the UNIFIED app (app_tui) does: feed the
    bridge/state and read STATE.snapshot(). This is the single-source-of-truth
    path -- one goal, one heading, shared by the web and the TUI. Returns a
    small namespace exposing snapshot() plus the pieces a test needs to poke.
    """
    from hmi_state import SystemState
    from hmi_heading import HeadingMonitor
    from hmi_bridge import Bridge

    class Telemetry:
        def __init__(self):
            self.state = SystemState()
            self.monitor = HeadingMonitor(self.state)
            self.bridge = Bridge(self.state, self.monitor)
            self.bridge.heading_registry.note_claim(0xF8, 229, 140, 1039212)

        def feed_compass(self, deg):
            self.bridge.on_decoded({"compass_heading": deg, "heading_sa": 0xF8,
                                    "heading_value": deg})
            self.state.set_signal("compass_heading", deg)
            self.bridge.derive_fused()

        def set_goal(self, g, source="web"):
            return self.state.set_goal(g, source)

        def make_telemetry_snapshot(self):
            return self.state.snapshot()

    return Telemetry()


# The keys the page reads from each telemetry message (unified STATE.snapshot).
REQUIRED_KEYS = [
    "heading_deg", "compass_deg", "cog_deg", "heading_goal",
    "shaft_goal", "ts",
]


class TestTelemetryFields:
    def test_snapshot_has_all_display_keys(self, telemetry):
        snap = telemetry.make_telemetry_snapshot()
        missing = [k for k in REQUIRED_KEYS if k not in snap]
        assert not missing, f"telemetry missing display keys: {missing}"

    def test_heading_present_when_compass_present(self, telemetry):
        """
        THE parked-boat regression: with a compass but no COG, the page must
        still have a heading to show. heading_deg or compass_deg must be
        non-null.
        """
        telemetry.feed_compass(179.4)
        snap = telemetry.make_telemetry_snapshot()
        heading = snap.get("heading_deg")
        compass = snap.get("compass_deg")
        assert (heading is not None) or (compass is not None), \
            "no heading field populated while the compass is valid -- the web " \
            "Heading readout would be blank on a parked boat"
        assert compass == pytest.approx(179.4)

    def test_heading_deg_is_real_not_stuck_zero_when_parked(self, telemetry):
        """
        The compass-wheel bug: heading_deg must reflect the REAL heading. In the
        unified app it is fused_heading, which on a parked boat is the compass
        (~179), never a stuck 0. If it were 0, the web wheel would stick at north
        (0 is present and non-null, so the page never falls back to compass_deg).
        """
        telemetry.feed_compass(179.4)
        snap = telemetry.make_telemetry_snapshot()
        h = snap.get("heading_deg")
        assert h is not None
        assert abs(((h - 179.4 + 180) % 360) - 180) < 5.0, (
            f"heading_deg={h} is not the compass heading -- the wheel would not turn"
        )

    def test_goal_is_a_single_normalized_value(self, telemetry):
        """The broadcast goal must be a clean [0,360) bearing, never 360."""
        telemetry.set_goal(360.0)
        snap = telemetry.make_telemetry_snapshot()
        g = snap["heading_goal"]
        assert g is None or (0.0 <= g < 360.0), f"goal {g} not normalized"

    def test_goal_and_heading_are_distinct_keys(self, telemetry):
        """
        Goal and heading must be separate fields -- conflating them was the bug.
        Set them to different values and confirm the snapshot keeps them apart.
        """
        telemetry.feed_compass(179.4)
        telemetry.set_goal(90.0)
        snap = telemetry.make_telemetry_snapshot()
        assert snap["heading_goal"] == pytest.approx(90.0)
        assert snap["compass_deg"] == pytest.approx(179.4)
        assert snap["heading_goal"] != snap["compass_deg"]

    def test_goal_is_single_source_of_truth(self, telemetry):
        """
        The whole point of the unified app: ONE goal. Setting it updates the
        single state value that both the web snapshot and the TUI read. There is
        no second copy to diverge.
        """
        telemetry.set_goal(123.0, source="web")
        snap = telemetry.make_telemetry_snapshot()
        assert snap["heading_goal"] == pytest.approx(123.0)
        assert snap.get("goal_source") == "web"
        # the state value the TUI reads is the same object
        assert telemetry.state.heading_goal == pytest.approx(123.0)

    def test_numeric_fields_are_numbers_or_none(self, telemetry):
        telemetry.feed_compass(179.4)
        snap = telemetry.make_telemetry_snapshot()
        for k in ("heading_deg", "compass_deg", "cog_deg", "heading_goal",
                  "shaft_goal"):
            v = snap.get(k)
            assert v is None or isinstance(v, (int, float)), \
                f"{k} is {type(v).__name__}, not a number"


# ---------------------------------------------------------------------------
# 2. compass.js reads the correct fields (static analysis)
# ---------------------------------------------------------------------------


@pytest.fixture
def compass_js():
    p = pathlib.Path(__file__).parent.parent / "static" / "js" / "compass.js"
    if not p.exists():
        pytest.skip("compass.js not present in this tree")
    return p.read_text(encoding="utf-8", errors="replace")


class TestCompassJsFields:
    def test_reads_a_real_heading_field(self, compass_js):
        """
        The heading display must read heading_deg or compass_deg, not only COG.
        This is the fix for the blank-heading-when-parked bug.
        """
        assert ("heading_deg" in compass_js or "compass_deg" in compass_js), \
            "compass.js does not read heading_deg/compass_deg -- Heading will " \
            "be blank whenever COG is absent (parked boat)"

    def test_heading_not_assigned_only_from_cog(self, compass_js):
        """
        Guard against the specific regression: `heading = data.COG` as the sole
        heading source. If COG is used, a real heading field must be used too.
        """
        uses_cog_for_heading = re.search(r"heading\s*=\s*data\.COG", compass_js)
        if uses_cog_for_heading:
            # then it must ALSO assign heading from a real heading field
            assert re.search(r"heading\s*=\s*data\.(heading_deg|compass_deg)",
                             compass_js), \
                "heading is assigned from COG but never from a real heading field"

    def test_goal_and_heading_are_separate_variables(self, compass_js):
        """
        The page must track goal and heading as distinct variables (it computes
        the dial goal relative to heading). Both must exist.
        """
        assert "heading_goal_string" in compass_js
        assert "heading_string" in compass_js

    def test_reads_heading_goal_from_data(self, compass_js):
        assert "data.heading_goal" in compass_js or "'heading_goal' in data" in compass_js


class TestCompassWheelRotation:
    """
    The compass wheel must rotate with heading (heading-up card): facing south,
    S sits at the bow. The bug was that cardinal letters were counter-rotated by
    +heading to stay upright, which cancelled the ring's -heading rotation and
    pinned N to the top forever, so the wheel looked static.
    """

    def _js(self):
        import pathlib
        p = pathlib.Path(__file__).parent.parent / "static" / "js" / "compass.js"
        if not p.exists():
            import pytest
            pytest.skip("compass.js not present")
        return p.read_text(encoding="utf-8", errors="replace")

    def test_letters_positioned_by_bearing_minus_heading(self, ):
        js = self._js()
        # the fix computes a screen angle from (bearing - heading)
        assert "bearing - heading" in js, \
            "cardinal letters are not positioned by (bearing - heading) -- the " \
            "wheel will not rotate"

    def test_no_per_letter_heading_counter_rotation(self):
        js = self._js()
        # the old bug: ctx.rotate(heading ...) applied per-letter to 'stay
        # upright', which cancelled the ring rotation. It must be gone.
        import re
        # find the cardinal-letter block and ensure it does not rotate by +heading
        assert "ctx.rotate(heading * Math.PI / 180)" not in js, \
            "per-letter +heading rotation is back -- it cancels the wheel rotation"


class TestCompassFaceLabels:
    """The value labels (Heading/Speed/Rudder/Goal) must be drawn as the top
    layer so the boat sprite cannot cover them."""

    def _js(self):
        import pathlib
        p = pathlib.Path(__file__).parent.parent / "static" / "js" / "compass.js"
        if not p.exists():
            import pytest
            pytest.skip("compass.js not present")
        return p.read_text(encoding="utf-8", errors="replace")

    def test_labels_drawn_after_boat_and_crosshair(self):
        js = self._js()
        boat = js.find("ctx.drawImage(boat")
        labels = js.find("VALUE LABELS")
        crosshair = js.find("ctx.lineTo(0, R);")  # last crosshair stroke
        assert boat != -1 and labels != -1
        # labels must come AFTER the boat draw and after the crosshair
        assert labels > boat, "value labels are drawn before the boat -- can be covered"
        assert labels > crosshair, "value labels are not the top layer"

    def test_labels_present(self):
        js = self._js()
        for s in ("heading_string", "speed_string", "rudder_string",
                  "heading_goal_string"):
            assert s in js, f"{s} label missing from the compass face"


class TestGraphsButtonAndVersion:
    def _js(self):
        import pathlib
        p = pathlib.Path(__file__).parent.parent / "static" / "js" / "compass.js"
        if not p.exists():
            import pytest
            pytest.skip("compass.js not present")
        return p.read_text(encoding="utf-8", errors="replace")

    def test_graphs_button_wired(self):
        js = self._js()
        assert "graphsBtn" in js, "Graphs button has no handler in compass.js"
        assert "/plot" in js, "Graphs button does not navigate to /plot"

    def test_version_stamp_present(self):
        js = self._js()
        # a console version stamp so the loaded compass.js version is verifiable
        assert "compass.js v" in js, "no version stamp to identify the loaded JS"

    def test_plot_route_exists(self):
        import pathlib
        app = (pathlib.Path(__file__).parent.parent / "app_tui.py").read_text()
        assert '"/plot"' in app or "r\"/plot\"" in app, "no /plot route for graphs"
