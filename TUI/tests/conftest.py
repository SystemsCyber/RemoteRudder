"""
Shared pytest fixtures for the RemoteRudder HMI test suite.

Design notes
------------
* Fixtures are function-scoped by default so tests cannot leak state into each
  other. SystemState carries latched faults and a deque of events; a shared
  instance would make test order significant, which is the fastest way to get
  a suite nobody trusts.

* Log fixtures are real captures, carved down to a few thousand frames. They
  are checked in because regression tests against synthetic data only prove
  the code is self-consistent, not that it decodes what the boat actually
  sends. See tests/fixtures/README.md for provenance.

* Time is injectable. Anything that ages out (staleness, rolling windows,
  backoff) takes an explicit `when` or reads a clock we can control, because
  a test that calls time.sleep(3) to check a 2-second timeout is both slow
  and flaky.
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import pytest

# The HMI modules are flat in the parent directory, not a package.
HMI_DIR = Path(__file__).resolve().parent.parent
if str(HMI_DIR) not in sys.path:
    sys.path.insert(0, str(HMI_DIR))

FIXTURE_DIR = Path(__file__).resolve().parent / "fixtures"


# ---------------------------------------------------------------------------
# Log fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope="session")
def fixture_dir() -> Path:
    return FIXTURE_DIR


@pytest.fixture(scope="session")
def underway_log(fixture_dir) -> Path:
    """
    Real capture, boat underway at 3-4 mph. Contains all eight decoded PGNs
    including autopilot status frames. This is the "everything works" case.
    """
    p = fixture_dir / "underway_fast.log"
    if not p.exists():
        pytest.skip(f"missing fixture {p}")
    return p


@pytest.fixture(scope="session")
def trolling_log(fixture_dir) -> Path:
    """
    Real capture at trolling speed: 98% of SOG samples below the 1.6 mph
    threshold where COG stops being a usable heading source. Also contains
    no autopilot status frames at all, so it doubles as a real-world
    missing-message case rather than a synthetic one.
    """
    p = fixture_dir / "trolling_slow.log"
    if not p.exists():
        pytest.skip(f"missing fixture {p}")
    return p


@pytest.fixture(scope="session")
def smoke_log(fixture_dir) -> Path:
    """400 frames. For tests that just need the decoder to turn over."""
    p = fixture_dir / "smoke.log"
    if not p.exists():
        pytest.skip(f"missing fixture {p}")
    return p


@pytest.fixture(scope="session")
def planing_log(fixture_dir) -> Path:
    """
    Real 2026 capture at planing speed (13.6-25.1 mph, none below the COG
    threshold). The COG-primary "everything working at speed" case.
    """
    p = fixture_dir / "planing_2026.log"
    if not p.exists():
        pytest.skip(f"missing fixture {p}")
    return p


@pytest.fixture(scope="session")
def docking_log(fixture_dir) -> Path:
    """
    Real 2026 capture at docking/idle speed: 86% of SOG samples below the COG
    threshold (0.2-2.5 mph). Exercises the no-lock / wait-for-motion path.
    """
    p = fixture_dir / "docking_2026.log"
    if not p.exists():
        pytest.skip(f"missing fixture {p}")
    return p


# ---------------------------------------------------------------------------
# Core objects
# ---------------------------------------------------------------------------


@pytest.fixture
def state():
    """A fresh SystemState. Function-scoped: faults and events must not leak."""
    from hmi_state import SystemState

    return SystemState()


@pytest.fixture
def monitor(state):
    from hmi_heading import HeadingMonitor

    return HeadingMonitor(state)


@pytest.fixture
def bridge(state, monitor):
    from hmi_bridge import Bridge

    return Bridge(state, monitor)


@pytest.fixture
def can_iface():
    """
    A CANinterface with all its decode state but no bus.

    CANinterface.__init__ opens a real bus and calls sys.exit() on failure,
    which is exactly the behaviour app_tui works around. We construct via
    __new__ and populate fields the same way, so process_message() is under
    test without any hardware.
    """
    from can_interface import CANinterface
    from app_tui import HMIApp

    ci = CANinterface.__new__(CANinterface)
    HMIApp._init_can_fields(ci)
    ci.bus = None
    return ci


@pytest.fixture
def wired(state, monitor, bridge, can_iface):
    """
    Decoder + bridge + state wired together, as the app runs them.

    Returns a small namespace so tests can say `wired.feed(path)` and then
    assert on `wired.state`.
    """

    class Wired:
        def __init__(self):
            self.state = state
            self.monitor = monitor
            self.bridge = bridge
            self.can = can_iface
            self.can.add_listener(bridge.on_decoded)
            self.frames = 0
            self.decoded = 0
            self.errors = []

        def feed(self, path, limit=None, tick_every=None, base_time=None,
                 defeat_rate_limits=True):
            """
            Replay a log through the real decode path.

            base_time: if given, signals are stamped on a synthetic clock
            starting there, so freshness is deterministic instead of
            depending on how fast the test machine runs.

            defeat_rate_limits: some decoders throttle on wall clock so the
            websocket is not flooded (engine RPM uses GUI_TIMEOUT = 0.35 s).
            Replay feeds thousands of frames instantly, so that gate
            suppresses every frame after the first and RPM never appears.
            That is an artifact of replay speed, not a decode bug -- at
            realistic pacing all 2150 engine frames decode. Rewinding the
            throttle timestamp before each frame reproduces live behaviour.
            Set False to test the throttle itself.
            """
            from can.io.canutils import CanutilsLogReader

            t0 = base_time
            for msg in CanutilsLogReader(str(path)):
                when = time.time() if t0 is None else t0
                self.state.mark_source(msg.arbitration_id, when)
                if defeat_rate_limits:
                    self.can.rpm_start_time = time.time() - 10.0
                try:
                    data = self.can.process_message(msg)
                except Exception as e:  # noqa: BLE001 - recorded, asserted on
                    self.errors.append((msg.arbitration_id, repr(e)))
                    data = None
                if data:
                    self.decoded += 1
                    for cb in self.can.listeners:
                        cb(data)
                self.frames += 1
                if tick_every and self.frames % tick_every == 0:
                    self.tick()
                if limit and self.frames >= limit:
                    break
            return self

        def tick(self):
            """One health-tick equivalent."""
            self.bridge.derive_fused()
            self.monitor.update_state()

        def feed_sampling(self, path, signals, tick_every=100,
                          bind_claims=True, limit=None):
            """
            Replay a log and, at each tick, record the current value of each
            named signal. Returns {signal_name: [values...]} so a test can
            assert on the stability (standard deviation) of the numbers that
            actually feed the control loop and the display.

            bind_claims: also decode J1939 address claims into the heading
            registry, so registry-bound sources (the wall compass, 24xd)
            resolve exactly as they do live.
            """
            from can.io.canutils import CanutilsLogReader
            from j1939_name import is_address_claim, decode_name, source_address

            samples = {name: [] for name in signals}
            extra = {"heading_source": []}  # always track which source is chosen
            for msg in CanutilsLogReader(str(path)):
                self.state.mark_source(msg.arbitration_id, time.time())
                if bind_claims and is_address_claim(msg.arbitration_id):
                    nm = decode_name(msg.data)
                    if nm:
                        self.bridge.heading_registry.note_claim(
                            source_address(msg.arbitration_id),
                            nm.mfg_code, nm.function, nm.identity)
                self.can.rpm_start_time = time.time() - 10.0
                try:
                    data = self.can.process_message(msg)
                except Exception as e:  # noqa: BLE001
                    self.errors.append((msg.arbitration_id, repr(e)))
                    data = None
                if data:
                    for cb in self.can.listeners:
                        cb(data)
                self.frames += 1
                if self.frames % tick_every == 0:
                    self.bridge.derive_fused()
                    for name in signals:
                        v = self.state.signal_value(name)
                        if v is not None:
                            samples[name].append(v)
                    extra["heading_source"].append(self.state.heading_source)
                if limit and self.frames >= limit:
                    break
            samples["_heading_source"] = extra["heading_source"]
            return samples

    return Wired()


def circular_stdev(values):
    """
    Standard deviation of angle values, computed on the circle so that a
    steady heading near the 0/360 wrap does not look noisy. Returns degrees.
    """
    import statistics
    if len(values) < 2:
        return 0.0
    mean = statistics.mean(values)
    dev = [((v - mean + 180.0) % 360.0) - 180.0 for v in values]
    return statistics.pstdev(dev)


# ---------------------------------------------------------------------------
# Time control
# ---------------------------------------------------------------------------


@pytest.fixture
def fake_clock(monkeypatch):
    """
    A controllable clock.

    Usage:
        def test_x(fake_clock):
            fake_clock.advance(5.0)

    Patches time.time in the modules that age things out. Signals, sources,
    and the heading window all read time.time() directly, so patching it at
    the module level is enough and avoids threading a clock parameter through
    every call site.
    """

    class Clock:
        def __init__(self):
            self.now = 1_700_000_000.0

        def __call__(self):
            return self.now

        def advance(self, secs):
            self.now += secs
            return self.now

    clock = Clock()
    import hmi_state
    import hmi_heading

    monkeypatch.setattr(hmi_state.time, "time", clock)
    monkeypatch.setattr(hmi_heading.time, "time", clock)
    return clock


# ---------------------------------------------------------------------------
# Bus fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def virtual_link(state):
    """An open CANLink on python-can's virtual bus. Closed on teardown."""
    from hmi_canlink import CANLink

    link = CANLink(state, channel="vcan_test", bitrate=250000, backend="virtual")
    assert link.open() is True
    yield link
    link.close()


@pytest.fixture
def free_port():
    """An OS-assigned free TCP port, to avoid collisions in parallel runs."""
    import socket

    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


# ---------------------------------------------------------------------------
# Markers
# ---------------------------------------------------------------------------


def pytest_configure(config):
    config.addinivalue_line("markers", "slow: replays a full log, takes seconds")
    config.addinivalue_line("markers", "integration: spawns a server or bus")
    config.addinivalue_line("markers", "regression: golden-value checks on real logs")
    config.addinivalue_line("markers", "hardware: needs a real CAN adapter (skipped by default)")


def pytest_collection_modifyitems(config, items):
    """Skip hardware tests unless --hardware is passed."""
    if config.getoption("--hardware"):
        return
    skip = pytest.mark.skip(reason="needs real CAN hardware; pass --hardware to run")
    for item in items:
        if "hardware" in item.keywords:
            item.add_marker(skip)


def pytest_addoption(parser):
    parser.addoption(
        "--hardware",
        action="store_true",
        default=False,
        help="run tests that require a real CAN adapter",
    )


@pytest.fixture
def stationary_south_log():
    """Real capture: boat parked facing south, wall compass steady ~180."""
    import pathlib
    p = pathlib.Path(__file__).parent / "fixtures" / "stationary_south.log"
    return str(p)


@pytest.fixture
def parked_south_log():
    """
    Real capture (2026-07-25 11:23), compact carve: boat parked facing south,
    all devices claiming (incl. the wall compass on 0xF8 and the 24xd on 0x19),
    so the registry binds and fused heading resolves. The true heading is
    steady, so this is the fixture for stability tests.
    """
    import pathlib
    p = pathlib.Path(__file__).parent / "fixtures" / "parked_south.log"
    return str(p)
