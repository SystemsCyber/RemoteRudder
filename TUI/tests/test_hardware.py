"""
Tests that need a real CAN adapter. Skipped unless --hardware is passed.

Run on the boat or the bench with the PCAN plugged in:

    python3 -m pytest --hardware -m hardware -v

These are the checks that no amount of virtual-bus testing can substitute for:
whether the adapter actually enumerates, whether the configured bitrate matches
what the other nodes use, and whether the kernel error counters behave the way
the fault model assumes.

Deliberately read-only. Nothing here sends a steering command, because a test
suite that can move the rudder is a test suite that will move the rudder at the
wrong moment.
"""

from __future__ import annotations

import subprocess
import time

import pytest

from hmi_bridge import Bridge
from hmi_canlink import CANLink, read_socketcan_health, socketcan_iface_exists
from hmi_heading import HeadingMonitor
from hmi_state import F_NO_TRAFFIC, SystemState

pytestmark = pytest.mark.hardware

CHANNEL = "can0"
EXPECTED_BITRATE = 250000


@pytest.fixture(scope="module")
def live_link():
    """
    An open link on the real adapter.

    Fails loudly with setup instructions rather than skipping, because if you
    passed --hardware you meant it and a silent skip would be misleading.
    """
    if not socketcan_iface_exists(CHANNEL):
        pytest.fail(
            f"{CHANNEL} not present. Plug in the PCAN and run:\n"
            f"  sudo ip link set {CHANNEL} up type can bitrate {EXPECTED_BITRATE}"
        )

    state = SystemState()
    link = CANLink(state, channel=CHANNEL, bitrate=EXPECTED_BITRATE, backend="socketcan")
    if not link.open():
        pytest.fail(f"could not open {CHANNEL}: {state.link_detail}")
    yield link, state
    link.close()


class TestAdapterPresent:
    def test_interface_exists(self):
        assert socketcan_iface_exists(CHANNEL), (
            f"{CHANNEL} missing -- adapter unplugged or link not brought up"
        )

    def test_interface_is_up(self):
        out = subprocess.run(
            ["ip", "link", "show", CHANNEL], capture_output=True, text=True
        )
        assert "state UP" in out.stdout or "UP" in out.stdout, (
            f"{CHANNEL} exists but is down; run: "
            f"sudo ip link set {CHANNEL} up type can bitrate {EXPECTED_BITRATE}"
        )

    def test_opens_cleanly(self, live_link):
        link, state = live_link
        assert state.link_up is True


class TestBitrateConfiguration:
    def test_kernel_reports_expected_bitrate(self, live_link):
        """
        The single most common misconfiguration. If the netdev came up at a
        different rate than the nodes use, nothing decodes and the symptom
        looks like a dead bus.
        """
        health = read_socketcan_health(CHANNEL)
        assert health, "could not read interface statistics"
        actual = health.get("bitrate")
        assert actual == EXPECTED_BITRATE, (
            f"{CHANNEL} is at {actual}, expected {EXPECTED_BITRATE}. "
            f"Re-run: sudo ip link set {CHANNEL} down && "
            f"sudo ip link set {CHANNEL} up type can bitrate {EXPECTED_BITRATE}"
        )

    def test_bus_state_healthy(self, live_link):
        health = read_socketcan_health(CHANNEL)
        state = health.get("bus_state")
        assert state != "BUS-OFF", (
            "controller is bus-off -- check termination (120 ohm at both ends) "
            "and that the bitrate matches the other nodes"
        )

    def test_error_counters_not_climbing(self, live_link):
        """
        Rising counters with the bus otherwise quiet is the bitrate-mismatch
        signature. Sample twice a second apart and compare.
        """
        first = read_socketcan_health(CHANNEL)
        time.sleep(2.0)
        second = read_socketcan_health(CHANNEL)

        d_rx = second.get("rx_error_count", 0) - first.get("rx_error_count", 0)
        d_tx = second.get("tx_error_count", 0) - first.get("tx_error_count", 0)
        assert d_rx + d_tx < 10, (
            f"error counters climbing (rx +{d_rx}, tx +{d_tx}) -- "
            "likely a bitrate mismatch or a termination problem"
        )


class TestLiveTraffic:
    def test_frames_arrive(self, live_link):
        """
        With the boat powered up, something should be transmitting within a
        couple of seconds. Silence here means either nothing is powered or
        the bitrate is wrong.
        """
        link, state = live_link
        deadline = time.time() + 5.0
        got = None
        while time.time() < deadline:
            msg = link.bus.recv(timeout=0.5)
            if msg is not None:
                got = msg
                break
        assert got is not None, (
            "no frames in 5s. Check that the nodes are powered, the bitrate "
            f"is {EXPECTED_BITRATE}, and the bus is terminated."
        )

    def test_expected_pgns_present(self, live_link):
        """
        Collect for a few seconds and report which of the watched PGNs are
        actually on the wire. Missing ones are usually an unpowered node.
        """
        from hmi_bridge import WATCHED

        link, state = live_link
        seen = set()
        deadline = time.time() + 6.0
        while time.time() < deadline:
            msg = link.bus.recv(timeout=0.5)
            if msg is not None and msg.arbitration_id in WATCHED:
                seen.add(msg.arbitration_id)

        missing = {WATCHED[i][0] for i in set(WATCHED) - seen}
        assert seen, f"none of the watched PGNs seen; got nothing in 6s"
        if missing:
            pytest.skip(
                f"saw {len(seen)}/{len(WATCHED)} PGNs. "
                f"Not seen: {', '.join(sorted(missing))} "
                "(usually an unpowered node rather than a decode problem)"
            )

    def test_decode_produces_plausible_values(self, live_link):
        """End-to-end on live data: decode, bridge, and sanity-check."""
        from app_tui import HMIApp
        from can_interface import CANinterface

        link, state = live_link
        st = SystemState()
        mon = HeadingMonitor(st)
        br = Bridge(st, mon)
        ci = CANinterface.__new__(CANinterface)
        HMIApp._init_can_fields(ci)
        ci.bus = link.bus
        ci.add_listener(br.on_decoded)

        deadline = time.time() + 8.0
        while time.time() < deadline:
            msg = link.bus.recv(timeout=0.5)
            if msg is None:
                continue
            st.mark_source(msg.arbitration_id, time.time())
            data = ci.process_message(msg)
            if data:
                for cb in ci.listeners:
                    cb(data)

        br.derive_fused()
        mon.update_state()

        checked = 0
        for name, lo, hi in (
            ("compass_heading", -360, 720),
            ("sog", 0, 100),
            ("rpm", 0, 8000),
            ("pitch", -45, 45),
            ("lat", -90, 90),
            ("lon", -180, 180),
        ):
            v = st.signal_value(name)
            if v is not None:
                checked += 1
                assert lo <= v <= hi, f"{name}={v} outside [{lo}, {hi}]"
        assert checked > 0, "no signals decoded from live traffic"


class TestNoTrafficDetection:
    def test_fault_raised_when_bus_silent(self, live_link):
        """
        Only meaningful with the adapter connected but the boat powered down.
        Skips if traffic is present, since that is the normal case.
        """
        link, state = live_link
        msg = link.bus.recv(timeout=1.0)
        if msg is not None:
            pytest.skip("bus is active; power down the nodes to test this path")

        link._opened_at = time.time() - 20
        state.rx_total = 0
        state.last_rx_any = 0.0
        link.poll_health()
        assert F_NO_TRAFFIC in state.faults
