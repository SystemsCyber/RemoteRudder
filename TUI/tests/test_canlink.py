"""
Tests for hmi_canlink: bus lifecycle, platform resolution, fault detection.

These cover the four failure modes seen on the water:

  hardware not plugged in -> LINK_DOWN with a retry, never sys.exit()
  wrong baud rate         -> NO_TRAFFIC, plus BUS_ERRORS from kernel counters
  missing messages        -> STALE naming the specific PGN
  (noisy heading lives in test_heading.py)

The kernel-counter path is Linux-only and needs a real socketcan device, so
the parsing is tested against captured `ip -details -statistics link` output
rather than a live interface.
"""

from __future__ import annotations

import time

import can
import pytest

from hmi_canlink import (
    BACKOFF,
    NO_TRAFFIC_TIMEOUT,
    CANLink,
    backend_for_platform,
    read_socketcan_health,
    socketcan_iface_exists,
)
from hmi_state import (
    F_BUS_ERRORS,
    F_BUS_OFF,
    F_LINK_DOWN,
    F_NO_TRAFFIC,
    F_STALE,
    F_TX_FAIL,
    SystemState,
)


# ---------------------------------------------------------------------------
# Platform resolution -- the Windows port hinges on this
# ---------------------------------------------------------------------------


class TestBackendResolution:
    def test_virtual_forced_to_vcan0(self):
        assert backend_for_platform("virtual", "can0") == ("virtual", "vcan0")

    def test_explicit_socketcan(self):
        assert backend_for_platform("socketcan", "can1") == ("socketcan", "can1")

    def test_explicit_pcan_channel_preserved(self):
        assert backend_for_platform("pcan", "PCAN_USBBUS2") == ("pcan", "PCAN_USBBUS2")

    def test_linux_default(self, monkeypatch):
        monkeypatch.setattr("hmi_canlink.IS_WINDOWS", False)
        assert backend_for_platform(None, "can0") == ("socketcan", "can0")

    def test_windows_default(self, monkeypatch):
        monkeypatch.setattr("hmi_canlink.IS_WINDOWS", True)
        iface, chan = backend_for_platform(None, "can0")
        assert iface == "pcan"
        assert chan.startswith("PCAN_")

    def test_windows_preserves_explicit_pcan_channel(self, monkeypatch):
        monkeypatch.setattr("hmi_canlink.IS_WINDOWS", True)
        assert backend_for_platform(None, "PCAN_USBBUS3") == ("pcan", "PCAN_USBBUS3")


# ---------------------------------------------------------------------------
# Failure mode 1: hardware not plugged in
# ---------------------------------------------------------------------------


class TestAdapterMissing:
    def test_open_returns_false_and_does_not_exit(self, state):
        """
        The original CANinterface called sys.exit() inside a bare except.
        Unplugging the adapter took the whole HMI down, including the display
        that would have told you why.
        """
        link = CANLink(state, channel="can_absent_99", backend="socketcan")
        assert link.open() is False  # no SystemExit

    def test_raises_link_down(self, state):
        link = CANLink(state, channel="can_absent_99", backend="socketcan")
        link.open()
        assert F_LINK_DOWN in state.faults
        assert state.link_up is False

    def test_detail_tells_operator_what_to_do(self, state):
        link = CANLink(state, channel="can_absent_99", backend="socketcan")
        link.open()
        detail = state.faults[F_LINK_DOWN].detail
        assert "ip link set" in detail or "not present" in detail
        assert "can_absent_99" in detail

    def test_backoff_grows_then_saturates(self, state):
        link = CANLink(state, channel="can_absent_99", backend="socketcan")
        seen = []
        for _ in range(len(BACKOFF) + 3):
            link.open()
            seen.append(link.next_backoff())
        assert seen[0] < seen[-1]
        assert seen[-1] == BACKOFF[-1]
        assert all(b <= BACKOFF[-1] for b in seen)

    def test_reconnect_attempts_counted(self, state):
        link = CANLink(state, channel="can_absent_99", backend="socketcan")
        for _ in range(3):
            link.open()
        assert state.reconnect_attempts == 3

    def test_iface_exists_check(self):
        assert socketcan_iface_exists("definitely_not_a_real_iface_9x") is False


# ---------------------------------------------------------------------------
# Failure mode 2: wrong baud rate
# ---------------------------------------------------------------------------


class TestWrongBitrate:
    def test_silence_after_open_raises_no_traffic(self, virtual_link, state):
        """
        socketcan opens successfully regardless of bitrate -- the mismatch
        only shows up as silence plus rising error counters. Silence alone is
        the portable signal.
        """
        virtual_link._opened_at = time.time() - (NO_TRAFFIC_TIMEOUT + 5)
        state.rx_total = 0
        state.last_rx_any = 0.0
        virtual_link.poll_health()
        assert F_NO_TRAFFIC in state.faults

    def test_detail_names_bitrate_as_suspect(self, virtual_link, state):
        virtual_link._opened_at = time.time() - (NO_TRAFFIC_TIMEOUT + 5)
        state.rx_total = 0
        state.last_rx_any = 0.0
        virtual_link.poll_health()
        assert "bitrate" in state.faults[F_NO_TRAFFIC].detail

    def test_distinguishes_never_from_stopped(self, virtual_link, state):
        """
        'Never received anything' means check the bitrate. 'Traffic stopped'
        means something died mid-run. Different problems, different messages.
        """
        virtual_link._opened_at = time.time() - 20
        state.rx_total = 0
        state.last_rx_any = 0.0
        virtual_link.poll_health()
        assert "bitrate" in state.faults[F_NO_TRAFFIC].detail

        state.clear_fault(F_NO_TRAFFIC)
        state.rx_total = 500
        state.last_rx_any = time.time() - 20
        virtual_link.poll_health()
        assert "stopped" in state.faults[F_NO_TRAFFIC].detail

    def test_clears_when_traffic_resumes(self, virtual_link, state):
        virtual_link._opened_at = time.time() - 20
        state.rx_total = 0
        state.last_rx_any = 0.0
        virtual_link.poll_health()
        assert F_NO_TRAFFIC in state.faults

        state.rx_total = 10
        state.last_rx_any = time.time()
        virtual_link.poll_health()
        assert F_NO_TRAFFIC not in state.faults

    def test_no_fault_within_timeout(self, virtual_link, state):
        state.rx_total = 5
        state.last_rx_any = time.time() - 1.0
        virtual_link.poll_health()
        assert F_NO_TRAFFIC not in state.faults


class TestKernelHealthParsing:
    """
    Rising berr-counter with zero delivered frames is the definitive bitrate
    mismatch signature: the controller half-hears traffic, generates form and
    stuff errors, and NAKs. A correct-but-quiet bus shows zero errors.
    """

    SAMPLE = (
        "3: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT "
        "group default qlen 10\n"
        "    link/can  promiscuity 0 minmtu 0 maxmtu 0 \n"
        "    can state ERROR-ACTIVE restart-ms 0 \n"
        "    bitrate 250000 sample-point 0.875 \n"
        "    tq 50 prop-seg 6 phase-seg1 7 phase-seg2 2 sjw 1\n"
        "    berr-counter tx 12 rx 34 \n"
        "    re-started bus-errors arbit-lost error-warn error-pass bus-off\n"
        "    0          7          0          1          0          0         \n"
    )

    def test_parses_state(self, monkeypatch):
        self._patch(monkeypatch, self.SAMPLE)
        h = read_socketcan_health("can0")
        assert h["bus_state"] == "ERROR-ACTIVE"

    def test_parses_bitrate(self, monkeypatch):
        self._patch(monkeypatch, self.SAMPLE)
        assert read_socketcan_health("can0")["bitrate"] == 250000

    def test_parses_error_counters(self, monkeypatch):
        self._patch(monkeypatch, self.SAMPLE)
        h = read_socketcan_health("can0")
        assert h["tx_error_count"] == 12
        assert h["rx_error_count"] == 34

    def test_bus_off_state(self, monkeypatch):
        self._patch(monkeypatch, self.SAMPLE.replace("ERROR-ACTIVE", "BUS-OFF"))
        assert read_socketcan_health("can0")["bus_state"] == "BUS-OFF"

    def test_missing_interface_returns_empty(self, monkeypatch):
        import subprocess

        def fake_run(*a, **k):
            class R:
                returncode = 1
                stdout = ""

            return R()

        monkeypatch.setattr(subprocess, "run", fake_run)
        assert read_socketcan_health("can0") == {}

    def test_non_linux_returns_empty(self, monkeypatch):
        monkeypatch.setattr("hmi_canlink.IS_LINUX", False)
        assert read_socketcan_health("can0") == {}

    def test_ip_command_absent_returns_empty(self, monkeypatch):
        import subprocess

        def boom(*a, **k):
            raise FileNotFoundError("no ip")

        monkeypatch.setattr(subprocess, "run", boom)
        assert read_socketcan_health("can0") == {}

    @staticmethod
    def _patch(monkeypatch, text):
        import subprocess

        monkeypatch.setattr("hmi_canlink.IS_LINUX", True)

        def fake_run(*a, **k):
            class R:
                returncode = 0
                stdout = text

            return R()

        monkeypatch.setattr(subprocess, "run", fake_run)


# ---------------------------------------------------------------------------
# Failure mode 3: missing messages
# ---------------------------------------------------------------------------


class TestStaleSources:
    def test_names_the_quiet_source(self, virtual_link, state, bridge):
        now = time.time()
        state.mark_source(0x09F112F8, now - 10)  # compass_F8, stale
        state.mark_source(0x0CF00400, now)       # engine, fresh
        state.last_rx_any = now
        state.rx_total = 100
        virtual_link.poll_health()
        assert F_STALE in state.faults
        detail = state.faults[F_STALE].detail
        assert "compass_F8" in detail
        assert "engine" not in detail

    def test_clears_when_all_fresh(self, virtual_link, state, bridge):
        now = time.time()
        state.mark_source(0x09F112F8, now - 10)
        state.last_rx_any = now
        state.rx_total = 100
        virtual_link.poll_health()
        assert F_STALE in state.faults

        for cid in state.sources:
            state.mark_source(cid, time.time())
        virtual_link.poll_health()
        assert F_STALE not in state.faults

    def test_never_seen_source_is_not_stale(self, virtual_link, state, bridge):
        """
        A PGN that never arrived at all is a different condition from one that
        stopped. Flagging every unseen source at startup would bury the real
        signal in noise.
        """
        state.last_rx_any = time.time()
        state.rx_total = 10
        virtual_link.poll_health()
        assert F_STALE not in state.faults

    def test_multiple_stale_sources_all_named(self, virtual_link, state, bridge):
        now = time.time()
        state.mark_source(0x09F112F8, now - 10)
        state.mark_source(0x19F10D13, now - 10)
        state.last_rx_any = now
        state.rx_total = 100
        virtual_link.poll_health()
        detail = state.faults[F_STALE].detail
        assert "compass_F8" in detail and "rudder" in detail


# ---------------------------------------------------------------------------
# Send path
# ---------------------------------------------------------------------------


class TestSend:
    def test_send_on_closed_bus_returns_false(self, state):
        link = CANLink(state, channel="can_absent_99", backend="socketcan")
        link.open()
        ok = link.send(can.Message(arbitration_id=0x123, data=b"\x01", is_extended_id=True))
        assert ok is False
        assert F_TX_FAIL in state.faults

    def test_successful_send_increments_and_clears(self, virtual_link, state):
        ok = virtual_link.send(
            can.Message(arbitration_id=0x123, data=b"\x01", is_extended_id=True)
        )
        assert ok is True
        assert state.tx_total == 1
        assert F_TX_FAIL not in state.faults

    def test_adapter_yanked_mid_send_marks_link_down(self, virtual_link, state):
        """OSError (ENETDOWN) is what a mid-run unplug looks like."""

        def boom(*a, **k):
            raise OSError(100, "Network is down")

        virtual_link.bus.send = boom
        ok = virtual_link.send(
            can.Message(arbitration_id=0x123, data=b"\x01", is_extended_id=True)
        )
        assert ok is False
        assert F_LINK_DOWN in state.faults
        assert state.link_up is False


# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------


class TestLifecycle:
    def test_open_clears_prior_link_down(self, state):
        state.raise_fault(F_LINK_DOWN, "stale from earlier")
        link = CANLink(state, channel="vcan_lifecycle", backend="virtual")
        assert link.open() is True
        assert F_LINK_DOWN not in state.faults
        link.close()

    def test_close_marks_down(self, state):
        link = CANLink(state, channel="vcan_lifecycle2", backend="virtual")
        link.open()
        link.close()
        assert state.link_up is False

    def test_close_is_idempotent(self, state):
        link = CANLink(state, channel="vcan_lifecycle3", backend="virtual")
        link.open()
        link.close()
        link.close()  # must not raise

    def test_reopen_after_failure(self, state):
        """Plugging the adapter back in must recover without a restart."""
        link = CANLink(state, channel="can_absent_99", backend="socketcan")
        assert link.open() is False
        link.interface, link.channel = "virtual", "vcan_recover"
        assert link.open() is True
        assert state.link_up is True
        assert F_LINK_DOWN not in state.faults
        link.close()

    def test_state_records_connection_params(self, state):
        link = CANLink(state, channel="can5", bitrate=500000, backend="socketcan")
        assert state.channel == "can5"
        assert state.bitrate == 500000
        assert state.backend == "socketcan"
