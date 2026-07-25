"""
Tests for the kernel-counter path in CANLink.poll_health.

This is the code that distinguishes a *wrong bitrate* from a *quiet bus*, and
it is the hardest part of the fault model to exercise: it only runs when the
interface is socketcan, and it reads real kernel statistics. Neither condition
holds on the virtual bus used elsewhere in the suite.

So these tests build a CANLink whose interface says "socketcan" but whose bus
is virtual, and feed it synthetic `ip -details -statistics link` output. That
isolates the decision logic from the plumbing.

The distinction being tested:

    silent bus, zero errors      -> nothing is talking (check wiring, or the
                                    other nodes are off)
    silent bus, rising errors    -> we are on the wire but cannot frame it
                                    (check bitrate)
    bus-off                      -> controller gave up entirely
"""

from __future__ import annotations

import time

import pytest

from hmi_canlink import CANLink
from hmi_state import F_BUS_ERRORS, F_BUS_OFF, F_NO_TRAFFIC


def ip_output(state="ERROR-ACTIVE", bitrate=250000, tx_err=0, rx_err=0, bus_errors=0):
    """Synthesize `ip -details -statistics link show can0` output."""
    return (
        "3: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP "
        "mode DEFAULT group default qlen 10\n"
        "    link/can  promiscuity 0 minmtu 0 maxmtu 0 \n"
        f"    can state {state} restart-ms 0 \n"
        f"    bitrate {bitrate} sample-point 0.875 \n"
        "    tq 50 prop-seg 6 phase-seg1 7 phase-seg2 2 sjw 1\n"
        f"    berr-counter tx {tx_err} rx {rx_err} \n"
        "    re-started bus-errors arbit-lost error-warn error-pass bus-off\n"
        f"    0          {bus_errors}          0          0          0         \n"
    )


@pytest.fixture
def sc_link(state, monkeypatch):
    """
    A CANLink that believes it is on socketcan but is backed by a virtual bus.

    poll_health branches on self.interface, so overriding it after
    construction is enough to reach the kernel-counter code without needing a
    real CAN device.
    """
    link = CANLink(state, channel="can0", bitrate=250000, backend="virtual")
    assert link.open() is True
    link.interface = "socketcan"
    link._last_health_poll = 0.0  # force the poll to run
    yield link
    link.close()


def patch_ip(monkeypatch, text):
    import subprocess

    monkeypatch.setattr("hmi_canlink.IS_LINUX", True)

    def fake_run(*a, **k):
        class R:
            returncode = 0
            stdout = text

        return R()

    monkeypatch.setattr(subprocess, "run", fake_run)


class TestBitrateMismatchSignature:
    def test_rising_counters_raise_bus_errors(self, sc_link, state, monkeypatch):
        """
        The definitive wrong-bitrate signature: error counters climbing while
        no valid frames are delivered. The controller is hearing edges at the
        wrong times and NAKing everything.
        """
        patch_ip(monkeypatch, ip_output(tx_err=5, rx_err=5))
        sc_link.poll_health()

        sc_link._last_health_poll = 0.0
        patch_ip(monkeypatch, ip_output(tx_err=40, rx_err=60))
        sc_link.poll_health()

        assert F_BUS_ERRORS in state.faults

    def test_detail_names_configured_bitrate(self, sc_link, state, monkeypatch):
        patch_ip(monkeypatch, ip_output(tx_err=5, rx_err=5))
        sc_link.poll_health()
        sc_link._last_health_poll = 0.0
        patch_ip(monkeypatch, ip_output(tx_err=40, rx_err=60))
        sc_link.poll_health()

        detail = state.faults[F_BUS_ERRORS].detail
        assert "bitrate" in detail.lower()
        assert "250000" in detail

    def test_quiet_bus_with_zero_errors_is_not_a_bitrate_fault(
        self, sc_link, state, monkeypatch
    ):
        """
        A correctly configured but idle bus must not be blamed on bitrate.
        Reporting BUS_ERRORS here would send the operator chasing the wrong
        problem while the actual cause is that nothing is powered on.
        """
        patch_ip(monkeypatch, ip_output(tx_err=0, rx_err=0))
        sc_link.poll_health()
        assert F_BUS_ERRORS not in state.faults

    def test_static_nonzero_counters_do_not_re_raise(self, sc_link, state, monkeypatch):
        """
        Historical errors that are not increasing are not an active fault.
        Only the delta matters.
        """
        patch_ip(monkeypatch, ip_output(tx_err=50, rx_err=50))
        sc_link.poll_health()
        state.clear_fault(F_BUS_ERRORS)

        sc_link._last_health_poll = 0.0
        patch_ip(monkeypatch, ip_output(tx_err=50, rx_err=50))
        sc_link.poll_health()
        assert F_BUS_ERRORS not in state.faults

    def test_small_error_count_below_threshold_ignored(self, sc_link, state, monkeypatch):
        """A handful of errors is normal on a busy bus; only sustained counts matter."""
        patch_ip(monkeypatch, ip_output(tx_err=0, rx_err=0))
        sc_link.poll_health()
        sc_link._last_health_poll = 0.0
        patch_ip(monkeypatch, ip_output(tx_err=2, rx_err=3))
        sc_link.poll_health()
        assert F_BUS_ERRORS not in state.faults

    def test_counters_returning_to_zero_clears(self, sc_link, state, monkeypatch):
        patch_ip(monkeypatch, ip_output(tx_err=5, rx_err=5))
        sc_link.poll_health()
        sc_link._last_health_poll = 0.0
        patch_ip(monkeypatch, ip_output(tx_err=40, rx_err=60))
        sc_link.poll_health()
        assert F_BUS_ERRORS in state.faults

        sc_link._last_health_poll = 0.0
        patch_ip(monkeypatch, ip_output(tx_err=0, rx_err=0))
        sc_link.poll_health()
        assert F_BUS_ERRORS not in state.faults


class TestBusOff:
    def test_bus_off_raises_fault(self, sc_link, state, monkeypatch):
        patch_ip(monkeypatch, ip_output(state="BUS-OFF"))
        sc_link.poll_health()
        assert F_BUS_OFF in state.faults

    def test_detail_mentions_termination(self, sc_link, state, monkeypatch):
        """
        Bus-off usually means termination or bitrate, and the message should
        say so -- it is the operator's first check.
        """
        patch_ip(monkeypatch, ip_output(state="BUS-OFF"))
        sc_link.poll_health()
        detail = state.faults[F_BUS_OFF].detail.lower()
        assert "termination" in detail or "bitrate" in detail

    def test_recovery_clears(self, sc_link, state, monkeypatch):
        patch_ip(monkeypatch, ip_output(state="BUS-OFF"))
        sc_link.poll_health()
        assert F_BUS_OFF in state.faults

        sc_link._last_health_poll = 0.0
        patch_ip(monkeypatch, ip_output(state="ERROR-ACTIVE"))
        sc_link.poll_health()
        assert F_BUS_OFF not in state.faults

    def test_error_passive_is_not_bus_off(self, sc_link, state, monkeypatch):
        patch_ip(monkeypatch, ip_output(state="ERROR-PASSIVE"))
        sc_link.poll_health()
        assert F_BUS_OFF not in state.faults
        assert state.bus_state == "ERROR-PASSIVE"


class TestStatePopulation:
    def test_bus_state_recorded(self, sc_link, state, monkeypatch):
        patch_ip(monkeypatch, ip_output(state="ERROR-ACTIVE"))
        sc_link.poll_health()
        assert state.bus_state == "ERROR-ACTIVE"

    def test_error_counts_recorded(self, sc_link, state, monkeypatch):
        patch_ip(monkeypatch, ip_output(tx_err=7, rx_err=11))
        sc_link.poll_health()
        assert state.tx_error_count == 7
        assert state.rx_error_count == 11

    def test_actual_bitrate_overrides_configured(self, sc_link, state, monkeypatch):
        """
        If the netdev was brought up at a different rate than the HMI was
        told, believe the kernel. This is how a bitrate typo in the systemd
        unit becomes visible instead of silent.
        """
        patch_ip(monkeypatch, ip_output(bitrate=500000))
        sc_link.poll_health()
        assert state.bitrate == 500000

    def test_poll_is_rate_limited(self, sc_link, state, monkeypatch):
        """Shelling out to `ip` on every tick would be wasteful at 4 Hz."""
        calls = []
        import subprocess

        monkeypatch.setattr("hmi_canlink.IS_LINUX", True)

        def counting_run(*a, **k):
            calls.append(1)

            class R:
                returncode = 0
                stdout = ip_output()

            return R()

        monkeypatch.setattr(subprocess, "run", counting_run)

        sc_link.poll_health()
        sc_link.poll_health()
        sc_link.poll_health()
        assert len(calls) == 1, "health poll should be throttled to ~1 Hz"


class TestNonLinuxFallback:
    def test_windows_skips_kernel_check(self, state, monkeypatch):
        """
        On Windows there are no kernel counters, so bitrate problems surface
        only through the no-traffic timeout. That is less specific but still
        catches the failure.
        """
        monkeypatch.setattr("hmi_canlink.IS_LINUX", False)
        monkeypatch.setattr("hmi_canlink.IS_WINDOWS", True)

        link = CANLink(state, channel="PCAN_USBBUS1", bitrate=250000, backend="virtual")
        link.open()
        link.interface = "pcan"
        link._opened_at = time.time() - 20
        state.rx_total = 0
        state.last_rx_any = 0.0

        link.poll_health()
        assert F_NO_TRAFFIC in state.faults
        assert "bitrate" in state.faults[F_NO_TRAFFIC].detail
        link.close()
