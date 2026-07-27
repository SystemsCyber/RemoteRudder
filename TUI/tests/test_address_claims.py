"""
Tests for J1939 address-claim decoding and the expanded source table.

Motivated by the stationary-south logs: two heading sources disagree (the
Garmin compass on SA 0xF8 reads steady ~170 deg, the GPS vehicle-direction on
SA 0x1C reads a stale ~60 deg course), and the old source table showed neither
one's identity. Address claims turn the table into "who is actually saying
what," which is how you tell the good heading source from the bad one.

The real claim in the logs is from SA 0x1C: manufacturer 114 (Garmin),
industry 4 (Marine), identity 328706. These tests pin the decode against that
and against the request mechanism that pulls claims from nodes (like the
compass) that do not announce on their own.
"""

from __future__ import annotations

import can
import pytest

from j1939_name import (
    MANUFACTURERS,
    REQUEST_ADDRESS_CLAIM_DATA,
    build_request_for_address_claim,
    decode_name,
    is_address_claim,
    source_address,
)


# The real address claim captured from SA 0x1C in the stationary logs.
REAL_CLAIM_1C = bytes.fromhex("0204450e00000042")


class TestNameDecode:
    def test_real_garmin_claim(self):
        """
        The actual claim from the boat. manufacturer 114 = Garmin,
        industry 4 = Marine, identity 328706. If this breaks, the bit layout
        is wrong.
        """
        n = decode_name(REAL_CLAIM_1C)
        assert n is not None
        assert n.mfg_code == 114
        assert n.manufacturer == "Garmin"
        assert n.industry_group == 4
        assert n.industry_name == "Marine"
        assert n.identity == 328706

    def test_short_payload_returns_none(self):
        assert decode_name(b"\x01\x02\x03") is None

    def test_none_payload(self):
        assert decode_name(None) is None

    def test_bit_fields_independent(self):
        """
        Construct a NAME with distinct values in each field and confirm they
        decode without bleeding into each other.
        """
        name = 0
        name |= 12345               # identity (21 bits)
        name |= 114 << 21           # mfg
        name |= 3 << 32             # ecu instance
        name |= 7 << 35             # function instance
        name |= 140 << 40           # function (heading sensor, marine)
        name |= 10 << 49            # vehicle system
        name |= 4 << 60             # industry (marine)
        name |= 1 << 63             # arbitrary address
        data = name.to_bytes(8, "little")

        n = decode_name(data)
        assert n.identity == 12345
        assert n.mfg_code == 114
        assert n.ecu_instance == 3
        assert n.function_instance == 7
        assert n.function == 140
        assert n.vehicle_system == 10
        assert n.industry_group == 4
        assert n.arbitrary_address is True

    def test_marine_function_names(self):
        """Function meaning is marine-specific; a heading sensor should read so."""
        name = (140 << 40) | (4 << 60)  # function 140, industry marine
        n = decode_name(name.to_bytes(8, "little"))
        assert "Heading" in n.function_name

    def test_unknown_mfg_shows_number(self):
        name = (999 << 21)  # a code not in the table
        n = decode_name(name.to_bytes(8, "little"))
        assert "999" in n.manufacturer

    def test_known_manufacturers_present(self):
        """A few marine makers we ship in the lookup."""
        assert MANUFACTURERS[114] == "Garmin"
        assert 135 in MANUFACTURERS  # Airmar
        assert 172 in MANUFACTURERS  # Simrad


class TestAddressClaimDetection:
    def test_detects_claim_id(self):
        assert is_address_claim(0x18EEFF1C) is True

    def test_detects_claim_any_source(self):
        assert is_address_claim(0x18EEFFF8) is True

    def test_non_claim_rejected(self):
        assert is_address_claim(0x18FEE81C) is False  # vehicle direction
        assert is_address_claim(0x09F112F8) is False  # vessel heading

    def test_source_address_extraction(self):
        assert source_address(0x18EEFF1C) == 0x1C
        assert source_address(0x09F112F8) == 0xF8


class TestRequestForAddressClaim:
    def test_request_id_structure(self):
        """
        The request must be PGN 59904 (0xEA) to the global address 0xFF, so
        every node answers.
        """
        req_id = build_request_for_address_claim()
        pf = (req_id >> 16) & 0xFF
        ps = (req_id >> 8) & 0xFF
        assert pf == 0xEA
        assert ps == 0xFF

    def test_request_payload_targets_pgn_60928(self):
        """The 3-byte data is PGN 60928 little-endian: 00 EE 00."""
        assert REQUEST_ADDRESS_CLAIM_DATA == bytes([0x00, 0xEE, 0x00])

    def test_priority_is_six(self):
        req_id = build_request_for_address_claim()
        priority = (req_id >> 26) & 0x7
        assert priority == 6


# ---------------------------------------------------------------------------
# Decoder integration
# ---------------------------------------------------------------------------


def claim_frame(sa, data):
    return can.Message(arbitration_id=0x18EE0000 | (0xFF << 8) | sa,
                       data=data, is_extended_id=True)


class TestSourceTableIdentity:
    def test_claim_recorded_by_source_address(self, state):
        state.record_claim(0x1C, {"manufacturer": "Garmin", "function": "Navigation",
                                  "identity": 328706})
        got = state.claim_for(0x09F8021C)  # a PGN from SA 0x1C
        assert got is not None
        assert got["manufacturer"] == "Garmin"

    def test_claim_joins_all_pgns_from_same_sa(self, state):
        """
        One node sends several PGNs (several full CAN IDs) but has one NAME.
        Every PGN from SA 0x1C should resolve to the same identity.
        """
        state.record_claim(0x1C, {"manufacturer": "Garmin"})
        for cid in (0x09F8021C, 0x09F8011C, 0x18FEE81C):
            assert state.claim_for(cid)["manufacturer"] == "Garmin"

    def test_unclaimed_source_returns_none(self, state):
        """
        The compass (SA 0xF8) did not claim in the logs. Its identity is
        unknown until requested -- the table shows blank, not wrong.
        """
        assert state.claim_for(0x09F112F8) is None

    def test_snapshot_includes_identity_columns(self, state):
        from hmi_bridge import WATCHED

        for cid, (name, stale) in WATCHED.items():
            state.register_source(cid, name, stale)
        state.mark_source(0x09F8021C, data=b"\x01\x02")
        state.record_claim(0x1C, {"manufacturer": "Garmin", "function": "Nav",
                                  "vehicle_system": "Navigation", "identity": 328706})
        snap = state.snapshot()
        gps = [s for s in snap["sources"] if s["id"] == 0x09F8021C][0]
        assert gps["manufacturer"] == "Garmin"
        assert gps["data"] == "0102"

    def test_data_column_populated(self, state):
        from hmi_bridge import WATCHED

        cid = 0x18F01D21
        state.register_source(cid, "steering", 1.5)
        state.mark_source(cid, data=bytes.fromhex("0F821680FFFFFFFF"))
        snap = state.snapshot()
        src = [s for s in snap["sources"] if s["id"] == cid][0]
        assert src["data"] == "0F821680FFFFFFFF"


class TestTwoHeadingSources:
    """
    The diagnostic the whole feature exists for: distinguishing the two heading
    sources. The compass (F8) is steady and correct; the GPS vehicle-direction
    (1C) is a stale course. With identity in the table, they are told apart by
    manufacturer/function rather than by guessing from the hex ID.
    """

    def test_compass_and_gps_are_distinct_sources(self, state):
        from hmi_bridge import WATCHED

        # compass on F8, gps vehicle-dir on 1C
        assert 0x09F112F8 in WATCHED  # compass_F8
        assert 0x18FEE81C in WATCHED  # gps_vehicle_dir
        assert (0x09F112F8 & 0xFF) != (0x18FEE81C & 0xFF)

    def test_compass_source_renamed_clearly(self, state):
        """The compass source name should make it obvious it's the compass."""
        from hmi_bridge import WATCHED

        name = WATCHED[0x09F112F8][0]
        assert "compass" in name.lower()


class TestRequestCommand:
    """The request-addresses command sends a real J1939 request frame."""

    def test_command_sends_request_frame(self):
        from unittest.mock import MagicMock
        from app_tui import HMIApp
        from hmi_state import SystemState

        app = HMIApp.__new__(HMIApp)
        app.state = SystemState()
        app.autopilot = MagicMock()
        app.can = MagicMock()
        app.monitor = MagicMock()
        app.tui = None

        app.handle_command("request_addresses", "tui")

        assert app.can.bus.send.called
        sent = app.can.bus.send.call_args[0][0]
        # PGN 59904 (0xEA) to global 0xFF, payload targets PGN 60928
        assert (sent.arbitration_id >> 16) & 0xFF == 0xEA
        assert (sent.arbitration_id >> 8) & 0xFF == 0xFF
        assert bytes(sent.data) == bytes([0x00, 0xEE, 0x00])

    def test_command_safe_without_can(self):
        from unittest.mock import MagicMock
        from app_tui import HMIApp
        from hmi_state import SystemState

        app = HMIApp.__new__(HMIApp)
        app.state = SystemState()
        app.autopilot = None
        app.can = None
        app.monitor = MagicMock()
        app.tui = None
        app.handle_command("request_addresses", "tui")  # must not raise
