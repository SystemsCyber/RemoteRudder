"""
Tests for the filtered CAN log recorder.

The recorder must capture exactly the frames the HMI decodes -- no more, no
less -- so the on-water log is small and is precisely what the fusion saw. It
must also keep address claims (needed to rebuild the NAME registry on replay)
and produce a candump-format file the existing replay path reads.
"""

from __future__ import annotations

import os
import tempfile

import can
import pytest

from can_recorder import FilteredCanRecorder, is_used, USED_CAN_IDS, USED_PGNS


class TestUsedFilter:
    def test_known_ids_used(self):
        for cid in USED_CAN_IDS:
            assert is_used(cid)

    def test_heading_any_source_used(self):
        # PGN 127250 from the wall compass and the 24xd, different addresses
        assert is_used(0x09F112F8)  # wall compass
        assert is_used(0x09F11219)  # 24xd
        assert is_used(0x09F11200)  # any source

    def test_address_claims_used(self):
        assert is_used(0x18EEFF1C)
        assert is_used(0x18EEFFF8)

    def test_unrelated_id_not_used(self):
        assert not is_used(0x0CF00203)  # some other engine PGN
        assert not is_used(0x18FEF100)  # unrelated


class TestRecorder:
    def test_writes_only_used_frames(self):
        with tempfile.TemporaryDirectory() as d:
            rec = FilteredCanRecorder(directory=d, channel="can0")
            # one used, one not
            rec.record(0x18F01D21, bytes.fromhex("0102030405060708"))
            rec.record(0x0CF00203, bytes.fromhex("0102030405060708"))  # unused
            rec.close()
            with open(rec.path) as f:
                lines = f.readlines()
            assert len(lines) == 1
            assert "18F01D21" in lines[0]

    def test_candump_format_roundtrips(self):
        from can.io.canutils import CanutilsLogReader
        with tempfile.TemporaryDirectory() as d:
            rec = FilteredCanRecorder(directory=d, channel="can0")
            rec.record(0x18F01D21, bytes.fromhex("DEADBEEF00000000"), when=1000.5)
            rec.close()
            msgs = list(CanutilsLogReader(rec.path))
            assert len(msgs) == 1
            assert msgs[0].arbitration_id == 0x18F01D21
            assert msgs[0].data == bytes.fromhex("DEADBEEF00000000")

    def test_stats_reports_count_and_path(self):
        with tempfile.TemporaryDirectory() as d:
            rec = FilteredCanRecorder(directory=d)
            rec.record(0x18F01D21, b"\x00" * 8)
            rec.record(0x19F10D13, b"\x00" * 8)
            st = rec.stats()
            assert st["count"] == 2
            assert st["path"].endswith(".log")
            rec.close()

    def test_record_message_convenience(self):
        with tempfile.TemporaryDirectory() as d:
            rec = FilteredCanRecorder(directory=d)
            msg = can.Message(arbitration_id=0x18F01D21, data=b"\x01" * 8,
                              is_extended_id=True, timestamp=123.0)
            rec.record_message(msg)
            rec.close()
            assert rec.stats()["count"] == 1
