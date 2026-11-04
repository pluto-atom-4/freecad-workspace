#!/usr/bin/env python3
"""
Unit tests for the ctrl-wheels CSV log writer (ctrl_wheels_log), issue FW #452.

Test-first: the module under test is a flat, stdlib-only module in poc/ctrl-wheels
that exists alongside it. Every test writes only under pytest's tmp_path,
never to a real path, and opens no network port. SIM ONLY. The sign convention
is UNVERIFIED.

Usage:
    cd poc/ctrl-wheels
    python3 -m pytest test_ctrl_wheels_log.py -v
"""

import csv
import io
import math
import sys
from pathlib import Path

import pytest

# Add the script directory to path for imports
sys.path.insert(0, str(Path(__file__).resolve().parent))

from ctrl_wheels_log import COLUMN_DOC, COLUMNS, CsvLog


EXPECTED_COLUMNS = (
    "host_t_s",
    "whl_seq",
    "whl_t_us",
    "whl_model_rad_s",
    "whl_raw",
    "whl_state",
    "cmd_seq",
    "cmd_rad_s",
    "stale",
    "echo_cmd_seq",
    "sim_t_s",
    "angle_rad",
    "vel_rad_s",
    "host_rtt_ms",
    "missed_whl",
)

FLOAT_COLUMNS = (
    "host_t_s",
    "whl_model_rad_s",
    "cmd_rad_s",
    "sim_t_s",
    "angle_rad",
    "vel_rad_s",
    "host_rtt_ms",
)

HEADER = list(EXPECTED_COLUMNS)

FULL_ROW_TEXT = [
    "1.500",
    "7",
    "4000000000",
    "0.500000",
    "-12",
    "0",
    "3",
    "0.250000",
    "0",
    "3",
    "2.000000",
    "-0.100000",
    "0.333333",
    "12.500",
    "0",
]


def _full_row():
    """Build a full row whose text is FULL_ROW_TEXT."""
    return {
        "host_t_s": 1.5,
        "whl_seq": 7,
        "whl_t_us": 4000000000,
        "whl_model_rad_s": 0.5,
        "whl_raw": -12,
        "whl_state": 0,
        "cmd_seq": 3,
        "cmd_rad_s": 0.25,
        "stale": 0,
        "echo_cmd_seq": 3,
        "sim_t_s": 2.0,
        "angle_rad": -0.1,
        "vel_rad_s": 1 / 3,
        "host_rtt_ms": 12.5,
        "missed_whl": 0,
    }


def _no_echo_row(missing):
    """Command with no echo: echo_*, sim_t_s, angle_rad, vel_rad_s, host_rtt_ms empty.

    missing=True omits those keys; missing=False sets them to None.
    """
    row = {
        "host_t_s": 2.0,
        "cmd_seq": 4,
        "cmd_rad_s": 0.5,
        "stale": 1,
        "missed_whl": 0,
    }
    if not missing:
        for name in COLUMNS:
            row.setdefault(name, None)
    return row


NO_ECHO_TEXT = [
    "2.000",
    "",
    "",
    "",
    "",
    "",
    "4",
    "0.500000",
    "1",
    "",
    "",
    "",
    "",
    "",
    "0",
]


def _read_rows(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.reader(fh))


def _parse(text):
    return list(csv.reader(io.StringIO(text)))


def _assert_rejected(tmp_path, row):
    """A rejected row raises ValueError and leaves only the header in the file."""
    path = tmp_path / "reject.csv"
    log = CsvLog(path)
    try:
        with pytest.raises(ValueError):
            log.write_row(row)
    finally:
        log.close()
    assert _read_rows(path) == [HEADER]


class TestColumns:
    """Test: COLUMNS is the pinned 15-name tuple in order."""

    def test_columns_exact(self):
        assert isinstance(COLUMNS, tuple)
        assert COLUMNS == EXPECTED_COLUMNS
        assert len(COLUMNS) == 15


class TestColumnDoc:
    """Test: COLUMN_DOC documents every column, one line each."""

    def test_keys_equal_columns(self):
        assert list(COLUMN_DOC) == list(COLUMNS)

    def test_each_doc_is_one_non_empty_line(self):
        for name, text in COLUMN_DOC.items():
            assert isinstance(text, str), name
            assert text.strip(), name
            assert "\n" not in text and "\r" not in text, name


class TestHeader:
    """Test: the header row is written at construction and equals COLUMNS."""

    def test_path_header_written_on_init(self, tmp_path):
        path = tmp_path / "log.csv"
        log = CsvLog(path)
        try:
            assert _read_rows(path) == [HEADER]
        finally:
            log.close()

    def test_file_object_header_written_on_init(self):
        buf = io.StringIO()
        log = CsvLog(buf)
        try:
            assert _parse(buf.getvalue()) == [HEADER]
        finally:
            log.close()

    def test_header_matches_columns_after_rows(self, tmp_path):
        path = tmp_path / "log.csv"
        with CsvLog(path) as log:
            log.write_row(_full_row())
        assert _read_rows(path)[0] == list(COLUMNS)


class TestRoundTrip:
    """Test: a full row is written with exact text."""

    def test_full_row_exact_text(self, tmp_path):
        path = tmp_path / "log.csv"
        log = CsvLog(path)
        log.write_row(_full_row())
        log.close()
        assert _read_rows(path) == [HEADER, FULL_ROW_TEXT]

    @pytest.mark.parametrize("as_str", [True, False], ids=["str", "pathlib"])
    def test_path_str_and_pathlib_accepted(self, tmp_path, as_str):
        path = tmp_path / "t.csv"
        target = str(path) if as_str else path
        with CsvLog(target) as log:
            log.write_row(_full_row())
        assert _read_rows(path) == [HEADER, FULL_ROW_TEXT]


class TestMissingAndNone:
    """Test: None and missing keys both give empty cells."""

    @pytest.mark.parametrize("missing", [True, False], ids=["missing", "none"])
    def test_no_echo_row_has_empty_cells(self, tmp_path, missing):
        path = tmp_path / "noecho.csv"
        log = CsvLog(path)
        log.write_row(_no_echo_row(missing))
        log.close()
        assert _read_rows(path) == [HEADER, NO_ECHO_TEXT]

    def test_all_none_row_is_all_empty(self, tmp_path):
        path = tmp_path / "allnone.csv"
        log = CsvLog(path)
        log.write_row({name: None for name in COLUMNS})
        log.close()
        assert _read_rows(path) == [HEADER, [""] * len(COLUMNS)]


class TestColumnOrder:
    """Test: output order follows COLUMNS, not the dict order."""

    def test_reversed_dict_order_same_line(self, tmp_path):
        path = tmp_path / "order.csv"
        row = _full_row()
        reversed_row = {name: row[name] for name in reversed(COLUMNS)}
        log = CsvLog(path)
        log.write_row(reversed_row)
        log.close()
        assert _read_rows(path) == [HEADER, FULL_ROW_TEXT]


class TestUnknownKey:
    """Test: an unknown key is refused and nothing is written."""

    @pytest.mark.parametrize("key", ["bogus", "whl_model", "WHL_SEQ"])
    def test_unknown_key_raises_value_error(self, tmp_path, key):
        row = _full_row()
        row[key] = 1
        _assert_rejected(tmp_path, row)


class TestRejectedValues:
    """Test: bool, NaN and inf are refused and nothing is written."""

    @pytest.mark.parametrize(
        "col", ["whl_seq", "stale", "missed_whl", "cmd_rad_s", "host_rtt_ms"]
    )
    def test_bool_rejected(self, tmp_path, col):
        row = _full_row()
        row[col] = True
        _assert_rejected(tmp_path, row)

    @pytest.mark.parametrize("col", FLOAT_COLUMNS)
    @pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf], ids=["nan", "inf", "-inf"])
    def test_non_finite_rejected(self, tmp_path, col, bad):
        row = _full_row()
        row[col] = bad
        _assert_rejected(tmp_path, row)


class TestOverwrite:
    """Test: an existing path is refused unless overwrite=True."""

    def test_existing_file_refused_by_default(self, tmp_path):
        path = tmp_path / "exists.csv"
        path.write_text("old\n", encoding="utf-8")
        with pytest.raises(FileExistsError):
            CsvLog(path)
        assert path.read_text(encoding="utf-8") == "old\n"

    def test_existing_file_replaced_with_overwrite(self, tmp_path):
        path = tmp_path / "exists.csv"
        path.write_text("old\n", encoding="utf-8")
        log = CsvLog(path, overwrite=True)
        log.write_row(_full_row())
        log.close()
        assert _read_rows(path) == [HEADER, FULL_ROW_TEXT]


class TestFileObjectTarget:
    """Test: a caller-owned file object is never closed by CsvLog."""

    def test_stringio_not_closed(self):
        buf = io.StringIO()
        log = CsvLog(buf)
        log.write_row(_full_row())
        log.close()
        assert buf.closed is False
        assert _parse(buf.getvalue()) == [HEADER, FULL_ROW_TEXT]

    def test_open_file_not_closed(self, tmp_path):
        path = tmp_path / "fh.csv"
        with open(path, "w", newline="", encoding="utf-8") as fh:
            log = CsvLog(fh)
            log.write_row(_full_row())
            log.close()
            assert fh.closed is False
        assert _read_rows(path) == [HEADER, FULL_ROW_TEXT]


class TestClose:
    """Test: close is idempotent and write_row after close is refused."""

    def test_close_idempotent(self, tmp_path):
        path = tmp_path / "close.csv"
        log = CsvLog(path)
        log.write_row(_full_row())
        log.close()
        log.close()
        assert _read_rows(path) == [HEADER, FULL_ROW_TEXT]

    def test_file_object_close_twice_leaves_open(self):
        buf = io.StringIO()
        log = CsvLog(buf)
        log.close()
        log.close()
        assert buf.closed is False

    def test_write_after_close_raises(self, tmp_path):
        path = tmp_path / "closed.csv"
        log = CsvLog(path)
        log.close()
        with pytest.raises(ValueError):
            log.write_row(_full_row())
        assert _read_rows(path) == [HEADER]


class TestContextManager:
    """Test: CsvLog works as a context manager and closes on exit."""

    def test_enter_returns_self(self, tmp_path):
        log = CsvLog(tmp_path / "cm.csv")
        with log as entered:
            assert entered is log
        with pytest.raises(ValueError):
            log.write_row(_full_row())

    def test_with_block_writes_and_closes(self, tmp_path):
        path = tmp_path / "cm.csv"
        log = CsvLog(path)
        with log:
            log.write_row(_full_row())
        assert _read_rows(path) == [HEADER, FULL_ROW_TEXT]
        with pytest.raises(ValueError):
            log.write_row(_full_row())

    def test_closes_when_body_raises(self, tmp_path):
        path = tmp_path / "cm_exc.csv"
        log = CsvLog(path)
        with pytest.raises(RuntimeError):
            with log:
                log.write_row(_full_row())
                raise RuntimeError("boom")
        assert _read_rows(path) == [HEADER, FULL_ROW_TEXT]
        with pytest.raises(ValueError):
            log.write_row(_full_row())


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
