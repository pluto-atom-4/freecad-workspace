#!/usr/bin/env python3
"""
Unit tests for the ctrl-wheels CSV log writer (ctrl_wheels_log), task F09
(freecad-workspace #493, umbrella #482).

SIM ONLY. Not hardware-validated. Chip-side signs UNCONFIRMED; Webots +1/+1 owner-observed (2026-11-06, sim only).

This suite pins contract S6 of poc/ctrl-wheels/CONTRACT.md for ctrl_wheels_log:
the 19-column tuple, the column docs, the cell formatting rules (int columns,
float columns with %.6f and %.3f), the refusals (bool, NaN, inf, values out of
float range, unknown columns), the atomic-row rule, and the file and close
behaviour. The expected text is hand-written. The column tuple is duplicated
here by design. When the contract changes, edit the contract and this test
together.

Every test writes only under pytest's tmp_path, never to a real path, and
opens no network port.

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

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ctrl_wheels_log import COLUMN_DOC, COLUMNS, CsvLog


EXPECTED_COLUMNS = (
    "host_t_s",
    "whl_seq",
    "whl_t_us",
    "whl_model_rad_s",
    "whl_raw_l",
    "whl_raw_r",
    "whl_state",
    "cmd_seq",
    "cmd_left_rad_s",
    "cmd_right_rad_s",
    "stale",
    "echo_cmd_seq",
    "sim_t_s",
    "angle_rad_l",
    "angle_rad_r",
    "vel_rad_s_l",
    "vel_rad_s_r",
    "host_rtt_ms",
    "missed_whl",
)

INT_COLUMNS = (
    "whl_seq",
    "whl_t_us",
    "whl_raw_l",
    "whl_raw_r",
    "whl_state",
    "cmd_seq",
    "stale",
    "echo_cmd_seq",
    "missed_whl",
)

FLOAT_COLUMNS = (
    "host_t_s",
    "whl_model_rad_s",
    "cmd_left_rad_s",
    "cmd_right_rad_s",
    "sim_t_s",
    "angle_rad_l",
    "angle_rad_r",
    "vel_rad_s_l",
    "vel_rad_s_r",
    "host_rtt_ms",
)

NO_ECHO_KEYS = (
    "echo_cmd_seq",
    "sim_t_s",
    "angle_rad_l",
    "angle_rad_r",
    "vel_rad_s_l",
    "vel_rad_s_r",
    "host_rtt_ms",
)

HEADER = list(EXPECTED_COLUMNS)

FULL_ROW_VALUES = {
    "host_t_s": 1.5,
    "whl_seq": 7,
    "whl_t_us": 4000000000,
    "whl_model_rad_s": 0.5,
    "whl_raw_l": -12,
    "whl_raw_r": 12,
    "whl_state": 0,
    "cmd_seq": 3,
    "cmd_left_rad_s": 0.25,
    "cmd_right_rad_s": -0.25,
    "stale": 0,
    "echo_cmd_seq": 3,
    "sim_t_s": 2.0,
    "angle_rad_l": -0.1,
    "angle_rad_r": 0.1,
    "vel_rad_s_l": 1 / 3,
    "vel_rad_s_r": -1 / 3,
    "host_rtt_ms": 12.5,
    "missed_whl": 0,
}

FULL_ROW_TEXT = [
    "1.500",
    "7",
    "4000000000",
    "0.500000",
    "-12",
    "12",
    "0",
    "3",
    "0.250000",
    "-0.250000",
    "0",
    "3",
    "2.000000",
    "-0.100000",
    "0.100000",
    "0.333333",
    "-0.333333",
    "12.500",
    "0",
]

NO_ECHO_VALUES = {
    "host_t_s": 2.0,
    "whl_seq": 8,
    "whl_t_us": 4000100000,
    "whl_model_rad_s": 0.5,
    "whl_raw_l": 10,
    "whl_raw_r": -10,
    "whl_state": 0,
    "cmd_seq": 4,
    "cmd_left_rad_s": 0.5,
    "cmd_right_rad_s": -0.5,
    "stale": 1,
    "missed_whl": 0,
}

NO_ECHO_TEXT = [
    "2.000",
    "8",
    "4000100000",
    "0.500000",
    "10",
    "-10",
    "0",
    "4",
    "0.500000",
    "-0.500000",
    "1",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "0",
]

ALL_EMPTY_TEXT = [""] * 19


def _full_row():
    """Build a full row whose text is FULL_ROW_TEXT."""
    return dict(FULL_ROW_VALUES)


def _no_echo_row(missing):
    """Command with no echo.

    missing=True omits the NO_ECHO_KEYS; missing=False sets them to None.
    """
    row = dict(NO_ECHO_VALUES)
    if not missing:
        for name in NO_ECHO_KEYS:
            row[name] = None
    return row


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
    """Test: COLUMNS is the pinned 19-name tuple in contract order."""

    def test_columns_is_tuple_of_19(self):
        assert isinstance(COLUMNS, tuple)
        assert len(COLUMNS) == 19

    def test_columns_exact_order(self):
        assert COLUMNS == EXPECTED_COLUMNS


class TestColumnDoc:
    """Test: COLUMN_DOC documents every column, one line each."""

    def test_keys_equal_columns(self):
        assert list(COLUMN_DOC) == list(COLUMNS)

    def test_each_doc_is_one_non_empty_line(self):
        for name, text in COLUMN_DOC.items():
            assert isinstance(text, str), name
            assert text.strip(), name
            assert "\n" not in text and "\r" not in text, name

    @pytest.mark.parametrize("name", ["whl_raw_l", "whl_raw_r"])
    def test_raw_docs_do_not_claim_servo_readback(self, name):
        assert "reported by the servo" not in COLUMN_DOC[name]

    @pytest.mark.parametrize("name", ["whl_raw_l", "whl_raw_r"])
    def test_raw_docs_say_not_a_servo_readback(self, name):
        assert "not a servo readback" in COLUMN_DOC[name]


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

    def test_header_line_unchanged_after_row(self, tmp_path):
        path = tmp_path / "log.csv"
        with CsvLog(path) as log:
            header_before = path.read_text(encoding="utf-8").splitlines()[0]
            log.write_row(_full_row())
        lines = path.read_text(encoding="utf-8").splitlines()
        assert lines[0] == header_before
        assert lines[0] == ",".join(EXPECTED_COLUMNS)
        assert len(lines) == 2


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


class TestIntegerFormat:
    """Test: integer columns are decimal integers, and a float is refused."""

    def test_int_extremes_exact_text(self, tmp_path):
        path = tmp_path / "ints.csv"
        row = _full_row()
        row.update(
            whl_seq=65535,
            whl_t_us=4294967295,
            whl_raw_l=-2147483648,
            whl_raw_r=2147483647,
            whl_state=2,
            cmd_seq=0,
            stale=1,
            echo_cmd_seq=0,
            missed_whl=0,
        )
        log = CsvLog(path)
        log.write_row(row)
        log.close()
        expected = [
            "1.500",
            "65535",
            "4294967295",
            "0.500000",
            "-2147483648",
            "2147483647",
            "2",
            "0",
            "0.250000",
            "-0.250000",
            "1",
            "0",
            "2.000000",
            "-0.100000",
            "0.100000",
            "0.333333",
            "-0.333333",
            "12.500",
            "0",
        ]
        assert _read_rows(path) == [HEADER, expected]

    @pytest.mark.parametrize("col", INT_COLUMNS)
    def test_float_refused_for_int_column(self, tmp_path, col):
        row = _full_row()
        row[col] = 7.0
        _assert_rejected(tmp_path, row)


class TestFloatFormat:
    """Test: float columns use %.6f, host_t_s and host_rtt_ms use %.3f."""

    @pytest.mark.parametrize(
        "col, value, text",
        [
            ("cmd_left_rad_s", 0.1234567, "0.123457"),
            ("host_t_s", 1.0, "1.000"),
            ("host_rtt_ms", 12.3456, "12.346"),
        ],
        ids=["six-decimals", "host-t-three-decimals", "rtt-three-decimals"],
    )
    def test_float_cell_text(self, tmp_path, col, value, text):
        path = tmp_path / "fmt.csv"
        row = _full_row()
        row[col] = value
        expected = list(FULL_ROW_TEXT)
        expected[EXPECTED_COLUMNS.index(col)] = text
        log = CsvLog(path)
        log.write_row(row)
        log.close()
        assert _read_rows(path) == [HEADER, expected]


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
        assert _read_rows(path) == [HEADER, ALL_EMPTY_TEXT]


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
    """Test: an unknown key is refused and nothing is written.

    The single-wheel names are included as a drift guard: they must not be
    accepted by the twin writer.
    """

    @pytest.mark.parametrize(
        "key",
        ["bogus", "whl_raw", "cmd_rad_s", "angle_rad", "vel_rad_s"],
    )
    def test_unknown_key_raises_value_error(self, tmp_path, key):
        row = _full_row()
        row[key] = 1
        _assert_rejected(tmp_path, row)


class TestRejectedValues:
    """Test: bool, NaN and inf are refused and nothing is written."""

    @pytest.mark.parametrize(
        "col", ["whl_seq", "cmd_left_rad_s", "whl_raw_l", "whl_raw_r"]
    )
    def test_bool_rejected(self, tmp_path, col):
        row = _full_row()
        row[col] = True
        _assert_rejected(tmp_path, row)

    @pytest.mark.parametrize("col", FLOAT_COLUMNS)
    @pytest.mark.parametrize(
        "bad", [math.nan, math.inf, -math.inf], ids=["nan", "inf", "-inf"]
    )
    def test_non_finite_rejected(self, tmp_path, col, bad):
        row = _full_row()
        row[col] = bad
        _assert_rejected(tmp_path, row)

    @pytest.mark.parametrize(
        "col",
        [
            "whl_seq",
            "whl_raw_l",
            "missed_whl",
            "cmd_left_rad_s",
            "host_t_s",
            "whl_model_rad_s",
        ],
    )
    @pytest.mark.parametrize("text", ["7", "1.5", ""], ids=["digits", "decimal", "empty"])
    def test_str_rejected(self, tmp_path, col, text):
        row = _full_row()
        row[col] = text
        _assert_rejected(tmp_path, row)


class TestOutOfFloatRange:
    """Test: an int too large for float() is refused with ValueError, nothing written."""

    @pytest.mark.parametrize("col", ["cmd_left_rad_s", "angle_rad_r", "host_rtt_ms"])
    @pytest.mark.parametrize(
        "bad", [10**400, -(10**400)], ids=["huge-positive", "huge-negative"]
    )
    def test_out_of_float_range_rejected(self, tmp_path, col, bad):
        row = _full_row()
        row[col] = bad
        _assert_rejected(tmp_path, row)


class TestAtomicRow:
    """Test: a refused row between two valid rows leaves no partial line."""

    def test_refused_row_leaves_no_partial_line(self, tmp_path):
        path = tmp_path / "atomic.csv"
        bad = _full_row()
        bad["missed_whl"] = 1.5
        log = CsvLog(path)
        log.write_row(_full_row())
        with pytest.raises(ValueError):
            log.write_row(bad)
        log.write_row(_full_row())
        log.close()
        assert _read_rows(path) == [HEADER, FULL_ROW_TEXT, FULL_ROW_TEXT]


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

    def test_with_block_writes_and_closes(self, tmp_path):
        path = tmp_path / "cm.csv"
        log = CsvLog(path)
        with log:
            log.write_row(_full_row())
        assert _read_rows(path) == [HEADER, FULL_ROW_TEXT]

    def test_closes_when_body_raises(self, tmp_path):
        path = tmp_path / "cm_exc.csv"
        log = CsvLog(path)
        with pytest.raises(RuntimeError):
            with log:
                log.write_row(_full_row())
                raise RuntimeError("boom")
        assert _read_rows(path) == [HEADER, FULL_ROW_TEXT]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
