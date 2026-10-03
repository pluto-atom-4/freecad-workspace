#!/usr/bin/env python3
"""
Unit tests for telemetry.py (issue #305, sub-issue of #303).

Pure Python: io.StringIO and a small fake file (counts flush/close calls and
can raise OSError on write, flush or close). No Webots, no hardware, must NOT
`import controller` or `serial`.

Reference row: _vals() has 17 values in COLUMNS order; its formatted fields are
EXPECTED_FIELDS (msgs 3 is an integer, warn 0, floats with 6 decimals).

Usage:
    cd poc/servo-wheel-dof
    mamba run -n servo-wheel-dof python3 -m pytest -q \
        webots/controllers/servo_wheel_dof/test_swd_telemetry.py
"""

from __future__ import annotations

import dataclasses
import io
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from telemetry import (  # noqa: E402
    COLUMNS,
    TelemetryWriter,
    format_row,
    parse_telemetry_env,
)

HEADER = ",".join(COLUMNS) + "\n"
NAN = float("nan")
INF = float("inf")
FLOAT_IDX = [i for i in range(17) if i not in (1, 16)]
EXPECTED_FIELDS = [
    "0.500000", "3", "0.020000", "0.100000", "0.200000", "0.300000",
    "0.400000", "0.000000", "0.000000", "0.030000", "0.030000",
    "0.000000", "0.000000", "0.026000", "0.000500", "0.999990", "0",
]


def _vals(warn=0, msgs=3):
    return [
        0.5, msgs, 0.02, 0.1, 0.2, 0.3, 0.4, 0.0, 0.0, 0.03, 0.03,
        0.0, 0.0, 0.026, 0.0005, 0.99999, warn,
    ]


class FakeFile:
    """Minimal file: records writes, counts flush/close, can raise OSError."""

    def __init__(self, *, fail_write=False, fail_flush=False, fail_close=False):
        self.fail_write = fail_write
        self.fail_flush = fail_flush
        self.fail_close = fail_close
        self.chunks = []
        self.flush_count = 0
        self.close_count = 0
        self.closed = False

    def write(self, text):
        if self.fail_write:
            raise OSError("disk full")
        self.chunks.append(text)
        return len(text)

    def flush(self):
        self.flush_count += 1
        if self.fail_flush:
            raise OSError("flush failed")

    def close(self):
        self.close_count += 1
        self.closed = True
        if self.fail_close:
            raise OSError("close failed")


class TestColumns:
    def test_exact_tuple(self):
        assert COLUMNS == (
            "t_s", "msgs", "age_s", "wheel_cmd", "wheel_sensor", "wheel_err",
            "vel_rad_s", "robot_x", "robot_y", "robot_z", "wheel_z",
            "rel_x", "rel_y", "rel_z", "anchor_dev_m", "axis_dot", "warn",
        )

    def test_length_is_17(self):
        assert len(COLUMNS) == 17

    def test_warn_is_last(self):
        assert COLUMNS[-1] == "warn"
        assert COLUMNS.index("warn") == 16

    def test_names_unique(self):
        assert len(set(COLUMNS)) == len(COLUMNS)


class TestFormatRow:
    def test_reference_row(self):
        assert format_row(_vals()).split(",") == EXPECTED_FIELDS

    def test_tuple_accepted(self):
        assert format_row(tuple(_vals())).split(",") == EXPECTED_FIELDS

    def test_shape_no_newline_16_commas(self):
        line = format_row(_vals())
        assert "\n" not in line
        assert line.count(",") == 16

    @pytest.mark.parametrize("bad", [None, INF, -INF, NAN])
    def test_non_finite_float_columns_are_nan(self, bad):
        for idx in FLOAT_IDX:
            vals = _vals()
            vals[idx] = bad
            fields = format_row(vals).split(",")
            assert len(fields) == 17
            for j, field in enumerate(fields):
                want = "nan" if j == idx else EXPECTED_FIELDS[j]
                assert field == want

    def test_non_numeric_string_is_nan(self):
        vals = _vals()
        vals[0] = "abc"
        assert format_row(vals).split(",")[0] == "nan"

    @pytest.mark.parametrize(
        "msgs, want",
        [
            (7, "7"),
            (7.0, "7"),
            (1234567, "1234567"),
            (None, "nan"),
            (NAN, "nan"),
            (INF, "nan"),
        ],
    )
    def test_msgs_is_integer(self, msgs, want):
        assert format_row(_vals(msgs=msgs)).split(",")[1] == want

    @pytest.mark.parametrize(
        "value, want",
        [
            (1 / 3, "0.333333"),
            (2 / 3, "0.666667"),
            (1e-9, "0.000000"),
            (-0.5, "-0.500000"),
            (12, "12.000000"),
        ],
    )
    def test_six_decimals(self, value, want):
        vals = _vals()
        vals[0] = value
        assert format_row(vals).split(",")[0] == want

    @pytest.mark.parametrize(
        "warn, want",
        [(True, "1"), (False, "0"), (1, "1"), (0, "0"), (None, "0")],
    )
    def test_warn_is_0_or_1(self, warn, want):
        assert format_row(_vals(warn=warn)).split(",")[16] == want

    @pytest.mark.parametrize("n", [0, 16, 18])
    def test_wrong_length_raises(self, n):
        with pytest.raises(ValueError, match="17"):
            format_row([0.0] * n)


class TestParseTelemetryEnv:
    @pytest.mark.parametrize(
        "raw", ["1", "true", "TRUE", "yes", "YES", " On ", "on"]
    )
    def test_on_values(self, raw):
        cfg = parse_telemetry_env({"SWD_TELEMETRY": raw})
        assert cfg.enabled is True
        assert cfg.warnings == ()

    @pytest.mark.parametrize("raw", ["", "0", "false", "no", "off", " OFF "])
    def test_off_values(self, raw):
        cfg = parse_telemetry_env({"SWD_TELEMETRY": raw})
        assert cfg.enabled is False
        assert cfg.warnings == ()

    @pytest.mark.parametrize("raw", ["junk", "2", "enable"])
    def test_junk_is_off_with_one_warning(self, raw):
        cfg = parse_telemetry_env({"SWD_TELEMETRY": raw})
        assert cfg.enabled is False
        assert len(cfg.warnings) == 1
        assert "SWD_TELEMETRY" in cfg.warnings[0]
        assert repr(raw) in cfg.warnings[0]

    def test_unset_is_off_without_warning(self):
        cfg = parse_telemetry_env({})
        assert cfg.enabled is False
        assert cfg.path is None
        assert cfg.warnings == ()

    def test_path_alone_does_not_enable(self):
        cfg = parse_telemetry_env({"SWD_TELEMETRY_FILE": "out.csv"})
        assert cfg.enabled is False
        assert cfg.path == "out.csv"
        assert cfg.warnings == ()

    def test_enabled_with_path(self):
        env = {"SWD_TELEMETRY": "1", "SWD_TELEMETRY_FILE": "run.csv"}
        cfg = parse_telemetry_env(env)
        assert cfg.enabled is True
        assert cfg.path == "run.csv"

    def test_empty_path_is_none(self):
        cfg = parse_telemetry_env({"SWD_TELEMETRY_FILE": ""})
        assert cfg.path is None

    def test_whitespace_path_is_none(self):
        cfg = parse_telemetry_env({"SWD_TELEMETRY_FILE": "   "})
        assert cfg.path is None

    def test_path_is_stripped(self):
        cfg = parse_telemetry_env({"SWD_TELEMETRY_FILE": " out.csv "})
        assert cfg.path == "out.csv"

    def test_junk_still_reports_path(self):
        env = {"SWD_TELEMETRY": "junk", "SWD_TELEMETRY_FILE": "x.csv"}
        cfg = parse_telemetry_env(env)
        assert cfg.enabled is False
        assert cfg.path == "x.csv"
        assert len(cfg.warnings) == 1

    def test_result_is_frozen(self):
        cfg = parse_telemetry_env({})
        with pytest.raises(dataclasses.FrozenInstanceError):
            cfg.enabled = True


class TestTelemetryWriter:
    def test_header_written_in_constructor(self):
        buf = io.StringIO()
        writer = TelemetryWriter(buf, owns_file=False)
        assert buf.getvalue() == HEADER
        assert writer.rows == 0
        assert writer.errors == 0
        assert writer.failed is False

    def test_header_once(self):
        buf = io.StringIO()
        writer = TelemetryWriter(buf, owns_file=False)
        for _ in range(3):
            writer.write_row(_vals())
        text = buf.getvalue()
        assert text.count(HEADER) == 1
        assert len(text.splitlines()) == 4

    def test_rows_count_and_content(self):
        buf = io.StringIO()
        writer = TelemetryWriter(buf, owns_file=False)
        for _ in range(3):
            writer.write_row(_vals())
        assert writer.rows == 3
        lines = buf.getvalue().splitlines()
        assert lines[0] == ",".join(COLUMNS)
        assert lines[1] == format_row(_vals())

    def test_prefix_on_every_line(self):
        buf = io.StringIO()
        writer = TelemetryWriter(buf, prefix="telemetry: ", owns_file=False)
        writer.write_row(_vals())
        writer.write_row(_vals(warn=1))
        lines = buf.getvalue().splitlines()
        assert len(lines) == 3
        assert all(line.startswith("telemetry: ") for line in lines)
        assert lines[0] == "telemetry: " + ",".join(COLUMNS)
        assert lines[1] == "telemetry: " + format_row(_vals())

    def test_default_has_no_prefix(self):
        buf = io.StringIO()
        TelemetryWriter(buf, owns_file=False)
        assert buf.getvalue().startswith("t_s,msgs,")

    def test_flush_cadence(self):
        fake = FakeFile()
        writer = TelemetryWriter(fake, flush_every=3)
        for _ in range(7):
            writer.write_row(_vals())
        assert fake.flush_count == 2
        writer.close()
        assert fake.flush_count == 3

    def test_default_flush_every_is_62(self):
        fake = FakeFile()
        writer = TelemetryWriter(fake)
        for _ in range(61):
            writer.write_row(_vals())
        assert fake.flush_count == 0
        writer.write_row(_vals())
        assert fake.flush_count == 1

    def test_warn_row_forces_flush_and_resets_count(self):
        fake = FakeFile()
        writer = TelemetryWriter(fake, flush_every=100)
        writer.write_row(_vals())
        assert fake.flush_count == 0
        writer.write_row(_vals(warn=1))
        assert fake.flush_count == 1
        for _ in range(99):
            writer.write_row(_vals())
        assert fake.flush_count == 1
        writer.write_row(_vals())
        assert fake.flush_count == 2

    def test_flush_every_one_flushes_each_row(self):
        fake = FakeFile()
        writer = TelemetryWriter(fake, flush_every=1)
        for _ in range(4):
            writer.write_row(_vals())
        assert fake.flush_count == 4

    def test_write_error_counted_then_rows_ignored(self):
        fake = FakeFile()
        writer = TelemetryWriter(fake)
        fake.fail_write = True
        writer.write_row(_vals())
        assert writer.errors == 1
        assert writer.failed is True
        assert writer.rows == 0
        fake.fail_write = False
        writer.write_row(_vals())
        assert writer.errors == 1
        assert writer.rows == 0
        assert len(fake.chunks) == 1

    def test_header_write_error_is_soft(self):
        fake = FakeFile(fail_write=True)
        writer = TelemetryWriter(fake)
        assert writer.errors == 1
        assert writer.failed is True
        fake.fail_write = False
        writer.write_row(_vals())
        assert writer.rows == 0
        assert fake.chunks == []

    def test_flush_error_counted_then_rows_ignored(self):
        fake = FakeFile(fail_flush=True)
        writer = TelemetryWriter(fake, flush_every=1)
        writer.write_row(_vals())
        assert writer.rows == 1
        assert writer.errors == 1
        assert writer.failed is True
        writer.write_row(_vals())
        assert writer.rows == 1
        assert writer.errors == 1
        assert fake.flush_count == 1
        assert len(fake.chunks) == 2

    def test_write_to_externally_closed_file_is_soft(self):
        buf = io.StringIO()
        writer = TelemetryWriter(buf)
        buf.close()
        writer.write_row(_vals())
        assert writer.errors == 1
        assert writer.failed is True
        writer.close()

    def test_bad_length_row_counted_but_not_fatal(self):
        buf = io.StringIO()
        writer = TelemetryWriter(buf, owns_file=False)
        writer.write_row([1.0])
        assert writer.errors == 1
        assert writer.failed is False
        assert writer.rows == 0
        writer.write_row(_vals())
        assert writer.rows == 1
        assert len(buf.getvalue().splitlines()) == 2

    def test_non_sequence_row_counted_but_not_fatal(self):
        buf = io.StringIO()
        writer = TelemetryWriter(buf, owns_file=False)
        writer.write_row(None)
        assert writer.errors == 1
        assert writer.failed is False

    def test_close_is_idempotent(self):
        fake = FakeFile()
        writer = TelemetryWriter(fake)
        assert writer.closed is False
        writer.close()
        writer.close()
        assert writer.closed is True
        assert fake.flush_count == 1
        assert fake.close_count == 1

    def test_close_does_not_close_unowned_file(self):
        buf = io.StringIO()
        writer = TelemetryWriter(buf, owns_file=False)
        writer.write_row(_vals())
        writer.close()
        assert buf.closed is False
        assert buf.getvalue().startswith(HEADER)

    def test_close_closes_owned_file(self):
        fake = FakeFile()
        writer = TelemetryWriter(fake, owns_file=True)
        writer.close()
        assert fake.closed is True
        buf = io.StringIO()
        TelemetryWriter(buf).close()
        assert buf.closed is True

    def test_close_with_failing_flush_does_not_raise(self):
        fake = FakeFile(fail_flush=True)
        writer = TelemetryWriter(fake)
        writer.close()
        assert writer.errors == 1
        assert writer.failed is True
        assert fake.closed is True

    def test_close_with_failing_close_does_not_raise(self):
        fake = FakeFile(fail_close=True)
        writer = TelemetryWriter(fake)
        writer.close()
        assert writer.errors == 1
        assert writer.failed is False
        assert fake.closed is True

    def test_close_after_failure_skips_flush_but_closes(self):
        fake = FakeFile(fail_write=True)
        writer = TelemetryWriter(fake)
        writer.close()
        assert fake.flush_count == 0
        assert fake.closed is True
        assert writer.errors == 1

    def test_header_not_rewritten_after_close(self):
        buf = io.StringIO()
        writer = TelemetryWriter(buf, owns_file=False)
        writer.write_row(_vals())
        snapshot = buf.getvalue()
        writer.close()
        writer.close()
        writer.write_row(_vals())
        assert buf.getvalue() == snapshot
        assert snapshot.count(HEADER) == 1

    def test_write_after_close_is_ignored(self):
        fake = FakeFile()
        writer = TelemetryWriter(fake)
        writer.close()
        writer.write_row(_vals())
        assert writer.rows == 0
        assert writer.errors == 0
        assert writer.failed is False
        assert len(fake.chunks) == 1

    @pytest.mark.parametrize("bad", [0, -1])
    def test_flush_every_below_one_raises(self, bad):
        buf = io.StringIO()
        with pytest.raises(ValueError, match="flush_every"):
            TelemetryWriter(buf, flush_every=bad)
        assert buf.getvalue() == ""
