#!/usr/bin/env python3
"""
Tests for monitor/telemetry_summary.py (issue #307, sub-issue of #303).

Tiny inline CSVs are built with the real telemetry.COLUMNS / format_row, so the
`nan` text and the `telemetry: ` prefix match what the controller writes. No
Webots, no hardware; must NOT import `controller`, numpy or `serial`.

Usage:
    cd poc/servo-wheel-dof
    mamba run -n servo-wheel-dof python3 -m pytest -q monitor
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

import pytest

_MONITOR_DIR = Path(__file__).resolve().parent
_CONTROLLER_DIR = (
    _MONITOR_DIR.parent / "webots" / "controllers" / "servo_wheel_dof"
)
sys.path.insert(0, str(_MONITOR_DIR))
sys.path.insert(0, str(_CONTROLLER_DIR))

import telemetry_summary as ts  # noqa: E402
from telemetry import COLUMNS, format_row  # noqa: E402

SCRIPT = _MONITOR_DIR / "telemetry_summary.py"
HEADER = ",".join(COLUMNS)
NAN = float("nan")


def _row(t, **kw):
    """One real-format telemetry row (healthy defaults) as CSV text."""
    values = {"t_s": t, "msgs": 1, "axis_dot": 1.0, "warn": 0}
    values.update(kw)
    return format_row([values.get(c, 0.0) for c in COLUMNS])


def _csv(rows, prefix=""):
    return "\n".join(prefix + ln for ln in [HEADER, *rows]) + "\n"


def _run(tmp_path, capsys, text, *args, name="run.csv"):
    path = tmp_path / name
    path.write_bytes(text if isinstance(text, bytes) else text.encode())
    code = ts.main([str(path), *args])
    cap = capsys.readouterr()
    return code, cap.out, cap.err


def _kv(out):
    result = {}
    for line in out.splitlines():
        key, sep, val = line.partition(":")
        if sep:
            result[key.strip()] = val.strip()
    return result


def _healthy_rows():
    return [
        _row(0.0, msgs=1, wheel_err=0.1, anchor_dev_m=0.0001),
        _row(0.25, msgs=2, wheel_err=-0.3, anchor_dev_m=0.0005,
             axis_dot=0.99995),
        _row(0.5, msgs=3, wheel_err=0.2, anchor_dev_m=0.0002),
        _row(0.75, msgs=4, wheel_err=0.0, anchor_dev_m=0.0003),
        _row(1.0, msgs=5, wheel_err=0.1, anchor_dev_m=0.0004),
    ]


def _warn_rows():
    rows = [_row(0.5 * i, anchor_dev_m=0.0001) for i in range(5)]
    rows[2] = _row(1.0, anchor_dev_m=0.002, axis_dot=0.999, warn=1)
    return rows


def test_healthy_run(tmp_path, capsys):
    code, out, err = _run(tmp_path, capsys, _csv(_healthy_rows()))
    kv = _kv(out)
    assert code == 0 and err == ""
    assert kv["rows"] == "5"
    assert kv["skipped"] == "0"
    assert kv["t_start_s"] == "0.000000"
    assert kv["t_end_s"] == "1.000000"
    assert kv["duration_s"] == "1.000000"
    assert kv["msgs_last"] == "5"
    assert kv["wheel_err_max_abs"] == "0.300000"
    assert kv["wheel_err_mean_abs"] == "0.140000"
    assert kv["anchor_dev_m_max"] == "0.000500"
    assert kv["axis_dot_min"] == "0.999950"
    assert kv["warn_rows"] == "0"
    assert kv["first_warn_t_s"] == "n/a"
    assert "window_rows" not in kv


def test_one_warn_row_and_window(tmp_path, capsys):
    code, out, _ = _run(tmp_path, capsys, _csv(_warn_rows()))
    kv = _kv(out)
    lines = out.splitlines()
    assert code == 0
    assert kv["warn_rows"] == "1"
    assert kv["first_warn_t_s"] == "1.000000"
    assert kv["anchor_dev_m_max"] == "0.002000"
    assert kv["axis_dot_min"] == "0.999000"
    assert kv["window_s"] == "0.500000"
    assert kv["window_rows"] == "3"  # default window 0.5: t = 0.5, 1.0, 1.5
    assert lines[-4] == HEADER
    assert lines[-3].startswith("0.500000,")
    assert lines[-2].startswith("1.000000,")
    assert lines[-1].startswith("1.500000,")
    assert lines[-2].endswith(",1")


@pytest.mark.parametrize(
    "window, expected", [("0.1", "1"), ("0", "1"), ("1", "5"), ("0.49", "1")]
)
def test_window_option(tmp_path, capsys, window, expected):
    _, out, _ = _run(tmp_path, capsys, _csv(_warn_rows()), "--window", window)
    assert _kv(out)["window_rows"] == expected


def test_window_edge_survives_float_rounding(tmp_path, capsys):
    # 1.0 - 0.7 is 0.30000000000000004 in floating point, still "0.3 away"
    rows = [_row(0.7), _row(1.0, warn=1), _row(1.3)]
    _, out, _ = _run(tmp_path, capsys, _csv(rows), "--window", "0.3")
    assert _kv(out)["window_rows"] == "3"


def test_only_warn_equal_to_one_counts(tmp_path, capsys):
    rows = [_row(0.0), _row(0.5), _row(1.0)]
    cells = [r.split(",") for r in rows]
    cells[1][COLUMNS.index("warn")] = "2"
    cells[2][COLUMNS.index("warn")] = "0.5"
    text = _csv([",".join(c) for c in cells])
    _, out, _ = _run(tmp_path, capsys, text)
    kv = _kv(out)
    assert kv["warn_rows"] == "0"
    assert kv["first_warn_t_s"] == "n/a"


def test_msgs_last_is_the_last_row_not_the_maximum(tmp_path, capsys):
    rows = [_row(0.0, msgs=9), _row(0.5, msgs=3)]
    _, out, _ = _run(tmp_path, capsys, _csv(rows))
    assert _kv(out)["msgs_last"] == "3"


def test_first_warn_is_first_in_file_order(tmp_path, capsys):
    rows = _warn_rows()
    rows[3] = _row(1.5, axis_dot=0.5, warn=1)
    _, out, _ = _run(tmp_path, capsys, _csv(rows))
    kv = _kv(out)
    assert kv["warn_rows"] == "2"
    assert kv["first_warn_t_s"] == "1.000000"
    assert kv["axis_dot_min"] == "0.500000"


def test_prefixed_stdout_capture_ignores_log_noise(tmp_path, capsys):
    plain = _run(tmp_path, capsys, _csv(_warn_rows()))[1]
    lines = _csv(_warn_rows(), prefix="telemetry: ").splitlines()
    noisy = "\n".join(["INFO: controller boot", *lines, "INFO: done"]) + "\n"
    _, out, _ = _run(tmp_path, capsys, noisy)
    assert out.splitlines()[1:] == plain.splitlines()[1:]  # all but "file"
    assert "INFO" not in out


def test_strip_prefix_unit():
    assert ts.strip_prefix(["telemetry: a,b\n", "x\n", "telemetry:1,2\n"]) == [
        "a,b",
        "1,2",
    ]
    assert ts.strip_prefix(["a,b\r\n", "\n", "1,2\r\n"]) == ["a,b", "1,2"]
    assert ts.strip_prefix([]) == []


def test_nan_cells_are_ignored(tmp_path, capsys):
    rows = [
        _row(0.0, msgs=7, wheel_err=0.1, anchor_dev_m=0.0001),
        _row(0.5, msgs=NAN, wheel_err=NAN, anchor_dev_m=NAN, axis_dot=NAN,
             warn=1),
    ]
    assert ",nan," in rows[1]
    _, out, _ = _run(tmp_path, capsys, _csv(rows))
    kv = _kv(out)
    assert kv["msgs_last"] == "7"
    assert kv["wheel_err_max_abs"] == "0.100000"
    assert kv["anchor_dev_m_max"] == "0.000100"
    assert kv["axis_dot_min"] == "1.000000"
    assert kv["warn_rows"] == "1"
    assert kv["first_warn_t_s"] == "0.500000"


def test_all_nan_prints_na(tmp_path, capsys):
    row = format_row([NAN] * (len(COLUMNS) - 1) + [0])
    code, out, _ = _run(tmp_path, capsys, _csv([row]))
    kv = _kv(out)
    assert code == 0 and kv["rows"] == "1"
    for key in ("t_start_s", "t_end_s", "duration_s", "msgs_last",
                "wheel_err_max_abs", "wheel_err_mean_abs",
                "anchor_dev_m_max", "axis_dot_min"):
        assert kv[key] == "n/a", key


def test_nan_time_rows_do_not_break_span_or_window(tmp_path, capsys):
    rows = [_row(0.0), _row(NAN), _row(1.0, warn=1), _row(NAN)]
    _, out, _ = _run(tmp_path, capsys, _csv(rows), "--window", "5")
    kv = _kv(out)
    assert kv["rows"] == "4"
    assert kv["duration_s"] == "1.000000"
    assert kv["window_rows"] == "2"  # the nan-time rows are excluded


def test_warn_row_with_nan_time_has_no_window(tmp_path, capsys):
    _, out, _ = _run(tmp_path, capsys, _csv([_row(0.0), _row(NAN, warn=1)]))
    kv = _kv(out)
    assert kv["first_warn_t_s"] == "nan"
    assert kv["window_rows"] == "0"


def test_non_numeric_and_inf_cells_become_nan(tmp_path, capsys):
    good = _row(0.0)
    cells = good.split(",")
    cells[COLUMNS.index("axis_dot")] = "abc"
    cells[COLUMNS.index("anchor_dev_m")] = "inf"
    text = HEADER + "\n" + ",".join(cells) + "\n" + _row(1.0) + "\n"
    code, out, _ = _run(tmp_path, capsys, text)
    kv = _kv(out)
    assert code == 0 and kv["rows"] == "2"
    assert kv["axis_dot_min"] == "1.000000"
    assert kv["anchor_dev_m_max"] == "0.000000"


def test_header_only_file(tmp_path, capsys):
    code, out, err = _run(tmp_path, capsys, HEADER + "\n")
    kv = _kv(out)
    assert code == 0 and err == ""
    assert kv["rows"] == "0"
    assert kv["duration_s"] == "n/a"
    assert kv["warn_rows"] == "0"
    assert kv["first_warn_t_s"] == "n/a"


def test_empty_file_exits_2(tmp_path, capsys):
    for text in ("", "\n\n   \n"):
        code, out, err = _run(tmp_path, capsys, text)
        assert code == 2 and out == ""
        assert "empty" in err


def test_log_without_telemetry_lines_is_a_bad_header(tmp_path, capsys):
    code, _, err = _run(tmp_path, capsys, "INFO: boot\nINFO: done\n")
    assert code == 2
    assert "missing columns" in err


def test_ragged_rows_are_skipped_and_counted(tmp_path, capsys):
    rows = [_row(0.0), "1,2,3", ",".join(["0"] * (len(COLUMNS) + 1)), _row(1.0)]
    code, out, _ = _run(tmp_path, capsys, _csv(rows))
    kv = _kv(out)
    assert code == 0
    assert kv["rows"] == "2"
    assert kv["skipped"] == "2"
    assert kv["duration_s"] == "1.000000"


def test_missing_file_exits_2(tmp_path, capsys):
    code = ts.main([str(tmp_path / "nope.csv")])
    cap = capsys.readouterr()
    assert code == 2 and cap.out == ""
    assert cap.err.startswith("error:") and "nope.csv" in cap.err


def test_directory_path_exits_2(tmp_path, capsys):
    assert ts.main([str(tmp_path)]) == 2
    assert capsys.readouterr().err.startswith("error:")


def test_missing_column_exits_2_and_names_it(tmp_path, capsys):
    names = [c for c in COLUMNS if c != "axis_dot"]
    row = ",".join(["0"] * len(names))
    code, out, err = _run(tmp_path, capsys, ",".join(names) + "\n" + row + "\n")
    assert code == 2 and out == ""
    assert "axis_dot" in err


def test_unused_columns_may_be_absent(tmp_path, capsys):
    keep = ("t_s", "msgs", "wheel_err", "anchor_dev_m", "axis_dot", "warn")
    text = ",".join(keep) + "\n0.0,1,0.1,0.0,1.0,0\n"
    code, out, _ = _run(tmp_path, capsys, text)
    assert code == 0 and _kv(out)["rows"] == "1"


def test_columns_are_read_by_name_not_position(tmp_path, capsys):
    rows = _warn_rows()
    forward = _run(tmp_path, capsys, _csv(rows))[1].splitlines()
    rev_header = ",".join(reversed(COLUMNS))
    rev_rows = [",".join(reversed(r.split(","))) for r in rows]
    _, out, _ = _run(tmp_path, capsys, rev_header + "\n" + "\n".join(rev_rows))
    lines = out.splitlines()
    assert lines[1:13] == forward[1:13]  # same statistics, whatever the order
    assert _kv(out)["window_rows"] == "3"


def test_bom_crlf_and_blank_lines(tmp_path, capsys):
    text = "﻿" + _csv(_healthy_rows()).replace("\n", "\r\n\r\n")
    code, out, _ = _run(tmp_path, capsys, text)
    assert code == 0
    assert _kv(out)["rows"] == "5"


def test_invalid_utf8_header_is_a_clean_exit_2(tmp_path, capsys):
    prefixed = _csv(_healthy_rows(), prefix="telemetry: ").encode()
    data = b"telemetry: \xff junk\n" + prefixed
    code, _, err = _run(tmp_path, capsys, data)
    assert code == 2
    assert "missing columns" in err


def test_invalid_utf8_in_data_row_is_skipped(tmp_path, capsys):
    prefixed = _csv(_healthy_rows(), prefix="telemetry: ").encode()
    data = prefixed + b"telemetry: \xff junk\n"
    code, out, _ = _run(tmp_path, capsys, data)
    kv = _kv(out)
    assert code == 0
    assert kv["rows"] == "5"
    assert kv["skipped"] == "1"


@pytest.mark.parametrize("bad", ["-1", "nan", "inf", "abc"])
def test_bad_window_is_an_argparse_error(tmp_path, capsys, bad):
    path = tmp_path / "run.csv"
    path.write_text(_csv(_healthy_rows()))
    with pytest.raises(SystemExit) as exc:
        ts.main([str(path), "--window", bad])
    assert exc.value.code == 2
    assert "--window" in capsys.readouterr().err


def test_missing_csv_argument_is_an_argparse_error(capsys):
    with pytest.raises(SystemExit) as exc:
        ts.main([])
    assert exc.value.code == 2


def test_script_exit_codes_via_subprocess(tmp_path):
    bad = subprocess.run(
        [sys.executable, str(SCRIPT), str(tmp_path / "nope.csv")],
        capture_output=True, text=True, timeout=30,
    )
    assert bad.returncode == 2 and "error:" in bad.stderr
    good = tmp_path / "ok.csv"
    good.write_text(_csv(_warn_rows(), prefix="telemetry: "))
    ok = subprocess.run(
        [sys.executable, str(SCRIPT), str(good), "--window", "0"],
        capture_output=True, text=True, timeout=30,
    )
    assert ok.returncode == 0
    assert "first_warn_t_s    : 1.000000" in ok.stdout


def test_module_has_no_forbidden_imports():
    text = SCRIPT.read_text(encoding="utf-8")
    forbidden = re.compile(
        r"^\s*(import|from)\s+(numpy|serial|controller|telemetry)\b", re.M
    )
    assert forbidden.search(text) is None


@pytest.mark.parametrize("path", [SCRIPT, Path(__file__)])
def test_lines_are_at_most_88_columns(path):
    long = [
        n
        for n, ln in enumerate(path.read_text(encoding="utf-8").splitlines(), 1)
        if len(ln) > 88
    ]
    assert long == []
