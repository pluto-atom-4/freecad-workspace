#!/usr/bin/env python3
"""
Offline summary of a servo-wheel-dof telemetry CSV (issue #307, sub of #303).

Reads a CSV written by the controller telemetry writer (columns in
webots/controllers/servo_wheel_dof/telemetry.py COLUMNS) and prints when the
wheel stopped being aligned. Stdlib only; reads columns by header name, so
column order and extra columns do not matter. Must NOT import numpy, serial
or the Webots controller module.

Input handling:
- If any line starts with "telemetry:" (stdout capture), only those lines are
  used and the prefix is stripped; other log lines are ignored. Otherwise
  every non-blank line is used. The first used line is the header.
- A row whose cell count differs from the header is skipped and counted.
- A cell that is not a number, or is nan/inf, counts as nan; every statistic
  ignores nan values. A BOM and CRLF line ends are tolerated.

Printed (key : value lines): file, rows, skipped, t_start_s, t_end_s, duration_s,
msgs_last (last finite msgs), wheel_err_max_abs and wheel_err_mean_abs
(absolute values), anchor_dev_m_max, axis_dot_min, warn_rows, first_warn_t_s
(first row in file order with warn == 1). When a warn row exists the rows
whose t_s is within +-window of first_warn_t_s follow the window_s and
window_rows lines, as CSV (header first). A statistic with no finite value prints n/a.

Exit codes: 0 summary printed (also for a header-only file); 2 bad arguments,
unreadable file, empty file (no header) or a missing required column.

Usage:
    python3 monitor/telemetry_summary.py run.csv [--window 0.5]
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from typing import Iterable

PREFIX = "telemetry:"
DEFAULT_WINDOW_S = 0.5
REQUIRED = ("t_s", "msgs", "wheel_err", "anchor_dev_m", "axis_dot", "warn")
NA = "n/a"
_LABEL_WIDTH = 18
_EDGE_EPS = 1e-9  # slack so a row exactly +-window away is included


@dataclass(frozen=True)
class Row:
    """One data row: the parsed columns used here plus the raw CSV text."""

    t_s: float
    msgs: float
    wheel_err: float
    anchor_dev_m: float
    axis_dot: float
    warn: float
    raw: str


@dataclass(frozen=True)
class Parsed:
    """Result of parse_telemetry."""

    names: tuple[str, ...]
    rows: tuple[Row, ...]
    skipped: int


@dataclass(frozen=True)
class Summary:
    """Result of summarize; None means no finite value (printed as n/a)."""

    names: tuple[str, ...]
    rows: int
    skipped: int
    t_start: float | None
    t_end: float | None
    duration: float | None
    msgs_last: float | None
    err_max_abs: float | None
    err_mean_abs: float | None
    dev_max: float | None
    dot_min: float | None
    warn_rows: int
    first_warn_t: float | None  # None: no warn row; may be nan
    window_s: float
    window_rows: tuple[Row, ...]


def strip_prefix(lines: Iterable[str]) -> list[str]:
    """
    Return the CSV lines to parse.

    If any line starts with PREFIX only those lines are kept, with the prefix
    removed; otherwise all lines are kept. Blank lines are dropped, trailing
    CR/LF and a BOM are removed.
    """
    raw = [ln.rstrip("\r\n").lstrip("﻿") for ln in lines]
    prefixed = [ln for ln in raw if ln.startswith(PREFIX)]
    if prefixed:
        raw = [ln[len(PREFIX):].lstrip() for ln in prefixed]
    return [ln for ln in raw if ln.strip()]


def _num(text: str) -> float:
    """Parse one cell; anything that is not a finite number becomes nan."""
    try:
        value = float(text)
    except ValueError:
        return math.nan
    return value if math.isfinite(value) else math.nan


def parse_telemetry(lines: Iterable[str], source: str = "<csv>") -> Parsed:
    """
    Parse telemetry CSV lines (pure: no file access).

    Raises:
        ValueError: no header line, or a REQUIRED column is missing. The
        message starts with source.
    """
    data = strip_prefix(lines)
    if not data:
        raise ValueError(f"{source}: empty file (no header)")
    names = tuple(n.strip() for n in next(csv.reader([data[0]])))
    missing = [c for c in REQUIRED if c not in names]
    if missing:
        raise ValueError(f"{source}: missing columns {missing}")
    idx = {}
    for i, name in enumerate(names):
        idx.setdefault(name, i)  # first duplicate wins
    rows: list[Row] = []
    skipped = 0
    for cells in csv.reader(data[1:]):
        if len(cells) != len(names):
            skipped += 1
            continue
        rows.append(
            Row(
                *(_num(cells[idx[c]]) for c in REQUIRED),
                raw=",".join(cells),
            )
        )
    return Parsed(names, tuple(rows), skipped)


def _finite(values: Iterable[float]) -> list[float]:
    return [v for v in values if math.isfinite(v)]


def summarize(parsed: Parsed, window_s: float = DEFAULT_WINDOW_S) -> Summary:
    """Compute the nan-aware statistics and the window around the first warn."""
    rows = parsed.rows
    times = _finite(r.t_s for r in rows)
    t_start = min(times) if times else None
    t_end = max(times) if times else None
    duration = t_end - t_start if times else None
    msgs = _finite(r.msgs for r in rows)
    errs = [abs(v) for v in _finite(r.wheel_err for r in rows)]
    devs = _finite(r.anchor_dev_m for r in rows)
    dots = _finite(r.axis_dot for r in rows)
    warns = [r for r in rows if r.warn == 1.0]
    first_warn_t = warns[0].t_s if warns else None
    window: tuple[Row, ...] = ()
    if first_warn_t is not None and math.isfinite(first_warn_t):
        window = tuple(
            r
            for r in rows
            if math.isfinite(r.t_s)
            and abs(r.t_s - first_warn_t) <= window_s + _EDGE_EPS
        )
    return Summary(
        names=parsed.names,
        rows=len(rows),
        skipped=parsed.skipped,
        t_start=t_start,
        t_end=t_end,
        duration=duration,
        msgs_last=msgs[-1] if msgs else None,
        err_max_abs=max(errs) if errs else None,
        err_mean_abs=math.fsum(errs) / len(errs) if errs else None,
        dev_max=max(devs) if devs else None,
        dot_min=min(dots) if dots else None,
        warn_rows=len(warns),
        first_warn_t=first_warn_t,
        window_s=window_s,
        window_rows=window,
    )


def _f(value: float | None) -> str:
    return NA if value is None else "%.6f" % value


def _line(key: str, value: object) -> str:
    return f"{key:<{_LABEL_WIDTH}}: {value}"


def render(summary: Summary, source: str) -> list[str]:
    """Return the output lines for a Summary."""
    msgs = NA if summary.msgs_last is None else str(int(summary.msgs_last))
    out = [
        _line("file", source),
        _line("rows", summary.rows),
        _line("skipped", summary.skipped),
        _line("t_start_s", _f(summary.t_start)),
        _line("t_end_s", _f(summary.t_end)),
        _line("duration_s", _f(summary.duration)),
        _line("msgs_last", msgs),
        _line("wheel_err_max_abs", _f(summary.err_max_abs)),
        _line("wheel_err_mean_abs", _f(summary.err_mean_abs)),
        _line("anchor_dev_m_max", _f(summary.dev_max)),
        _line("axis_dot_min", _f(summary.dot_min)),
        _line("warn_rows", summary.warn_rows),
        _line("first_warn_t_s", _f(summary.first_warn_t)),
    ]
    if summary.first_warn_t is not None:
        out.append(_line("window_s", "%.6f" % summary.window_s))
        out.append(_line("window_rows", len(summary.window_rows)))
        if summary.window_rows:
            out.append(",".join(summary.names))
            out.extend(r.raw for r in summary.window_rows)
    return out


def load_telemetry(path) -> Parsed:
    """Read a telemetry file (BOM-safe, undecodable bytes replaced) and parse."""
    with open(path, encoding="utf-8-sig", errors="replace") as fh:
        return parse_telemetry(fh, source=str(path))


def _window_type(text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not a number: {text!r}") from None
    if not math.isfinite(value) or value < 0:
        raise argparse.ArgumentTypeError(f"must be finite and >= 0: {text!r}")
    return value


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI parser."""
    parser = argparse.ArgumentParser(
        prog="telemetry_summary.py",
        description="Summarize a servo-wheel-dof telemetry CSV: row count, "
        "time span, worst alignment values and the rows around the first warn.",
    )
    parser.add_argument(
        "csv",
        metavar="CSV",
        help="telemetry CSV (a 'telemetry: ' prefix on lines is accepted)",
    )
    parser.add_argument(
        "--window",
        type=_window_type,
        default=DEFAULT_WINDOW_S,
        metavar="SECONDS",
        help="print rows within +-SECONDS of the first warn "
        "(default %(default)s)",
    )
    return parser


def main(argv=None) -> int:
    """CLI entry point; returns the exit code (argparse itself exits 2)."""
    args = build_parser().parse_args(argv)
    try:
        parsed = load_telemetry(args.csv)
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    for line in render(summarize(parsed, args.window), args.csv):
        print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
