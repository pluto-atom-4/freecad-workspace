#!/usr/bin/env python3
"""
Telemetry row format, env parsing and fail-soft writer (issue #305, sub of #303).

Pure Python: only the standard library; no Webots, serial or numpy dependency
(must NOT `import controller`). The controller wiring comes later (#306).

Conventions:
- COLUMNS is the CSV header and the exact order of the values given to
  format_row (17 values; the last one is warn).
- Float columns are written as %.6f. None, non-numeric or non-finite values
  become the text nan.
- msgs (valid datagram count) is an integer: written with int() (floats are
  truncated); None, nan or inf becomes nan.
- warn is written as 1 when bool(value) is true, else 0.
- parse_telemetry_env reads SWD_TELEMETRY (on: 1,true,yes,on; off: empty,
  0,false,no,off; case-insensitive, stripped; anything else is off plus one
  warning) and SWD_TELEMETRY_FILE (path, stripped, empty means None). The file
  variable alone never enables telemetry and adds no warning.
- TelemetryWriter writes the header ONCE, in the constructor (a failing write
  counts as an error). write_row never raises. A failed write or flush counts
  one error, sets failed and then every later row is ignored. A malformed row
  (wrong length, not a sequence) counts one error and is dropped, failed stays
  False. flush_every is the number of rows written since the last flush
  (a warn row also flushes and resets the count); values below 1 raise
  ValueError. close() is idempotent, flushes unless failed, closes the file
  only when owns_file is true (never pass sys.stdout as owned) and never
  raises.

Usage:
    from telemetry import TelemetryWriter, format_row, parse_telemetry_env

    cfg = parse_telemetry_env(os.environ)
    if cfg.enabled:
        writer = TelemetryWriter(sys.stdout, prefix="telemetry: ",
                                 owns_file=False)
        writer.write_row(values)  # 17 values in COLUMNS order
        writer.close()
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

COLUMNS = (
    "t_s",
    "msgs",
    "age_s",
    "wheel_cmd",
    "wheel_sensor",
    "wheel_err",
    "vel_rad_s",
    "robot_x",
    "robot_y",
    "robot_z",
    "wheel_z",
    "rel_x",
    "rel_y",
    "rel_z",
    "anchor_dev_m",
    "axis_dot",
    "warn",
)

DEFAULT_FLUSH_EVERY = 62
ENV_ENABLE = "SWD_TELEMETRY"
ENV_FILE = "SWD_TELEMETRY_FILE"

_ON = frozenset(("1", "true", "yes", "on"))
_OFF = frozenset(("", "0", "false", "no", "off"))
_MSGS_INDEX = COLUMNS.index("msgs")
_WARN_INDEX = len(COLUMNS) - 1


def _fmt_float(value: Any) -> str:
    """Return %.6f, or nan for None, non-numeric and non-finite values."""
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return "nan"
    if not math.isfinite(number):
        return "nan"
    return "%.6f" % number


def _fmt_int(value: Any) -> str:
    """Return the integer text, or nan for None, nan and inf."""
    try:
        return str(int(value))
    except (TypeError, ValueError, OverflowError):
        return "nan"


def format_row(values: Sequence[Any]) -> str:
    """
    Format one telemetry row (no newline, no prefix).

    Args:
        values: len(COLUMNS) values in COLUMNS order.

    Returns:
        Comma-joined text: msgs as an integer, warn as 0/1, every other column
        as %.6f or nan.

    Raises:
        ValueError: values does not have len(COLUMNS) items.
    """
    if len(values) != len(COLUMNS):
        raise ValueError(
            f"row must have {len(COLUMNS)} values, got {len(values)}"
        )
    out = []
    for i, value in enumerate(values):
        if i == _MSGS_INDEX:
            out.append(_fmt_int(value))
        elif i == _WARN_INDEX:
            out.append("1" if value else "0")
        else:
            out.append(_fmt_float(value))
    return ",".join(out)


@dataclass(frozen=True)
class TelemetryConfig:
    """Result of parse_telemetry_env."""

    enabled: bool
    path: str | None
    warnings: tuple[str, ...]


def parse_telemetry_env(environ: Mapping[str, str]) -> TelemetryConfig:
    """
    Read SWD_TELEMETRY and SWD_TELEMETRY_FILE from a mapping.

    Never raises. A junk SWD_TELEMETRY value means disabled plus one warning.
    The path is reported even when telemetry is disabled.
    """
    warnings: list[str] = []
    raw = environ.get(ENV_ENABLE)
    text = "" if raw is None else str(raw).strip().lower()
    enabled = False
    if text in _ON:
        enabled = True
    elif text not in _OFF:
        warnings.append(
            f"{ENV_ENABLE}={raw!r} is not one of 1/true/yes/on or "
            "0/false/no/off; telemetry stays off"
        )

    path_raw = environ.get(ENV_FILE)
    path = None
    if path_raw is not None:
        path = str(path_raw).strip() or None
    return TelemetryConfig(enabled, path, tuple(warnings))


class TelemetryWriter:
    """Fail-soft CSV writer shared by the file and stdout modes."""

    def __init__(
        self,
        fileobj: Any,
        flush_every: int = DEFAULT_FLUSH_EVERY,
        prefix: str = "",
        owns_file: bool = True,
    ) -> None:
        """
        Args:
            fileobj: Object with write() and flush() (close() if owned).
            flush_every: Flush after this many rows since the last flush.
            prefix: Text put before the header and before every row.
            owns_file: close() closes fileobj only when true.

        Raises:
            ValueError: flush_every is below 1 (nothing is written).
        """
        if flush_every < 1:
            raise ValueError(f"flush_every must be >= 1, got {flush_every}")
        self._file = fileobj
        self._flush_every = flush_every
        self._prefix = prefix
        self._owns = owns_file
        self._rows = 0
        self._errors = 0
        self._failed = False
        self._closed = False
        self._since = 0
        self._write(prefix + ",".join(COLUMNS) + "\n")

    @property
    def rows(self) -> int:
        """Rows written successfully (the header is not counted)."""
        return self._rows

    @property
    def errors(self) -> int:
        """Errors counted so far (write, flush, bad row, close)."""
        return self._errors

    @property
    def failed(self) -> bool:
        """True after a write or flush error; later rows are ignored."""
        return self._failed

    @property
    def closed(self) -> bool:
        """True after close()."""
        return self._closed

    def _fail(self) -> None:
        self._errors += 1
        self._failed = True

    def _write(self, text: str) -> bool:
        try:
            self._file.write(text)
        except (OSError, ValueError):
            self._fail()
            return False
        return True

    def _flush(self) -> None:
        self._since = 0
        try:
            self._file.flush()
        except (OSError, ValueError):
            self._fail()

    def write_row(self, values: Sequence[Any]) -> None:
        """Write one row; never raises. Ignored once failed or closed."""
        if self._failed or self._closed:
            return
        try:
            line = format_row(values)
        except (TypeError, ValueError):
            self._errors += 1
            return
        if not self._write(self._prefix + line + "\n"):
            return
        self._rows += 1
        self._since += 1
        if line.endswith(",1") or self._since >= self._flush_every:
            self._flush()

    def close(self) -> None:
        """Flush (unless failed) and close an owned file; idempotent."""
        if self._closed:
            return
        self._closed = True
        if not self._failed:
            self._flush()
        if self._owns:
            try:
                self._file.close()
            except (OSError, ValueError):
                self._errors += 1
