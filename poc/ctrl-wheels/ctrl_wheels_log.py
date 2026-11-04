"""
CSV log writer for the ctrl-wheels proof of concept (issue FW #452).

SIM ONLY. Not hardware-validated. The sign convention is UNVERIFIED.
Latency UNMEASURED.

One row per host tick. Columns are fixed by COLUMNS and written in that order,
whatever the key order of the input mapping. A row is fully validated before
anything is written, so a rejected row leaves no partial line.

Cell rules:
- None or a missing key gives an empty cell.
- bool, NaN and inf are refused with ValueError.
- Float columns use "%.6f", except host_t_s and host_rtt_ms which use "%.3f".
- Integer columns are written as decimal integers.

host_rtt_ms: wall-clock milliseconds from sending the command with cmd_seq to
receiving the echo whose cmd_seq_echo equals that cmd_seq. It includes up to one
simulation step (16 ms in the world file and in fake_wheels, UNVERIFIED for Webots)
and one bridge poll, so it is NOT a pure link latency. Empty when no echo has
arrived.

echo_cmd_seq: the value of the echo JSON key cmd_seq_echo.
"""

import csv
import math
import os

COLUMNS = (
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

# Documentation data for humans. It is read only by test_ctrl_wheels_log.py, which pins its keys to COLUMNS,
# so adding a column needs a description here. It is not used at run time.
COLUMN_DOC = {
    "host_t_s": "host wall-clock seconds at the row, 3 decimals",
    "whl_seq": "wheel telemetry sequence number from the device",
    "whl_t_us": "wheel telemetry timestamp in microseconds, device clock",
    "whl_model_rad_s": "wheel model velocity in rad/s (SIM)",
    "whl_raw": "wheel raw velocity units as reported by the servo",
    "whl_state": "wheel state code as reported by the device",
    "cmd_seq": "sequence number of the command sent by the host",
    "cmd_rad_s": "commanded wheel velocity in rad/s",
    "stale": "stale flag, 0 or 1, as supplied by the caller",
    "echo_cmd_seq": "cmd_seq_echo from the echo JSON; empty when no echo",
    "sim_t_s": "Webots simulation time in seconds; empty when no echo",
    "angle_rad": "simulated frame angle in rad; empty when no echo",
    "vel_rad_s": "simulated frame velocity in rad/s; empty when no echo",
    "host_rtt_ms": "wall ms from sending cmd_seq to its echo (not link latency); empty when no echo",
    "missed_whl": "count of missed wheel telemetry frames as supplied by the caller",
}

_INT_COLUMNS = (
    "whl_seq",
    "whl_t_us",
    "whl_raw",
    "whl_state",
    "cmd_seq",
    "stale",
    "echo_cmd_seq",
    "missed_whl",
)

_MS3_COLUMNS = ("host_t_s", "host_rtt_ms")

_COLUMN_SET = frozenset(COLUMNS)


def _format_cell(name, value):
    """Return the text of one cell, or raise ValueError for a refused value."""
    if value is None:
        return ""
    if isinstance(value, bool):
        raise ValueError(f"{name}: bool is not allowed")
    if name in _INT_COLUMNS:
        if not isinstance(value, int):
            raise ValueError(f"{name}: expected int, got {type(value).__name__}")
        return str(value)
    if not isinstance(value, (int, float)):
        raise ValueError(f"{name}: expected float, got {type(value).__name__}")
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name}: value too large") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name}: non-finite value {number!r}")
    fmt = "%.3f" if name in _MS3_COLUMNS else "%.6f"
    return fmt % number


def _row_cells(row):
    """Validate the whole row and return its cells in COLUMNS order."""
    unknown = [key for key in row if key not in _COLUMN_SET]
    if unknown:
        raise ValueError(f"unknown column(s): {sorted(str(k) for k in unknown)}")
    return [_format_cell(name, row.get(name)) for name in COLUMNS]


class CsvLog:
    """CSV writer for ctrl-wheels rows. The header is written on construction.

    A str or os.PathLike target is opened and owned: it is closed by close().
    With overwrite=False an existing path raises FileExistsError.
    Any other target is a caller-owned file object: it is flushed, never closed.
    """

    def __init__(self, target, overwrite=False):
        self._closed = False
        if isinstance(target, (str, os.PathLike)):
            mode = "w" if overwrite else "x"
            self._fh = open(target, mode, newline="", encoding="utf-8")
            self._owned = True
        else:
            self._fh = target
            self._owned = False
        self._writer = csv.writer(self._fh, lineterminator="\n")
        try:
            self._writer.writerow(COLUMNS)
            self._fh.flush()  # header visible on disk at once; a crash keeps the rows so far
        except BaseException:
            self._release()
            raise

    def write_row(self, row):
        """Write one row. Raises ValueError (and writes nothing) if the row is refused."""
        if self._closed:
            raise ValueError("write_row on a closed CsvLog")
        cells = _row_cells(row)
        self._writer.writerow(cells)
        self._fh.flush()  # a live relay log: keep rows on disk if the run is killed

    def close(self):
        """Close the log. Safe to call more than once."""
        if self._closed:
            return
        self._closed = True
        self._release()

    def _release(self):
        if self._owned:
            self._fh.close()
        elif not getattr(self._fh, "closed", False):
            self._fh.flush()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return None
