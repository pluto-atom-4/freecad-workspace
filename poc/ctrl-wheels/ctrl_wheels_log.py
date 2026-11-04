"""
CSV log writer for the ctrl-wheels twin-wheel proof of concept (freecad-workspace
#494, task F10, under umbrella FW #482). Contract: poc/ctrl-wheels/CONTRACT.md, S6.

SIM ONLY. Not hardware-validated. All signs are UNCONFIRMED (chip-side mirror,
bridge sign and Webots sign). Latency UNMEASURED.

One row per host tick, 19 columns. Columns are fixed by COLUMNS and written in
that order, whatever the key order of the input mapping. A row is fully
validated before anything is written, so a rejected row leaves no partial line.

whl_raw_l and whl_raw_r are the raw goal velocities the planned firmware writes
to each chip, taken from the WHL2 line. They are not a servo readback. This log
does not range-check whl_raw_l or whl_raw_r to int32: it accepts any Python int
in an int column. The int32 range is enforced by the WHL2 parser,
ctrl_wheels_msg.parse_line as rewritten by PR #511 (task F04), CONTRACT.md S1;
this branch does not contain that change.

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

# Documentation data for humans. It is read only by test_ctrl_wheels_log.py, which pins its keys to COLUMNS,
# so adding a column needs a description here. It is not used at run time.
COLUMN_DOC = {
    "host_t_s": "host wall-clock seconds at the row, 3 decimals",
    "whl_seq": "wheel telemetry sequence number from the device (WHL2 seq)",
    "whl_t_us": "wheel telemetry timestamp in microseconds, device clock",
    "whl_model_rad_s": "wheel model velocity in rad/s, one value for the pair (SIM, sign UNCONFIRMED)",
    "whl_raw_l": "left raw goal velocity the firmware writes to the left chip; not a servo readback",
    "whl_raw_r": "right raw goal velocity the firmware writes to the right chip; not a servo readback",
    "whl_state": "pair state code from the WHL2 line: 0 run, 1 hold after a fall, 2 bus down",
    "cmd_seq": "sequence number of the command sent by the host",
    "cmd_left_rad_s": "commanded left wheel velocity in rad/s, after the host sign and clamp",
    "cmd_right_rad_s": "commanded right wheel velocity in rad/s, after the host sign and clamp",
    "stale": "stale flag, 0 or 1, as supplied by the caller",
    "echo_cmd_seq": "cmd_seq_echo from the echo JSON; empty when no echo or the echo value is null",
    "sim_t_s": "Webots simulation time in seconds; empty when no echo",
    "angle_rad_l": "simulated left hinge angle in rad; empty when no echo",
    "angle_rad_r": "simulated right hinge angle in rad; empty when no echo",
    "vel_rad_s_l": "simulated left hinge velocity in rad/s; empty when no echo or the echo value is null",
    "vel_rad_s_r": "simulated right hinge velocity in rad/s; empty when no echo or the echo value is null",
    "host_rtt_ms": "wall ms from sending cmd_seq to its echo (not link latency); empty when no echo",
    "missed_whl": "count of missed wheel telemetry frames as supplied by the caller",
}

_INT_COLUMNS = (
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
