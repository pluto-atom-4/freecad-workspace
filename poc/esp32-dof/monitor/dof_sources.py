#!/usr/bin/env python3
"""Frame line sources. A source is an Iterator[str] of raw frame lines, non-empty,
stripped of trailing \r\n (uniform with #235 serial_lines). Consumers call dof_frame.parse_line.
mock motion is fusion-model-consistent: gx=d(roll)/dt, gy=d(pitch)/dt (deg/s), gz=0."""
import csv
import math
import os
import time
from typing import Iterator

from dof_frame import Frame, format_frame

_MAX_REPLAY_SLEEP_S = 1.0
_REQUIRED_COLUMNS = ("seq", "t_us", "ax", "ay", "az", "gx", "gy", "gz")
_ROLL_AMP, _ROLL_HZ = 0.5, 0.2
_PITCH_AMP, _PITCH_HZ = 0.3, 0.1


def mock_angles(t_s: float) -> tuple:
    """(roll, pitch) radians at time t_s.

    Args:
        t_s: Time in seconds

    Returns:
        tuple: (roll, pitch) in radians
    """
    return (_ROLL_AMP * math.sin(2 * math.pi * _ROLL_HZ * t_s),
            _PITCH_AMP * math.sin(2 * math.pi * _PITCH_HZ * t_s))


def mock_frame(k: int, rate_hz: float = 50.0) -> Frame:
    """Generate a synthetic frame at sample index k.

    Fusion-model-consistent motion: roll and pitch evolve as sinusoids;
    gyro rates are analytic derivatives (gx=d(roll)/dt, gy=d(pitch)/dt, gz=0);
    accel derived from gravity and pitch/roll rotations (no linear acceleration).

    Args:
        k: Sample index (0-based)
        rate_hz: Sampling rate in Hz (default 50.0)

    Returns:
        Frame: Synthetic IMU frame at time t=k/rate_hz
    """
    t = k / rate_hz
    roll, pitch = mock_angles(t)
    accel = (-math.sin(pitch),
             math.sin(roll) * math.cos(pitch),
             math.cos(roll) * math.cos(pitch))
    gx = math.degrees(_ROLL_AMP * 2 * math.pi * _ROLL_HZ * math.cos(2 * math.pi * _ROLL_HZ * t))
    gy = math.degrees(_PITCH_AMP * 2 * math.pi * _PITCH_HZ * math.cos(2 * math.pi * _PITCH_HZ * t))
    return Frame(seq=k % 65536,
                 t_us=round(k * 1_000_000 / rate_hz) % 2**32,
                 accel=accel, gyro=(gx, gy, 0.0))


def _line(frame: Frame) -> str:
    """Convert Frame to raw frame line (no trailing whitespace).

    Args:
        frame: Frame to format

    Returns:
        str: Frame formatted as protocol line, stripped of \\r\\n
    """
    return format_frame(frame).rstrip("\r\n")


def mock_lines(rate_hz: float = 50.0, duration_s=None, realtime: bool = True,
               drop_every: int = 0) -> Iterator[str]:
    """Synthesize infinite or time-bounded frame lines.

    Deterministic synthetic motion: roll=0.5*sin(2π*0.2*t), pitch=0.3*sin(2π*0.1*t) radians.
    Accelerometer derived from gravity + pitch/roll rotations.
    Gyroscope: gx=d(roll)/dt, gy=d(pitch)/dt (degrees/s), gz=0 (matching fusion's simple axis-rate model).

    Args:
        rate_hz: Sampling rate in Hz (default 50.0, must be finite > 0)
        duration_s: Duration in seconds; None means infinite (use itertools.islice to limit).
                    Finite duration must be > 0. (default None)
        realtime: If True, yield frames with sleep delays matching wall-clock time
                  (respect frame spacing for real-time replay). If False, yield as fast
                  as possible. (default True)
        drop_every: Skip every Nth frame slot (0 = no drops, N > 0 skips frames at slots
                    where (k+1) % drop_every == 0). Dropped slots still advance time and seq;
                    creates detectable gaps in sequence numbers. (default 0)

    Returns:
        Iterator[str]: Infinite or time-bounded sequence of raw frame lines.

    Raises:
        ValueError: If rate_hz is non-finite or <= 0, duration_s is given but non-finite
                    or <= 0, or drop_every < 0. These errors are EAGER (raised at call time,
                    not during iteration).

    Notes:
        - drop_every=1 yields nothing (acceptable; every frame is dropped).
        - duration_s=None is infinite; use itertools.islice to limit iteration.
        - Argument errors are eager (raised on the call to mock_lines()).
    """
    # EAGER validation (runs at call time, not first next()):
    if not (isinstance(rate_hz, (int, float)) and math.isfinite(rate_hz) and rate_hz > 0):
        raise ValueError(f"rate_hz must be finite > 0, got {rate_hz!r}")
    if duration_s is not None and not (math.isfinite(duration_s) and duration_s > 0):
        raise ValueError(f"duration_s must be None or finite > 0, got {duration_s!r}")
    if drop_every < 0:
        raise ValueError(f"drop_every must be >= 0, got {drop_every!r}")
    return _mock_gen(float(rate_hz), duration_s, realtime, int(drop_every))


def _mock_gen(rate_hz, duration_s, realtime, drop_every):
    """Internal generator for mock_lines.

    Args:
        rate_hz: Sampling rate (already validated, float)
        duration_s: Duration or None (already validated)
        realtime: Boolean
        drop_every: Drop frequency (already validated, int)

    Yields:
        str: Raw frame lines
    """
    n = None if duration_s is None else int(round(duration_s * rate_hz))
    start = time.monotonic()
    k = 0
    while n is None or k < n:
        if drop_every > 0 and (k + 1) % drop_every == 0:
            k += 1
            continue                      # slot skipped: seq gap, time still advances
        if realtime:
            delay = start + k / rate_hz - time.monotonic()   # absolute deadline, no drift
            if delay > 0:
                time.sleep(delay)
        yield _line(mock_frame(k, rate_hz))
        k += 1                            # no try/except: KeyboardInterrupt/GeneratorExit propagate


def csv_replay_lines(path, realtime: bool = True) -> Iterator[str]:
    """Replay frame lines from a CSV file.

    Reads a CSV with columns (any order, extras ignored): seq, t_us, ax, ay, az, gx, gy, gz.
    Reconstructs Frame objects, formats them as protocol lines, and optionally sleeps by
    inter-frame time deltas (clamped to max 1 s; no sleep for negative/zero deltas or first row).

    Args:
        path: Path to CSV file (str or PathLike). Header row required, with lowercase
              column names. Blank lines skipped. BOM (UTF-8 signature) stripped.
        realtime: If True, yield frames with sleep delays matching t_us deltas
                  (clamped to _MAX_REPLAY_SLEEP_S). If False, yield as fast as possible.
                  (default True)

    Returns:
        Iterator[str]: Sequence of raw frame lines from the file.

    Raises:
        ValueError: If path is not str/PathLike, on missing required columns (eager, before
                    first iteration), or on bad row data (lazy, raised while iterating).
                    Row errors include line number.

    Notes:
        - Argument errors (path type, missing columns) are EAGER.
        - Row format/value errors are LAZY (raised during list() or iteration).
        - Blank rows (fully empty after parsing) are silently skipped by csv.DictReader.
        - CSV header is case-insensitive but columns names must match exactly.
        - Negative or zero t_us delta: no sleep (don't replay backwards/stall).
    """
    if not isinstance(path, (str, os.PathLike)):
        raise ValueError(f"path must be str or PathLike, got {type(path).__name__}")
    return _csv_gen(path, realtime)


def _csv_gen(path, realtime):
    """Internal generator for csv_replay_lines.

    Args:
        path: Path to CSV (already validated as str/PathLike)
        realtime: Boolean

    Yields:
        str: Raw frame lines

    Raises:
        ValueError: On missing columns (before first yield) or bad row data
    """
    prev_t = None
    with open(path, newline="", encoding="utf-8-sig") as fh:   # utf-8-sig strips BOM; with-> closes on early exit
        rd = csv.DictReader(fh)
        names = [(n or "").strip() for n in (rd.fieldnames or [])]
        missing = [c for c in _REQUIRED_COLUMNS if c not in names]
        if missing:
            raise ValueError(f"{path}: missing columns {missing}")
        rd.fieldnames = names              # extras ignored; any order OK
        for row in rd:                     # DictReader skips fully blank rows
            try:
                frame = Frame(seq=int(row["seq"]), t_us=int(row["t_us"]),
                              accel=(float(row["ax"]), float(row["ay"]), float(row["az"])),
                              gyro=(float(row["gx"]), float(row["gy"]), float(row["gz"])))
                line = _line(frame)        # FrameError (ValueError) for out-of-range; no modding
            except (ValueError, TypeError) as e:   # TypeError: short row -> None
                raise ValueError(f"{path}:{rd.line_num}: {e}") from e
            if realtime and prev_t is not None:
                dt = (frame.t_us - prev_t) / 1e6
                if dt > 0:                 # negative/zero (wrap, out-of-order): no sleep
                    time.sleep(min(dt, _MAX_REPLAY_SLEEP_S))
            prev_t = frame.t_us
            yield line
