#!/usr/bin/env python3
"""ESP32 6-DOF IMU monitor CLI: prints a live table + stats. No fusion / UDP here (see #238).

Usage (from poc/esp32-dof):
    mamba run -n esp32-dof python3 monitor/dof_monitor.py --mock --duration 3
    mamba run -n esp32-dof python3 monitor/dof_monitor.py --port /dev/ttyACM0
    mamba run -n esp32-dof python3 monitor/dof_monitor.py --replay capture.csv --quiet-every 1
    mamba run -n esp32-dof python3 monitor/dof_monitor.py --list-ports

Exit codes: 0 ok (incl. Ctrl-C), 2 usage error or source error (message on stderr).
Rows + summary go to stdout; errors go to stderr.
"""
import argparse
import math
import sys
from time import monotonic
from typing import Iterable, Iterator

from dof_frame import Frame, FrameError, parse_line
from dof_serial import list_ports, serial_lines
from dof_sources import csv_replay_lines, mock_lines
from dof_stats import StreamStats

_ROW_FMT = ("{seq:>5d} {t_ms:>10.1f} {ax:>8.3f} {ay:>8.3f} {az:>8.3f} "
            "{gx:>9.2f} {gy:>9.2f} {gz:>9.2f} {rate:>6.1f} {drops:>5d}")


def _header() -> str:
    return (f"{'seq':>5} {'t(ms)':>10} {'ax(g)':>8} {'ay(g)':>8} {'az(g)':>8} "
            f"{'gx(dps)':>9} {'gy(dps)':>9} {'gz(dps)':>9} {'rate':>6} {'drops':>5}")


def _format_row(frame: Frame, rate_hz: float, dropped: int) -> str:
    return _ROW_FMT.format(seq=frame.seq, t_ms=frame.t_us / 1000.0,
                           ax=frame.accel[0], ay=frame.accel[1], az=frame.accel[2],
                           gx=frame.gyro[0], gy=frame.gyro[1], gz=frame.gyro[2],
                           rate=rate_hz, drops=dropped)


def _print_summary(stats: StreamStats, out, interrupted: bool) -> None:
    title = "--- summary (interrupted) ---" if interrupted else "--- summary ---"
    rows = [("received", str(stats.received)),
            ("dropped", str(stats.dropped)),
            ("out_of_order", str(stats.out_of_order)),
            ("resets", str(stats.resets)),
            ("bad_frames", str(stats.bad_frames)),
            ("avg rate (Hz)", f"{stats.avg_rate_hz:.1f} (device time, {stats.elapsed_s:.2f} s)")]
    print(title, file=out)
    for label, value in rows:
        print(f"{label:<13}: {value}", file=out)
    out.flush()


def run(lines: Iterable[str], out=None, quiet_every: int = 5) -> StreamStats:
    """Consume raw frame lines; print a table row every `quiet_every` frames (all frames are
    counted); print a summary at the end. KeyboardInterrupt -> summary + normal return.
    Errors raised by the source (OSError/ValueError) propagate to the caller."""
    if isinstance(quiet_every, bool) or not isinstance(quiet_every, int) or quiet_every < 1:
        raise ValueError(f"quiet_every must be int >= 1, got {quiet_every!r}")
    if out is None:
        out = sys.stdout
    stats = StreamStats()
    interrupted = False
    print(_header(), file=out, flush=True)
    try:
        for line in lines:
            try:
                frame = parse_line(line)
            except FrameError:
                stats.note_bad()
                continue
            stats.update(frame.seq, frame.t_us)
            if (stats.received - 1) % quiet_every == 0:
                print(_format_row(frame, stats.rate_hz, stats.dropped), file=out, flush=True)
    except KeyboardInterrupt:
        interrupted = True
    _print_summary(stats, out, interrupted)
    return stats


def _limit_duration(lines: Iterable[str], seconds: float) -> Iterator[str]:
    """Stop after `seconds` of wall clock (checked between lines; deadline set at first next())."""
    deadline = monotonic() + seconds
    for line in lines:
        if monotonic() >= deadline:
            return
        yield line


def _positive_float(text: str) -> float:
    try:
        v = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not a number: {text!r}")
    if not (math.isfinite(v) and v > 0):
        raise argparse.ArgumentTypeError(f"must be finite > 0, got {text!r}")
    return v


def _positive_int(text: str) -> int:
    try:
        v = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not an integer: {text!r}")
    if v < 1:
        raise argparse.ArgumentTypeError(f"must be >= 1, got {text!r}")
    return v


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="dof_monitor",
                                description="ESP32 6-DOF IMU frame monitor (table + stats).")
    src = p.add_mutually_exclusive_group()
    src.add_argument("--port", metavar="PATH", help="serial device or pyserial URL, e.g. /dev/ttyACM0")
    src.add_argument("--mock", action="store_true", help="synthetic 50 Hz frames (real time)")
    src.add_argument("--replay", metavar="CSV", help="replay a CSV capture (seq,t_us,ax..gz)")
    p.add_argument("--baud", type=_positive_int, default=115200, help="serial baud (default 115200)")
    p.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    p.add_argument("--duration", type=_positive_float, default=None, metavar="SECONDS",
                   help="stop after SECONDS (default: run until Ctrl-C / end of input)")
    p.add_argument("--quiet-every", type=_positive_int, default=5, metavar="N",
                   help="print every Nth frame (default 5); all frames are still counted")
    return p


def _make_source(args) -> Iterable[str]:
    if args.mock:
        return mock_lines(duration_s=args.duration, realtime=True)
    if args.replay is not None:
        lines = csv_replay_lines(args.replay, realtime=True)
        return _limit_duration(lines, args.duration) if args.duration is not None else lines
    return serial_lines(args.port, args.baud, max_duration_s=args.duration)


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.list_ports:
        ports = list_ports()
        if ports:
            for p in ports:
                print(p)
        else:
            print("(no serial ports found)")
        return 0
    if not (args.mock or args.replay is not None or args.port is not None):
        parser.error("one of --port, --mock, --replay (or --list-ports) is required")
    try:
        lines = _make_source(args)
        run(lines, sys.stdout, args.quiet_every)
    except (OSError, ValueError) as exc:     # DofSerialError is an OSError
        print(f"dof_monitor: error: {exc}", file=sys.stderr, flush=True)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
