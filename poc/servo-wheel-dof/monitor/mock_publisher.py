#!/usr/bin/env python3
"""
Standalone mock/replay UDP publisher for the servo-wheel-dof POC (issue #284,
sub-issue of #258).

Feeds the Webots follower controller with IMU + wheel_angle datagrams on
127.0.0.1 (default port 5006, env SWD_UDP_PORT), so the follower can be driven
without hardware. Stdlib only; reuses the wire format from imu_wheel_msg.py
(issue #282). Must NOT import numpy, `controller` or `serial`.

Sources (exactly one required):
    --mock          synthetic sinusoids from imu_wheel_msg.mock_imu_wheel
    --replay CSV    rows of a CSV with columns roll,pitch,yaw,wheel_angle
                    (radians) and an optional t_us column (microseconds)

Pacing: --mock sends at --rate Hz. --replay follows the t_us deltas when the
column exists (each gap clamped to 1 s, a non-positive gap means no wait),
otherwise --rate. A replay stops at the end of the file. --duration (seconds)
caps either source.

Exit codes: 0 normal end or Ctrl-C; 2 bad arguments or unreadable/invalid CSV.

Usage:
    python3 monitor/mock_publisher.py --mock --duration 5
    python3 monitor/mock_publisher.py --replay run.csv --port 5006
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import socket
import sys
import time
from collections.abc import Iterable, Iterator
from pathlib import Path

# monitor/ -> ../webots/controllers/servo_wheel_dof (pure modules, no Webots).
_CONTROLLER_DIR = (
    Path(__file__).resolve().parent.parent
    / "webots"
    / "controllers"
    / "servo_wheel_dof"
)
sys.path.insert(0, str(_CONTROLLER_DIR))

from imu_wheel_msg import (  # noqa: E402
    ImuWheel,
    format_imu_wheel_msg,
    mock_imu_wheel,
)

HOST = "127.0.0.1"
DEFAULT_PORT = 5006
PORT_ENV = "SWD_UDP_PORT"
DEFAULT_RATE_HZ = 50.0
MAX_REPLAY_GAP_S = 1.0
_SEND_TIMEOUT_S = 0.05
_ANGLE_COLUMNS = ("roll", "pitch", "yaw", "wheel_angle")


def _parse_float(text: str | None, name: str) -> float:
    """Parse one finite float cell; ValueError names the column."""
    if text is None or not text.strip():
        raise ValueError(f"{name}: missing value")
    try:
        value = float(text)
    except ValueError:
        raise ValueError(f"{name}: not a number {text!r}") from None
    if not math.isfinite(value):
        raise ValueError(f"{name}: non-finite value {text!r}")
    return value


def _parse_int(text: str | None, name: str) -> int:
    """Parse one integer cell; ValueError names the column."""
    if text is None or not text.strip():
        raise ValueError(f"{name}: missing value")
    try:
        return int(text)
    except ValueError:
        raise ValueError(f"{name}: not an integer {text!r}") from None


def parse_replay_csv(
    lines: Iterable[str], source: str = "<csv>"
) -> tuple[list[ImuWheel], list[int]]:
    """
    Parse replay CSV lines (pure: no file or socket access).

    Args:
        lines: Iterable of text lines; the first non-blank line is the header.
        source: Name used as the prefix of error messages.

    Returns:
        (samples, t_us). t_us is empty when the file has no t_us column,
        otherwise it has one int per sample.

    Raises:
        ValueError: missing required column, or bad row data. Row errors read
        "<source>:<line>: <column>: <problem>"; the header is line 1.
        Empty/NaN/inf/non-numeric cells are rejected.

    Notes:
        Column order is free, extra columns are ignored, blank lines are
        skipped, and a UTF-8 BOM on the first header cell is stripped.
    """
    reader = csv.DictReader(lines)
    header = reader.fieldnames or []
    names = [(n or "").lstrip("\ufeff").strip() for n in header]
    missing = [c for c in _ANGLE_COLUMNS if c not in names]
    if missing:
        raise ValueError(f"{source}: missing columns {missing}")
    reader.fieldnames = names
    has_t = "t_us" in names
    samples: list[ImuWheel] = []
    t_us: list[int] = []
    for row in reader:
        try:
            angles = [_parse_float(row[c], c) for c in _ANGLE_COLUMNS]
            t = _parse_int(row["t_us"], "t_us") if has_t else None
        except ValueError as exc:
            raise ValueError(f"{source}:{reader.line_num}: {exc}") from None
        samples.append(ImuWheel(*angles))
        if t is not None:
            t_us.append(t)
    return samples, t_us


def load_replay_csv(path) -> tuple[list[ImuWheel], list[int]]:
    """Read a replay CSV file (BOM-safe) and parse it; see parse_replay_csv."""
    with open(path, newline="", encoding="utf-8-sig") as fh:
        return parse_replay_csv(fh, source=str(path))


def mock_timeline(rate_hz: float) -> Iterator[tuple[float, ImuWheel]]:
    """
    Endless (t_s, sample) pairs: sample k is mock_imu_wheel(k / rate_hz).

    Args:
        rate_hz: Sample rate in Hz; must be > 0 (the CLI validates it).
    """
    k = 0
    while True:
        t_s = k / rate_hz
        yield t_s, mock_imu_wheel(t_s)
        k += 1


def replay_timeline(
    samples: list[ImuWheel], t_us: list[int], rate_hz: float
) -> Iterator[tuple[float, ImuWheel]]:
    """
    (t_s, sample) pairs for a parsed CSV; the first sample is at t = 0.

    Args:
        samples: Parsed samples.
        t_us: Per-sample microsecond stamps, or [] to pace by rate_hz.
        rate_hz: Fallback rate in Hz (> 0); ignored when t_us is given.

    Notes:
        With t_us, each gap is (t_us[i] - t_us[i-1]) / 1e6 clamped to
        MAX_REPLAY_GAP_S; a zero or negative gap adds no time.
    """
    t_s = 0.0
    for i, sample in enumerate(samples):
        if i > 0:
            if t_us:
                gap = (t_us[i] - t_us[i - 1]) / 1e6
                t_s += min(gap, MAX_REPLAY_GAP_S) if gap > 0 else 0.0
            else:
                t_s += 1.0 / rate_hz
        yield t_s, sample


def _is_valid(sample) -> bool:
    """True if all four angle fields are real, finite numbers (not bool)."""
    for name in _ANGLE_COLUMNS:
        value = getattr(sample, name, None)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return False
        try:
            finite = math.isfinite(value)
        except OverflowError:
            return False
        if not finite:
            return False
    return True


class Publisher:
    """
    UDP sender that never raises from send().

    Counters: sent (successful sendto), errors (swallowed OSError), skipped
    (invalid sample or seq, nothing sent). Use an IPv4 literal for host.
    """

    def __init__(self, host: str = HOST, port: int = DEFAULT_PORT):
        if not isinstance(host, str) or not host:
            raise ValueError(f"host must be a non-empty str, got {host!r}")
        if (
            isinstance(port, bool)
            or not isinstance(port, int)
            or not 1 <= port <= 65535
        ):
            raise ValueError(f"port must be int 1..65535, got {port!r}")
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(_SEND_TIMEOUT_S)
        self.sent = 0
        self.errors = 0
        self.skipped = 0

    def send(self, seq: int, sample: ImuWheel) -> bool:
        """Send one datagram. Never raises. Returns True only if it was sent."""
        if self._sock is None:  # closed: silent no-op
            return False
        if (
            isinstance(seq, bool)
            or not isinstance(seq, int)
            or seq < 0
            or not _is_valid(sample)
        ):
            self.skipped += 1
            return False
        try:
            data = format_imu_wheel_msg(seq, sample)
            self._sock.sendto(data, self._addr)
        except OSError:
            self.errors += 1
            return False
        self.sent += 1
        return True

    def close(self) -> None:
        """Close the socket; later send() calls are silent no-ops."""
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass


def run_timeline(
    pub,
    timeline: Iterable[tuple[float, ImuWheel]],
    duration_s: float | None = None,
    clock=time.monotonic,
    sleep=time.sleep,
) -> None:
    """
    Send each (t_s, sample) at start + t_s; seq is the pair's index.

    Args:
        pub: Object with send(seq, sample).
        timeline: Iterable of (t_s, sample), t_s non-decreasing.
        duration_s: Stop before the first pair with t_s >= duration_s.
        clock: Monotonic clock (injectable for tests).
        sleep: Sleep function (injectable for tests).
    """
    start = clock()
    for seq, (t_s, sample) in enumerate(timeline):
        if duration_s is not None and t_s >= duration_s:
            break
        delay = start + t_s - clock()
        if delay > 0:
            sleep(delay)
        pub.send(seq, sample)


def parse_port(text) -> int:
    """Return a port int 1..65535 from text, else ValueError."""
    try:
        port = int(str(text).strip())
    except ValueError:
        raise ValueError(f"port must be an integer, got {text!r}") from None
    if not 1 <= port <= 65535:
        raise ValueError(f"port must be 1..65535, got {port}")
    return port


def _port_type(text: str) -> int:
    try:
        return parse_port(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from None


def _positive_float(text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not a number: {text!r}") from None
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError(f"must be finite and > 0: {text!r}")
    return value


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI parser (--mock xor --replay is required)."""
    parser = argparse.ArgumentParser(
        prog="mock_publisher.py",
        description="Publish mock or replayed IMU + wheel_angle UDP datagrams "
        "to the servo-wheel-dof Webots follower on 127.0.0.1.",
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "--mock", action="store_true", help="synthetic sinusoid motion"
    )
    source.add_argument(
        "--replay",
        metavar="CSV",
        help="replay CSV (roll,pitch,yaw,wheel_angle[,t_us]); stops at the end",
    )
    parser.add_argument(
        "--port",
        type=_port_type,
        default=None,
        help=f"UDP port (default: env {PORT_ENV} or {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--rate",
        type=_positive_float,
        default=DEFAULT_RATE_HZ,
        help="send rate in Hz (default %(default)s; ignored for replay t_us)",
    )
    parser.add_argument(
        "--duration",
        type=_positive_float,
        default=None,
        metavar="SECONDS",
        help="stop after this many seconds (default: mock runs until Ctrl-C)",
    )
    return parser


def main(argv=None, clock=time.monotonic, sleep=time.sleep) -> int:
    """CLI entry point; returns the process exit code (argparse exits 2)."""
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.port is not None:
        port = args.port
    else:
        try:
            port = parse_port(os.environ.get(PORT_ENV, str(DEFAULT_PORT)))
        except ValueError as exc:
            parser.error(f"{PORT_ENV}: {exc}")

    if args.mock:
        timeline = mock_timeline(args.rate)
    else:
        try:
            samples, t_us = load_replay_csv(args.replay)
        except (OSError, ValueError) as exc:
            parser.error(str(exc))
        timeline = replay_timeline(samples, t_us, args.rate)

    try:
        pub = Publisher(HOST, port)
    except OSError as exc:
        print(f"cannot create UDP socket: {exc}", file=sys.stderr)
        return 1

    try:
        run_timeline(pub, timeline, args.duration, clock=clock, sleep=sleep)
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
    finally:
        pub.close()

    print(f"published : {pub.sent}")
    print(f"errors    : {pub.errors}")
    print(f"skipped   : {pub.skipped}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
