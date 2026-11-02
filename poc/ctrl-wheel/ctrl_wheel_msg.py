#!/usr/bin/env python3
"""
WHL wheel-velocity line protocol (ctrl_wheel_msg): formatter, parser and a
step-input generator for the ctrl-wheel POC.

SIM ONLY. The sign convention of the model and raw values is UNVERIFIED.
The wire format is not validated against any real device or receiver.
This module must stay in step with the BRC firmware header
lib/wheel_frame/wheel_frame.h (format, field order, limits, raw conversion).

Line format (ASCII, one line per sample):
    WHL,<seq>,<t_us>,<model %.6f>,<raw>,<state>*<HH>\\n

Protocol details:
- seq: uint16 sequence number, wraps at 65536 (format takes seq mod 65536)
- t_us: uint32 device microseconds (0 .. 2^32-1)
- model: model wheel velocity in rad/s, %.6f; non-finite formats as 0.000000
- raw: Dynamixel goal velocity raw units (0.229 rpm per unit), capped at +-125
- state: 0, 1 or 2
- HH: 2-digit UPPERCASE hex XOR checksum of the body (WHL up to the *)
- "-0.000000" is normalised to "0.000000"
- Line terminator: \\n (parser also accepts \\r\\n)

Usage (run from poc/ctrl-wheel):
    python3 -m ctrl_wheel_msg --step 1.0 --seconds 5
    python3 -m ctrl_wheel_msg --step 1.0 --seconds 5 --no-sleep
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from collections.abc import Callable, Iterator
from dataclasses import dataclass


PREFIX = "WHL"
SEQ_MOD = 65536
T_US_MAX = 2**32 - 1
STATES = (0, 1, 2)
FIELD_COUNT = 6
RAW_CAP = 125
RAW_RPM_PER_UNIT = 0.229

_UINT_RE = re.compile(r"[0-9]+")
_INT_RE = re.compile(r"[+-]?[0-9]+")
_HEX_DIGITS = "0123456789ABCDEF"


class WhlError(ValueError):
    """Raised when WHL formatting or parsing fails."""


@dataclass(frozen=True)
class Whl:
    """Parsed or to-be-formatted WHL sample."""
    seq: int
    t_us: int
    model_rad_s: float
    raw: int
    state: int


def checksum(body: str) -> int:
    """XOR of ord values of an ASCII body (WHL up to the *, exclusive)."""
    result = 0
    for c in body:
        if not c.isascii():
            raise WhlError(f"non-ASCII character in body: {c!r}")
        result ^= ord(c)
    return result


def _check_int(name: str, value: object) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise WhlError(f"{name} must be int, got {value!r}")


def _parse_uint(name: str, tok: str) -> int:
    if not _UINT_RE.fullmatch(tok):
        raise WhlError(f"{name} must be decimal digits, got {tok!r}")
    return int(tok)


def _parse_int(name: str, tok: str) -> int:
    if not _INT_RE.fullmatch(tok):
        raise WhlError(f"{name} must be a decimal integer, got {tok!r}")
    return int(tok)


def _parse_float(name: str, tok: str) -> float:
    # Reject any whitespace and underscores before float() sees the token.
    if any(c.isspace() for c in tok):
        raise WhlError(f"{name} has embedded whitespace: {tok!r}")
    if "_" in tok:
        raise WhlError(f"{name} contains underscore: {tok!r}")
    try:
        val = float(tok)
    except ValueError:
        raise WhlError(f"{name} is not a valid float: {tok!r}")
    if not math.isfinite(val):
        raise WhlError(f"{name} is not finite: {tok!r}")
    return val


def format_whl(w: Whl) -> str:
    """Format a Whl into one protocol line with checksum and newline."""
    _check_int("seq", w.seq)
    _check_int("t_us", w.t_us)
    _check_int("raw", w.raw)
    _check_int("state", w.state)

    if not (0 <= w.t_us <= T_US_MAX):
        raise WhlError(f"t_us out of range [0, {T_US_MAX}], got {w.t_us}")
    if w.state not in STATES:
        raise WhlError(f"state must be one of {STATES}, got {w.state}")

    seq = w.seq % SEQ_MOD

    model = float(w.model_rad_s)
    if not math.isfinite(model):
        model = 0.0
    model_str = f"{model:.6f}"
    if model_str == "-0.000000":
        model_str = "0.000000"

    body = f"{PREFIX},{seq},{w.t_us},{model_str},{w.raw},{w.state}"
    return f"{body}*{checksum(body):02X}\n"


def parse_line(line: str) -> Whl:
    """
    Parse one WHL line. Accepts a trailing \\n or \\r\\n.

    Float tokens are lenient (exponent, leading '+') but must be finite and
    free of whitespace and underscores. Integer tokens are checked with an
    explicit pattern, not int(), so "1_0" is rejected.

    Raises:
        WhlError: on any malformed, mis-checksummed or out-of-range line.
    """
    line = line.rstrip("\r\n")

    star = line.rfind("*")
    if star == -1:
        raise WhlError("missing checksum marker *")

    body = line[:star]
    hh = line[star + 1:]

    if len(hh) != 2:
        raise WhlError("checksum must be exactly 2 characters")
    if any(c not in _HEX_DIGITS for c in hh):
        raise WhlError("checksum must be uppercase hex (0-9A-F)")

    computed = checksum(body)
    expected = int(hh, 16)
    if computed != expected:
        raise WhlError(f"checksum mismatch: computed {computed:02X}, expected {hh}")

    fields = body.split(",")
    if len(fields) != FIELD_COUNT:
        raise WhlError(f"expected {FIELD_COUNT} fields, got {len(fields)}")
    if fields[0] != PREFIX:
        raise WhlError(f"bad prefix: expected {PREFIX!r}, got {fields[0]!r}")

    seq = _parse_uint("seq", fields[1])
    if seq >= SEQ_MOD:
        raise WhlError(f"seq out of range [0, {SEQ_MOD - 1}], got {seq}")

    t_us = _parse_uint("t_us", fields[2])
    if t_us > T_US_MAX:
        raise WhlError(f"t_us out of range [0, {T_US_MAX}], got {t_us}")

    model = _parse_float("model_rad_s", fields[3])
    raw = _parse_int("raw", fields[4])

    state = _parse_uint("state", fields[5])
    if state not in STATES:
        raise WhlError(f"state must be one of {STATES}, got {state}")

    return Whl(seq=seq, t_us=t_us, model_rad_s=model, raw=raw, state=state)


def _raw_for(value_rad_s: float) -> int:
    """rad/s -> Dynamixel raw velocity, clamped to +-RAW_CAP."""
    raw = round(value_rad_s * 60.0 / (2.0 * math.pi) / RAW_RPM_PER_UNIT)
    return max(-RAW_CAP, min(RAW_CAP, raw))


def step_lines(
    value_rad_s: float,
    seconds: float,
    rate_hz: float = 50.0,
    realtime: bool = False,
    start_seq: int = 0,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], object] = time.sleep,
) -> Iterator[str]:
    """
    Yield WHL lines holding a constant model value for `seconds` at `rate_hz`.

    State is 0. seq counts up from start_seq (format wraps it at 65536).
    t_us steps by round(1e6 / rate_hz) and wraps at 2^32 like the device.
    When realtime is True, each line is released against the injected clock
    by sleeping with the injected sleep.
    """
    value = float(value_rad_s)
    if not math.isfinite(value):
        raise WhlError(f"step value must be finite, got {value_rad_s!r}")
    if not (math.isfinite(rate_hz) and rate_hz > 0):
        raise WhlError(f"rate must be a positive finite number, got {rate_hz!r}")
    if not (math.isfinite(seconds) and seconds >= 0):
        raise WhlError(f"seconds must be a non-negative finite number, got {seconds!r}")

    count = round(seconds * rate_hz)
    t_step = round(1e6 / rate_hz)
    period = 1.0 / rate_hz
    raw = _raw_for(value)

    t0 = clock() if realtime else 0.0
    for i in range(count):
        if realtime:
            delay = (t0 + i * period) - clock()
            if delay > 0:
                sleep(delay)
        yield format_whl(
            Whl(
                seq=(start_seq + i) % SEQ_MOD,
                t_us=(i * t_step) % (T_US_MAX + 1),
                model_rad_s=value,
                raw=raw,
                state=0,
            )
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Print a constant-step WHL stream (SIM ONLY, sign UNVERIFIED)."
    )
    parser.add_argument("--step", type=float, required=True, help="model value in rad/s")
    parser.add_argument("--seconds", type=float, required=True, help="duration in seconds")
    parser.add_argument("--rate", type=float, default=50.0, help="lines per second (default 50)")
    parser.add_argument(
        "--no-sleep",
        action="store_true",
        help="print as fast as possible instead of pacing in real time",
    )
    args = parser.parse_args(argv)

    try:
        for line in step_lines(
            args.step,
            args.seconds,
            rate_hz=args.rate,
            realtime=not args.no_sleep,
        ):
            sys.stdout.write(line)
            sys.stdout.flush()
    except WhlError as exc:
        print(f"ctrl_wheel_msg: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
