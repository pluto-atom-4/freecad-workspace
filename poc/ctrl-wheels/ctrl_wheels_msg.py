#!/usr/bin/env python3
"""WHL2 twin-wheel line protocol: formatter, parser and step generator."""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from collections.abc import Callable, Iterator
from dataclasses import dataclass


PREFIX = "WHL2"
FIELD_COUNT = 7
_V1_PREFIX = "WHL"
SEQ_MOD = 65536
T_US_MAX = 2**32 - 1
INT32_MIN = -(2**31)
INT32_MAX = 2**31 - 1
STATES = (0, 1, 2)
RAW_CAP = 125
RAW_RPM_PER_UNIT = 0.229
_MAX_TOKEN_LEN = 20

_UINT_RE = re.compile(r"[0-9]+")
_INT_RE = re.compile(r"[+-]?[0-9]+")
_HEX_DIGITS = "0123456789ABCDEF"


class WhlError(ValueError):
    """Raised when WHL2 parsing or formatting, or WHL v1 formatting, fails."""


@dataclass(frozen=True)
class Whl:
    """Single-wheel sample (WHL v1 shape), built by split() and written by format_whl."""
    seq: int
    t_us: int
    model_rad_s: float
    raw: int
    state: int


@dataclass(frozen=True)
class Whl2:
    """Parsed or to-be-formatted twin sample (WHL2): one model, two raws."""
    seq: int
    t_us: int
    model_rad_s: float
    raw_l: int
    raw_r: int
    state: int


def checksum(body: str) -> int:
    """XOR of ord values of an ASCII body (WHL or WHL2 body, up to the *, exclusive)."""
    result = 0
    for c in body:
        if not c.isascii():
            raise WhlError(f"non-ASCII character in body: {c!r}")
        result ^= ord(c)
    return result


def _check_int(name: str, value: object) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise WhlError(f"{name} must be int, got {value!r}")


def _check_int32(name: str, value: int) -> None:
    if not (INT32_MIN <= value <= INT32_MAX):
        raise WhlError(f"{name} out of int32 range [{INT32_MIN}, {INT32_MAX}], got {value}")


def _model_to_float(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise WhlError(f"model_rad_s must be a real number, got {value!r}")
    try:
        return float(value)
    except OverflowError:
        raise WhlError(f"model_rad_s out of float range, got {value!r}")


def _show(tok: str) -> str:
    """Repr of a token for an error message; a long token is shown by length only."""
    if len(tok) > _MAX_TOKEN_LEN:
        return f"<{len(tok)} chars>"
    return repr(tok)


def _check_token_len(name: str, tok: str) -> None:
    if len(tok) > _MAX_TOKEN_LEN:
        raise WhlError(
            f"{name} token too long: {len(tok)} chars, limit {_MAX_TOKEN_LEN}"
        )


def _parse_uint(name: str, tok: str) -> int:
    _check_token_len(name, tok)
    if not _UINT_RE.fullmatch(tok):
        raise WhlError(f"{name} must be decimal digits, got {_show(tok)}")
    return int(tok)


def _parse_int(name: str, tok: str) -> int:
    _check_token_len(name, tok)
    if not _INT_RE.fullmatch(tok):
        raise WhlError(f"{name} must be a decimal integer, got {_show(tok)}")
    return int(tok)


def _parse_float(name: str, tok: str) -> float:
    _check_token_len(name, tok)
    if any(c.isspace() for c in tok):
        raise WhlError(f"{name} has embedded whitespace: {_show(tok)}")
    if "_" in tok:
        raise WhlError(f"{name} contains underscore: {_show(tok)}")
    try:
        val = float(tok)
    except ValueError:
        raise WhlError(f"{name} is not a valid float: {_show(tok)}")
    if not math.isfinite(val):
        raise WhlError(f"{name} is not finite: {_show(tok)}")
    return val


def _model_text(model: float) -> str:
    """Format a finite model value with %.6f, turning -0.000000 into 0.000000."""
    model_str = f"{model:.6f}"
    if model_str == "-0.000000":
        model_str = "0.000000"
    if len(model_str) > _MAX_TOKEN_LEN:
        raise WhlError(
            f"model_rad_s magnitude exceeds the supported range: "
            f"text is {len(model_str)} chars, limit {_MAX_TOKEN_LEN}"
        )
    return model_str


def format_whl2(w: Whl2) -> str:
    """Format a Whl2 into one WHL2 line with checksum and newline."""
    _check_int("seq", w.seq)
    _check_int("t_us", w.t_us)
    _check_int("raw_l", w.raw_l)
    _check_int("raw_r", w.raw_r)
    _check_int("state", w.state)

    if not (0 <= w.t_us <= T_US_MAX):
        raise WhlError(f"t_us out of range [0, {T_US_MAX}], got {w.t_us}")
    _check_int32("raw_l", w.raw_l)
    _check_int32("raw_r", w.raw_r)
    if w.state not in STATES:
        raise WhlError(f"state must be one of {STATES}, got {w.state}")

    seq = w.seq % SEQ_MOD

    model = _model_to_float(w.model_rad_s)
    if not math.isfinite(model):
        raise WhlError(f"model_rad_s must be finite, got {w.model_rad_s!r}")
    model_str = _model_text(model)

    body = f"{PREFIX},{seq},{w.t_us},{model_str},{w.raw_l},{w.raw_r},{w.state}"
    return f"{body}*{checksum(body):02X}\n"


def format_whl(w: Whl) -> str:
    """Format a single-wheel Whl into one WHL v1 line with checksum and newline."""
    _check_int("seq", w.seq)
    _check_int("t_us", w.t_us)
    _check_int("raw", w.raw)
    _check_int("state", w.state)

    if not (0 <= w.t_us <= T_US_MAX):
        raise WhlError(f"t_us out of range [0, {T_US_MAX}], got {w.t_us}")
    _check_int32("raw", w.raw)
    if w.state not in STATES:
        raise WhlError(f"state must be one of {STATES}, got {w.state}")

    seq = w.seq % SEQ_MOD

    model = _model_to_float(w.model_rad_s)
    if not math.isfinite(model):
        raise WhlError(f"model_rad_s must be finite, got {w.model_rad_s!r}")
    model_str = _model_text(model)

    body = f"{_V1_PREFIX},{seq},{w.t_us},{model_str},{w.raw},{w.state}"
    return f"{body}*{checksum(body):02X}\n"


def parse_line(line: str) -> Whl2:
    """Parse one WHL2 line into a Whl2, or raise WhlError."""
    if line.endswith("\r\n"):
        line = line[:-2]
    elif line.endswith("\n"):
        line = line[:-1]

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
        raise WhlError(f"bad prefix: expected {PREFIX!r}, got {_show(fields[0])}")

    seq = _parse_uint("seq", fields[1])
    if seq >= SEQ_MOD:
        raise WhlError(f"seq out of range [0, {SEQ_MOD - 1}], got {seq}")

    t_us = _parse_uint("t_us", fields[2])
    if t_us > T_US_MAX:
        raise WhlError(f"t_us out of range [0, {T_US_MAX}], got {t_us}")

    model = _parse_float("model_rad_s", fields[3])
    raw_l = _parse_int("raw_l", fields[4])
    _check_int32("raw_l", raw_l)
    raw_r = _parse_int("raw_r", fields[5])
    _check_int32("raw_r", raw_r)

    state = _parse_uint("state", fields[6])
    if state not in STATES:
        raise WhlError(f"state must be one of {STATES}, got {state}")

    return Whl2(
        seq=seq,
        t_us=t_us,
        model_rad_s=model,
        raw_l=raw_l,
        raw_r=raw_r,
        state=state,
    )


def split(w: Whl2) -> tuple[Whl, Whl]:
    """Split a twin sample into (left, right) single-wheel samples."""
    left = Whl(seq=w.seq, t_us=w.t_us, model_rad_s=w.model_rad_s, raw=w.raw_l, state=w.state)
    right = Whl(seq=w.seq, t_us=w.t_us, model_rad_s=w.model_rad_s, raw=w.raw_r, state=w.state)
    return left, right


def _raw_for(value_rad_s: float) -> int:
    """rad/s -> Dynamixel raw velocity, clamped to +-RAW_CAP before rounding."""
    raw_f = value_rad_s * 60.0 / (2.0 * math.pi) / RAW_RPM_PER_UNIT
    return round(max(-RAW_CAP, min(RAW_CAP, raw_f)))


def _check_sign(name: str, value: object) -> int:
    """Accept only the int +1 or -1 (bool, float, None and str are refused)."""
    if isinstance(value, bool) or not isinstance(value, int) or value not in (1, -1):
        raise WhlError(f"{name} must be the int +1 or -1, got {value!r}")
    return value


def step_lines(
    value_rad_s: float,
    seconds: float,
    rate_hz: float = 50.0,
    realtime: bool = False,
    start_seq: int = 0,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], object] = time.sleep,
    sign_l: int = 1,
    sign_r: int = -1,
    state: int = 0,
) -> Iterator[str]:
    """Yield WHL2 twin lines holding a constant model value for a duration."""
    value = float(value_rad_s)
    if not math.isfinite(value):
        raise WhlError(f"step value must be finite, got {value_rad_s!r}")
    if isinstance(start_seq, bool) or not isinstance(start_seq, int):
        raise WhlError(f"start_seq must be int, got {start_seq!r}")
    if not (math.isfinite(rate_hz) and rate_hz > 0):
        raise WhlError(f"rate must be a positive finite number, got {rate_hz!r}")
    if not (math.isfinite(seconds) and seconds >= 0):
        raise WhlError(f"seconds must be a non-negative finite number, got {seconds!r}")
    _check_sign("sign_l", sign_l)
    _check_sign("sign_r", sign_r)
    _check_int("state", state)
    if state not in STATES:
        raise WhlError(f"state must be one of {STATES}, got {state}")

    count_f = seconds * rate_hz
    if not math.isfinite(count_f):
        raise WhlError(f"seconds * rate overflows, got {seconds!r} * {rate_hz!r}")
    count = round(count_f)

    t_step_f = 1e6 / rate_hz
    if not math.isfinite(t_step_f) or round(t_step_f) < 1:
        raise WhlError(
            f"rate {rate_hz!r} Hz gives a t_us step below 1 us (1e6/rate = {t_step_f!r})"
        )
    t_step = round(t_step_f)
    period = 1.0 / rate_hz
    raw = _raw_for(value)
    raw_l = sign_l * raw
    raw_r = sign_r * raw

    t0 = clock() if realtime else 0.0
    for i in range(count):
        if realtime:
            delay = (t0 + i * period) - clock()
            if delay > 0:
                sleep(delay)
        yield format_whl2(
            Whl2(
                seq=(start_seq + i) % SEQ_MOD,
                t_us=(i * t_step) % (T_US_MAX + 1),
                model_rad_s=value,
                raw_l=raw_l,
                raw_r=raw_r,
                state=state,
            )
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Print a constant-step WHL2 twin stream "
            "(SIM ONLY, signs UNCONFIRMED)."
        )
    )
    parser.add_argument("--step", type=float, required=True, help="model value in rad/s")
    parser.add_argument("--seconds", type=float, required=True, help="duration in seconds")
    parser.add_argument("--rate", type=float, default=50.0, help="lines per second (default 50)")
    parser.add_argument(
        "--sign-left",
        type=int,
        choices=[1, -1],
        default=1,
        help="sign applied to the left raw (default 1)",
    )
    parser.add_argument(
        "--sign-right",
        type=int,
        choices=[1, -1],
        default=-1,
        help="sign applied to the right raw (default -1)",
    )
    parser.add_argument(
        "--state",
        type=int,
        choices=list(STATES),
        default=0,
        help="state on every line: 0 run, 1 hold after a fall, 2 bus down (default 0)",
    )
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
            sign_l=args.sign_left,
            sign_r=args.sign_right,
            state=args.state,
        ):
            sys.stdout.write(line)
            sys.stdout.flush()
    except WhlError as exc:
        print(f"ctrl_wheels_msg: {exc}", file=sys.stderr)
        return 2
    except BrokenPipeError:
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
