#!/usr/bin/env python3
"""
ESP32 6-DOF IMU frame protocol parser and formatter.

Frame format (ASCII, 50 Hz):
    IMU,<seq>,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>*<HH>\\n

Protocol details:
- seq: uint16 sequence number, wraps at 65536
- t_us: uint32 device microseconds (capped at 2^32-1)
- ax, ay, az: accelerometer in g
- gx, gy, gz: gyroscope in deg/s
- HH: 2-digit UPPERCASE hex XOR checksum of body (IMU to *)
- Floats formatted as %.6f (fixed, no exponent); "-0.000000" normalized to "0.000000"
- Line terminator: \\n (receiver also accepts \\r\\n)

Usage:
    from dof_frame import Frame, FrameError, parse_line, format_frame

    # Parse an incoming line
    try:
        frame = parse_line("IMU,0,1000000,0.100000,-9.806650,0.050000,1.5,2.5,3.5*XX\\n")
        print(f"seq={frame.seq}, t_us={frame.t_us}, accel={frame.accel}")
    except FrameError as e:
        print(f"Parse error: {e}")

    # Format a frame
    frame = Frame(seq=0, t_us=1000000, accel=(0.1, -9.80665, 0.05), gyro=(1.5, 2.5, 3.5))
    line = format_frame(frame)
    print(line)
"""

from dataclasses import dataclass
import math


PREFIX = "IMU"
SEQ_MOD = 65536
T_US_MAX = 2**32 - 1


class FrameError(ValueError):
    """Raised when frame parsing or formatting fails."""
    pass


@dataclass(frozen=True)
class Frame:
    """Parsed IMU frame."""
    seq: int
    t_us: int
    accel: tuple  # (ax, ay, az) in g
    gyro: tuple   # (gx, gy, gz) in deg/s


def checksum(body: str) -> int:
    """
    Compute XOR checksum of frame body (IMU prefix through * exclusive).

    Args:
        body: ASCII string to checksum

    Returns:
        int: XOR of all character ord values

    Raises:
        FrameError: if body contains non-ASCII characters
    """
    result = 0
    for c in body:
        if not c.isascii():
            raise FrameError(f"non-ASCII character in body: {c!r}")
        result ^= ord(c)
    return result


def parse_line(line: str) -> Frame:
    """
    Parse a single frame line.

    The parser is lenient for float tokens: it accepts any finite decimal representation
    that Python's float() recognizes, including exponent notation (1e-3, 2E+5) and leading
    '+' signs (+2.5). It rejects embedded whitespace, underscores, and non-finite values
    (nan, inf).

    Note: The *sender* (firmware / format_frame) emits floats with fixed-point %.6f format
    only; the receiver here is lenient for compatibility and robustness during validation.

    Args:
        line: Raw frame line (may contain trailing whitespace)

    Returns:
        Frame: Parsed frame object

    Raises:
        FrameError: if parsing fails (bad prefix, field count, checksum, etc.)
    """
    # Remove only trailing \r\n, not all whitespace
    line = line.rstrip("\r\n")

    # Find checksum marker
    star_idx = line.rfind("*")
    if star_idx == -1:
        raise FrameError("missing checksum marker *")

    body = line[:star_idx]
    hh_str = line[star_idx + 1:]

    # Validate checksum format
    if len(hh_str) != 2:
        raise FrameError("checksum must be exactly 2 characters")

    for c in hh_str:
        if c not in "0123456789ABCDEF":
            raise FrameError("checksum must be uppercase hex (0-9A-F)")

    # Verify checksum
    computed = checksum(body)
    expected = int(hh_str, 16)
    if computed != expected:
        raise FrameError(f"checksum mismatch: computed {computed:02X}, expected {hh_str}")

    # Parse fields
    fields = body.split(",")
    if len(fields) != 9:
        raise FrameError(f"expected 9 fields, got {len(fields)}")

    if fields[0] != PREFIX:
        raise FrameError(f"bad prefix: expected {PREFIX!r}, got {fields[0]!r}")

    # Parse seq
    seq_str = fields[1]
    if not (seq_str.isascii() and seq_str.isdigit()):
        raise FrameError(f"seq must be decimal digits, got {seq_str!r}")
    seq = int(seq_str)
    if seq >= SEQ_MOD:
        raise FrameError(f"seq out of range [0, {SEQ_MOD - 1}], got {seq}")

    # Parse t_us
    t_us_str = fields[2]
    if not (t_us_str.isascii() and t_us_str.isdigit()):
        raise FrameError(f"t_us must be decimal digits, got {t_us_str!r}")
    t_us = int(t_us_str)
    if t_us > T_US_MAX:
        raise FrameError(f"t_us out of range [0, {T_US_MAX}], got {t_us}")

    # Parse floats (accel and gyro)
    float_fields = fields[3:9]
    values = []
    for i, tok in enumerate(float_fields):
        # Reject any whitespace and underscores
        if any(c.isspace() for c in tok):
            raise FrameError(f"field {i + 3} has embedded whitespace: {tok!r}")
        if "_" in tok:
            raise FrameError(f"field {i + 3} contains underscore: {tok!r}")

        try:
            val = float(tok)
        except ValueError:
            raise FrameError(f"field {i + 3} is not a valid float: {tok!r}")

        if not math.isfinite(val):
            raise FrameError(f"field {i + 3} is not finite: {tok!r}")

        values.append(val)

    accel = tuple(values[0:3])
    gyro = tuple(values[3:6])

    return Frame(seq=seq, t_us=t_us, accel=accel, gyro=gyro)


def format_frame(f: Frame) -> str:
    """
    Format a Frame into a protocol line.

    Args:
        f: Frame to format

    Returns:
        str: Protocol line with checksum and newline terminator

    Raises:
        FrameError: if frame values are invalid
    """
    # Validate seq
    if not (0 <= f.seq < SEQ_MOD):
        raise FrameError(f"seq out of range [0, {SEQ_MOD - 1}], got {f.seq}")

    # Validate t_us
    if not (0 <= f.t_us <= T_US_MAX):
        raise FrameError(f"t_us out of range [0, {T_US_MAX}], got {f.t_us}")

    # Validate floats are finite
    for val in (*f.accel, *f.gyro):
        if not math.isfinite(val):
            raise FrameError(f"non-finite float: {val}")

    # Format floats with %.6f
    formatted_floats = []
    for val in (*f.accel, *f.gyro):
        formatted = f"{val:.6f}"
        # Normalize "-0.000000" to "0.000000"
        if formatted == "-0.000000":
            formatted = "0.000000"
        formatted_floats.append(formatted)

    # Build body
    body = f"{PREFIX},{f.seq},{f.t_us}," + ",".join(formatted_floats)

    # Compute checksum
    csum = checksum(body)

    return f"{body}*{csum:02X}\n"
