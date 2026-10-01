#!/usr/bin/env python3
"""
Pure IMU + wheel_angle message module for the servo-wheel-dof POC (issue #282,
sub-issue of #258).

Creates, formats and validates the UDP JSON messages that drive the follower
controller: an IMU orientation (roll, pitch, yaw) plus a wheel angle. Contains
**no** Webots or serial dependency (must NOT `import controller` or `import
serial`), so it stays importable and unit-testable outside Webots.

COPIED (not imported) from esp32-dof. Fixes must be mirrored by hand:
  - poc/esp32-dof/webots/controllers/esp32_dof_follower/dof_webots_math.py
    (parse_orientation_msg)
  - poc/esp32-dof/monitor/dof_sources.py (mock_angles)

**Wire format (UDP JSON, one datagram per message):**
    {"seq": <uint>, "roll": f, "pitch": f, "yaw": f, "wheel_angle": f}
- All angles in radians; roll/pitch/yaw are ZYX Euler angles.
- `seq` and any extra keys are ignored on parse; all four angle keys are
  required.

Usage:
    from imu_wheel_msg import parse_imu_wheel_msg, mock_imu_wheel, format_imu_wheel_msg

    sample = mock_imu_wheel(1.25)
    data = format_imu_wheel_msg(seq=0, msg=sample)
    assert parse_imu_wheel_msg(data) == sample
"""


from __future__ import annotations

import json
import math
from dataclasses import dataclass

# Mock amplitudes (rad) and frequencies (Hz); same style as dof_sources.py.
_ROLL_AMP, _ROLL_HZ = 0.5, 0.2
_PITCH_AMP, _PITCH_HZ = 0.3, 0.1
_YAW_AMP, _YAW_HZ = 0.8, 0.05
_WHEEL_AMP, _WHEEL_HZ = 1.5, 0.25

# Required angle keys, in wire order.
_KEYS: tuple[str, ...] = ("roll", "pitch", "yaw", "wheel_angle")

@dataclass(frozen=True)
class ImuWheel:
    """IMU orientation plus wheel angle, all in radians."""

    roll: float
    pitch: float
    yaw: float
    wheel_angle: float

def _reject_constant(name: str) -> None:
    """json.loads hook: reject the NaN / Infinity / -Infinity literals."""
    raise ValueError(f"non-finite literal: {name}")

def parse_imu_wheel_msg(data: bytes) -> ImuWheel | None:
    """
    Parse a UDP JSON datagram into an ImuWheel.

    Args:
        data: Byte string containing the JSON payload.

    Returns:
        ImuWheel, or None if parsing or validation fails.
        - Ints and floats are accepted; strings, null, lists, objects and bool
          are rejected.
        - Extra keys and `seq` are ignored.
        - NaN, +-Infinity and values that overflow to infinity are rejected.
        - All four angle keys are required.

    Raises:
        None (every error returns None).
    """
    try:
        obj = json.loads(data, parse_constant=_reject_constant)
    except (ValueError, TypeError, RecursionError):
        return None

    if not isinstance(obj, dict):
        return None

    values: list[float] = []
    for key in _KEYS:
        v = obj.get(key)  # missing key -> None -> fails the type check below
        # bool is a subclass of int in Python, so reject it first.
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            return None
        try:
            f = float(v)
        except (OverflowError, ValueError):  # e.g. a 400-digit integer
            return None
        if not math.isfinite(f):  # e.g. 1e999 parses to inf
            return None
        values.append(f)

    return ImuWheel(*values)


def mock_imu_wheel(t_s: float) -> ImuWheel:
    """
    Deterministic mock sample: AMP * sin(2*pi*HZ*t) for each field.

    Args:
        t_s: Elapsed time in seconds.

    Returns:
        ImuWheel with roll 0.5 @ 0.2 Hz, pitch 0.3 @ 0.1 Hz, yaw 0.8 @ 0.05 Hz
        and wheel_angle 1.5 @ 0.25 Hz (radians).
    """
    two_pi_t = 2.0 * math.pi * t_s
    return ImuWheel(
        roll=_ROLL_AMP * math.sin(two_pi_t * _ROLL_HZ),
        pitch=_PITCH_AMP * math.sin(two_pi_t * _PITCH_HZ),
        yaw=_YAW_AMP * math.sin(two_pi_t * _YAW_HZ),
        wheel_angle=_WHEEL_AMP * math.sin(two_pi_t * _WHEEL_HZ),
    )


def format_imu_wheel_msg(seq: int, msg: ImuWheel) -> bytes:
    """
    Encode an ImuWheel as a UTF-8 JSON datagram.

    Args:
        seq: Sender sequence number (non-negative int; not validated).
        msg: The sample to encode.

    Returns:
        UTF-8 JSON bytes with keys seq, roll, pitch, yaw, wheel_angle.
    """
    payload = {
        "seq": seq,
        "roll": msg.roll,
        "pitch": msg.pitch,
        "yaw": msg.yaw,
        "wheel_angle": msg.wheel_angle,
    }
    return json.dumps(payload).encode("utf-8")