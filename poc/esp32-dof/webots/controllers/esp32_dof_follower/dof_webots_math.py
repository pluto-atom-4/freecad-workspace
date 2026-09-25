#!/usr/bin/env python3
"""
Webots-side orientation parsing and Euler-to-axis-angle conversion.

**IPC Contract (monitor → Webots controller):**
- Protocol: UDP over 127.0.0.1:5005
- Payload: One JSON datagram per UDP packet
- Format: {"seq": <uint>, "roll": <float>, "pitch": <float>, "yaw": <float>}
- Units: All angles in radians
- Convention: ZYX Euler angles; seq is ignored by this module

**Rotation Convention:**
- ZYX: Composed as R = Rz(yaw) * Ry(pitch) * Rx(roll)
  - Roll: rotation about X-axis (±π)
  - Pitch: rotation about Y-axis [−π/2, π/2]
  - Yaw: rotation about Z-axis (−π, π]
- Webots rotation: (x, y, z, angle) unit axis-angle representation
  - Output of euler_to_axis_angle() is compatible with Webots Node.rotation field

Usage:
    from dof_webots_math import parse_orientation_msg, euler_to_axis_angle

    # Parse incoming UDP datagram
    data = b'{"seq": 12, "roll": 0.10, "pitch": -0.05, "yaw": 0.0}'
    roll, pitch, yaw = parse_orientation_msg(data)
    if roll is None:
        print("Invalid message")
    else:
        # Convert to Webots axis-angle
        x, y, z, angle = euler_to_axis_angle(roll, pitch, yaw)
        # Use in: node.rotation = [x, y, z, angle]
"""

from __future__ import annotations

import json
import math
from typing import Optional


def parse_orientation_msg(data: bytes) -> Optional[tuple[float, float, float]]:
    """
    Parse a UDP JSON datagram to extract roll, pitch, yaw Euler angles.

    Args:
        data: Byte string containing JSON payload.

    Returns:
        Tuple (roll, pitch, yaw) in radians, or None if parsing/validation fails.
        - Accepts ints and floats for angle values; strings, null, bool rejected.
        - Extra keys in the JSON are ignored.
        - seq key is optional (ignored if present).
        - Rejects NaN, ±Infinity, and values that overflow to infinity.

    Raises:
        None (all errors return None).
    """

    def _reject_constant(name: str) -> None:
        """Reject NaN/Infinity during JSON parsing."""
        raise ValueError(f"non-finite literal: {name}")

    # Parse JSON with rejection of NaN/Infinity literals
    try:
        obj = json.loads(data, parse_constant=_reject_constant)
    except (ValueError, TypeError, RecursionError):
        return None

    # Top-level must be a dict
    if not isinstance(obj, dict):
        return None

    # Extract roll, pitch, yaw with type checking
    roll_val = None
    pitch_val = None
    yaw_val = None

    for key, var_name in [("roll", "roll"), ("pitch", "pitch"), ("yaw", "yaw")]:
        v = obj.get(key)

        # Reject bool (which is a subclass of int in Python)
        if isinstance(v, bool):
            return None

        # Accept int or float only
        if not isinstance(v, (int, float)):
            return None

        # Convert to float
        try:
            f = float(v)
        except (OverflowError, ValueError):
            return None

        # Reject non-finite values
        if not math.isfinite(f):
            return None

        if var_name == "roll":
            roll_val = f
        elif var_name == "pitch":
            pitch_val = f
        elif var_name == "yaw":
            yaw_val = f

    # All three values must have been present
    if roll_val is None or pitch_val is None or yaw_val is None:
        return None

    return (roll_val, pitch_val, yaw_val)


def euler_to_axis_angle(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    """
    Convert ZYX Euler angles to axis-angle representation (Webots-compatible).

    Args:
        roll: Rotation about X-axis (radians).
        pitch: Rotation about Y-axis (radians).
        yaw: Rotation about Z-axis (radians).

    Returns:
        Tuple (x, y, z, angle) representing a unit axis and rotation angle (radians).
        - Axis is unit-length (within numerical precision).
        - Angle is in range [0, π] for a normalized quaternion.
        - Special case: zero rotation returns (0, 0, 1, 0).

    Notes:
        - No input wrapping or clamping applied.
        - Internally uses quaternion representation: q = qz(yaw) * qy(pitch) * qx(roll).
        - Quaternion is normalized to ensure angle ∈ [0, π] (w ≥ 0).
        - Uses stable atan2 for angle computation; avoids acos to prevent numerical instability.
    """
    # Half angles
    hr = roll / 2.0
    hp = pitch / 2.0
    hy = yaw / 2.0

    # Precompute cos/sin of half angles
    cr = math.cos(hr)
    sr = math.sin(hr)
    cp = math.cos(hp)
    sp = math.sin(hp)
    cy = math.cos(hy)
    sy = math.sin(hy)

    # Quaternion: q = qz(yaw) * qy(pitch) * qx(roll)
    # q = (w, x, y, z)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    # Normalize to ensure w >= 0 (angle ∈ [0, π])
    if w < 0.0:
        w = -w
        x = -x
        y = -y
        z = -z

    # Compute vector magnitude (sine of half-angle)
    s = math.sqrt(x * x + y * y + z * z)

    # Handle zero rotation case
    if s < 1e-12:
        return (0.0, 0.0, 1.0, 0.0)

    # Compute angle using stable atan2; normalize axis
    angle = 2.0 * math.atan2(s, w)
    axis_x = x / s
    axis_y = y / s
    axis_z = z / s

    return (axis_x, axis_y, axis_z, angle)
