#!/usr/bin/env python3
"""
purpose, issue #285, sub-issue of #258, and "no controller, no numpy"
converts roll, pitch and yaw to Webots axis-angle (x, y, z, angle). #291 and #286 will call it
"""

from __future__ import annotations

import math

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

