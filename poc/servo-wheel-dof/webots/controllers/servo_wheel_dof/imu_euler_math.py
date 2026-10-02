#!/usr/bin/env python3
"""
Pure Euler-to-axis-angle conversion for the servo-wheel-dof POC (issue #285,
sub-issue of #258).

Converts roll, pitch and yaw (ZYX, radians) to Webots axis-angle
(x, y, z, angle). Used by #291 (upright pose) and #286 (follower controller).
Contains no Webots, serial or numpy dependency (must NOT `import controller`).

COPIED (not imported) from esp32-dof. Fixes must be mirrored by hand:
  - poc/esp32-dof/webots/controllers/esp32_dof_follower/dof_webots_math.py
    (euler_to_axis_angle)

Convention: R = Rz(yaw) * Ry(pitch) * Rx(roll); angle in [0, pi] (w >= 0
canonical form); a zero or near-zero rotation returns (0.0, 0.0, 1.0, 0.0).

Usage:
    from imu_euler_math import euler_to_axis_angle

    x, y, z, angle = euler_to_axis_angle(roll, pitch, yaw)
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

