#!/usr/bin/env python3
"""
Upright pose for the pitch+yaw follower in the servo-wheel-dof POC (issue #291,
sub-issue of #258).

Given IMU pitch and yaw (radians) returns the Webots Robot translation and
rotation that keep the wheel CENTRE fixed in the world while the wheel rim stays
on the floor. IMU roll is ignored by design. Pure Python: no Webots, serial or
numpy dependency (must NOT `import controller`).

NEW logic (not copied from esp32-dof), built on euler_to_axis_angle (issue #285).

Convention: rotation R = Rz(yaw) * Ry(pitch) * Rx(pi/2); the Rx(+90 deg) is the
base rotation of the world file (Robot-local hinge axis Z maps to world -Y).
The wheel centre is the Robot-local point a = (0, 0, wheel_offset), so it sits at
T + R*a in the world and is pinned at c0 = (0, -wheel_offset, radius + gap):
    T = c0 - R*a = (-wo*sin(yaw), -wo + wo*cos(yaw), radius + gap)
Pitch cancels: R*ez = (sin(yaw), -cos(yaw), 0) does not depend on pitch (the
wheel centre lies on the pitch axis), and the hinge axis stays horizontal, so
the lowest rim point is centre_z - radius = gap.

Usage:
    from upright_pose import upright_pose

    translation, axis_angle = upright_pose(pitch, yaw)
    robot.getField("translation").setSFVec3f(list(translation))
    robot.getField("rotation").setSFRotation(list(axis_angle))
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from imu_euler_math import euler_to_axis_angle  # noqa: E402

BASE_ROLL = math.pi / 2  # world file Robot rotation: 1 0 0 pi/2
WHEEL_RADIUS = 0.03  # Cylinder radius in servo_wheel_dof.wbt (m)
WHEEL_OFFSET = 0.026  # wheel center along Robot-local Z (m)


def upright_pose(
    pitch: float,
    yaw: float,
    *,
    radius: float = WHEEL_RADIUS,
    wheel_offset: float = WHEEL_OFFSET,
    gap: float = 0.0,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """
    Compute the Robot pose that keeps the wheel centre fixed.

    Args:
        pitch: IMU pitch about Y (radians), any real value.
        yaw: IMU yaw about Z (radians), any real value.
        radius: Wheel radius (m).
        wheel_offset: Distance from the Robot origin to the wheel centre along
            Robot-local Z (m).
        gap: Clearance between the lowest rim point and the floor (m).

    Returns:
        (translation, axis_angle): translation is (x, y, z) in metres and
        axis_angle is (x, y, z, angle) for Webots setSFRotation.

    Notes:
        - Roll is not a parameter; the base Rx(+90 deg) is used as the roll.
        - No validation: NaN or inf inputs propagate to the output.
    """
    axis_angle = euler_to_axis_angle(BASE_ROLL, pitch, yaw)

    # T = c0 - R*a, with R*a = wheel_offset * (sin(yaw), -cos(yaw), 0).
    # Pitch does not appear: Ry leaves the hinge axis (world -Y at roll 0) alone.
    tx = -wheel_offset * math.sin(yaw)
    ty = -wheel_offset + wheel_offset * math.cos(yaw)
    tz = radius + gap

    return (tx, ty, tz), axis_angle
