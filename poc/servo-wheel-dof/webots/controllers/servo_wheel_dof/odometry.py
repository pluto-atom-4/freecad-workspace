#!/usr/bin/env python3
"""
Kinematic odometry for the servo-wheel-dof POC (issue #321, sub-issue of #301).

Pure step function: turns the change of the COMMANDED wheel_angle (radians) into a
world-frame offset of the wheel centre, assuming no-slip rolling on the floor.
Pure Python: only `math` and `dataclasses`; no Webots, serial or numpy dependency
(must NOT `import controller`).

Conventions:
- wheel_angle is unbounded radians (no wrap). The delta between two calls is
  applied literally, so a jump of 10 rad moves the robot 10 * radius.
- Positive wheel_angle means the wheel top turns toward -X, i.e. the robot rolls
  toward -X (west) at yaw 0. Negative wheel_angle rolls it toward +X (east).
- rim_direction(yaw) = (-cos(yaw), -sin(yaw)) is the unit horizontal vector the
  robot travels along for a positive delta. It is perpendicular to the hinge
  axis (sin(yaw), -cos(yaw)) from upright_pose.
- OdomState is immutable. x, y are the world-frame offset in metres of the wheel
  centre from its start point (0, -0.026). prev_angle is the last accepted
  wheel_angle (None before the first message).
- The first message never moves the robot: it only records prev_angle.
- Non-finite wheel_angle or yaw: the state is returned unchanged, prev_angle
  included, so the delta accumulates over the gap and is applied along the
  next finite yaw.

Usage:
    from odometry import OdomState, step_odometry

    state = OdomState()
    state = step_odometry(state, wheel_angle, yaw)
    # state.x, state.y: offset of the wheel centre (m)
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from upright_pose import WHEEL_RADIUS  # noqa: E402


@dataclass(frozen=True)
class OdomState:
    """Odometry state: offset (m) from the start point and last wheel angle."""

    x: float = 0.0
    y: float = 0.0
    prev_angle: float | None = None


def rim_direction(yaw: float) -> tuple[float, float]:
    """
    Unit horizontal vector along the rim for a positive wheel_angle delta.

    Args:
        yaw: Robot heading about Z (radians).

    Returns:
        (-cos(yaw), -sin(yaw)). At yaw 0 this is (-1, 0), i.e. toward -X.
    """
    return (-math.cos(yaw), -math.sin(yaw))


def step_odometry(
    state: OdomState,
    wheel_angle: float,
    yaw: float,
    *,
    radius: float = WHEEL_RADIUS,
) -> OdomState:
    """
    Advance the odometry by one wheel_angle sample.

    Args:
        state: Current state (not modified).
        wheel_angle: Commanded wheel angle (radians, unbounded).
        yaw: Robot heading about Z (radians).
        radius: Wheel radius (m).

    Returns:
        The new OdomState. If wheel_angle or yaw is not finite the same state
        is returned. On the first finite call only prev_angle is set.
    """
    if not (math.isfinite(wheel_angle) and math.isfinite(yaw)):
        return state
    if state.prev_angle is None:
        return OdomState(state.x, state.y, wheel_angle)

    delta = wheel_angle - state.prev_angle
    ux, uy = rim_direction(yaw)
    x = state.x + radius * delta * ux
    y = state.y + radius * delta * uy
    # Later issues (#322/#323): clamp x, y to the floor box right here,
    # before building the new state.
    return OdomState(x, y, wheel_angle)
