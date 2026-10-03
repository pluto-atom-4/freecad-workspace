#!/usr/bin/env python3
"""
Kinematic odometry for the servo-wheel-dof POC (issues #321/#322, sub-issues of #301).

Pure step function: turns the change of the COMMANDED wheel_angle (radians) into a
world-frame offset of the wheel centre, assuming no-slip rolling on the floor.
Pure Python: only `math`, `dataclasses` and `typing`; no Webots, serial or numpy
dependency
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
  and clamped included, so the delta accumulates over the gap and is applied
  along the next finite yaw.
- Floor clamp (issue #322): the floor is 2 m x 2 m centred at the origin and
  OFFSET_LIMIT_M = 0.9 keeps the wheel centre inside a +-0.9 m box. With
  step_odometry(..., limit=L) x and y are clamped independently into
  [-L, +L] (a box, not a circle). OdomState.clamped is True only if THIS call
  clamped a component; landing exactly on the limit is not clamped. limit=None
  never clamps. A limit that is not finite or <= 0 raises ValueError, checked
  first, before any other argument.
- No windup: the clamped x, y are stored and prev_angle still follows the
  commanded wheel_angle, so reversing the angle moves the robot back at once.
- apply_offset(translation, state) adds (state.x, state.y) to the x, y of an
  upright_pose translation; z is returned untouched, so rim contact holds.
- Opt-in (issue #323): parse_odometry_env reads SWD_ODOMETRY (on: 1,true,yes,on;
  off: empty,0,false,no,off; case-insensitive, stripped; anything else is off
  plus exactly one warning). A missing key or a None value is off, no warning.
  Same rules as telemetry.parse_telemetry_env (not imported). Never raises.

Usage:
    from odometry import OFFSET_LIMIT_M, OdomState, apply_offset, step_odometry

    state = OdomState()
    state = step_odometry(state, wheel_angle, yaw, limit=OFFSET_LIMIT_M)
    # state.x, state.y: offset of the wheel centre (m); state.clamped: hit edge
    translation = apply_offset(upright_translation, state)
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

sys.path.insert(0, str(Path(__file__).resolve().parent))

from upright_pose import WHEEL_RADIUS  # noqa: E402

OFFSET_LIMIT_M = 0.9  # half-width of the allowed box on the 2 m x 2 m floor (m)

ENV_ENABLE = "SWD_ODOMETRY"

_ON = frozenset(("1", "true", "yes", "on"))
_OFF = frozenset(("", "0", "false", "no", "off"))


@dataclass(frozen=True)
class OdomState:
    """Odometry state: offset (m) from the start point and last wheel angle."""

    x: float = 0.0
    y: float = 0.0
    prev_angle: float | None = None
    clamped: bool = False  # True iff the last step_odometry call clamped x or y


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
    limit: float | None = None,
) -> OdomState:
    """
    Advance the odometry by one wheel_angle sample.

    Args:
        state: Current state (not modified).
        wheel_angle: Commanded wheel angle (radians, unbounded).
        yaw: Robot heading about Z (radians).
        radius: Wheel radius (m).
        limit: Box half-width (m). None disables the clamp. Otherwise x and y
            are clamped independently into [-limit, +limit].

    Returns:
        The new OdomState. If wheel_angle or yaw is not finite the same state
        is returned (clamped included). On the first finite call only
        prev_angle is set. clamped is True iff this call clamped x or y.

    Raises:
        ValueError: limit is not None and is not finite or is <= 0. Checked
            before anything else, also on the first call and for bad angles.
    """
    if limit is not None and not (math.isfinite(limit) and limit > 0):
        raise ValueError(f"limit must be a finite number > 0 or None, got {limit!r}")
    if not (math.isfinite(wheel_angle) and math.isfinite(yaw)):
        return state
    if state.prev_angle is None:
        return OdomState(state.x, state.y, wheel_angle)

    delta = wheel_angle - state.prev_angle
    ux, uy = rim_direction(yaw)
    x = state.x + radius * delta * ux
    y = state.y + radius * delta * uy
    if limit is None:
        return OdomState(x, y, wheel_angle)
    cx = min(max(x, -limit), limit)
    cy = min(max(y, -limit), limit)
    # Compare after clamping: landing exactly on the limit is not "clamped".
    return OdomState(cx, cy, wheel_angle, clamped=(cx != x or cy != y))


def apply_offset(
    translation: tuple[float, float, float], state: OdomState
) -> tuple[float, float, float]:
    """
    Add the odometry offset to an upright_pose translation.

    Args:
        translation: (tx, ty, tz) from upright_pose (metres).
        state: Odometry state; only x and y are used.

    Returns:
        (tx + state.x, ty + state.y, tz). z is the same value, untouched, so
        the lowest rim point stays on the floor.
    """
    tx, ty, tz = translation
    return (tx + state.x, ty + state.y, tz)


@dataclass(frozen=True)
class OdometryConfig:
    """Result of parse_odometry_env."""

    enabled: bool
    warnings: tuple[str, ...]


def parse_odometry_env(environ: Mapping[str, str]) -> OdometryConfig:
    """Read SWD_ODOMETRY from a mapping; never raises, junk means off + warning."""
    raw = environ.get(ENV_ENABLE)
    text = "" if raw is None else str(raw).strip().lower()
    if text in _ON:
        return OdometryConfig(True, ())
    if text in _OFF:
        return OdometryConfig(False, ())
    return OdometryConfig(
        False,
        (
            f"{ENV_ENABLE}={raw!r} is not one of 1/true/yes/on or "
            "0/false/no/off; odometry stays off",
        ),
    )
