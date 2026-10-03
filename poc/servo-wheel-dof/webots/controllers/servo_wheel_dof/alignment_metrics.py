#!/usr/bin/env python3
"""
Alignment metrics for the servo-wheel-dof POC (issue #304, sub-issue of #303).

Pure function that measures how far the wheel centre is from the hinge anchor
and how well the wheel's local Z axis lines up with the Robot's local Z axis.
Used by the opt-in telemetry stream (#303). Pure Python: only `math`; no Webots,
serial or numpy dependency (must NOT `import controller`).

Conventions:
- Positions are 3 floats (x, y, z) in metres.
- Orientations are 9 floats, row-major 3x3, exactly as Webots
  Node.getOrientation() returns them, so column k is o[k], o[k+3], o[k+6].
- Anchor (world) = robot_pos + robot_column2 * anchor_offset, which is the
  hinge point Robot-local (0, 0, anchor_offset).
- deviation_m = distance from the wheel centre to the anchor.
- axis_dot = dot(wheel column 2, Robot column 2), clamped to [-1, 1].
- Non-finite input (or output) never raises: the three metrics become NaN and
  warn is True. Wrong sequence lengths raise ValueError.
- relative_position (issue #312) gives the wheel centre relative to the Robot in
  the Robot's local frame: rel_k = dot(Robot column k, wheel_pos - robot_pos).
  Healthy value is (0, 0, anchor_offset). Non-finite input gives three NaN.

Usage:
    from alignment_metrics import anchor_metrics

    m = anchor_metrics(robot_pos, robot_ori, wheel_pos, wheel_ori)
    if m.warn:
        print("WARN", m.deviation_m, m.axis_dot)
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

sys.path.insert(0, str(Path(__file__).resolve().parent))

from upright_pose import WHEEL_OFFSET  # noqa: E402

DEV_WARN_M = 0.001  # anchor deviation warning threshold (m)
DOT_WARN = 0.9999  # axis alignment warning threshold (dot product)


@dataclass(frozen=True)
class AlignmentMetrics:
    """Wheel alignment result for one simulation step."""

    deviation_m: float
    axis_dot: float
    wheel_z: float
    warn: bool


def _floats(name: str, values: Sequence[float], n: int) -> list[float]:
    """Check the length and return the values as a list of floats."""
    if len(values) != n:
        raise ValueError(f"{name} must have {n} values, got {len(values)}")
    return [float(v) for v in values]


def anchor_metrics(
    robot_pos: Sequence[float],
    robot_ori: Sequence[float],
    wheel_pos: Sequence[float],
    wheel_ori: Sequence[float],
    *,
    anchor_offset: float = WHEEL_OFFSET,
    dev_warn_m: float = DEV_WARN_M,
    dot_warn: float = DOT_WARN,
) -> AlignmentMetrics:
    """
    Measure the wheel's deviation from the hinge anchor and its axis alignment.

    Args:
        robot_pos: Robot world position (x, y, z), 3 floats (m).
        robot_ori: Robot world orientation, 9 floats row-major.
        wheel_pos: Wheel centre world position (x, y, z), 3 floats (m).
        wheel_ori: Wheel world orientation, 9 floats row-major.
        anchor_offset: Hinge anchor distance along Robot-local Z (m).
        dev_warn_m: Warn when deviation_m is greater than this (m).
        dot_warn: Warn when axis_dot is less than this.

    Returns:
        AlignmentMetrics(deviation_m, axis_dot, wheel_z, warn). If any input or
        computed value is non-finite, the three metrics are NaN and warn is True.

    Raises:
        ValueError: A position is not 3 values or an orientation is not 9.
    """
    rp = _floats("robot_pos", robot_pos, 3)
    ro = _floats("robot_ori", robot_ori, 9)
    wp = _floats("wheel_pos", wheel_pos, 3)
    wo = _floats("wheel_ori", wheel_ori, 9)

    nan = float("nan")
    bad = AlignmentMetrics(nan, nan, nan, True)
    if not all(math.isfinite(v) for v in rp + ro + wp + wo):
        return bad
    if not math.isfinite(anchor_offset):
        return bad

    anchor = [rp[i] + ro[3 * i + 2] * anchor_offset for i in range(3)]
    deviation = math.hypot(*(wp[i] - anchor[i] for i in range(3)))

    dot = sum(wo[3 * i + 2] * ro[3 * i + 2] for i in range(3))
    if math.isfinite(dot):
        dot = max(-1.0, min(1.0, dot))

    wheel_z = wp[2]
    if not all(math.isfinite(v) for v in (deviation, dot, wheel_z)):
        return bad

    warn = bool(deviation > dev_warn_m or dot < dot_warn)
    return AlignmentMetrics(deviation, dot, wheel_z, warn)


def relative_position(
    robot_pos: Sequence[float],
    robot_ori: Sequence[float],
    wheel_pos: Sequence[float],
) -> tuple[float, float, float]:
    """
    Wheel centre position relative to the Robot, in the Robot's local frame.

    Applies the transpose of the row-major Robot rotation to the world offset
    (wheel_pos - robot_pos): rel_k = sum_i robot_ori[3*i + k] * offset[i], that
    is the dot product of Robot column k with the offset. Healthy value is
    (0, 0, anchor_offset); x/y show radial drift, z shows axial drift.

    Args:
        robot_pos: Robot world position (x, y, z), 3 floats (m).
        robot_ori: Robot world orientation, 9 floats row-major.
        wheel_pos: Wheel centre world position (x, y, z), 3 floats (m).

    Returns:
        (rel_x, rel_y, rel_z) in metres. If any input or computed value is
        non-finite, all three are NaN.

    Raises:
        ValueError: A position is not 3 values or the orientation is not 9.
    """
    rp = _floats("robot_pos", robot_pos, 3)
    ro = _floats("robot_ori", robot_ori, 9)
    wp = _floats("wheel_pos", wheel_pos, 3)

    nan = float("nan")
    bad = (nan, nan, nan)
    if not all(math.isfinite(v) for v in rp + ro + wp):
        return bad

    d = [wp[i] - rp[i] for i in range(3)]
    rel_x, rel_y, rel_z = (
        sum(ro[3 * i + k] * d[i] for i in range(3)) for k in range(3)
    )
    if not all(math.isfinite(v) for v in (rel_x, rel_y, rel_z)):
        return bad
    return (rel_x, rel_y, rel_z)
