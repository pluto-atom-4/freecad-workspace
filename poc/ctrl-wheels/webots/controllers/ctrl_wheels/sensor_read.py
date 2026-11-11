#!/usr/bin/env python3
"""Pure sensor-readback module for the servo-wheel-dof Webots POC."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional, Union


class SensorReadError(ValueError):
    """Raised when a sensor readback or its rate computation is invalid."""
    pass


@dataclass(frozen=True)
class SensorReading:
    """Validated, timestamped sensor sample."""
    t_s: float
    angle_rad: float
    velocity_rad_s: Optional[float]


def _validate_finite(value: float, name: str) -> float:
    """Validate that value is a finite float, else raise SensorReadError."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise SensorReadError(f"{name} must be a real number, got {value!r}")
    if not math.isfinite(v):
        raise SensorReadError(f"{name} must be finite, got {v!r}")
    return v


def angular_velocity_from_positions(
    angle_prev_rad: float,
    angle_curr_rad: float,
    dt_s: float,
) -> float:
    """Compute angular velocity from two angle readings by finite difference."""
    angle_prev_rad = _validate_finite(angle_prev_rad, "angle_prev_rad")
    angle_curr_rad = _validate_finite(angle_curr_rad, "angle_curr_rad")
    dt_s = _validate_finite(dt_s, "dt_s")
    if not (dt_s > 0.0):
        raise SensorReadError(f"dt_s must be > 0, got {dt_s!r}")
    return (angle_curr_rad - angle_prev_rad) / dt_s


def read_sensor(
    t_s: float,
    angle_source: Union[float, Callable[[], float]],
    angle_prev_rad: Optional[float] = None,
    t_prev_s: Optional[float] = None,
) -> SensorReading:
    """Read and validate one angle sample with a finite-difference velocity."""
    raw_angle = angle_source() if callable(angle_source) else angle_source
    angle_rad = _validate_finite(raw_angle, "angle_source reading")
    t_s = _validate_finite(t_s, "t_s")

    if (angle_prev_rad is None) != (t_prev_s is None):
        raise SensorReadError(
            "angle_prev_rad and t_prev_s must both be provided or both be None"
        )

    if angle_prev_rad is None:
        return SensorReading(t_s=t_s, angle_rad=angle_rad, velocity_rad_s=None)

    t_prev_s = _validate_finite(t_prev_s, "t_prev_s")
    dt_s = t_s - t_prev_s
    velocity_rad_s = angular_velocity_from_positions(angle_prev_rad, angle_rad, dt_s)
    return SensorReading(t_s=t_s, angle_rad=angle_rad, velocity_rad_s=velocity_rad_s)
