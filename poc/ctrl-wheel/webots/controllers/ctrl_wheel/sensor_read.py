#!/usr/bin/env python3
"""
Pure sensor-readback module for the servo-wheel-dof Webots POC (issue #263).

Processes raw PositionSensor angle readings (radians, per Webots convention) into
validated (time, angle, angular-velocity) samples. Contains **no** Webots dependency
(must NOT `import controller`) so it stays importable and unit-testable outside Webots,
per this repo's dependency-injection convention (see poc/esp32-dof/webots/controllers/
esp32_dof_follower/dof_webots_math.py and poc/esp32-dof/CLAUDE.md's "Conventions"
section: math/logic modules stay controller-free; only the top-level Webots controller
script imports `controller`).

**Units:**
- Angle: radians, as returned directly by Webots' PositionSensor.getValue() (no unit
  conversion needed here, unlike a raw ADC/gyroscope reading).
- Time / dt: seconds, float.
- Angular velocity: radians/second, computed as a simple finite difference between two
  successive angle readings -- Webots' PositionSensor has no companion velocity sensor,
  so this is the only way to obtain a rate from it in this POC.

**Validation:**
- Readings must be finite (no NaN/+-inf). Non-finite input raises SensorReadError,
  mirroring poc/esp32-dof/monitor/dof_frame.py's FrameError pattern (a dedicated
  ValueError subclass) rather than silently substituting a default -- a bad readback
  here should surface immediately in the calling controller/tests, not be masked.
- dt_s must be > 0 to compute a velocity; dt_s <= 0 raises SensorReadError (mirrors
  poc/esp32-dof/monitor/dof_stats.py's guard against a non-positive time span, but
  raises instead of silently returning 0.0 -- an issue #265 controller calling this
  every timestep should never legitimately see dt <= 0, so treat it as a bug signal).

Usage:
    from sensor_read import angular_velocity_from_positions, read_sensor

    # Low-level: compute velocity from two raw angle samples
    velocity_rad_s = angular_velocity_from_positions(angle_prev_rad=0.10,
                                                       angle_curr_rad=0.12,
                                                       dt_s=0.02)

    # High-level: package a validated (t, angle, velocity) sample, injecting the raw
    # angle source as a callable or plain number (dependency-injection pattern, so no
    # running Webots instance / PositionSensor is required to exercise this):
    reading = read_sensor(t_s=1.02, angle_source=lambda: sensor.getValue(),
                           angle_prev_rad=0.10, t_prev_s=1.00)
    print(reading.angle_rad, reading.velocity_rad_s)
"""

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
    velocity_rad_s: Optional[float]  # None if no previous sample was available


def _validate_finite(value: float, name: str) -> float:
    """
    Validate that `value` is a finite float (not NaN/+-inf, not a non-numeric type).

    Args:
        value: Value to validate.
        name: Field name, used only in the error message.

    Returns:
        float: `value`, unchanged.

    Raises:
        SensorReadError: if `value` is not a finite real number.
    """
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
    """
    Compute angular velocity from two successive angle readings (finite difference).

    Webots' PositionSensor has no companion velocity sensor, so this is the standard
    way to derive a rate from it: velocity = (angle_curr - angle_prev) / dt.

    Args:
        angle_prev_rad: Previous angle reading, in radians.
        angle_curr_rad: Current angle reading, in radians.
        dt_s: Elapsed time between the two readings, in seconds. Must be > 0.

    Returns:
        float: Angular velocity in radians/second.

    Raises:
        SensorReadError: if either angle is non-finite, or dt_s is non-finite or <= 0.
    """
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
    """
    Read and validate one angle sample, packaging it with a finite-difference velocity.

    `angle_source` is injected rather than read from a live PositionSensor directly, so
    this stays testable without a running Webots instance (mirrors poc/esp32-dof's
    dependency-injection pattern of passing in a source rather than importing
    `controller`). Pass a plain number for a fixed/mock reading, or a zero-argument
    callable (e.g. `sensor.getValue`) for a live one -- the caller (issue #265's
    controller) decides which.

    Args:
        t_s: Current elapsed time in seconds (e.g. from Webots robot.getTime()).
        angle_source: Either a float (the raw angle reading, radians) or a
            zero-argument callable returning one.
        angle_prev_rad: Previous angle reading in radians, or None if this is the
            first sample (no velocity can be computed yet).
        t_prev_s: Timestamp of the previous reading in seconds, required (not None)
            whenever angle_prev_rad is not None.

    Returns:
        SensorReading: (t_s, angle_rad, velocity_rad_s). velocity_rad_s is None when
            angle_prev_rad is None (first sample); otherwise a finite-difference rate.

    Raises:
        SensorReadError: if the raw angle is non-finite, t_s/t_prev_s are non-finite,
            angle_prev_rad is given without t_prev_s (or vice versa), or the resulting
            dt_s is <= 0.
    """
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
