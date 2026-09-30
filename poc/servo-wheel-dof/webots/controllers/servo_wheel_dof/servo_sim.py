#!/usr/bin/env python3
"""
Pure servo motion-profile module for the servo-wheel-dof Webots POC (issue #262).

Computes a target angle / angular velocity profile as a function of elapsed
simulation time. Contains **no** Webots dependency (must NOT `import controller`)
so it stays importable and unit-testable outside Webots, per this repo's
dependency-injection convention (see poc/esp32-dof/webots/controllers/
esp32_dof_follower/dof_webots_math.py and poc/esp32-dof/CLAUDE.md's
"Conventions" section: math/logic modules stay controller-free; only the
top-level Webots controller script imports `controller`).

**Units (all angles/velocities in radians / radians-per-second):**
- Elapsed time `t`: seconds, float, >= 0.
- Target angle: radians.
- Target angular velocity: radians/second.

**Motion profiles implemented:**

1. Sinusoidal sweep (default) — angle oscillates about 0 rad:
       angle(t)    = amplitude_rad * sin(2*pi*frequency_hz*t)
       velocity(t) = amplitude_rad * 2*pi*frequency_hz * cos(2*pi*frequency_hz*t)
   (velocity is the exact analytic derivative of angle, not a finite difference)

2. Constant ramp — angle increases linearly without bound:
       angle(t)    = rate_rad_s * t
       velocity(t) = rate_rad_s   (constant)

A later controller (issue #265) calls `sinusoidal_sweep`/`constant_ramp` (or the
`motion_profile` dispatcher) once per Webots timestep with the elapsed
simulation time, then commands the "wheel_motor" RotationalMotor with the
returned velocity (and/or the angle, e.g. for a position-mode motor or for
comparison against "wheel_sensor" PositionSensor readback). All functions here
are pure and stateless: same `t` and parameters always return the same result,
with no reads of a live clock or device and no mutable module state -- this
keeps the exact-value pytest assertions in issue #264 straightforward.

Usage:
    from servo_sim import sinusoidal_sweep, constant_ramp, motion_profile

    t = 0.5  # seconds, e.g. from Webots robot.getTime()
    angle_rad, velocity_rad_s = sinusoidal_sweep(t)
    # angle_rad, velocity_rad_s = constant_ramp(t, rate_rad_s=0.5)
    # angle_rad, velocity_rad_s = motion_profile(t, profile="sinusoidal")
"""

from __future__ import annotations

import math

# Default sinusoidal sweep parameters: +-30 deg amplitude, 0.2 Hz (5 s period).
DEFAULT_AMPLITUDE_RAD: float = math.radians(30.0)
DEFAULT_FREQUENCY_HZ: float = 0.2

# Default constant-ramp rate: ~1 rev per ~6.28 s.
DEFAULT_RAMP_RATE_RAD_S: float = 1.0


def sinusoidal_sweep(
    t: float,
    amplitude_rad: float = DEFAULT_AMPLITUDE_RAD,
    frequency_hz: float = DEFAULT_FREQUENCY_HZ,
) -> tuple[float, float]:
    """
    Sinusoidal angle sweep about 0 rad.

    Formula:
        angle(t)    = amplitude_rad * sin(2*pi*frequency_hz*t)
        velocity(t) = amplitude_rad * 2*pi*frequency_hz * cos(2*pi*frequency_hz*t)

    Args:
        t: Elapsed time in seconds (float, any real value; typically >= 0).
        amplitude_rad: Peak angle magnitude in radians. Default +-30 deg.
        frequency_hz: Sweep frequency in Hz (cycles per second). Default 0.2 Hz.

    Returns:
        Tuple (angle_rad, velocity_rad_s):
            angle_rad: Target angle in radians.
            velocity_rad_s: Target angular velocity in radians/second
                (exact analytic derivative of angle_rad w.r.t. t).
    """
    omega = 2.0 * math.pi * frequency_hz
    angle_rad = amplitude_rad * math.sin(omega * t)
    velocity_rad_s = amplitude_rad * omega * math.cos(omega * t)
    return (angle_rad, velocity_rad_s)


def constant_ramp(
    t: float,
    rate_rad_s: float = DEFAULT_RAMP_RATE_RAD_S,
) -> tuple[float, float]:
    """
    Constant-velocity ramp: angle grows linearly and unbounded with time.

    Formula:
        angle(t)    = rate_rad_s * t
        velocity(t) = rate_rad_s

    Args:
        t: Elapsed time in seconds (float, any real value; typically >= 0).
        rate_rad_s: Constant target angular velocity in radians/second.
            Default 1.0 rad/s. May be negative (reverse direction).

    Returns:
        Tuple (angle_rad, velocity_rad_s):
            angle_rad: Target angle in radians (unwrapped, grows without bound).
            velocity_rad_s: Target angular velocity in radians/second (constant).
    """
    angle_rad = rate_rad_s * t
    velocity_rad_s = rate_rad_s
    return (angle_rad, velocity_rad_s)


def motion_profile(
    t: float,
    profile: str = "sinusoidal",
    **params: float,
) -> tuple[float, float]:
    """
    Dispatch to a named motion profile. Convenience wrapper for a controller
    that selects the profile once (e.g. via an environment variable or
    constant) and calls this once per timestep thereafter.

    Args:
        t: Elapsed time in seconds.
        profile: One of "sinusoidal" or "ramp".
        **params: Forwarded to the selected profile function
            (e.g. amplitude_rad=, frequency_hz= for "sinusoidal";
            rate_rad_s= for "ramp").

    Returns:
        Tuple (angle_rad, velocity_rad_s) -- see sinusoidal_sweep / constant_ramp.

    Raises:
        ValueError: if `profile` is not a recognized name.
    """
    if profile == "sinusoidal":
        return sinusoidal_sweep(t, **params)
    if profile == "ramp":
        return constant_ramp(t, **params)
    raise ValueError(f"unknown motion profile: {profile!r}")
