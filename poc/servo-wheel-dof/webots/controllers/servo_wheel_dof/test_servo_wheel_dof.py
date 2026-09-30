#!/usr/bin/env python3
"""
Unit tests for the pure servo-wheel-dof motion-profile and sensor-readout
modules (issue #264): servo_sim.py and sensor_read.py only.

Pure Python / pytest -- no Webots dependency, no live Webots needed, and must
NOT `import controller` anywhere in this file (per poc/esp32-dof/CLAUDE.md's
"controller is imported only in the top-level Webots controller script"
convention, which this POC mirrors -- see servo_sim.py's own module docstring).

Usage:
    cd poc/servo-wheel-dof/webots/controllers/servo_wheel_dof
    mamba run -n servo-wheel-dof python3 -m pytest test_servo_wheel_dof.py -v
"""

import dataclasses
import math
import sys
from pathlib import Path

import pytest

# Add the script directory to path for imports (this dir is not a package).
sys.path.insert(0, str(Path(__file__).resolve().parent))

from servo_sim import sinusoidal_sweep, constant_ramp, motion_profile
from sensor_read import SensorReadError, SensorReading, angular_velocity_from_positions, read_sensor


class TestSinusoidalSweep:
    """Test: sinusoidal_sweep() angle/velocity values at known points."""

    def test_zero_time_zero_angle_peak_velocity(self):
        """t=0: angle=0, velocity at its positive peak (amplitude * omega)."""
        angle_rad, velocity_rad_s = sinusoidal_sweep(0.0)
        # omega = 2*pi*0.2 = 0.4*pi; amplitude = radians(30) = pi/6.
        # velocity = amplitude*omega = (pi/6)*(0.4*pi) = pi**2/15.
        assert angle_rad == pytest.approx(0.0, abs=1e-9)
        assert velocity_rad_s == pytest.approx(math.pi**2 / 15.0)

    def test_quarter_period_peak_angle_zero_velocity(self):
        """t=1.25s (quarter of the 5s period): angle at +amplitude, velocity=0."""
        angle_rad, velocity_rad_s = sinusoidal_sweep(1.25)
        assert angle_rad == pytest.approx(math.radians(30.0))
        assert velocity_rad_s == pytest.approx(0.0, abs=1e-9)

    def test_half_period_zero_angle_negative_peak_velocity(self):
        """t=2.5s (half period): angle back near 0, velocity at its negative peak."""
        angle_rad, velocity_rad_s = sinusoidal_sweep(2.5)
        assert angle_rad == pytest.approx(0.0, abs=1e-9)
        assert velocity_rad_s == pytest.approx(-(math.pi**2 / 15.0))

    def test_velocity_is_analytic_derivative_of_angle(self):
        """velocity_rad_s matches the central-difference derivative of angle_rad."""
        t0 = 0.3
        h = 1e-6
        angle_plus, _ = sinusoidal_sweep(t0 + h)
        angle_minus, _ = sinusoidal_sweep(t0 - h)
        finite_diff_velocity = (angle_plus - angle_minus) / (2.0 * h)
        _, velocity_rad_s = sinusoidal_sweep(t0)
        assert velocity_rad_s == pytest.approx(finite_diff_velocity, abs=1e-4)

    def test_custom_amplitude_and_frequency(self):
        """Non-default amplitude_rad=1.0, frequency_hz=1.0 at t=0.5s (half period)."""
        angle_rad, velocity_rad_s = sinusoidal_sweep(0.5, amplitude_rad=1.0, frequency_hz=1.0)
        # omega = 2*pi*1.0 = 2*pi; omega*t = 2*pi*0.5 = pi.
        # angle = 1.0*sin(pi) ~= 0; velocity = 1.0*2*pi*cos(pi) = -2*pi.
        assert angle_rad == pytest.approx(0.0, abs=1e-9)
        assert velocity_rad_s == pytest.approx(-2.0 * math.pi)


class TestConstantRamp:
    """Test: constant_ramp() angle/velocity for zero, default, and negative rates."""

    def test_zero_rate_angle_and_velocity_stay_zero(self):
        """rate_rad_s=0.0: angle stays 0 and velocity is 0, regardless of t."""
        angle_rad, velocity_rad_s = constant_ramp(2.0, rate_rad_s=0.0)
        assert angle_rad == pytest.approx(0.0)
        assert velocity_rad_s == pytest.approx(0.0)

    def test_default_rate(self):
        """Default rate_rad_s=1.0 at t=2.0s: angle=2.0, velocity=1.0."""
        angle_rad, velocity_rad_s = constant_ramp(2.0)
        assert angle_rad == pytest.approx(2.0)
        assert velocity_rad_s == pytest.approx(1.0)

    def test_negative_rate_angle_decreases_velocity_negative(self):
        """Negative rate_rad_s: angle grows more negative with t, velocity is constant negative."""
        angle_at_1s, velocity_at_1s = constant_ramp(1.0, rate_rad_s=-2.0)
        angle_at_3s, velocity_at_3s = constant_ramp(3.0, rate_rad_s=-2.0)
        assert angle_at_1s == pytest.approx(-2.0)
        assert angle_at_3s == pytest.approx(-6.0)
        assert angle_at_3s < angle_at_1s  # angle decreases as t increases
        assert velocity_at_1s == pytest.approx(-2.0)
        assert velocity_at_3s == pytest.approx(-2.0)


class TestMotionProfileDispatch:
    """Test: motion_profile() dispatches to the right function and rejects unknown names."""

    def test_sinusoidal_dispatch_matches_direct_call(self):
        """profile="sinusoidal" matches calling sinusoidal_sweep() directly."""
        dispatched = motion_profile(0.75, profile="sinusoidal")
        direct = sinusoidal_sweep(0.75)
        assert dispatched[0] == pytest.approx(direct[0])
        assert dispatched[1] == pytest.approx(direct[1])

    def test_ramp_dispatch_matches_direct_call_with_kwargs(self):
        """profile="ramp" forwards kwargs and matches constant_ramp() directly."""
        dispatched = motion_profile(2.0, profile="ramp", rate_rad_s=0.5)
        assert dispatched[0] == pytest.approx(1.0)
        assert dispatched[1] == pytest.approx(0.5)

    def test_unknown_profile_raises_value_error(self):
        """An unrecognized profile name raises ValueError."""
        with pytest.raises(ValueError):
            motion_profile(0.0, profile="bogus")


class TestAngularVelocityFromPositions:
    """Test: angular_velocity_from_positions() normal case and error paths."""

    def test_normal_case(self):
        """(0.4 - 0.1) / 0.1 = 3.0 rad/s."""
        velocity = angular_velocity_from_positions(0.1, 0.4, 0.1)
        assert velocity == pytest.approx(3.0)

    def test_zero_dt_raises(self):
        """dt_s == 0 raises SensorReadError."""
        with pytest.raises(SensorReadError):
            angular_velocity_from_positions(0.0, 0.1, 0.0)

    def test_negative_dt_raises(self):
        """dt_s < 0 raises SensorReadError."""
        with pytest.raises(SensorReadError):
            angular_velocity_from_positions(0.0, 0.1, -0.1)

    def test_nonfinite_angle_prev_raises(self):
        """Non-finite angle_prev_rad (NaN) raises SensorReadError."""
        with pytest.raises(SensorReadError):
            angular_velocity_from_positions(float("nan"), 0.1, 0.1)

    def test_nonfinite_angle_curr_raises(self):
        """Non-finite angle_curr_rad (+inf) raises SensorReadError."""
        with pytest.raises(SensorReadError):
            angular_velocity_from_positions(0.0, float("inf"), 0.1)


class TestReadSensor:
    """Test: read_sensor() first-sample vs second-sample behavior and error paths."""

    def test_first_sample_velocity_is_none(self):
        """No angle_prev_rad/t_prev_s given: velocity_rad_s is None."""
        reading = read_sensor(0.0, 0.1)
        assert isinstance(reading, SensorReading)
        assert reading.t_s == pytest.approx(0.0)
        assert reading.angle_rad == pytest.approx(0.1)
        assert reading.velocity_rad_s is None

    def test_second_sample_computes_velocity(self):
        """Second sample with prior angle/time computes velocity via the position diff."""
        reading = read_sensor(0.1, 0.4, angle_prev_rad=0.1, t_prev_s=0.0)
        assert reading.t_s == pytest.approx(0.1)
        assert reading.angle_rad == pytest.approx(0.4)
        assert reading.velocity_rad_s == pytest.approx(3.0)

    def test_angle_prev_without_t_prev_raises(self):
        """angle_prev_rad given but t_prev_s omitted (XOR mismatch) raises SensorReadError."""
        with pytest.raises(SensorReadError):
            read_sensor(0.1, 0.4, angle_prev_rad=0.1, t_prev_s=None)

    def test_t_prev_without_angle_prev_raises(self):
        """t_prev_s given but angle_prev_rad omitted (XOR mismatch) raises SensorReadError."""
        with pytest.raises(SensorReadError):
            read_sensor(0.1, 0.4, angle_prev_rad=None, t_prev_s=0.0)

    def test_nonfinite_angle_source_raises(self):
        """Non-finite angle_source (NaN) raises SensorReadError."""
        with pytest.raises(SensorReadError):
            read_sensor(0.0, float("nan"))

    def test_reading_is_frozen(self):
        """SensorReading is a frozen dataclass -- mutation raises."""
        reading = read_sensor(0.0, 0.1)
        with pytest.raises(dataclasses.FrozenInstanceError):
            reading.t_s = 99.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
