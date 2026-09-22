#!/usr/bin/env python3
"""
Unit tests for PlantPID discrete PID controller.

Pure Python / pytest, no external dependencies beyond pytest.

Usage:
    cd inverted-pendulum-project
    mamba run -n pendulum-tools python3 -m pytest 07_Simulation/test_plant_pid.py -v
"""

import sys
from pathlib import Path

import pytest

# Add the script directory to path for imports
sys.path.insert(0, str(Path(__file__).resolve().parent))

from plant_pid import PlantPID


class TestStepResponseConstantError:
    """Test: constant error accumulates integral term over time"""

    def test_step_response_constant_error(self):
        """Call .step(error=1.0, dt=0.02) twice with kp=1, ki=0.5, kd=0.

        Assert second call's output > first call's output (integral accumulating).
        """
        pid = PlantPID(kp=1.0, ki=0.5, kd=0.0, output_min=-100, output_max=100)

        # First call: error=1.0, dt=0.02
        # P = 1.0 * 1.0 = 1.0
        # I = 0.5 * (1.0 * 0.02) = 0.5 * 0.02 = 0.01
        # D = 0 (first call, no prev_error)
        # output1 = 1.0 + 0.01 + 0 = 1.01
        output1 = pid.step(error=1.0, dt=0.02)

        # Second call: error=1.0, dt=0.02 (same error, same dt)
        # P = 1.0 * 1.0 = 1.0
        # I = 0.5 * (0.02 + 0.02) = 0.5 * 0.04 = 0.02
        # D = 0 * (1.0 - 1.0) / 0.02 = 0
        # output2 = 1.0 + 0.02 + 0 = 1.02
        output2 = pid.step(error=1.0, dt=0.02)

        assert output2 > output1, "Second output should be greater due to integral accumulation"


class TestStepResponseConvergence:
    """Test: proportional-only PID tracks decreasing error"""

    def test_step_response_convergence(self):
        """Proportional-only PID (kp=2, ki=0, kd=0), feed decreasing error sequence.

        Assert output tracks proportionally and hits 0 when error is 0.
        """
        pid = PlantPID(kp=2.0, ki=0.0, kd=0.0, output_min=-100, output_max=100)

        errors = [1.0, 0.5, 0.25, 0.0]
        expected_outputs = [2.0, 1.0, 0.5, 0.0]

        for error, expected in zip(errors, expected_outputs):
            output = pid.step(error=error, dt=0.02)
            assert abs(output - expected) < 1e-9, f"Expected {expected}, got {output}"


class TestVariableDtDerivative:
    """Test: kd-only PID with variable dt (simulating jittery control loop)"""

    def test_variable_dt_derivative(self):
        """kd-only PID (kp=0, ki=0, kd=1.0).

        First call: .step(error=0.0, dt=0.02) (establishes prev_error=0, output should be 0)
        Second call: .step(error=0.036, dt=0.036) (simulating combined jittery fires)

        Expected output ≈ kd * (0.036-0.0)/0.036 == kd == 1.0, within tolerance.
        """
        pid = PlantPID(kp=0.0, ki=0.0, kd=1.0, output_min=-100, output_max=100)

        # First call: error=0.0, dt=0.02
        # P = 0, I = 0, D = 0 (no prev_error yet)
        # output = 0.0
        output1 = pid.step(error=0.0, dt=0.02)
        assert abs(output1 - 0.0) < 1e-9

        # Second call: error=0.036, dt=0.036
        # P = 0, I = 0
        # D = 1.0 * (0.036 - 0.0) / 0.036 = 1.0 * 1.0 = 1.0
        # output = 1.0
        output2 = pid.step(error=0.036, dt=0.036)
        assert abs(output2 - 1.0) < 0.01, f"Expected ~1.0, got {output2}"


class TestIntegralAntiWindupClamp:
    """Test: integral anti-windup clamping"""

    def test_integral_anti_windup_clamp(self):
        """ki=1.0, kp=0, kd=0, integral_max=5.0, integral_min=-5.0.

        Call .step(error=10.0, dt=1.0) repeatedly (~10 times) with large error.
        Assert output never exceeds integral_max (5.0).
        """
        pid = PlantPID(
            kp=0.0, ki=1.0, kd=0.0,
            output_min=-100, output_max=100,
            integral_min=-5.0, integral_max=5.0
        )

        # With ki=1.0 and error=10.0, dt=1.0:
        # Each step accumulates integral += 10.0 * 1.0 = 10.0
        # But integral is clamped to [-5.0, 5.0], so after first step integral = 5.0
        # i_term = 1.0 * 5.0 = 5.0
        # output = 0 + 5.0 + 0 = 5.0

        for _ in range(10):
            output = pid.step(error=10.0, dt=1.0)
            # Output should never exceed integral_max (5.0) with wide output bounds
            assert output <= 5.0 + 1e-9, f"Output {output} exceeded integral_max 5.0"


class TestOutputSaturation:
    """Test: output saturation bounds"""

    def test_output_saturation(self):
        """kp=100, ki=0, kd=0, output_min=-1.0, output_max=1.0.

        Call .step(error=10.0, dt=0.02).
        Assert output == 1.0 (saturated at upper bound, not raw 1000).
        """
        pid = PlantPID(
            kp=100.0, ki=0.0, kd=0.0,
            output_min=-1.0, output_max=1.0
        )

        # Raw P term would be 100.0 * 10.0 = 1000.0
        # But saturated to [−1.0, 1.0], so output = 1.0
        output = pid.step(error=10.0, dt=0.02)
        assert output == 1.0


class TestDtZeroReturnsPOnly:
    """Test: dt==0 returns P-only output without state update"""

    def test_dt_zero_returns_p_only(self):
        """kp=2.0, ki=1.0, kd=1.0, wide bounds.

        Call .step(error=1.0, dt=0.02) first (to establish non-None prev_error/some integral state).
        Then call .step(error=1.0, dt=0).

        Assert second call's output == kp * 1.0 == 2.0 exactly (P-only, I/D untouched).
        """
        pid = PlantPID(kp=2.0, ki=1.0, kd=1.0, output_min=-100, output_max=100)

        # First call: build up some integral state and prev_error
        output1 = pid.step(error=1.0, dt=0.02)
        # After first call:
        # integral = 1.0 * 0.02 = 0.02
        # prev_error = 1.0
        # output1 = 2.0 + 1.0*0.02 + 1.0*(1.0-None)/0.02
        #         = 2.0 + 0.02 + 0
        #         = 2.02

        # Second call with dt=0: should return P-only, no state update
        output2 = pid.step(error=1.0, dt=0)

        # output2 = kp * error = 2.0 * 1.0 = 2.0
        assert output2 == 2.0, f"Expected 2.0, got {output2}"

        # Verify state was not updated (integral should still be 0.02, prev_error still 1.0)
        # by making another call and checking the integral contribution
        output3 = pid.step(error=1.0, dt=0.02)
        # integral should now be 0.02 + 1.0*0.02 = 0.04
        # i_term = 1.0 * 0.04 = 0.04
        # output3 = 2.0 + 0.04 + 0 = 2.04
        assert abs(output3 - 2.04) < 1e-9, f"Integral state was incorrectly updated by dt=0 call"


class TestDtNegativeRaises:
    """Test: negative dt raises ValueError"""

    def test_dt_negative_raises(self):
        """Any gains; assert calling .step(error=1.0, dt=-0.01) raises ValueError."""
        pid = PlantPID(kp=1.0, ki=1.0, kd=1.0, output_min=-100, output_max=100)

        with pytest.raises(ValueError):
            pid.step(error=1.0, dt=-0.01)


class TestResetClearsIntegral:
    """Test: reset() clears integral state"""

    def test_reset_clears_integral(self):
        """ki=1.0, kp=0, kd=0, wide bounds.

        Call .step(error=5.0, dt=1.0) a few times to build up integral.
        Note the output. Call .reset().
        Call .step(error=5.0, dt=1.0) once more.
        Assert this post-reset output is smaller than the last pre-reset output.
        """
        pid = PlantPID(kp=0.0, ki=1.0, kd=0.0, output_min=-100, output_max=100)

        # Build up integral over several calls
        for _ in range(3):
            pid.step(error=5.0, dt=1.0)

        # After 3 calls with error=5.0, dt=1.0:
        # integral = 5.0 + 5.0 + 5.0 = 15.0
        # i_term = 1.0 * 15.0 = 15.0
        # output (last pre-reset) = 15.0
        last_pre_reset_output = pid.step(error=5.0, dt=1.0)
        # Now integral = 20.0, output = 20.0

        # Reset the controller
        pid.reset()

        # After reset, make one more call
        post_reset_output = pid.step(error=5.0, dt=1.0)
        # integral = 0.0 + 5.0*1.0 = 5.0
        # i_term = 1.0 * 5.0 = 5.0
        # output = 5.0

        assert post_reset_output < last_pre_reset_output, \
            f"Post-reset output {post_reset_output} should be < pre-reset {last_pre_reset_output}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
