#!/usr/bin/env python3
"""
FakeHal and LinearPitchPlant tests.

Verify mock HAL backend, plant simulation, tick periods, command logging,
plant integration via RK4, and importability contract.

Usage:
    mamba run -n pendulum-tools python3 -m pytest -q inverted-pendulum-project/07_Simulation/hal/test_fake_hal.py
"""

import ast
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from hal.hal import Hal, HalFault, ImuSample
from hal.fake_hal import FakeHal, LinearPitchPlant

import pytest


class TestFakeHalDefault:
    """F1: Default tick periods (0.016, 0.032)."""

    def test_default_periods_and_now_s(self):
        """5 calls return [0.016,0.032,0.016,0.032,0.016]; now_s() before any tick == 0.0."""
        hal = FakeHal()
        assert hal.now_s() == 0.0

        dt1 = hal.wait_next_tick()
        assert dt1 == pytest.approx(0.016, abs=1e-12)
        assert hal.now_s() == pytest.approx(0.016, abs=1e-12)

        dt2 = hal.wait_next_tick()
        assert dt2 == pytest.approx(0.032, abs=1e-12)
        assert hal.now_s() == pytest.approx(0.048, abs=1e-12)

        dt3 = hal.wait_next_tick()
        assert dt3 == pytest.approx(0.016, abs=1e-12)
        assert hal.now_s() == pytest.approx(0.064, abs=1e-12)

        dt4 = hal.wait_next_tick()
        assert dt4 == pytest.approx(0.032, abs=1e-12)
        assert hal.now_s() == pytest.approx(0.096, abs=1e-12)

        dt5 = hal.wait_next_tick()
        assert dt5 == pytest.approx(0.016, abs=1e-12)
        assert hal.now_s() == pytest.approx(0.112, abs=1e-12)


class TestFakeHalSinglePeriod:
    """F2: Single tick period (0.02)."""

    def test_single_period_three_ticks(self):
        """First tick 0.02; 3 ticks give now_s ~ 0.06."""
        hal = FakeHal(tick_periods_s=(0.02,))

        dt1 = hal.wait_next_tick()
        assert dt1 == pytest.approx(0.02, abs=1e-12)

        dt2 = hal.wait_next_tick()
        assert dt2 == pytest.approx(0.02, abs=1e-12)

        dt3 = hal.wait_next_tick()
        assert dt3 == pytest.approx(0.02, abs=1e-12)
        assert hal.now_s() == pytest.approx(0.06, abs=1e-12)


class TestFakeHalMaxTicks:
    """F3: Max ticks limiting."""

    def test_max_ticks_three(self):
        """max_ticks=3: first 3 ticks [0.016,0.032,0.016], then -1.0, then -1.0 again; now_s ~0.064 unchanged."""
        hal = FakeHal(max_ticks=3)

        dt1 = hal.wait_next_tick()
        assert dt1 == pytest.approx(0.016, abs=1e-12)

        dt2 = hal.wait_next_tick()
        assert dt2 == pytest.approx(0.032, abs=1e-12)

        dt3 = hal.wait_next_tick()
        assert dt3 == pytest.approx(0.016, abs=1e-12)
        assert hal.now_s() == pytest.approx(0.064, abs=1e-12)

        dt4 = hal.wait_next_tick()
        assert dt4 == -1.0
        assert hal.now_s() == pytest.approx(0.064, abs=1e-12)

        dt5 = hal.wait_next_tick()
        assert dt5 == -1.0
        assert hal.now_s() == pytest.approx(0.064, abs=1e-12)

    def test_max_ticks_zero(self):
        """max_ticks=0: first call -1.0."""
        hal = FakeHal(max_ticks=0)
        dt = hal.wait_next_tick()
        assert dt == -1.0


class TestFakeHalImuScript:
    """F4: IMU script replay (mixed ImuSample and tuple)."""

    def test_imu_script_mixed_exhaustion(self):
        """Script of 3 samples (ImuSample and tuple mixed): read_imu returns equal objects in order; 4th raises HalFault."""
        sample1 = ImuSample(0.0, 0.0, 0.1, 0.0, (0.0, 0.1, 0.0))
        sample2_tuple = (0.0, 0.0, -0.1, 0.0, (0.0, 0.2, 0.0))

        hal = FakeHal(imu_script=[sample1, sample2_tuple, sample1])

        # Read 1st
        s1 = hal.read_imu()
        assert s1 == sample1

        # Read 2nd (from tuple)
        s2 = hal.read_imu()
        expected2 = ImuSample(0.0, 0.0, -0.1, 0.0, (0.0, 0.2, 0.0))
        assert s2 == expected2

        # Read 3rd
        s3 = hal.read_imu()
        assert s3 == sample1

        # Read 4th raises
        with pytest.raises(HalFault, match="exhausted"):
            hal.read_imu()


class TestFakeHalNoSource:
    """F5: No IMU source or mutual exclusivity."""

    def test_no_imu_source(self):
        """No imu_script or plant: read_imu raises HalFault."""
        hal = FakeHal()
        with pytest.raises(HalFault):
            hal.read_imu()

    def test_imu_script_and_plant_mutually_exclusive(self):
        """imu_script and plant together raise ValueError."""
        plant = LinearPitchPlant([[0, 1], [0, 0]], [[0], [1]])
        with pytest.raises(ValueError):
            FakeHal(imu_script=[ImuSample(0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 0.0))], plant=plant)

    def test_read_encoders_unsupported(self):
        """read_encoders raises HalFault."""
        hal = FakeHal()
        with pytest.raises(HalFault):
            hal.read_encoders()


class TestFakeHalCommandLog:
    """F6: Command logging and timing."""

    def test_command_logging_with_ticks(self):
        """write(0.5,0.5) at t=0 -> commands == [(0.0,0.5,0.5)]; after wait_next_tick(), write(-0.25,0.25)."""
        hal = FakeHal()

        hal.write_wheel_velocity(0.5, 0.5)
        assert hal.commands == [(0.0, 0.5, 0.5)]

        hal.wait_next_tick()
        hal.write_wheel_velocity(-0.25, 0.25)
        assert len(hal.commands) == 2
        assert hal.commands[1][0] == pytest.approx(0.016, abs=1e-12)
        assert hal.commands[1][1] == -0.25
        assert hal.commands[1][2] == 0.25

    def test_write_without_tick_never_raises(self):
        """Writing with no tick never raises."""
        hal = FakeHal()
        hal.write_wheel_velocity(0.1, 0.2)
        hal.write_wheel_velocity(0.3, 0.4)
        # Should not raise
        assert len(hal.commands) == 2


class TestLinearPitchPlantValidation:
    """F7: Input validation for plant and tick periods."""

    def test_empty_tick_periods(self):
        """tick_periods_s=() raises ValueError."""
        with pytest.raises(ValueError):
            FakeHal(tick_periods_s=())

    def test_zero_tick_period(self):
        """tick_periods_s=(0.0,) raises ValueError."""
        with pytest.raises(ValueError):
            FakeHal(tick_periods_s=(0.0,))

    def test_negative_tick_period(self):
        """tick_periods_s=(-0.01,) raises ValueError."""
        with pytest.raises(ValueError):
            FakeHal(tick_periods_s=(-0.01,))

    def test_plant_zero_substep(self):
        """LinearPitchPlant with substep_s=0.0 raises ValueError."""
        with pytest.raises(ValueError):
            LinearPitchPlant([[0, 1], [0, 0]], [[0], [1]], substep_s=0.0)


class TestLinearPitchPlantSimple:
    """F8: Plant integration with simple A, B matrices."""

    def test_plant_simple_dynamics(self):
        """Simple plant: A=[[0,1],[0,0]], B=[[0],[1]]; verify state evolution and pitch."""
        plant = LinearPitchPlant([[0, 1], [0, 0]], [[0], [1]])
        hal = FakeHal(
            tick_periods_s=(0.02,),
            plant=plant,
            theta0_rad=0.05,
            theta_ref_rad=-0.1086,
        )

        # Before any tick
        sample0 = hal.read_imu()
        assert sample0.pitch_rad == pytest.approx(-0.0586, abs=1e-12)
        assert sample0.gyro_rad_s[1] == 0.0
        assert sample0.t_s == 0.0

        # Write command and tick
        hal.write_wheel_velocity(1.0, 1.0)
        dt = hal.wait_next_tick()
        assert dt == pytest.approx(0.02, abs=1e-12)

        sample1 = hal.read_imu()
        # theta evolved from 0.05; pitch = theta + ref = theta - 0.1086
        assert sample1.pitch_rad == pytest.approx(-0.0584, abs=1e-12)
        # gyro_rad_s[1] ~ 0.02
        assert sample1.gyro_rad_s[1] == pytest.approx(0.02, abs=1e-12)
        assert sample1.t_s == pytest.approx(0.02, abs=1e-12)

        # Second tick with zero-order hold (no new write)
        dt2 = hal.wait_next_tick()
        assert dt2 == pytest.approx(0.02, abs=1e-12)

        sample2 = hal.read_imu()
        # theta continues to evolve
        assert sample2.pitch_rad == pytest.approx(-0.0578, abs=1e-12)
        assert sample2.gyro_rad_s[1] == pytest.approx(0.04, abs=1e-12)


class TestLinearPitchPlantFromLQR:
    """F9: from_plant_lqr construction and matrix values."""

    def test_from_plant_lqr_matrices(self):
        """Compare plant._a[1][0] and plant._b[1][0] to build_state_space() values."""
        from plant_lqr import build_state_space

        plant = LinearPitchPlant.from_plant_lqr()
        A, B = build_state_space()

        # A and B from build_state_space() are numpy arrays; access with indexing
        assert plant._a[1][0] == A[1][0]
        assert plant._b[1][0] == B[1][0]

        # Verify signs
        assert A[1][0] > 0
        assert B[1][0] < 0


class TestLinearPitchPlantDeterminism:
    """F10: Determinism and monotonic behavior."""

    def test_determinism_from_plant_lqr(self):
        """Two identically built FakeHal in plant mode with from_plant_lqr give identical read_imu tuples over 50 ticks."""
        # Build two identical instances
        hal1 = FakeHal(
            tick_periods_s=(0.016, 0.032),
            plant=LinearPitchPlant.from_plant_lqr(),
            theta0_rad=0.05,
        )
        hal2 = FakeHal(
            tick_periods_s=(0.016, 0.032),
            plant=LinearPitchPlant.from_plant_lqr(),
            theta0_rad=0.05,
        )

        samples1 = []
        samples2 = []

        for _ in range(50):
            hal1.wait_next_tick()
            hal2.wait_next_tick()
            samples1.append(hal1.read_imu())
            samples2.append(hal2.read_imu())

        # Verify identical
        for s1, s2 in zip(samples1, samples2):
            assert s1 == s2

    def test_free_fall_monotonic(self):
        """Free fall (u=0): pitch - ref grows monotonically from initial theta0."""
        plant = LinearPitchPlant.from_plant_lqr()
        hal = FakeHal(
            tick_periods_s=(0.016, 0.032),
            plant=plant,
            theta0_rad=0.05,
            theta_ref_rad=-0.1086,
        )

        samples = []
        for _ in range(20):
            hal.wait_next_tick()
            samples.append(hal.read_imu())

        # Extract pitch - theta_ref (approximates theta)
        pitches = [s.pitch_rad for s in samples]

        # Verify monotonic growth (free fall should increase pitch error)
        for i in range(1, len(pitches)):
            assert pitches[i] > pitches[i - 1]


class TestFakeHalInstance:
    """F11: Hal instance and close() method."""

    def test_isinstance_hal(self):
        """FakeHal(imu_script=[]) is instance of Hal."""
        hal = FakeHal(imu_script=[])
        assert isinstance(hal, Hal)

    def test_close_returns_none(self):
        """close() returns None."""
        hal = FakeHal()
        result = hal.close()
        assert result is None


class TestFakeHalAST:
    """F12: AST analysis of fake_hal.py."""

    def test_no_controller_import(self):
        """Parse fake_hal.py; no Import/ImportFrom with top-level module == 'controller'."""
        fake_hal_path = Path(__file__).resolve().parent / "fake_hal.py"
        code = fake_hal_path.read_text()
        tree = ast.parse(code)

        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    top_mod = alias.name.split(".")[0]
                    assert top_mod != "controller", f"Found controller import: {alias.name}"
            elif isinstance(node, ast.ImportFrom):
                if node.module:
                    top_mod = node.module.split(".")[0]
                    assert top_mod != "controller", f"Found controller import: {node.module}"

    def test_module_level_imports(self):
        """Module-level (tree.body) imports are subset of {'sys','math','pathlib','hal'}."""
        fake_hal_path = Path(__file__).resolve().parent / "fake_hal.py"
        code = fake_hal_path.read_text()
        tree = ast.parse(code)

        allowed_modules = {"sys", "math", "pathlib", "hal"}
        module_level_imports = set()

        for node in tree.body:
            if isinstance(node, ast.Import):
                for alias in node.names:
                    top_mod = alias.name.split(".")[0]
                    module_level_imports.add(top_mod)
            elif isinstance(node, ast.ImportFrom):
                if node.module:
                    top_mod = node.module.split(".")[0]
                    module_level_imports.add(top_mod)

        assert module_level_imports.issubset(
            allowed_modules
        ), f"Module-level imports not in allowed set: {module_level_imports - allowed_modules}"

    def test_numpy_plant_lqr_not_module_level(self):
        """numpy and plant_lqr are not imported at module level."""
        fake_hal_path = Path(__file__).resolve().parent / "fake_hal.py"
        code = fake_hal_path.read_text()
        tree = ast.parse(code)

        disallowed_at_module_level = {"numpy", "plant_lqr"}

        for node in tree.body:
            if isinstance(node, ast.Import):
                for alias in node.names:
                    top_mod = alias.name.split(".")[0]
                    assert (
                        top_mod not in disallowed_at_module_level
                    ), f"Found {top_mod} at module level"
            elif isinstance(node, ast.ImportFrom):
                if node.module:
                    top_mod = node.module.split(".")[0]
                    assert (
                        top_mod not in disallowed_at_module_level
                    ), f"Found {top_mod} at module level"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
