"""
Float32 emulation parity tests: verify quantization, Float32PidBalance, Float32LqrBalance.

Pure Python unit tests with no Webots or hardware dependencies.
Tests validate float32 numeric precision, PID/LQR operation under float32 quantization,
state management, output clamping, dt-handling, and import structure.

Usage:
    mamba run -n pendulum-tools python3 -m pytest -q inverted-pendulum-project/07_Simulation/hal/test_f32_parity.py
"""

import ast
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from hal.hal import ImuSample
from hal.control_core import PidBalance, LqrBalance
from hal.fake_hal import FakeHal, LinearPitchPlant
from hal.f32_emulation import (
    Float32PidBalance,
    Float32LqrBalance,
    quantize_f32,
    quantize_imu,
    run_closed_loop,
    compare_runs,
    replay_imu,
    LoopRecord,
    F32_TOL_ABS,
    F32_TOL_PID_LONG_ABS,
    F32_TOL_STATE_PITCH_ABS,
    F32_TOL_STATE_GYRO_ABS,
)


def imu(pitch, gyro_y=0.0):
    """Construct ImuSample with given pitch and gyro_y (pitch rate)."""
    return ImuSample(0.0, 0.0, pitch, 0.0, (0.0, gyro_y, 0.0))


def is_f32(v):
    """Check if a value is exactly representable as float32."""
    return float(np.float32(v)) == v


class TestQuantize:
    """Test suite for float32 quantization functions."""

    def test_a1_quantize_f32_precision(self):
        """A1: quantize_f32(0.1) differs from 0.1, quantize_f32(0.5)==0.5, idempotent."""
        # 0.1 cannot be exactly represented in binary, so quantize_f32 changes it
        assert quantize_f32(0.1) == 0.10000000149011612
        assert quantize_f32(0.1) != 0.1

        # 0.5 is exactly representable
        assert quantize_f32(0.5) == 0.5

        # Idempotent: quantizing twice yields the same result
        for x in (0.1, -0.1086, 3.3):
            once = quantize_f32(x)
            twice = quantize_f32(once)
            assert once == twice, f"Not idempotent for {x}"

    def test_a2_quantize_imu_sample(self):
        """A2: quantize_imu returns ImuSample with f32-quantized fields, gyro tuple of len 3."""
        sample = ImuSample(0.1, 0.2, 0.3, 0.4, (0.5, 0.6, 0.7))
        quantized = quantize_imu(sample)

        # Result is an ImuSample
        assert isinstance(quantized, ImuSample)

        # Gyro is a tuple of length 3
        assert isinstance(quantized.gyro_rad_s, tuple)
        assert len(quantized.gyro_rad_s) == 3

        # All 7 floats satisfy is_f32
        assert is_f32(quantized.t_s)
        assert is_f32(quantized.roll_rad)
        assert is_f32(quantized.pitch_rad)
        assert is_f32(quantized.yaw_rad)
        assert is_f32(quantized.gyro_rad_s[0])
        assert is_f32(quantized.gyro_rad_s[1])
        assert is_f32(quantized.gyro_rad_s[2])

        # t_s is quantized as expected
        assert quantized.t_s == quantize_f32(0.1)


class TestFloat32LqrExact:
    """Test suite for Float32LqrBalance controller with exact float32 precision checks."""

    def test_a3_lqr_at_reference_zero_error(self):
        """A3: pitch=-0.1086, gyro=0, dt=0.02 => raw==0.0 and cmd==0.0 EXACTLY."""
        lqr = Float32LqrBalance(K=(10.0, 2.0))
        out = lqr.step(imu(-0.1086, 0.0), 0.02)

        assert out.raw_cmd == 0.0
        assert out.cmd_rad_s == 0.0

    def test_a4_lqr_pitch_error_and_gyro(self):
        """A4: pitch=-0.0586, gyro=0 => raw/cmd≈-0.5; with gyro=0.1 => raw/cmd≈-0.7."""
        lqr = Float32LqrBalance(K=(10.0, 2.0))

        # First case: pitch=-0.0586, gyro=0
        out1 = lqr.step(imu(-0.0586, 0.0), 0.02)
        assert out1.raw_cmd == pytest.approx(-0.5, abs=1e-6)
        assert out1.cmd_rad_s == pytest.approx(-0.5, abs=1e-6)

        # Second case: pitch=-0.0586, gyro=0.1
        out2 = lqr.step(imu(-0.0586, 0.1), 0.02)
        assert out2.raw_cmd == pytest.approx(-0.7, abs=1e-6)
        assert out2.cmd_rad_s == pytest.approx(-0.7, abs=1e-6)

    def test_a5_lqr_large_error_clamped(self):
        """A5: pitch=0.0914, gyro=0.5 => raw≈-3.0, cmd==-1.0 EXACTLY."""
        lqr = Float32LqrBalance(K=(10.0, 2.0))
        out = lqr.step(imu(0.0914, 0.5), 0.02)

        assert out.raw_cmd == pytest.approx(-3.0, abs=1e-6)
        assert out.cmd_rad_s == -1.0

    def test_a6_lqr_negative_dt_fault(self):
        """A6: dt=-0.001 => cmd=0.0, raw=0.0, fault=True, len(debug)==2."""
        lqr = Float32LqrBalance(K=(10.0, 2.0))
        out = lqr.step(imu(-0.0586, 0.1), -0.001)

        assert out.cmd_rad_s == 0.0
        assert out.raw_cmd == 0.0
        assert out.fault is True
        assert len(out.debug) == 2

    def test_a7_lqr_validation(self):
        """A7: K of length 3 raises ValueError; out_limit=0 raises ValueError."""
        with pytest.raises(ValueError):
            Float32LqrBalance(K=(10.0, 2.0, 3.0))

        with pytest.raises(ValueError):
            Float32LqrBalance(out_limit=0)

    def test_a8_lqr_deterministic_f32_precision(self):
        """A8: 20 deterministic inputs yield cmd, raw, and all debug values as f32."""
        lqr = Float32LqrBalance(K=(10.0, 2.0))

        for k in range(20):
            pitch = 0.1 * math.sin(0.7 * k) - 0.02
            gyro = 0.05 * math.cos(0.3 * k)
            out = lqr.step(imu(pitch, gyro), 0.02)

            assert is_f32(out.cmd_rad_s)
            assert is_f32(out.raw_cmd)
            assert is_f32(out.debug[0])  # theta
            assert is_f32(out.debug[1])  # theta_dot


class TestFloat32PidExact:
    """Test suite for Float32PidBalance controller with exact float32 precision checks."""

    def test_a9_pid_fresh_step(self):
        """A9: Fresh step(imu(-0.1), 0.02) => cmd≈0.1002, debug≈(0.1,0.1,0.0002,0.0), fault=False."""
        pid = Float32PidBalance()
        out = pid.step(imu(-0.1), 0.02)

        assert out.cmd_rad_s == pytest.approx(0.1002, abs=1e-6)
        assert out.debug[0] == pytest.approx(0.1, abs=1e-6)
        assert out.debug[1] == pytest.approx(0.1, abs=1e-6)
        assert out.debug[2] == pytest.approx(0.0002, abs=1e-6)
        assert out.debug[3] == pytest.approx(0.0, abs=1e-6)
        assert out.fault is False

    def test_a10_pid_second_step(self):
        """A10: After A9, step(imu(-0.2), 0.02) => cmd≈0.4506, debug[3]≈0.25."""
        pid = Float32PidBalance()
        pid.step(imu(-0.1), 0.02)
        out = pid.step(imu(-0.2), 0.02)

        assert out.cmd_rad_s == pytest.approx(0.4506, abs=1e-6)
        assert out.debug[3] == pytest.approx(0.25, abs=1e-6)

    def test_a11_pid_third_step_clamping(self):
        """A11: After A9-A10, step(imu(-1.5), 0.02) => raw≈4.7536, cmd==1.0 EXACTLY."""
        pid = Float32PidBalance()
        pid.step(imu(-0.1), 0.02)
        pid.step(imu(-0.2), 0.02)
        out = pid.step(imu(-1.5), 0.02)

        assert out.raw_cmd == pytest.approx(4.7536, abs=1e-5)
        assert out.cmd_rad_s == 1.0

    def test_a12_pid_zero_dt(self):
        """A12: dt==0, fresh step(imu(-0.3), 0.0) => cmd≈0.3, debug[2]==0.0 and debug[3]==0.0 exactly."""
        pid = Float32PidBalance()
        out = pid.step(imu(-0.3), 0.0)

        assert out.cmd_rad_s == pytest.approx(0.3, abs=1e-6)
        assert out.debug[2] == 0.0
        assert out.debug[3] == 0.0

    def test_a13_pid_negative_dt(self):
        """A13: dt<0 fresh step(imu(-0.1), -0.01) => cmd=0.0, fault=True; then step(imu(-0.1), 0.02)≈0.1002."""
        pid = Float32PidBalance()
        out1 = pid.step(imu(-0.1), -0.01)

        assert out1.cmd_rad_s == 0.0
        assert out1.fault is True

        out2 = pid.step(imu(-0.1), 0.02)
        assert out2.cmd_rad_s == pytest.approx(0.1002, abs=1e-6)

    def test_a14_pid_reset(self):
        """A14: After A9-A11 sequence, reset(), step(imu(-0.1), 0.02) => cmd≈0.1002, debug[3]==0.0."""
        pid = Float32PidBalance()
        pid.step(imu(-0.1), 0.02)
        pid.step(imu(-0.2), 0.02)
        pid.step(imu(-1.5), 0.02)
        pid.reset()

        out = pid.step(imu(-0.1), 0.02)
        assert out.cmd_rad_s == pytest.approx(0.1002, abs=1e-6)
        assert out.debug[3] == 0.0

    def test_a15_pid_invalid_out_limit(self):
        """A15: Float32PidBalance(out_limit=0) raises ValueError."""
        with pytest.raises(ValueError):
            Float32PidBalance(out_limit=0)


class TestImports:
    """Test AST structure and imports of f32_emulation.py and this test file."""

    def test_no_top_level_controller_import_f32(self):
        """No top-level import/import-from with module=='controller' in f32_emulation.py."""
        f32_file = Path(__file__).resolve().parent / "f32_emulation.py"
        f32_text = f32_file.read_text()
        f32_ast = ast.parse(f32_text)

        for node in f32_ast.body:
            if isinstance(node, ast.Import):
                for alias in node.names:
                    assert not alias.name == "controller", \
                        f"Top-level import of controller: {alias.name}"
            elif isinstance(node, ast.ImportFrom):
                if node.module:
                    assert not node.module == "controller", \
                        f"Top-level import from controller: {node.module}"

    def test_no_top_level_controller_import_test(self):
        """No top-level import/import-from with module=='controller' in this test file."""
        test_file = Path(__file__).resolve()
        test_text = test_file.read_text()
        test_ast = ast.parse(test_text)

        for node in test_ast.body:
            if isinstance(node, ast.Import):
                for alias in node.names:
                    assert not alias.name == "controller", \
                        f"Top-level import of controller: {alias.name}"
            elif isinstance(node, ast.ImportFrom):
                if node.module:
                    assert not node.module == "controller", \
                        f"Top-level import from controller: {node.module}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])


# --- PART B (comparison scenarios) appended below ---


# --- Module-level helpers for PART B tests ---

PERIODS = (0.016, 0.032)
N_TICKS = 2600
THETA_REF = -0.1086


def _lqr_run(core, theta0):
    """Run LQR control loop with fresh LinearPitchPlant."""
    hal = FakeHal(
        tick_periods_s=PERIODS,
        plant=LinearPitchPlant.from_plant_lqr(),
        theta0_rad=theta0,
        max_ticks=N_TICKS,
    )
    return run_closed_loop(core, hal, N_TICKS)


def _pid_run(core, pitches):
    """Run PID control loop with IMU script from pitches."""
    hal = FakeHal(
        tick_periods_s=PERIODS,
        imu_script=[ImuSample(0.0, 0.0, p, 0.0, (0.0, 0.0, 0.0)) for p in pitches],
        max_ticks=len(pitches),
    )
    return run_closed_loop(core, hal, len(pitches))


def _report(tag, metrics):
    """Print metrics in F32-MEASURED format."""
    formatted = {
        k: f"{v:.3e}" if isinstance(v, float) else v
        for k, v in metrics.items()
    }
    print("F32-MEASURED", tag, formatted)


class TestLqrClosedLoop:
    """Closed-loop LQR tests: reference vs float32 comparison."""

    @pytest.mark.parametrize("theta0", (0.02, 0.05))
    def test_b1_initial_conditions(self, theta0):
        """B1: Verify run length and pitch error bounds for both ref and f32."""
        K = LqrBalance().K
        ref = _lqr_run(LqrBalance(K=K), theta0)
        f32 = _lqr_run(Float32LqrBalance(K=K), theta0)

        # Both runs must have exactly N_TICKS records
        assert len(ref) == N_TICKS, f"ref len={len(ref)}, expected {N_TICKS}"
        assert len(f32) == N_TICKS, f"f32 len={len(f32)}, expected {N_TICKS}"

        # Check pitch error bounds for both runs
        for run, run_name in [(ref, "ref"), (f32, "f32")]:
            max_pitch_error = max(abs(r.pitch_rad - THETA_REF) for r in run)
            assert max_pitch_error <= 2 * theta0, \
                f"{run_name} max pitch error {max_pitch_error} > {2*theta0}"

            # Mean pitch error over last 100 records
            last_100 = run[-100:]
            mean_pitch_error = sum(abs(r.pitch_rad - THETA_REF) for r in last_100) / len(last_100)
            assert mean_pitch_error < 1e-4, \
                f"{run_name} mean pitch error {mean_pitch_error} >= 1e-4"

            # All commands finite and bounded
            for r in run:
                assert math.isfinite(r.cmd), f"{run_name} non-finite cmd: {r.cmd}"
                assert abs(r.cmd) <= 1.0, f"{run_name} cmd out of bounds: {r.cmd}"

    @pytest.mark.parametrize("theta0", (0.02, 0.05))
    def test_b2_comparison(self, theta0):
        """B2: Compare ref and f32 metrics via compare_runs."""
        K = LqrBalance().K
        ref = _lqr_run(LqrBalance(K=K), theta0)
        f32 = _lqr_run(Float32LqrBalance(K=K), theta0)

        m = compare_runs(ref, f32)

        assert m["max_abs_cmd"] <= F32_TOL_ABS, \
            f"max_abs_cmd {m['max_abs_cmd']} > {F32_TOL_ABS}"
        assert m["max_abs_raw"] <= F32_TOL_ABS, \
            f"max_abs_raw {m['max_abs_raw']} > {F32_TOL_ABS}"
        assert m["max_abs_pitch"] <= F32_TOL_STATE_PITCH_ABS, \
            f"max_abs_pitch {m['max_abs_pitch']} > {F32_TOL_STATE_PITCH_ABS}"
        assert m["max_abs_gyro"] <= F32_TOL_STATE_GYRO_ABS, \
            f"max_abs_gyro {m['max_abs_gyro']} > {F32_TOL_STATE_GYRO_ABS}"

        # Non-vacuity: check that commands actually differ between ref and f32
        assert m["max_abs_cmd"] > 0.0, "max_abs_cmd is zero (vacuous test)"

        _report(f"lqr-closed-loop theta0={theta0}", m)

    @pytest.mark.parametrize("theta0", (0.02, 0.05))
    def test_b3_replay(self, theta0):
        """B3: Replay IMU through fresh float32 instance."""
        K = LqrBalance().K
        ref = _lqr_run(LqrBalance(K=K), theta0)
        rep = replay_imu(Float32LqrBalance(K=K), ref)

        m = compare_runs(ref, rep)

        assert m["max_abs_cmd"] <= F32_TOL_ABS, \
            f"replay max_abs_cmd {m['max_abs_cmd']} > {F32_TOL_ABS}"
        assert m["max_abs_cmd"] > 0.0, "replay max_abs_cmd is zero"

        _report(f"lqr-replay theta0={theta0}", m)

    def test_b4_saturation(self):
        """B4: Check saturation behavior at theta0=0.05."""
        K = LqrBalance().K
        ref = _lqr_run(LqrBalance(K=K), 0.05)

        assert ref[0].cmd == 1.0, f"First cmd not saturated: {ref[0].cmd}"
        assert ref[0].raw == pytest.approx(1.82, abs=0.05), \
            f"First raw {ref[0].raw} not approx 1.82 ± 0.05"


class TestPidScripted:
    """Scripted PID tests: reference vs float32 agreement, repeatability, boundedness.

    Note: PID stability is NOT tested here (#359); only agreement, boundedness and repeatability.
    """

    def test_c1_basic_wave(self):
        """C1: 50-point sine wave agreement and repeatability."""
        pitches = [0.1 * math.sin(0.7 * k) - 0.02 for k in range(50)]

        ref = _pid_run(PidBalance(), pitches)
        f32 = _pid_run(Float32PidBalance(), pitches)

        m = compare_runs(ref, f32)
        assert m["max_abs_cmd"] <= F32_TOL_ABS, \
            f"max_abs_cmd {m['max_abs_cmd']} > {F32_TOL_ABS}"
        assert m["max_abs_raw"] <= F32_TOL_ABS, \
            f"max_abs_raw {m['max_abs_raw']} > {F32_TOL_ABS}"

        # Repeatability: run f32 twice with same input
        f32b = _pid_run(Float32PidBalance(), pitches)
        assert f32 == f32b, "Float32PidBalance not deterministic"

        _report("pid-50", m)

    def test_c2_stress(self):
        """C2: Stress test with saturation and integral clamping."""
        pitches = [-0.5] * 120 + [-2.0] + [0.5] * 60 + [-0.05] * 119

        ref = _pid_run(PidBalance(), pitches)

        # Verify ref shows saturation and integral clamping
        assert any(abs(r.raw) > 1.0 for r in ref), \
            "No raw values exceeded 1.0 (saturation not triggered)"
        assert any(abs(r.cmd) == 1.0 for r in ref), \
            "No commands clamped to ±1.0"
        assert any(abs(r.debug[2] - 0.1) < 1e-12 for r in ref), \
            "No integral terms near clamp limit 0.1"

        f32 = _pid_run(Float32PidBalance(), pitches)

        m = compare_runs(ref, f32)
        assert m["max_abs_cmd"] <= F32_TOL_ABS, \
            f"stress max_abs_cmd {m['max_abs_cmd']} > {F32_TOL_ABS}"
        assert m["max_abs_raw"] <= F32_TOL_ABS, \
            f"stress max_abs_raw {m['max_abs_raw']} > {F32_TOL_ABS}"

        # All commands in both runs must be bounded
        for r in ref:
            assert abs(r.cmd) <= 1.0, f"ref cmd out of bounds: {r.cmd}"
        for r in f32:
            assert abs(r.cmd) <= 1.0, f"f32 cmd out of bounds: {r.cmd}"

        _report("pid-stress", m)

    def test_c3_long(self):
        """C3: Long run with near-zero steady-state input."""
        pitches = [-0.5] * 40 + [-1e-6] * N_TICKS

        ref = _pid_run(PidBalance(), pitches)
        assert len(ref) * 0.024 >= 60.0, \
            f"Run too short: {len(ref)} ticks * 0.024 < 60.0 s"

        f32 = _pid_run(Float32PidBalance(), pitches)

        m = compare_runs(ref, f32)
        assert m["max_abs_cmd"] <= F32_TOL_PID_LONG_ABS, \
            f"long max_abs_cmd {m['max_abs_cmd']} > {F32_TOL_PID_LONG_ABS}"

        # All commands bounded
        for r in ref:
            assert abs(r.cmd) <= 1.0, f"ref long cmd out of bounds: {r.cmd}"
        for r in f32:
            assert abs(r.cmd) <= 1.0, f"f32 long cmd out of bounds: {r.cmd}"

        # Repeatability
        f32b = _pid_run(Float32PidBalance(), pitches)
        assert f32 == f32b, "Float32PidBalance not deterministic on long run"

        _report("pid-long", m)
