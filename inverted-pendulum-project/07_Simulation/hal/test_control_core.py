"""
Control core (PidBalance, LqrBalance) integration tests.

Pure Python unit tests with no Webots or hardware dependencies.
Tests validate PID discrete integration, state management, LQR state feedback,
output clamping, dt-handling (zero, negative), and import structure.

Usage:
    mamba run -n pendulum-tools python3 -m pytest -q inverted-pendulum-project/07_Simulation/hal/test_control_core.py
"""

import ast
import dataclasses
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from plant_pid import PlantPID
from hal.hal import ImuSample
from hal.control_core import (
    PidBalance,
    LqrBalance,
    ControlOutput,
    DEFAULT_THETA_REF_RAD,
    DEFAULT_PID_KP,
    DEFAULT_PID_KI,
    DEFAULT_PID_KD,
    DEFAULT_OUT_LIMIT,
)


def imu(pitch, gyro_y=0.0):
    """Construct ImuSample with given pitch and gyro_y (pitch rate)."""
    return ImuSample(0.0, 0.0, pitch, 0.0, (0.0, gyro_y, 0.0))


class TestPidBalance:
    """Test suite for discrete PID controller."""

    def test_p1_fresh_step_negative_pitch(self):
        """P1: Fresh step(imu(-0.1), 0.02) with defaults (kp=1, ki=0.1, kd=0.05)."""
        pid = PidBalance()
        out = pid.step(imu(-0.1), 0.02)

        assert out.cmd_rad_s == pytest.approx(0.1002, abs=1e-12)
        assert out.raw_cmd == pytest.approx(0.1002, abs=1e-12)
        assert out.debug[0] == pytest.approx(0.1, abs=1e-12)
        assert out.debug[1] == pytest.approx(0.1, abs=1e-12)
        assert out.debug[2] == pytest.approx(0.0002, abs=1e-12)
        assert out.debug[3] == pytest.approx(0.0, abs=1e-12)
        assert out.fault is False

    def test_p2_sequence_second_step(self):
        """P2: After P1, step(imu(-0.2), 0.02) gives p=0.2, i=0.0006, d=0.25, raw=0.4506."""
        pid = PidBalance()
        pid.step(imu(-0.1), 0.02)
        out = pid.step(imu(-0.2), 0.02)

        assert out.debug[0] == pytest.approx(0.2, abs=1e-12)
        assert out.debug[1] == pytest.approx(0.2, abs=1e-12)
        assert out.debug[2] == pytest.approx(0.0006, abs=1e-12)
        assert out.debug[3] == pytest.approx(0.25, abs=1e-12)
        assert out.cmd_rad_s == pytest.approx(0.4506, abs=1e-12)

    def test_p3_third_step_clamping(self):
        """P3: After P1-P2, step(imu(-1.5), 0.02) gives clamped output."""
        pid = PidBalance()
        pid.step(imu(-0.1), 0.02)
        pid.step(imu(-0.2), 0.02)
        out = pid.step(imu(-1.5), 0.02)

        assert out.debug[0] == pytest.approx(1.5, abs=1e-12)
        assert out.debug[1] == pytest.approx(1.5, abs=1e-12)
        assert out.debug[2] == pytest.approx(0.0036, abs=1e-12)
        assert out.debug[3] == pytest.approx(3.25, abs=1e-12)
        assert out.raw_cmd == pytest.approx(4.7536, abs=1e-12)
        assert out.cmd_rad_s == pytest.approx(1.0, abs=1e-12)

    def test_p4_large_dt(self):
        """P4: Fresh step(imu(-5.0), 1.0) dt integration over 1 second."""
        pid = PidBalance()
        out = pid.step(imu(-5.0), 1.0)

        assert out.debug[2] == pytest.approx(0.1, abs=1e-12)
        assert out.debug[1] == pytest.approx(5.0, abs=1e-12)
        assert out.debug[3] == pytest.approx(0.0, abs=1e-12)
        assert out.raw_cmd == pytest.approx(5.1, abs=1e-12)
        assert out.cmd_rad_s == pytest.approx(1.0, abs=1e-12)

    def test_p5_dt_zero(self):
        """P5: dt==0 fresh step(imu(-0.3), 0.0) returns P-only, then step(imu(-2.0), 0.0)."""
        pid = PidBalance()
        out1 = pid.step(imu(-0.3), 0.0)

        assert out1.cmd_rad_s == pytest.approx(0.3, abs=1e-12)
        assert out1.raw_cmd == pytest.approx(0.3, abs=1e-12)
        assert out1.debug == (0.3, 0.3, 0.0, 0.0)

        out2 = pid.step(imu(-2.0), 0.0)
        assert out2.raw_cmd == pytest.approx(2.0, abs=1e-12)
        assert out2.cmd_rad_s == pytest.approx(1.0, abs=1e-12)

    def test_p6_mixed_dt_sequence(self):
        """P6: Sequence step(-0.1, 0.02), step(-0.2, 0.0), step(-0.2, 0.02)."""
        pid = PidBalance()
        pid.step(imu(-0.1), 0.02)
        pid.step(imu(-0.2), 0.0)
        out3 = pid.step(imu(-0.2), 0.02)

        assert out3.cmd_rad_s == pytest.approx(0.4506, abs=1e-12)

    def test_p7_negative_dt(self):
        """P7: dt<0 fresh step returns 0, fault=True; following normal step works."""
        pid = PidBalance()
        out1 = pid.step(imu(-0.1), -0.01)

        assert out1.cmd_rad_s == pytest.approx(0.0, abs=1e-12)
        assert out1.raw_cmd == pytest.approx(0.0, abs=1e-12)
        assert out1.fault is True
        assert out1.debug[0] == pytest.approx(0.1, abs=1e-12)
        assert out1.debug[1] == pytest.approx(0.0, abs=1e-12)
        assert out1.debug[2] == pytest.approx(0.0, abs=1e-12)
        assert out1.debug[3] == pytest.approx(0.0, abs=1e-12)

        out2 = pid.step(imu(-0.1), 0.02)
        assert out2.cmd_rad_s == pytest.approx(0.1002, abs=1e-12)

    def test_p8_reset(self):
        """P8: After P1-P3, reset(), step matches P1."""
        pid = PidBalance()
        pid.step(imu(-0.1), 0.02)
        pid.step(imu(-0.2), 0.02)
        pid.step(imu(-1.5), 0.02)
        pid.reset()

        out = pid.step(imu(-0.1), 0.02)
        assert out.cmd_rad_s == pytest.approx(0.1002, abs=1e-12)
        assert out.debug == pytest.approx((0.1, 0.1, 0.0002, 0.0), abs=1e-12)

        pid2 = PidBalance()
        out_fresh = pid2.step(imu(-0.3), 0.0)
        assert out_fresh.debug[2] == pytest.approx(0.0, abs=1e-12)
        assert out_fresh.debug[3] == pytest.approx(0.0, abs=1e-12)

    def test_p9_equivalence_with_plant_pid(self):
        """P9: Equivalence check with PlantPID reference."""
        pid_core = PidBalance(1.0, 0.1, 0.05, 1.0)
        pid_ref = PlantPID(1.0, 0.1, 0.05, -1.0, 1.0)

        pitch_vals = [0.1 * math.sin(0.7 * k) - 0.02 for k in range(20)]
        dt_vals = [0.016 if k % 2 == 0 else 0.032 for k in range(20)]

        for pitch, dt in zip(pitch_vals, dt_vals):
            out_core = pid_core.step(imu(-pitch), dt)
            out_ref = pid_ref.step(pitch, dt)

            assert out_core.cmd_rad_s == pytest.approx(out_ref, abs=1e-12)

    def test_p10_invalid_out_limit_and_attrs(self):
        """P10: PidBalance(out_limit=0) and (out_limit=-1) raise ValueError; public attrs check."""
        with pytest.raises(ValueError):
            PidBalance(out_limit=0)

        with pytest.raises(ValueError):
            PidBalance(out_limit=-1)

        pid = PidBalance()
        assert pid.kp == 1.0
        assert pid.ki == 0.1
        assert pid.kd == 0.05
        assert pid.out_limit == 1.0


class TestLqrBalance:
    """Test suite for LQR state feedback controller."""

    def test_l1_at_reference_zero_error(self):
        """L1: pitch=-0.1086 (default ref), gyro=0 => raw==0.0, cmd==0.0."""
        lqr = LqrBalance(K=(10.0, 2.0))
        out = lqr.step(imu(-0.1086, 0.0), 0.02)

        assert out.raw_cmd == 0.0
        assert out.cmd_rad_s == 0.0
        assert out.debug[0] == pytest.approx(0.0, abs=1e-12)
        assert out.debug[1] == pytest.approx(0.0, abs=1e-12)

    def test_l2_pitch_positive_error(self):
        """L2: pitch=-0.0586 (θ=0.05 from ref), gyro=0 => raw=cmd=-0.5."""
        lqr = LqrBalance(K=(10.0, 2.0))
        out = lqr.step(imu(-0.0586, 0.0), 0.02)

        assert out.raw_cmd == pytest.approx(-0.5, abs=1e-12)
        assert out.cmd_rad_s == pytest.approx(-0.5, abs=1e-12)
        assert out.debug[0] == pytest.approx(0.05, abs=1e-12)

    def test_l3_with_gyro_rate(self):
        """L3: pitch=-0.0586, gyro=0.1 => raw=cmd=-0.7."""
        lqr = LqrBalance(K=(10.0, 2.0))
        out = lqr.step(imu(-0.0586, 0.1), 0.02)

        assert out.raw_cmd == pytest.approx(-0.7, abs=1e-12)
        assert out.cmd_rad_s == pytest.approx(-0.7, abs=1e-12)
        assert out.debug == pytest.approx((0.05, 0.1), abs=1e-12)

    def test_l4_large_error_clamped(self):
        """L4: pitch=0.0914, gyro=0.5 => theta=0.2, raw=-3.0, cmd=-1.0."""
        lqr = LqrBalance(K=(10.0, 2.0))
        out = lqr.step(imu(0.0914, 0.5), 0.02)

        assert out.raw_cmd == pytest.approx(-3.0, abs=1e-12)
        assert out.cmd_rad_s == pytest.approx(-1.0, abs=1e-12)

    def test_l5_positive_clamp(self):
        """L5: pitch=-0.2086, gyro=-1.5 => theta=-0.1, raw=4.0, cmd=1.0."""
        lqr = LqrBalance(K=(10.0, 2.0))
        out = lqr.step(imu(-0.2086, -1.5), 0.02)

        assert out.raw_cmd == pytest.approx(4.0, abs=1e-12)
        assert out.cmd_rad_s == pytest.approx(1.0, abs=1e-12)

    def test_l6_custom_out_limit(self):
        """L6: out_limit=0.5, pitch=-0.0586, gyro=0.1 => raw=-0.7, cmd=-0.5."""
        lqr = LqrBalance(K=(10.0, 2.0), out_limit=0.5)
        out = lqr.step(imu(-0.0586, 0.1), 0.02)

        assert out.raw_cmd == pytest.approx(-0.7, abs=1e-12)
        assert out.cmd_rad_s == pytest.approx(-0.5, abs=1e-12)

    def test_l7_custom_theta_ref(self):
        """L7: theta_ref_rad=0.0, pitch=0.05, gyro=0 => raw=cmd=-0.5."""
        lqr = LqrBalance(K=(10.0, 2.0), theta_ref_rad=0.0)
        out = lqr.step(imu(0.05, 0.0), 0.02)

        assert out.raw_cmd == pytest.approx(-0.5, abs=1e-12)
        assert out.cmd_rad_s == pytest.approx(-0.5, abs=1e-12)

    def test_l8_dt_ignored(self):
        """L8: dt in {0.0, 0.016, 0.032} gives identical cmd/raw, fault=False."""
        lqr = LqrBalance(K=(10.0, 2.0))

        for dt in [0.0, 0.016, 0.032]:
            out = lqr.step(imu(-0.0586, 0.1), dt)
            assert out.raw_cmd == pytest.approx(-0.7, abs=1e-12)
            assert out.cmd_rad_s == pytest.approx(-0.7, abs=1e-12)
            assert out.fault is False

    def test_l9_negative_dt_fault(self):
        """L9: dt=-0.001 => cmd=0.0, raw=0.0, fault=True, debug=(theta, gyro)."""
        lqr = LqrBalance(K=(10.0, 2.0))
        out = lqr.step(imu(-0.0586, 0.1), -0.001)

        assert out.cmd_rad_s == pytest.approx(0.0, abs=1e-12)
        assert out.raw_cmd == pytest.approx(0.0, abs=1e-12)
        assert out.fault is True
        assert out.debug[0] == pytest.approx(0.05, abs=1e-12)
        assert out.debug[1] == pytest.approx(0.1, abs=1e-12)

    def test_l10_k_validation_and_attrs(self):
        """L10: K as list accepted, len(K)!=2 raises ValueError, out_limit=0 raises."""
        lqr = LqrBalance(K=[10.0, 2.0])
        assert lqr.K == (10.0, 2.0)

        with pytest.raises(ValueError):
            LqrBalance(K=[10.0, 2.0, 3.0])

        with pytest.raises(ValueError):
            LqrBalance(out_limit=0)

    def test_l11_default_gain(self):
        """L11: LqrBalance() without K uses default_gain(); K values match plant_lqr."""
        try:
            from plant_lqr import default_gain
        except ImportError:
            pytest.skip("plant_lqr not available")

        lqr = LqrBalance()
        g = default_gain()
        assert lqr.K[0] == pytest.approx(g[0][0], rel=1e-4)
        assert lqr.K[1] == pytest.approx(g[0][1], rel=1e-4)

        assert lqr.K[0] == pytest.approx(-23.81842951, rel=1e-4)
        assert lqr.K[1] == pytest.approx(-3.49402271, rel=1e-4)

        out1 = lqr.step(imu(-0.0986, 0.1), 0.02)
        assert out1.debug[0] == pytest.approx(0.01, abs=1e-9)
        assert out1.raw_cmd == pytest.approx(0.5875865661, abs=1e-6)
        assert out1.cmd_rad_s == pytest.approx(out1.raw_cmd, abs=1e-12)

        out2 = lqr.step(imu(0.0914, 0.0), 0.02)
        assert out2.raw_cmd == pytest.approx(4.76, abs=0.05)
        assert out2.cmd_rad_s == pytest.approx(1.0, abs=1e-12)

    def test_l12_monkeypatch_imports(self, monkeypatch):
        """L12: numpy/scipy monkeypatched to None; LqrBalance still works."""
        monkeypatch.setitem(sys.modules, "numpy", None)
        monkeypatch.setitem(sys.modules, "scipy", None)

        lqr = LqrBalance(K=(10.0, 2.0))
        out = lqr.step(imu(-0.0586, 0.0), 0.02)

        assert out.cmd_rad_s == pytest.approx(-0.5, abs=1e-12)

    def test_l13_default_theta_ref_and_frozen(self):
        """L13: DEFAULT_THETA_REF_RAD==-0.1086; ControlOutput frozen."""
        assert DEFAULT_THETA_REF_RAD == -0.1086

        out = ControlOutput(0.5, 0.7, (0.1, 0.2), False)
        with pytest.raises(dataclasses.FrozenInstanceError):
            out.cmd_rad_s = 0.6

        fields = [f.name for f in dataclasses.fields(ControlOutput)]
        assert fields == ["cmd_rad_s", "raw_cmd", "debug", "fault"]


class TestControlCoreImports:
    """Test AST structure and imports of control_core.py."""

    def test_no_top_level_controller_import(self):
        """No top-level import/import-from with module=='controller'."""
        core_file = Path(__file__).resolve().parent / "control_core.py"
        core_text = core_file.read_text()
        core_ast = ast.parse(core_text)

        for node in core_ast.body:
            if isinstance(node, ast.Import):
                for alias in node.names:
                    assert not alias.name.startswith("controller"), \
                        f"Top-level import of controller: {alias.name}"
            elif isinstance(node, ast.ImportFrom):
                if node.module:
                    assert not node.module.startswith("controller"), \
                        f"Top-level import from controller: {node.module}"

    def test_top_level_imports_allowed_set(self):
        """Top-level imported modules must be subset of {sys,pathlib,dataclasses,plant_pid,hal}."""
        core_file = Path(__file__).resolve().parent / "control_core.py"
        core_text = core_file.read_text()
        core_ast = ast.parse(core_text)

        allowed = {"sys", "pathlib", "dataclasses", "plant_pid", "hal"}
        found = set()

        for node in core_ast.body:
            if isinstance(node, ast.Import):
                for alias in node.names:
                    top_mod = alias.name.split(".")[0]
                    found.add(top_mod)
            elif isinstance(node, ast.ImportFrom):
                if node.level == 0 and node.module:
                    top_mod = node.module.split(".")[0]
                    found.add(top_mod)

        assert found.issubset(allowed), \
            f"Top-level imports not in allowed set: {found - allowed}"

    def test_plant_lqr_imported_nested(self):
        """plant_lqr is imported in some nested (non-module-level) context."""
        core_file = Path(__file__).resolve().parent / "control_core.py"
        core_text = core_file.read_text()
        core_ast = ast.parse(core_text)

        found_nested = False
        for node in ast.walk(core_ast):
            if isinstance(node, ast.ImportFrom):
                if node.module and "plant_lqr" in node.module:
                    if node not in core_ast.body:
                        found_nested = True
                        break

        assert found_nested, "plant_lqr not found in nested ImportFrom"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
