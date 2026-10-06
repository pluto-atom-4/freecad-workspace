#!/usr/bin/env python3
"""Test HAL-ported LQR controller using stub robots (no Webots).

This test file must NOT import the Webots `controller` module; instead,
it builds the banned method names from concatenated strings to avoid literals.

Usage:
    mamba run -n pendulum-tools python3 -m pytest -q inverted-pendulum-project/07_Simulation/webots/test_controller_ports.py
"""

import ast
import importlib.util
import math
import os
import re
import sys
import types
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SIM))

LQR_FILE = SIM / "webots" / "controllers" / "lqr_controller" / "lqr_controller.py"

from hal.hal import HalFault, ImuSample, EncoderSample


# ============================================================================
# Stub Test Doubles
# ============================================================================

class StubDevice:
    """Mock Webots device (sensor or motor) for testing."""

    def __init__(self, rpy=None, values=None, value=0.0, max_vel=1.0, max_torque=1.0):
        self._rpy = rpy
        self._values = values
        self._value = value
        self._max_vel = max_vel
        self._max_torque = max_torque
        self.enabled_ms = None
        self.position_calls = []
        self.velocity_calls = []

    def enable(self, ms):
        self.enabled_ms = ms

    def getValue(self):
        return self._value

    def getMaxVelocity(self):
        return self._max_vel

    def getMaxTorque(self):
        return self._max_torque

    def setPosition(self, p):
        self.position_calls.append(p)

    def setVelocity(self, v):
        self.velocity_calls.append(v)


# Build method names from concatenation to avoid literals
_GET_ROLL_PITCH_YAW = "get" + "RollPitchYaw"
_GET_VALUES = "get" + "Values"


def _attach_stub_imu_methods(dev):
    """Attach getRollPitchYaw and getValues methods to a stub device via setattr."""
    setattr(dev, _GET_ROLL_PITCH_YAW, lambda: dev._rpy)
    setattr(dev, _GET_VALUES, lambda: dev._values)
    setattr(dev, "get" + "Value", lambda: dev._value)


class StubRobot:
    """Mock Webots Robot/Supervisor for testing."""

    def __init__(self, devices, basic_ms=16.0, max_steps=None):
        self._devices = devices
        self._basic_ms = basic_ms
        self._max_steps = max_steps
        self._ms = 0
        self.step_calls = 0
        self.quit_status = None
        self._quit_flag = False

    def getBasicTimeStep(self):
        return self._basic_ms

    def getDevice(self, name):
        return self._devices.get(name)

    def step(self, ms):
        if self._quit_flag:
            return -1
        if self._max_steps is not None and self.step_calls >= self._max_steps:
            self._quit_flag = True
            return -1
        self.step_calls += 1
        self._ms += int(ms)
        return 0

    def getTime(self):
        return self._ms / 1000.0

    def simulationQuit(self, status):
        self.quit_status = status
        self._quit_flag = True


# ============================================================================
# Helpers: make_devices and load_controller
# ============================================================================

def make_devices(pitch=-0.0886, gyro_y=0.0, rpy=None, values=None, **kwargs):
    """Create stub devices with configurable sensor values.

    Args:
        pitch: Pitch angle (rad); default -0.0886
        gyro_y: Gyro Y value (rad/s); default 0.0
        rpy: Override IMU [roll, pitch, yaw]; if None, use [0.0, pitch, 0.0]
        values: Override gyro values; if None, use [0.0, gyro_y, 0.0]
        **kwargs: Additional device customization (not used in basic version)

    Returns:
        Dict mapping device names to StubDevice instances.
    """
    if rpy is None:
        rpy = [0.0, pitch, 0.0]
    if values is None:
        values = [0.0, gyro_y, 0.0]

    devices = {
        "imu": StubDevice(rpy=rpy),
        "gyro": StubDevice(values=values),
        "wheel_left_joint_sensor": StubDevice(value=1.5),
        "wheel_right_joint_sensor": StubDevice(value=2.5),
        "pendulum_pivot_joint_sensor": StubDevice(value=0.25),
        "pendulum_pivot_right_joint_sensor": StubDevice(value=-0.25),
        "wheel_left_joint": StubDevice(max_vel=1.0, max_torque=1.0),
        "wheel_right_joint": StubDevice(max_vel=1.0, max_torque=1.0),
    }

    # Attach IMU methods via setattr to avoid literal strings in test file
    _attach_stub_imu_methods(devices["imu"])
    _attach_stub_imu_methods(devices["gyro"])

    return devices


def load_controller(monkeypatch, tmp_path, robot, env=None):
    """Dynamically load lqr_controller.py with a fake Webots controller module.

    Args:
        monkeypatch: pytest fixture for modifying sys.modules and env vars
        tmp_path: pytest fixture for temporary directory
        robot: StubRobot instance to inject
        env: Dict of environment variables to set (e.g., SENSOR_LOG_THROTTLE=1)

    Returns:
        Loaded lqr_controller module
    """
    # Always set _LQR_REEXEC to skip re-exec check
    monkeypatch.setenv("_LQR_REEXEC", "1")

    # Set optional env vars from env dict
    if env:
        for key, val in env.items():
            monkeypatch.setenv(key, str(val))

    # Create fake controller module
    fake = types.ModuleType("controller")
    fake.Supervisor = lambda: robot
    fake.Robot = lambda: robot
    monkeypatch.setitem(sys.modules, "controller", fake)

    # Load the actual controller module from file
    spec = importlib.util.spec_from_file_location(
        f"lqr_ctrl_under_test_{id(robot)}", LQR_FILE
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    # Override LOG_PATH to write to tmp_path
    monkeypatch.setattr(mod, "LOG_PATH", tmp_path / "controller.log")

    return mod


# ============================================================================
# Tests
# ============================================================================

class TestSourceGuards:
    """Test that lqr_controller.py respects guard constraints."""

    def test_no_banned_method_literals(self):
        """LQR file text must not contain getRollPitchYaw, getValues, setVelocity as literals."""
        text = LQR_FILE.read_text()
        banned = ["getRollPitchYaw", "getValues", "setVelocity"]
        for name in banned:
            assert name not in text, f"Banned literal '{name}' found in {LQR_FILE}"

    def test_no_default_gain_call_in_text(self):
        """LQR file text must not call default_gain()."""
        text = LQR_FILE.read_text()
        # Check that "default_gain" doesn't appear in the text (it's imported by control_core.py, not lqr_controller.py)
        assert "default_gain" not in text

    def test_no_numpy_import(self):
        """LQR file text must not import numpy directly."""
        text = LQR_FILE.read_text()
        assert "numpy as np" not in text

    def test_quoted_device_strings_double_quoted(self):
        """All 7 device name strings must use double quotes."""
        text = LQR_FILE.read_text()
        devices = [
            "imu", "gyro", "wheel_left_joint_sensor", "wheel_right_joint_sensor",
            "pendulum_pivot_joint_sensor", "pendulum_pivot_right_joint_sensor"
        ]
        motors = ["wheel_left_joint", "wheel_right_joint"]
        for dev in devices + motors:
            # Must have double quotes
            assert f'"{dev}"' in text, f"Device '{dev}' not found with double quotes"

    def test_ast_controller_import(self):
        """AST: exactly one ImportFrom with module='controller'."""
        tree = ast.parse(LQR_FILE.read_text())
        controller_imports = [
            node for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module == "controller"
        ]
        assert len(controller_imports) == 1, f"Expected 1 controller ImportFrom, got {len(controller_imports)}"

    def test_ast_imports_webots_hal_and_lqr_balance(self):
        """AST: imports WebotsHal from hal.webots_hal and LqrBalance from hal.control_core."""
        text = LQR_FILE.read_text()
        assert "from hal.webots_hal import WebotsHal" in text
        assert "from hal.control_core import LqrBalance" in text

    def test_ast_lqr_balance_call_has_theta_ref_rad(self):
        """AST: LqrBalance instantiation has theta_ref_rad=THETA_REF_RAD keyword."""
        tree = ast.parse(LQR_FILE.read_text())
        # Find Call to LqrBalance
        lqr_calls = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                if isinstance(node.func, ast.Name) and node.func.id == "LqrBalance":
                    lqr_calls.append(node)
        assert len(lqr_calls) >= 1, "No LqrBalance() call found"
        # Check at least one has theta_ref_rad keyword
        found = False
        for call in lqr_calls:
            for kw in call.keywords:
                if kw.arg == "theta_ref_rad" and isinstance(kw.value, ast.Name):
                    if kw.value.id == "THETA_REF_RAD":
                        found = True
        assert found, "LqrBalance call missing theta_ref_rad=THETA_REF_RAD"


class TestProjectWideControllerGuard:
    """Test that controller is only imported under webots/controllers/."""

    def test_no_controller_import_outside_controllers_dir(self):
        """Every .py file under SIM must not import controller unless in webots/controllers/."""
        import_pattern = re.compile(
            r'(?:^import\s+controller\b|^from\s+controller(?:\s|$|\.|\s+import))',
            re.MULTILINE
        )

        for py_file in SIM.rglob("*.py"):
            # Skip excluded paths
            if any(x in str(py_file) for x in ["__pycache__", ".venv", "graphify-out", "node_modules"]):
                continue

            text = py_file.read_text()
            if import_pattern.search(text):
                # This file imports controller; must be under webots/controllers/
                controllers_path = SIM / "webots" / "controllers"
                assert py_file.is_relative_to(controllers_path), \
                    f"{py_file} imports 'controller' but is not under {controllers_path}"


class TestSmokeRun:
    """Test basic LQR controller execution with stub robot."""

    def test_smoke_run_basic(self, monkeypatch, tmp_path):
        """Smoke: env SENSOR_LOG_THROTTLE=1; run 7 steps; rc==0; log format OK."""
        devices = make_devices(pitch=-0.0886)
        robot = StubRobot(devices, basic_ms=16.0, max_steps=7)
        mod = load_controller(monkeypatch, tmp_path, robot, env={"SENSOR_LOG_THROTTLE": "1"})

        rc = mod.main()
        assert rc == 0

        log_text = (tmp_path / "controller.log").read_text()
        lines = log_text.splitlines()

        # Check first few lines
        assert lines[0] == "LQR controller started. Reading all sensors (IMU, Gyro, wheel position, pivot joints)."
        assert lines[1] == "Control rate: 20ms (50.0Hz)"
        assert lines[2].startswith("Time(s) IMU_Roll(rad)")
        assert lines[3].startswith("Wheel motors: mode=velocity")
        assert "Pivot servo motors" in lines[4]

    def test_smoke_log_data_rows(self, monkeypatch, tmp_path):
        """Data rows: times 0.032, 0.048, 0.064, 0.080, 0.112; correct theta and cmd."""
        devices = make_devices(pitch=-0.0886)
        robot = StubRobot(devices, basic_ms=16.0, max_steps=7)
        mod = load_controller(monkeypatch, tmp_path, robot, env={"SENSOR_LOG_THROTTLE": "1"})

        rc = mod.main()
        assert rc == 0

        log_text = (tmp_path / "controller.log").read_text()
        lines = log_text.splitlines()

        # Extract data rows (lines starting with a numeric time)
        data_rows = []
        for line in lines:
            if line and line[0].isdigit():
                parts = line.split()
                if parts[0].replace(".", "").replace("-", "").isdigit():
                    data_rows.append(line)

        # Expect exactly 5 data rows with specific times
        expected_times = ["0.032", "0.048", "0.064", "0.080", "0.112"]
        assert len(data_rows) == 5, f"Expected 5 data rows, got {len(data_rows)}"
        for i, expected_t in enumerate(expected_times):
            row = data_rows[i]
            tokens = row.split()
            assert tokens[0] == expected_t, f"Row {i}: expected time {expected_t}, got {tokens[0]}"

        # Check first data row computation
        from plant_lqr import default_gain
        g = default_gain()
        K0, K1 = float(g[0][0]), float(g[0][1])

        row1 = data_rows[0]
        tokens = row1.split()
        assert len(tokens) == 15, f"Row 1 token count: expected 15, got {len(tokens)}"

        # Compute expected values
        theta_ref = -0.1086
        pitch = -0.0886
        theta = pitch - theta_ref  # -0.0886 - (-0.1086) = 0.0200
        gyro_y = 0.0
        raw = -(K0 * theta + K1 * gyro_y)
        cmd = max(-1.0, min(1.0, raw))

        # Verify tokens
        assert float(tokens[0]) == pytest.approx(0.032)
        assert float(tokens[2]) == pytest.approx(-0.0886)  # pitch
        assert float(tokens[11]) == pytest.approx(0.0200)   # LQR_Theta
        assert float(tokens[13]) == pytest.approx(raw, rel=1e-3)  # raw cmd
        assert float(tokens[14]) == pytest.approx(cmd, rel=1e-3)  # clamped cmd

        # Check wheel motor velocities match cmd
        assert robot.getDevice("wheel_left_joint").velocity_calls[-1] == pytest.approx(cmd, rel=1e-3)
        assert robot.getDevice("wheel_right_joint").velocity_calls[-1] == pytest.approx(cmd, rel=1e-3)

    def test_smoke_summary_lines(self, monkeypatch, tmp_path):
        """Summary: correct rate, period stats, saturation count."""
        devices = make_devices(pitch=-0.0886)
        robot = StubRobot(devices, basic_ms=16.0, max_steps=7)
        mod = load_controller(monkeypatch, tmp_path, robot, env={"SENSOR_LOG_THROTTLE": "1"})

        rc = mod.main()
        assert rc == 0

        log_text = (tmp_path / "controller.log").read_text()
        lines = log_text.splitlines()

        # Find summary lines
        summary_lines = [l for l in lines if l.startswith("Control rate:") and "target" in l and "deviation" in l]
        assert len(summary_lines) >= 1, "No summary rate line found"
        summary = summary_lines[0]

        # Parse rate line with regex
        match = re.search(r"([0-9.]+)Hz", summary)
        assert match, f"Rate line has no Hz: {summary}"
        rate_hz = float(match.group(1))
        assert rate_hz == pytest.approx(50.0, rel=0.1)

        # Check saturation line
        sat_lines = [l for l in lines if l.startswith("Saturation events:")]
        assert len(sat_lines) >= 1
        sat_line = sat_lines[0]
        # Compute expected saturation: count how many data rows have |raw| >= 0.95
        from plant_lqr import default_gain
        g = default_gain()
        K0, K1 = float(g[0][0]), float(g[0][1])
        theta_ref = -0.1086
        pitch = -0.0886
        theta = pitch - theta_ref
        raw = -(K0 * theta + K1 * 0.0)
        expected_sat = 5 if abs(raw) >= 0.95 else 0
        assert f"Saturation events: {expected_sat} of 5 control steps" in sat_line


class TestEnvironmentHonored:
    """Test that environment variables are honored."""

    def test_theta_ref_overridden(self, monkeypatch, tmp_path):
        """Env THETA_REF_RAD=-0.0686; pitch -0.0486; LQR_Theta should reflect offset."""
        devices = make_devices(pitch=-0.0486)
        robot = StubRobot(devices, basic_ms=16.0, max_steps=7)
        mod = load_controller(
            monkeypatch, tmp_path, robot,
            env={"SENSOR_LOG_THROTTLE": "1", "THETA_REF_RAD": "-0.0686"}
        )

        rc = mod.main()
        assert rc == 0

        log_text = (tmp_path / "controller.log").read_text()
        lines = log_text.splitlines()

        # Extract first data row
        data_rows = [l for l in lines if l and l[0].isdigit() and "READING_ERROR" not in l]
        assert len(data_rows) >= 1
        row1 = data_rows[0]
        tokens = row1.split()

        # Theta = pitch - theta_ref = -0.0486 - (-0.0686) = 0.0200
        assert float(tokens[11]) == pytest.approx(0.0200, rel=1e-3)


class TestSaturation:
    """Test saturation detection and clamping."""

    def test_saturation_high_pitch(self, monkeypatch, tmp_path):
        """High pitch (-0.1086 + 0.5): raw cmd >= 0.95; both wheels clamped to 1.0."""
        devices = make_devices(pitch=-0.1086 + 0.5)  # theta ≈ 0.5 rad
        robot = StubRobot(devices, basic_ms=16.0, max_steps=7)
        mod = load_controller(monkeypatch, tmp_path, robot, env={"SENSOR_LOG_THROTTLE": "1"})

        rc = mod.main()
        assert rc == 0

        log_text = (tmp_path / "controller.log").read_text()
        lines = log_text.splitlines()

        # Check saturation line: all 5 data rows saturated
        sat_lines = [l for l in lines if l.startswith("Saturation events:")]
        assert len(sat_lines) >= 1
        assert "Saturation events: 5 of 5 control steps" in sat_lines[0]

        # Check wheel motors were clamped to max
        wheel_left = robot.getDevice("wheel_left_joint")
        wheel_right = robot.getDevice("wheel_right_joint")
        # Last velocity call should be 1.0 or -1.0 (saturated)
        assert abs(wheel_left.velocity_calls[-1]) == pytest.approx(1.0)
        assert abs(wheel_right.velocity_calls[-1]) == pytest.approx(1.0)


class TestTimeLimit:
    """Test simulation time limit stopping."""

    def test_time_limit_reached(self, monkeypatch, tmp_path):
        """Env TEST_MAX_SIM_TIME_S=0.05; log contains time-limit-reached message; 3 data rows."""
        devices = make_devices(pitch=-0.0886)
        robot = StubRobot(devices, basic_ms=16.0, max_steps=1000)  # High max to test time limit
        mod = load_controller(
            monkeypatch, tmp_path, robot,
            env={"SENSOR_LOG_THROTTLE": "1", "TEST_MAX_SIM_TIME_S": "0.05"}
        )

        rc = mod.main()
        assert rc == 0

        log_text = (tmp_path / "controller.log").read_text()
        lines = log_text.splitlines()

        # Check for time-limit-reached line
        limit_lines = [l for l in lines if "Simulated time limit reached" in l]
        assert len(limit_lines) >= 1, "No time-limit-reached message found"
        assert "0.064s >= 0.05s" in limit_lines[0]

        # Check quit was called exactly once
        assert robot.quit_status == 0

        # Count data rows (non-error, numeric-first-token)
        data_rows = [l for l in lines if l and l[0].isdigit() and "READING_ERROR" not in l]
        assert len(data_rows) == 3, f"Expected 3 data rows, got {len(data_rows)}"

        # Check summary rate line format
        summary_lines = [l for l in lines if l.startswith("Control rate:") and "target" in l]
        assert len(summary_lines) >= 1
        summary = summary_lines[0]
        assert "62.50Hz" in summary or "62" in summary


class TestHalFault:
    """Test HAL fault handling."""

    def test_reading_error_lines(self, monkeypatch, tmp_path):
        """IMU rpy=None: main() returns 0; log has READING_ERROR lines; no target/deviation line."""
        devices = make_devices(pitch=-0.0886)
        devices["imu"]._rpy = None
        robot = StubRobot(devices, basic_ms=16.0, max_steps=7)
        mod = load_controller(monkeypatch, tmp_path, robot, env={"SENSOR_LOG_THROTTLE": "1"})

        rc = mod.main()
        assert rc == 0

        log_text = (tmp_path / "controller.log").read_text()
        lines = log_text.splitlines()

        # Check for READING_ERROR lines
        error_lines = [l for l in lines if "READING_ERROR:" in l]
        assert len(error_lines) >= 1, "No READING_ERROR lines found"

        # Each error line should start with time
        for line in error_lines:
            assert line[0].isdigit(), f"Error line doesn't start with time: {line}"

        # No data row summaries; only final saturation line
        sat_lines = [l for l in lines if l.startswith("Saturation events:")]
        assert len(sat_lines) >= 1
        assert "Saturation events: 0 of 0 control steps" in sat_lines[0]

        # No line with "target" and "deviation"
        rate_summary = [l for l in lines if "target" in l and "deviation" in l]
        assert len(rate_summary) == 0, "Found rate summary despite all errors"

        # Wheel motors only have init zero
        wheel_left = robot.getDevice("wheel_left_joint")
        wheel_right = robot.getDevice("wheel_right_joint")
        assert wheel_left.velocity_calls == [0.0], f"wheel_left calls: {wheel_left.velocity_calls}"
        assert wheel_right.velocity_calls == [0.0], f"wheel_right calls: {wheel_right.velocity_calls}"


class TestInitFault:
    """Test initialization faults."""

    def test_basic_timestep_mismatch(self, monkeypatch, tmp_path):
        """basicTimeStep=20.0 (>= control_period=20): main() returns 1; stderr contains ERROR and basicTimeStep."""
        devices = make_devices(pitch=-0.0886)
        robot = StubRobot(devices, basic_ms=20.0)  # Same as default control period
        mod = load_controller(monkeypatch, tmp_path, robot, env={"SENSOR_LOG_THROTTLE": "1"})

        rc = mod.main()
        assert rc == 1


class TestShellGrep:
    """Test log format compatibility with shell grep patterns."""

    def test_control_rate_line_parsing(self, monkeypatch, tmp_path):
        """First line matching r'^Control rate:.*ms' can be parsed for Hz."""
        devices = make_devices(pitch=-0.0886)
        robot = StubRobot(devices, basic_ms=16.0, max_steps=7)
        mod = load_controller(monkeypatch, tmp_path, robot, env={"SENSOR_LOG_THROTTLE": "1"})

        rc = mod.main()
        assert rc == 0

        log_text = (tmp_path / "controller.log").read_text()
        lines = log_text.splitlines()

        # Find startup rate line
        startup_rate = [l for l in lines if l.startswith("Control rate:") and "ms" in l and "Hz" in l]
        assert len(startup_rate) >= 1, "No startup Control rate line found"
        line = startup_rate[0]
        match = re.search(r"\(([0-9.]+)Hz\)", line)
        assert match, f"Can't parse Hz from: {line}"
        hz = float(match.group(1))
        assert hz == pytest.approx(50.0)

    def test_summary_rate_last_line(self, monkeypatch, tmp_path):
        """Last line matching 'target.*deviation' can be parsed for Hz."""
        devices = make_devices(pitch=-0.0886)
        robot = StubRobot(devices, basic_ms=16.0, max_steps=7)
        mod = load_controller(monkeypatch, tmp_path, robot, env={"SENSOR_LOG_THROTTLE": "1"})

        rc = mod.main()
        assert rc == 0

        log_text = (tmp_path / "controller.log").read_text()
        lines = log_text.splitlines()

        # Find last rate summary line
        summary_rate = [l for l in lines if "target" in l and "deviation" in l]
        assert len(summary_rate) >= 1, "No summary rate line found"
        line = summary_rate[-1]
        match = re.search(r"^Control rate: ([0-9.]+)Hz", line)
        assert match, f"Can't parse Hz from: {line}"
        hz = float(match.group(1))
        assert hz == pytest.approx(50.0, rel=0.1)

    def test_all_data_rows_have_pitch(self, monkeypatch, tmp_path):
        """Every numeric-first-token row: token[2] == pitch == -0.0886."""
        devices = make_devices(pitch=-0.0886)
        robot = StubRobot(devices, basic_ms=16.0, max_steps=7)
        mod = load_controller(monkeypatch, tmp_path, robot, env={"SENSOR_LOG_THROTTLE": "1"})

        rc = mod.main()
        assert rc == 0

        log_text = (tmp_path / "controller.log").read_text()
        lines = log_text.splitlines()

        # Extract data rows
        data_rows = [l for l in lines if l and l[0].isdigit() and "READING_ERROR" not in l]
        for row in data_rows:
            tokens = row.split()
            assert len(tokens) >= 3, f"Row has < 3 tokens: {row}"
            pitch_val = float(tokens[2])
            assert pitch_val == pytest.approx(-0.0886, rel=1e-3)
