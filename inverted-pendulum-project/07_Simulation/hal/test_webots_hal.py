#!/usr/bin/env python3
"""
WebotsHal unit tests: stubs, initialization, tick sequencing, sensors, motors, lifecycle.

Verify WebotsHal initialization, timing accumulator, sensor/motor device management,
read_imu/read_encoders fault cases, write_wheel_velocity clamping, and close() idempotency.
No real controller import anywhere (test files scanned by the leak guard).

Usage:
    mamba run -n pendulum-tools python3 -m pytest -q inverted-pendulum-project/07_Simulation/hal/test_webots_hal.py
"""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from hal.hal import Hal, HalFault, ImuSample, EncoderSample
from hal.webots_hal import WebotsHal


class StubDevice:
    """Mock Webots device (sensor or motor) for testing."""

    def __init__(self, rpy=None, values=None, value=0.0, max_vel=1.0):
        self._rpy = rpy
        self._values = values
        self._value = value
        self._max_vel = max_vel
        self.enabled_ms = None
        self.position_calls = []
        self.velocity_calls = []

    def enable(self, ms):
        self.enabled_ms = ms

    def getRollPitchYaw(self):
        return self._rpy

    def getValues(self):
        return self._values

    def getValue(self):
        return self._value

    def getMaxVelocity(self):
        return self._max_vel

    def setPosition(self, p):
        self.position_calls.append(p)

    def setVelocity(self, v):
        self.velocity_calls.append(v)


class StubRobot:
    """Mock Webots Robot/Supervisor for testing."""

    def __init__(self, devices, basic_ms=16.0, max_steps=None):
        self._devices = devices
        self._basic_ms = basic_ms
        self._max_steps = max_steps
        self._ms = 0
        self.step_calls = 0

    def getBasicTimeStep(self):
        return self._basic_ms

    def getDevice(self, name):
        return self._devices.get(name)

    def step(self, ms):
        self.step_calls += 1
        if self._max_steps is not None and self.step_calls > self._max_steps:
            return -1
        self._ms += int(ms)
        return 0

    def getTime(self):
        return self._ms / 1000.0


SENSOR_NAMES = {
    "imu": "imu",
    "gyro": "gyro",
    "wheel_left": "wheel_left_joint_sensor",
    "wheel_right": "wheel_right_joint_sensor",
    "pivot_left": "pendulum_pivot_joint_sensor",
    "pivot_right": "pendulum_pivot_right_joint_sensor",
}

MOTOR_NAMES = {
    "wheel_left": "wheel_left_joint",
    "wheel_right": "wheel_right_joint",
}


def make_devices():
    """Create stub devices with default sensor/motor values."""
    return {
        "imu": StubDevice(rpy=[0.01, -0.1086, 0.02]),
        "gyro": StubDevice(values=[0.0, 0.3, 0.0]),
        "wheel_left_joint_sensor": StubDevice(value=1.5),
        "wheel_right_joint_sensor": StubDevice(value=2.5),
        "pendulum_pivot_joint_sensor": StubDevice(),
        "pendulum_pivot_right_joint_sensor": StubDevice(),
        "wheel_left_joint": StubDevice(max_vel=1.0),
        "wheel_right_joint": StubDevice(max_vel=1.0),
    }


def make_hal(**robot_kw):
    """Factory: create (robot, hal) tuple with WebotsHal using device stubs."""
    devices = make_devices()
    robot = StubRobot(devices, **robot_kw)
    hal = WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES)
    return robot, hal


class TestWebotsHalInit:
    """Test WebotsHal initialization and basic properties."""

    def test_init_default(self):
        """Init: all sensors enabled_ms==16, motors set to inf position & 0 velocity, hal properties OK."""
        robot, hal = make_hal()

        # All sensors enabled at timestep_ms
        for key in ("imu", "gyro", "wheel_left", "wheel_right", "pivot_left", "pivot_right"):
            assert robot.getDevice(SENSOR_NAMES[key]).enabled_ms == 16

        # Motors in velocity control mode
        for key in ("wheel_left", "wheel_right"):
            motor = robot.getDevice(MOTOR_NAMES[key])
            assert motor.position_calls == [float("inf")]
            assert motor.velocity_calls == [0.0]

        # Hal properties
        assert hal.timestep_ms == 16
        assert hal.robot is robot
        assert hal.sensor("pivot_left") is robot.getDevice(SENSOR_NAMES["pivot_left"])
        assert isinstance(hal, Hal)

    def test_init_timestep_override(self):
        """Init with timestep_ms=8 override: all sensors enabled_ms==8."""
        robot, hal = make_hal(basic_ms=16.0)
        robot2 = StubRobot(make_devices(), basic_ms=16.0)
        hal2 = WebotsHal(robot2, SENSOR_NAMES, MOTOR_NAMES, control_period_ms=20, timestep_ms=8)

        for key in ("imu", "gyro", "wheel_left", "wheel_right", "pivot_left", "pivot_right"):
            assert robot2.getDevice(SENSOR_NAMES[key]).enabled_ms == 8


class TestWebotsHalInitFaults:
    """Test initialization fault cases."""

    def test_init_fault_missing_gyro_device(self):
        """Init fault: robot lacks gyro device."""
        devices = make_devices()
        del devices["gyro"]
        robot = StubRobot(devices, basic_ms=16.0)

        with pytest.raises(HalFault):
            WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES)

    def test_init_fault_missing_motor_device(self):
        """Init fault: robot lacks a motor device."""
        devices = make_devices()
        del devices["wheel_left_joint"]
        robot = StubRobot(devices, basic_ms=16.0)

        with pytest.raises(HalFault):
            WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES)

    def test_init_fault_sensor_names_missing_gyro_key(self):
        """Init fault: sensor_names dict missing 'gyro' key."""
        robot, _ = make_hal()
        bad_sensor_names = {
            "imu": "imu",
            "wheel_left": "wheel_left_joint_sensor",
            "wheel_right": "wheel_right_joint_sensor",
            "pivot_left": "pendulum_pivot_joint_sensor",
            "pivot_right": "pendulum_pivot_right_joint_sensor",
        }

        with pytest.raises(HalFault):
            WebotsHal(robot, bad_sensor_names, MOTOR_NAMES)

    def test_init_fault_motor_names_missing_wheel_right_key(self):
        """Init fault: motor_names dict missing 'wheel_right' key."""
        robot, _ = make_hal()
        bad_motor_names = {"wheel_left": "wheel_left_joint"}

        with pytest.raises(HalFault):
            WebotsHal(robot, SENSOR_NAMES, bad_motor_names)

    def test_init_fault_control_period_equals_basic_timestep(self):
        """Init fault: control_period_ms == basicTimeStep (16ms)."""
        robot = StubRobot(make_devices(), basic_ms=16.0)

        with pytest.raises(HalFault):
            WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES, control_period_ms=16)

    def test_init_fault_control_period_less_than_basic_timestep(self):
        """Init fault: control_period_ms < basicTimeStep."""
        robot = StubRobot(make_devices(), basic_ms=16.0)

        with pytest.raises(HalFault):
            WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES, control_period_ms=10)

    def test_init_fault_timestep_zero(self):
        """Init fault: timestep_ms=0."""
        robot = StubRobot(make_devices(), basic_ms=16.0)

        with pytest.raises(HalFault):
            WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES, timestep_ms=0)


class TestWebotsHalTiming:
    """Test wait_next_tick timing accumulator and sequencing."""

    def test_first_tick(self):
        """First wait_next_tick() returns nominal 0.020s exactly; robot.step_calls==2."""
        robot, hal = make_hal()

        dt = hal.wait_next_tick()
        assert dt == 0.020
        assert robot.getTime() == pytest.approx(0.032, abs=1e-9)
        assert robot.step_calls == 2

    def test_dt_sequence_5_calls(self):
        """Basic 16ms/period 20ms: dts=[0.020, 0.016, 0.016, 0.016, 0.032]; step_calls==7."""
        robot, hal = make_hal()

        dts = [hal.wait_next_tick() for _ in range(5)]

        assert dts[0] == 0.020
        assert dts[1] == pytest.approx(0.016, abs=1e-9)
        assert dts[2] == pytest.approx(0.016, abs=1e-9)
        assert dts[3] == pytest.approx(0.016, abs=1e-9)
        assert dts[4] == pytest.approx(0.032, abs=1e-9)
        assert robot.step_calls == 7
        assert hal.now_s() == pytest.approx(0.112, abs=1e-9)

    def test_long_run_13_calls(self):
        """Long run: 13 calls with stats on dt sequence."""
        robot, hal = make_hal()

        dts = [hal.wait_next_tick() for _ in range(13)]

        assert dts[0] == 0.020
        for dt in dts[1:]:
            assert dt == pytest.approx(0.016, abs=1e-9) or dt == pytest.approx(0.032, abs=1e-9)
        assert sum(dts[1:]) == pytest.approx(0.240, abs=1e-9)
        assert (sum(dts[1:]) / len(dts[1:])) == pytest.approx(0.020, abs=1e-9)
        assert dts[1:5] == pytest.approx([0.016, 0.016, 0.016, 0.032], abs=1e-9)

    def test_basic_8ms_period_20ms(self):
        """Basic 8ms/period 20ms: dts=[0.020, 0.016, 0.024, 0.016, 0.024]."""
        robot = StubRobot(make_devices(), basic_ms=8.0)
        hal = WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES, control_period_ms=20)

        dts = [hal.wait_next_tick() for _ in range(5)]

        assert dts == pytest.approx([0.020, 0.016, 0.024, 0.016, 0.024], abs=1e-9)

    def test_basic_16_control_period_32(self):
        """Basic 16ms/period 32ms: dts=[0.032, 0.032, 0.032]."""
        robot = StubRobot(make_devices(), basic_ms=16.0)
        hal = WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES, control_period_ms=32)

        dts = [hal.wait_next_tick() for _ in range(3)]

        assert dts == pytest.approx([0.032, 0.032, 0.032], abs=1e-9)


class TestWebotsHalQuit:
    """Test quit/step return behavior on max_steps."""

    def test_quit_max_steps_3(self):
        """max_steps=3: calls 1-2 return dt; call 3 returns -1.0; call 4 returns -1.0, step_calls latches."""
        robot = StubRobot(make_devices(), basic_ms=16.0, max_steps=3)
        hal = WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES)

        dt1 = hal.wait_next_tick()
        assert dt1 == pytest.approx(0.020, abs=1e-9)

        dt2 = hal.wait_next_tick()
        assert dt2 == pytest.approx(0.016, abs=1e-9)

        dt3 = hal.wait_next_tick()
        assert dt3 == -1.0
        assert robot.step_calls == 4

        dt4 = hal.wait_next_tick()
        assert dt4 == -1.0
        assert robot.step_calls == 4

    def test_quit_max_steps_1(self):
        """max_steps=1: first wait_next_tick() returns -1.0."""
        robot = StubRobot(make_devices(), basic_ms=16.0, max_steps=1)
        hal = WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES)

        dt = hal.wait_next_tick()
        assert dt == -1.0

    def test_quit_never_raises(self):
        """Quit never raises an exception."""
        robot = StubRobot(make_devices(), basic_ms=16.0, max_steps=1)
        hal = WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES)

        try:
            hal.wait_next_tick()
        except Exception:
            pytest.fail("wait_next_tick() raised exception on quit")


class TestWebotsHalReadImu:
    """Test read_imu sensor fusion."""

    def test_read_imu_ok(self):
        """read_imu returns correct ImuSample with exact pitch."""
        robot, hal = make_hal()
        hal.wait_next_tick()

        imu = hal.read_imu()

        assert imu.t_s == robot.getTime()
        assert imu.roll_rad == 0.01
        assert imu.pitch_rad == -0.1086
        assert imu.yaw_rad == 0.02
        assert imu.gyro_rad_s == (0.0, 0.3, 0.0)
        assert isinstance(imu.gyro_rad_s, tuple)
        assert imu.gyro_rad_s[1] == 0.3

    @pytest.mark.parametrize("bad_rpy", [
        None,
        [float("nan"), -0.1086, 0.02],
        [0.01, float("inf"), 0.02],
        [0.01, None, 0.02],
        [0.01, -0.1086],
    ])
    def test_read_imu_fault_rpy(self, bad_rpy):
        """read_imu faults on bad rpy: None, NaN, inf, None element, wrong length."""
        devices = make_devices()
        devices["imu"]._rpy = bad_rpy
        robot = StubRobot(devices, basic_ms=16.0)
        hal = WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES)

        with pytest.raises(HalFault):
            hal.read_imu()

    @pytest.mark.parametrize("bad_gyro", [
        None,
        [0.0, float("nan"), 0.0],
        [0.0, 0.3],
        [0.0, None, 0.0],
    ])
    def test_read_imu_fault_gyro(self, bad_gyro):
        """read_imu faults on bad gyro: None, NaN, wrong length, None element."""
        devices = make_devices()
        devices["gyro"]._values = bad_gyro
        robot = StubRobot(devices, basic_ms=16.0)
        hal = WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES)

        with pytest.raises(HalFault):
            hal.read_imu()


class TestWebotsHalReadEncoders:
    """Test read_encoders wheel position fusion."""

    def test_read_encoders_ok(self):
        """read_encoders returns correct EncoderSample."""
        robot, hal = make_hal()

        enc = hal.read_encoders()

        assert enc == EncoderSample(1.5, 2.5)

    def test_read_encoders_fault_nan(self):
        """read_encoders faults on NaN wheel value."""
        devices = make_devices()
        devices["wheel_left_joint_sensor"]._value = float("nan")
        robot = StubRobot(devices, basic_ms=16.0)
        hal = WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES)

        with pytest.raises(HalFault):
            hal.read_encoders()

    def test_read_encoders_fault_none(self):
        """read_encoders faults on None wheel value."""
        devices = make_devices()
        devices["wheel_right_joint_sensor"]._value = None
        robot = StubRobot(devices, basic_ms=16.0)
        hal = WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES)

        with pytest.raises(HalFault):
            hal.read_encoders()


class TestWebotsHalWriteWheelVelocity:
    """Test write_wheel_velocity clamping and error handling."""

    def test_write_wheel_velocity_ok(self):
        """write_wheel_velocity(0.5, -0.25): motors set to those values."""
        robot, hal = make_hal()

        hal.write_wheel_velocity(0.5, -0.25)

        left_motor = robot.getDevice(MOTOR_NAMES["wheel_left"])
        right_motor = robot.getDevice(MOTOR_NAMES["wheel_right"])
        assert left_motor.velocity_calls[-1] == 0.5
        assert right_motor.velocity_calls[-1] == -0.25

    def test_write_wheel_velocity_clamp(self):
        """write_wheel_velocity(2.0, -3.0) with max_vel=1.0: clamped to (1.0, -1.0)."""
        robot, hal = make_hal()

        hal.write_wheel_velocity(2.0, -3.0)

        left_motor = robot.getDevice(MOTOR_NAMES["wheel_left"])
        right_motor = robot.getDevice(MOTOR_NAMES["wheel_right"])
        assert left_motor.velocity_calls[-1] == 1.0
        assert right_motor.velocity_calls[-1] == -1.0

    def test_write_wheel_velocity_nan_to_zero(self):
        """write_wheel_velocity(float('nan'), float('nan')): clamped to (0.0, 0.0)."""
        robot, hal = make_hal()

        hal.write_wheel_velocity(float("nan"), float("nan"))

        left_motor = robot.getDevice(MOTOR_NAMES["wheel_left"])
        right_motor = robot.getDevice(MOTOR_NAMES["wheel_right"])
        assert left_motor.velocity_calls[-1] == 0.0
        assert right_motor.velocity_calls[-1] == 0.0

    def test_write_wheel_velocity_non_numeric_to_zero(self):
        """write_wheel_velocity('x', ...): non-numeric clamped to 0.0, no raise."""
        robot, hal = make_hal()

        hal.write_wheel_velocity("x", 0.5)

        left_motor = robot.getDevice(MOTOR_NAMES["wheel_left"])
        assert left_motor.velocity_calls[-1] == 0.0

    def test_write_wheel_velocity_no_clamp_when_max_vel_zero(self):
        """Motor with max_vel=0.0: 2.0 passes through (no clamp)."""
        devices = make_devices()
        devices["wheel_left_joint"]._max_vel = 0.0
        robot = StubRobot(devices, basic_ms=16.0)
        hal = WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES)

        hal.write_wheel_velocity(2.0, 0.5)

        left_motor = robot.getDevice(MOTOR_NAMES["wheel_left"])
        assert left_motor.velocity_calls[-1] == 2.0

    def test_write_wheel_velocity_no_clamp_when_max_vel_nan(self):
        """Motor with max_vel=NaN: 2.0 passes through (no clamp)."""
        devices = make_devices()
        devices["wheel_left_joint"]._max_vel = float("nan")
        robot = StubRobot(devices, basic_ms=16.0)
        hal = WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES)

        hal.write_wheel_velocity(2.0, 0.5)

        left_motor = robot.getDevice(MOTOR_NAMES["wheel_left"])
        assert left_motor.velocity_calls[-1] == 2.0


class TestWebotsHalClose:
    """Test close() idempotency and motor shutdown."""

    def test_close_stops_motors(self):
        """close(): both motors' last velocity is 0.0."""
        robot, hal = make_hal()

        hal.close()

        left_motor = robot.getDevice(MOTOR_NAMES["wheel_left"])
        right_motor = robot.getDevice(MOTOR_NAMES["wheel_right"])
        assert left_motor.velocity_calls[-1] == 0.0
        assert right_motor.velocity_calls[-1] == 0.0

    def test_close_blocks_wait_next_tick(self):
        """After close(), wait_next_tick() returns -1.0, step_calls unchanged."""
        robot, hal = make_hal()

        hal.close()
        step_calls_before = robot.step_calls

        dt = hal.wait_next_tick()

        assert dt == -1.0
        assert robot.step_calls == step_calls_before

    def test_close_idempotent(self):
        """Second close() is idempotent and safe."""
        robot, hal = make_hal()

        hal.close()
        hal.close()

    def test_close_blocks_write(self):
        """After close(), write_wheel_velocity() adds no setVelocity call."""
        robot, hal = make_hal()

        hal.close()
        left_motor = robot.getDevice(MOTOR_NAMES["wheel_left"])
        calls_before = len(left_motor.velocity_calls)

        hal.write_wheel_velocity(0.5, 0.5)

        assert len(left_motor.velocity_calls) == calls_before


class TestWebotsHalNow:
    """Test now_s() timestamp."""

    def test_now_s_before_any_step(self):
        """now_s() before any step returns 0.0."""
        robot, hal = make_hal()

        assert hal.now_s() == 0.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
