#!/usr/bin/env python3
"""Pendulum robot controller: PID balance control for pitch stabilization.

Error = -(pitch - setpoint). Sign UNVERIFIED (#359) — requires manual GUI test:
tilt robot and confirm wheels drive to correct not amplify the fall.

PID_SETPOINT_RAD (#408) sets the pitch setpoint in rad. Default: unset (= 0), error = -pitch,
behaviour unchanged. The value -0.1086 is the robot's rest pitch, which is the LQR design
reference (lqr.theta_ref_rad in hal/parity_vectors.json); with it the PID error is ~0 at rest.
Implemented as a controller-local shift of the IMU sample before PidBalance.step, so the
core and its PID_Error sign are untouched. Non-numeric, empty, nan, inf, underscore and Unicode
digit text exits 1 at startup, with the message in controller.log and stderr. "1e-400" parses
to 0.0 and behaves as unset (accepted).
Logging: IMU_Pitch is the REAL measured pitch; PID_Error is the core's error (shifted by the
setpoint). The sign (#359) is still unresolved, so the setpoint sign is not verified either.

Logging cadence: data rows are logged every Nth control tick (mean 20 ms), not
every Nth 16 ms Webots step (changed in #346).

Gains are placeholders pending real tuning. PID stability is untested. Webots behavior
is unverified.

Loop runs through hal.webots_hal.WebotsHal and hal.control_core.PidBalance.

CONTROLLER_DISABLED=1 (exact value "1" only; any other value, including unset, leaves the
controller on) sends 0.0/0.0 rad/s to the wheels instead of the PID output. The PID core
still runs and its output is logged, so while the switch is on the PID_Cmd column is the
core output, NOT the applied wheel command. Used by tests/test_stage_d_balance.sh (#405).
"""

import sys
import os
import dataclasses
import math
import re
from pathlib import Path

from controller import Supervisor

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from hal.hal import HalFault
from hal.webots_hal import WebotsHal
from hal.control_core import PidBalance
from hal.loop_support import (
    is_saturated,
    should_log_row,
    time_limit_reached,
    require_value,
    format_startup_rate_line,
    format_wheel_motors_line,
    format_data_row,
    format_summary_lines,
)

# TODO(#187): Replace hardcoded SENSOR_NAMES with manifest resolver
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

# Fixed-rate control loop parameters
CONTROL_RATE_MS = int(os.environ.get("CONTROL_RATE_MS", "20"))
CONTROL_PERIOD_S = CONTROL_RATE_MS / 1000.0

# Sensor log throttling (configurable via env var)
# every Nth control tick (mean 20 ms); before #346 it was every Nth 16 ms Webots step
SENSOR_LOG_THROTTLE = int(os.environ.get("SENSOR_LOG_THROTTLE", "50"))

# Simulation time limit for automated testing (0 = disabled)
TEST_MAX_SIM_TIME_S = float(os.environ.get("TEST_MAX_SIM_TIME_S", "0"))

# Control-off switch for informational test runs (#405). CONTROLLER_DISABLED=1 sends
# 0.0 rad/s to both wheels instead of the PID output; the PID still runs and is logged.
CONTROLLER_DISABLED = os.environ.get("CONTROLLER_DISABLED", "0") == "1"

# Log file path
LOG_PATH = Path(__file__).parent / "controller.log"

# PID_SETPOINT_RAD (#408): plain ASCII decimal only (no nan, inf, underscores, Unicode digits, or empty text).
_PLAIN_DECIMAL = re.compile(r"[-+]?([0-9]+\.?[0-9]*|\.[0-9]+)([eE][-+]?[0-9]+)?")


def parse_pid_setpoint(text):
    """Return the PID setpoint in rad from PID_SETPOINT_RAD text; raise ValueError if invalid."""
    s = text.strip()
    if not _PLAIN_DECIMAL.fullmatch(s):
        raise ValueError(f"PID_SETPOINT_RAD={text!r} is not a plain decimal number (e.g. -0.1086)")
    value = float(s)
    if not math.isfinite(value):
        raise ValueError(f"PID_SETPOINT_RAD={text!r} is not finite")
    return value


def main():
    raw_setpoint = os.environ.get("PID_SETPOINT_RAD")
    try:
        pid_setpoint = 0.0 if raw_setpoint is None else parse_pid_setpoint(raw_setpoint)
    except ValueError as exc:
        msg = f"ERROR: {exc}. Controller not started."
        print(msg, file=sys.stderr)
        LOG_PATH.write_text(msg + "\n")
        return 1

    robot = Supervisor()
    try:
        hal = WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES, control_period_ms=CONTROL_RATE_MS)
    except HalFault as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    core = PidBalance(kp=1.0, ki=0.1, kd=0.05, out_limit=1.0)
    sensor_log_count = 0

    with open(LOG_PATH, "w") as log_file:
        def log_msg(msg, always_flush=False):
            print(msg)
            log_file.write(msg + "\n")
            if always_flush or (sensor_log_count % SENSOR_LOG_THROTTLE == 0):
                log_file.flush()

        log_msg("Pendulum controller started. Reading all 5 sensors.", always_flush=True)
        log_msg(format_startup_rate_line(CONTROL_RATE_MS), always_flush=True)
        log_msg("Time(s) IMU_Roll(rad) IMU_Pitch(rad) IMU_Yaw(rad) IMU_Ax(m/s2) IMU_Ay(m/s2) IMU_Az(m/s2) WheelL(rad) WheelR(rad) PivotL(rad) PivotR(rad) PID_Error(rad) PID_P PID_I PID_D PID_Cmd", always_flush=True)

        wheel_left_max_vel = float(hal.robot.getDevice(MOTOR_NAMES["wheel_left"]).getMaxVelocity())
        wheel_left_max_torque = float(hal.robot.getDevice(MOTOR_NAMES["wheel_left"]).getMaxTorque())
        wheel_right_max_vel = float(hal.robot.getDevice(MOTOR_NAMES["wheel_right"]).getMaxVelocity())
        wheel_right_max_torque = float(hal.robot.getDevice(MOTOR_NAMES["wheel_right"]).getMaxTorque())
        log_msg(format_wheel_motors_line(wheel_left_max_vel, wheel_left_max_torque, wheel_right_max_vel, wheel_right_max_torque), always_flush=True)
        log_msg("NOTE: Pivot servo motors (pendulum_pivot_joint/pendulum_pivot_right_joint) intentionally left at Webots default position-hold this stage.", always_flush=True)
        if pid_setpoint != 0.0:
            log_msg(f"PID_SETPOINT_RAD={pid_setpoint:g}rad: PID error = -(pitch - setpoint). IMU_Pitch column is the REAL pitch; PID_Error is the core's shifted error.", always_flush=True)
        if CONTROLLER_DISABLED:
            log_msg("CONTROLLER_DISABLED=1: control-off run, wheel commands forced to 0.0 rad/s. PID_Cmd column is the PID core output, NOT the applied wheel command.", always_flush=True)

        control_step_count = 0
        control_fire_times = []
        saturation_count = 0

        try:
            while True:
                dt = hal.wait_next_tick()
                if dt < 0:
                    break

                t = hal.now_s()

                if time_limit_reached(t, TEST_MAX_SIM_TIME_S):
                    log_msg(f"Simulated time limit reached ({t:.3f}s >= {TEST_MAX_SIM_TIME_S}s), stopping.", always_flush=True)
                    robot.simulationQuit(0)

                try:
                    imu = hal.read_imu()
                    enc = hal.read_encoders()
                    pl = require_value(hal.sensor("pivot_left").getValue(), "pivot_left")
                    pr = require_value(hal.sensor("pivot_right").getValue(), "pivot_right")
                except HalFault as exc:
                    log_msg(f"{t:.3f} READING_ERROR: {exc}", always_flush=True)
                    continue

                control_step_count += 1
                control_fire_times.append(t)

                # Setpoint shift (#408): the core sees pitch - setpoint; the log keeps the real imu.
                out = core.step(dataclasses.replace(imu, pitch_rad=imu.pitch_rad - pid_setpoint), dt)

                if is_saturated(out.cmd_rad_s):
                    saturation_count += 1

                if CONTROLLER_DISABLED:
                    hal.write_wheel_velocity(0.0, 0.0)
                else:
                    hal.write_wheel_velocity(out.cmd_rad_s, out.cmd_rad_s)

                sensor_log_count += 1
                if should_log_row(sensor_log_count, SENSOR_LOG_THROTTLE):
                    log_msg(format_data_row(t, imu, enc, pl, pr, out.debug + (out.cmd_rad_s,)))

        finally:
            log_msg(f"Controller finished at t={hal.now_s():.3f}s", always_flush=True)
            for line in format_summary_lines(control_fire_times, CONTROL_RATE_MS, saturation_count, control_step_count):
                log_msg(line, always_flush=True)

    return 0

if __name__ == "__main__":
    sys.exit(main())
