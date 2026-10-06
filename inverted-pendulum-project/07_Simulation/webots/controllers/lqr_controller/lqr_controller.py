#!/usr/bin/env python3
"""Pendulum robot controller: LQR balance control with gyro feedback.

LQR state targets the true resting equilibrium (~-0.1086 rad) via a reference
offset (THETA_REF_RAD), not an augmented integral state — see issue #220 for
why this method was chosen over integral augmentation (avoids reintroducing
windup risk and extra tuning).

Logging cadence: data rows are logged every Nth control tick (mean 20 ms), not
every Nth 16 ms Webots step (changed in #346).

Loop runs through hal.webots_hal.WebotsHal and hal.control_core.LqrBalance;
Webots behavior is unverified.
"""

import sys
import os

# Webots launches controllers with the system default python3, which does not
# have numpy/scipy (the PID controller never needed them). This controller's
# math (plant_lqr.py) does. Re-exec under the pendulum-tools mamba env's
# python3 if numpy isn't importable in the interpreter Webots handed us.
# Override the interpreter path via PENDULUM_TOOLS_PYTHON if it lives elsewhere.
if "_LQR_REEXEC" not in os.environ:
    try:
        import numpy  # noqa: F401
        import scipy  # noqa: F401
    except ImportError:
        mamba_python = os.environ.get(
            "PENDULUM_TOOLS_PYTHON",
            os.path.expanduser("~/miniforge3/envs/pendulum-tools/bin/python3"),
        )
        if os.path.exists(mamba_python):
            os.environ["_LQR_REEXEC"] = "1"
            os.execv(mamba_python, [mamba_python, "-u", __file__] + sys.argv[1:])
        raise RuntimeError(
            f"numpy/scipy not importable under {sys.executable}, and fallback "
            f"interpreter {mamba_python} does not exist. Set PENDULUM_TOOLS_PYTHON "
            f"to a python3 with numpy/scipy installed (see issue #219)."
        )

from controller import Supervisor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from hal.hal import HalFault
from hal.webots_hal import WebotsHal
from hal.control_core import LqrBalance
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

# True resting-equilibrium tilt (robot asymmetry causes a nonzero rest point,
# not the ideal theta=0 the LQR model assumes -- see issue #220). Consistent
# across both PID and LQR live logs (~-0.1084 to -0.1086 rad).
THETA_REF_RAD = float(os.environ.get("THETA_REF_RAD", "-0.1086"))

# Log file path
LOG_PATH = Path(__file__).parent / "controller.log"

def main():
    robot = Supervisor()
    try:
        hal = WebotsHal(robot, SENSOR_NAMES, MOTOR_NAMES, control_period_ms=CONTROL_RATE_MS)
    except HalFault as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    core = LqrBalance(theta_ref_rad=THETA_REF_RAD)
    sensor_log_count = 0

    with open(LOG_PATH, "w") as log_file:
        def log_msg(msg, always_flush=False):
            print(msg)
            log_file.write(msg + "\n")
            if always_flush or (sensor_log_count % SENSOR_LOG_THROTTLE == 0):
                log_file.flush()

        log_msg("LQR controller started. Reading all sensors (IMU, Gyro, wheel position, pivot joints).", always_flush=True)
        log_msg(format_startup_rate_line(CONTROL_RATE_MS), always_flush=True)
        # LQR_Theta is offset-corrected (pitch - THETA_REF_RAD) to target true resting equilibrium; see issue #220
        log_msg("Time(s) IMU_Roll(rad) IMU_Pitch(rad) IMU_Yaw(rad) IMU_Ax(m/s2) IMU_Ay(m/s2) IMU_Az(m/s2) WheelL(rad) WheelR(rad) PivotL(rad) PivotR(rad) LQR_Theta(rad) LQR_ThetaDot(rad/s) LQR_Cmd_Raw LQR_Cmd_Clamped", always_flush=True)

        wheel_left_max_vel = float(hal.robot.getDevice(MOTOR_NAMES["wheel_left"]).getMaxVelocity())
        wheel_left_max_torque = float(hal.robot.getDevice(MOTOR_NAMES["wheel_left"]).getMaxTorque())
        wheel_right_max_vel = float(hal.robot.getDevice(MOTOR_NAMES["wheel_right"]).getMaxVelocity())
        wheel_right_max_torque = float(hal.robot.getDevice(MOTOR_NAMES["wheel_right"]).getMaxTorque())
        log_msg(format_wheel_motors_line(wheel_left_max_vel, wheel_left_max_torque, wheel_right_max_vel, wheel_right_max_torque), always_flush=True)
        log_msg("NOTE: Pivot servo motors (pendulum_pivot_joint/pendulum_pivot_right_joint) intentionally left at Webots default position-hold this stage.", always_flush=True)

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

                out = core.step(imu, dt)

                if is_saturated(out.raw_cmd):
                    saturation_count += 1

                hal.write_wheel_velocity(out.cmd_rad_s, out.cmd_rad_s)

                sensor_log_count += 1
                if should_log_row(sensor_log_count, SENSOR_LOG_THROTTLE):
                    log_msg(format_data_row(t, imu, enc, pl, pr, out.debug + (out.raw_cmd, out.cmd_rad_s)))

        finally:
            log_msg(f"Controller finished at t={hal.now_s():.3f}s", always_flush=True)
            for line in format_summary_lines(control_fire_times, CONTROL_RATE_MS, saturation_count, control_step_count):
                log_msg(line, always_flush=True)

    return 0

if __name__ == "__main__":
    sys.exit(main())
