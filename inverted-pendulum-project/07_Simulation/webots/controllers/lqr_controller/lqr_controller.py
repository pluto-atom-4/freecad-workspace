#!/usr/bin/env python3
"""Pendulum robot controller: LQR balance control with gyro feedback."""

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

from controller import Robot, Supervisor
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from plant_lqr import default_gain

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
SENSOR_LOG_THROTTLE = int(os.environ.get("SENSOR_LOG_THROTTLE", "50"))

# Simulation time limit for automated testing (0 = disabled)
TEST_MAX_SIM_TIME_S = float(os.environ.get("TEST_MAX_SIM_TIME_S", "0"))

# LQR gain matrix (computed once at import time)
K = default_gain()

def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    # Validate that timestep < CONTROL_RATE_MS (required for single-fire-per-step assumption)
    if timestep >= CONTROL_RATE_MS:
        print(f"ERROR: basicTimeStep ({timestep}ms) >= CONTROL_RATE_MS ({CONTROL_RATE_MS}ms). Adjust world or CONTROL_RATE_MS env var.", file=sys.stderr)
        return 1

    # Get sensor devices
    sensors = {}
    for key, device_name in SENSOR_NAMES.items():
        device = robot.getDevice(device_name)
        if device is None:
            print(f"ERROR: Sensor '{device_name}' (key: {key}) not found", file=sys.stderr)
            return 1
        sensors[key] = device
        device.enable(timestep)

    # Get motor devices
    motors = {}
    for key, device_name in MOTOR_NAMES.items():
        device = robot.getDevice(device_name)
        if device is None:
            print(f"ERROR: Motor '{device_name}' (key: {key}) not found", file=sys.stderr)
            return 1
        motors[key] = device

    # Log output (dual-sink: stdout + file)
    log_path = Path(__file__).parent / "controller.log"

    with open(log_path, "w") as log_file:
        def log_msg(msg, always_flush=False):
            print(msg)
            log_file.write(msg + "\n")
            if always_flush or (sensor_log_count % SENSOR_LOG_THROTTLE == 0):
                log_file.flush()

        log_msg("LQR controller started. Reading all sensors (IMU, Gyro, wheel position, pivot joints).", always_flush=True)
        log_msg(f"Control rate: {CONTROL_RATE_MS}ms ({1000.0/CONTROL_RATE_MS:.1f}Hz)", always_flush=True)
        log_msg("Time(s) IMU_Roll(rad) IMU_Pitch(rad) IMU_Yaw(rad) IMU_Ax(m/s2) IMU_Ay(m/s2) IMU_Az(m/s2) WheelL(rad) WheelR(rad) PivotL(rad) PivotR(rad) LQR_Theta(rad) LQR_ThetaDot(rad/s) LQR_Cmd_Raw LQR_Cmd_Clamped", always_flush=True)

        # Initialize motors: switch to velocity control mode
        for key, motor in motors.items():
            motor.setPosition(float('inf'))
            motor.setVelocity(0)

        wheel_left_max_vel = motors["wheel_left"].getMaxVelocity()
        wheel_left_max_torque = motors["wheel_left"].getMaxTorque()
        wheel_right_max_vel = motors["wheel_right"].getMaxVelocity()
        wheel_right_max_torque = motors["wheel_right"].getMaxTorque()
        log_msg(f"Wheel motors: mode=velocity, wheel_left maxVelocity={wheel_left_max_vel:.2f} maxTorque={wheel_left_max_torque:.2f}, wheel_right maxVelocity={wheel_right_max_vel:.2f} maxTorque={wheel_right_max_torque:.2f}", always_flush=True)
        log_msg("NOTE: Pivot servo motors (pendulum_pivot_joint/pendulum_pivot_right_joint) intentionally left at Webots default position-hold this stage.", always_flush=True)

        # Fixed-rate control loop state
        control_accum_ms = 0.0
        control_step_count = 0
        control_fire_times = []  # for jitter measurement at shutdown
        control_prev_fire_time = None  # for computing dt in LQR step
        saturation_count = 0  # for tracking output saturation events
        sensor_log_count = 0  # for throttling sensor log lines

        # LQR state and command tracking for telemetry logging
        theta = 0.0
        theta_dot = 0.0
        raw_cmd = 0.0
        cmd_velocity = 0.0

        def _run_control_step(t, roll, pitch, yaw, ax, ay, az, wl, wr, pl, pr, theta_dot_gyro):
            """LQR balance control: state vector [theta, theta_dot] from pitch + gyro.

            theta = pitch (setpoint 0 = upright), theta_dot = gyro Y-axis rate.
            Sign convention UNVERIFIED — requires empirical test: perturb robot
            and confirm wheels drive to correct, not amplify, the fall.
            LQR gain matrix K pre-computed via plant_lqr.default_gain().
            """
            nonlocal control_prev_fire_time, saturation_count, theta, theta_dot, raw_cmd, cmd_velocity

            # State vector
            theta = pitch
            theta_dot = theta_dot_gyro

            # Compute command: u = -K @ x
            state = np.array([theta, theta_dot])
            raw_cmd = float(-(K @ state)[0])

            # Clamp to actuator maxVelocity
            cmd_velocity = max(-1.0, min(1.0, raw_cmd))

            # Track saturation events (BEFORE clamping, to reflect true saturation)
            if abs(raw_cmd) >= 0.95 * 1.0:
                saturation_count += 1

            # Command both wheels
            motors["wheel_left"].setVelocity(cmd_velocity)
            motors["wheel_right"].setVelocity(cmd_velocity)

        # Main loop: run indefinitely until Webots quit signal (-1)
        try:
            while robot.step(timestep) != -1:
                t = robot.getTime()

                # Check simulated time limit (for automated testing)
                if TEST_MAX_SIM_TIME_S > 0 and t >= TEST_MAX_SIM_TIME_S:
                    log_msg(f"Simulated time limit reached ({t:.3f}s >= {TEST_MAX_SIM_TIME_S}s), stopping.", always_flush=True)
                    robot.simulationQuit(0)

                # Read sensors
                imu_device = sensors["imu"]
                rpy = imu_device.getRollPitchYaw()
                gyro_device = sensors["gyro"]
                gyro_vals = gyro_device.getValues()
                wl = sensors["wheel_left"].getValue()
                wr = sensors["wheel_right"].getValue()
                pl = sensors["pivot_left"].getValue()
                pr = sensors["pivot_right"].getValue()

                # Log (check for NaN)
                if None in [rpy, gyro_vals, wl, wr, pl, pr] or any(x is None for x in rpy) or any(x is None for x in gyro_vals):
                    log_msg(f"{t:.3f} READING_ERROR: sensor returned None", always_flush=True)
                    continue

                roll, pitch, yaw = rpy
                ax, ay, az = 0.0, 0.0, 0.0  # No Accelerometer device on this robot; InertialUnit has no getAcceleration() (see #202)
                theta_dot_gyro = gyro_vals[1]  # pitch-rate is Y-axis, index 1

                # Fixed-rate control loop gating (accumulator pattern)
                # Note: if (not while) assumes timestep < CONTROL_RATE_MS per startup validation above.
                # Single fire per step ensures all samples have unique t values for jitter measurement.
                control_accum_ms += timestep
                if control_accum_ms >= CONTROL_RATE_MS:
                    control_accum_ms -= CONTROL_RATE_MS  # carry remainder
                    control_step_count += 1
                    control_fire_times.append(t)
                    _run_control_step(t, roll, pitch, yaw, ax, ay, az, wl, wr, pl, pr, theta_dot_gyro)

                # Log sensor data with throttle gate
                sensor_log_count += 1
                if sensor_log_count % SENSOR_LOG_THROTTLE == 0:
                    log_msg(f"{t:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f} {ax:.4f} {ay:.4f} {az:.4f} {wl:.4f} {wr:.4f} {pl:.4f} {pr:.4f} {theta:.4f} {theta_dot:.4f} {raw_cmd:.4f} {cmd_velocity:.4f}")
        finally:
            log_msg(f"Controller finished at t={robot.getTime():.3f}s", always_flush=True)

            # Jitter and frequency measurement
            if control_fire_times and len(control_fire_times) > 1:
                diffs = [control_fire_times[i+1] - control_fire_times[i] for i in range(len(control_fire_times)-1)]
                mean_period_ms = sum(diffs) * 1000 / len(diffs)
                mean_freq_hz = 1000 / mean_period_ms if mean_period_ms > 0 else 0
                min_period_ms = min(diffs) * 1000
                max_period_ms = max(diffs) * 1000
                max_jitter_ms = max(abs(d * 1000 - CONTROL_RATE_MS) for d in diffs) if diffs else 0
                log_msg(f"Control rate: {mean_freq_hz:.2f}Hz (target {1000/CONTROL_RATE_MS:.2f}Hz), periods {min_period_ms:.1f}-{max_period_ms:.1f}ms (target {CONTROL_RATE_MS}ms), max deviation {max_jitter_ms:.2f}ms", always_flush=True)

            log_msg(f"Saturation events: {saturation_count} of {control_step_count} control steps", always_flush=True)

    return 0

if __name__ == "__main__":
    sys.exit(main())
