#!/usr/bin/env python3
"""Pendulum robot controller: read IMU + wheel position sensors."""

from controller import Robot
import sys
import os
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from plant_pid import PlantPID

# TODO(#187): Replace hardcoded SENSOR_NAMES with manifest resolver
SENSOR_NAMES = {
    "imu": "imu",
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

# Placeholder PID gains (conservative starting point; real tuning is a follow-up
# once the loop is confirmed working via manual GUI sign-verification).
BALANCE_PID = PlantPID(
    kp=1.0,
    ki=0.1,
    kd=0.05,
    output_min=-1.0,
    output_max=1.0,
)

def main():
    robot = Robot()
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

        log_msg("Pendulum controller started. Reading all 5 sensors.", always_flush=True)
        log_msg(f"Control rate: {CONTROL_RATE_MS}ms ({1000.0/CONTROL_RATE_MS:.1f}Hz)", always_flush=True)
        log_msg("Time(s) IMU_Roll(rad) IMU_Pitch(rad) IMU_Yaw(rad) IMU_Ax(m/s2) IMU_Ay(m/s2) IMU_Az(m/s2) WheelL(rad) WheelR(rad) PivotL(rad) PivotR(rad) PID_Error(rad) PID_P PID_I PID_D PID_Cmd", always_flush=True)

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
        control_prev_fire_time = None  # for computing dt in PID step
        saturation_count = 0  # for tracking output saturation events
        sensor_log_count = 0  # for throttling sensor log lines

        # PID component tracking for telemetry logging
        error = 0.0
        p_term = 0.0
        i_term = 0.0
        d_term = 0.0
        cmd_velocity = 0.0

        def _run_control_step(t, roll, pitch, yaw, ax, ay, az, wl, wr, pl, pr):
            """PID balance control: tilt error measured from pitch angle.

            Error = -pitch (setpoint 0 = upright). Sign UNVERIFIED — requires manual GUI test:
            tilt robot and confirm wheels drive to correct not amplify the fall.
            Gains are placeholders pending real tuning.
            """
            nonlocal control_prev_fire_time, saturation_count, error, p_term, i_term, d_term, cmd_velocity

            # Compute dt: fallback to CONTROL_PERIOD_S on first call
            if control_prev_fire_time is None:
                dt = CONTROL_PERIOD_S
            else:
                dt = t - control_prev_fire_time
            control_prev_fire_time = t

            # Tilt error: negative pitch (upright at pitch=0)
            error = -pitch

            # Compute command velocity
            cmd_velocity = BALANCE_PID.step(error, dt)
            p_term, i_term, d_term = BALANCE_PID.last_components()

            # Track saturation events
            if abs(cmd_velocity) >= 0.95 * 1.0:
                saturation_count += 1

            # Command both wheels
            motors["wheel_left"].setVelocity(cmd_velocity)
            motors["wheel_right"].setVelocity(cmd_velocity)

        # Main loop: run indefinitely until Webots quit signal (-1)
        try:
            while robot.step(timestep) != -1:
                t = robot.getTime()

                # Read sensors
                imu_device = sensors["imu"]
                rpy = imu_device.getRollPitchYaw()
                wl = sensors["wheel_left"].getValue()
                wr = sensors["wheel_right"].getValue()
                pl = sensors["pivot_left"].getValue()
                pr = sensors["pivot_right"].getValue()

                # Log (check for NaN)
                if None in [rpy, wl, wr, pl, pr] or any(x is None for x in rpy):
                    log_msg(f"{t:.3f} READING_ERROR: sensor returned None", always_flush=True)
                    continue

                roll, pitch, yaw = rpy
                ax, ay, az = 0.0, 0.0, 0.0  # No Accelerometer device on this robot; InertialUnit has no getAcceleration() (see #202)

                # Fixed-rate control loop gating (accumulator pattern)
                # Note: if (not while) assumes timestep < CONTROL_RATE_MS per startup validation above.
                # Single fire per step ensures all samples have unique t values for jitter measurement.
                control_accum_ms += timestep
                if control_accum_ms >= CONTROL_RATE_MS:
                    control_accum_ms -= CONTROL_RATE_MS  # carry remainder
                    control_step_count += 1
                    control_fire_times.append(t)
                    _run_control_step(t, roll, pitch, yaw, ax, ay, az, wl, wr, pl, pr)

                # Log sensor data with throttle gate
                sensor_log_count += 1
                if sensor_log_count % SENSOR_LOG_THROTTLE == 0:
                    log_msg(f"{t:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f} {ax:.4f} {ay:.4f} {az:.4f} {wl:.4f} {wr:.4f} {pl:.4f} {pr:.4f} {error:.4f} {p_term:.4f} {i_term:.4f} {d_term:.4f} {cmd_velocity:.4f}")
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
