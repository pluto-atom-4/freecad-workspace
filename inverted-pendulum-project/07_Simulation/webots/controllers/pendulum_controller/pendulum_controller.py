#!/usr/bin/env python3
"""Pendulum robot controller: read IMU + wheel position sensors."""

from controller import Robot
import sys
import os
from pathlib import Path

# TODO(#187): Replace hardcoded SENSOR_NAMES with manifest resolver
SENSOR_NAMES = {
    "imu": "imu",
    "wheel_left": "wheel_left_joint_sensor",
    "wheel_right": "wheel_right_joint_sensor",
    "pivot_left": "pendulum_pivot_joint_sensor",
    "pivot_right": "pendulum_pivot_right_joint_sensor",
}

# Fixed-rate control loop parameters
CONTROL_RATE_MS = int(os.environ.get("CONTROL_RATE_MS", "20"))
CONTROL_PERIOD_S = CONTROL_RATE_MS / 1000.0

# Sensor log throttling (configurable via env var)
SENSOR_LOG_THROTTLE = int(os.environ.get("SENSOR_LOG_THROTTLE", "50"))

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
        log_msg("Time(s) IMU_Roll(rad) IMU_Pitch(rad) IMU_Yaw(rad) IMU_Ax(m/s2) IMU_Ay(m/s2) IMU_Az(m/s2) WheelL(rad) WheelR(rad) PivotL(rad) PivotR(rad)", always_flush=True)

        # Fixed-rate control loop state
        control_accum_ms = 0.0
        control_step_count = 0
        control_fire_times = []  # for jitter measurement at shutdown
        sensor_log_count = 0  # for throttling sensor log lines

        def _run_control_step(t, roll, pitch, yaw, ax, ay, az, wl, wr, pl, pr):
            """Placeholder control logic. TODO(Stage D): replace with motor commands."""
            # No motor writes yet; just a hook for Stage D to fill
            # TODO(Stage D): Add log_control_msg() for per-fire control telemetry if needed
            pass

        # Main loop: run indefinitely until Webots quit signal (-1)
        try:
            while robot.step(timestep) != -1:
                t = robot.getTime()

                # Read sensors
                imu_device = sensors["imu"]
                rpy = imu_device.getRollPitchYaw()
                accel = imu_device.getAcceleration()
                wl = sensors["wheel_left"].getValue()
                wr = sensors["wheel_right"].getValue()
                pl = sensors["pivot_left"].getValue()
                pr = sensors["pivot_right"].getValue()

                # Log (check for NaN)
                if None in [rpy, accel, wl, wr, pl, pr] or any(x is None for x in rpy) or any(x is None for x in accel):
                    log_msg(f"{t:.3f} READING_ERROR: sensor returned None", always_flush=True)
                    continue

                roll, pitch, yaw = rpy
                ax, ay, az = accel

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
                    log_msg(f"{t:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f} {ax:.4f} {ay:.4f} {az:.4f} {wl:.4f} {wr:.4f} {pl:.4f} {pr:.4f}")
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

    return 0

if __name__ == "__main__":
    sys.exit(main())
