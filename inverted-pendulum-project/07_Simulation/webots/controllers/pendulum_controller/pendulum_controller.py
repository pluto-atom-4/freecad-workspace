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

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

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
        def log_msg(msg):
            print(msg)
            log_file.write(msg + "\n")
            log_file.flush()

        log_msg("Pendulum controller started. Reading all 5 sensors.")
        log_msg("Time(s) IMU_Roll(rad) IMU_Pitch(rad) IMU_Yaw(rad) IMU_Ax(m/s2) IMU_Ay(m/s2) IMU_Az(m/s2) WheelL(rad) WheelR(rad) PivotL(rad) PivotR(rad)")

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
                    log_msg(f"{t:.3f} READING_ERROR: sensor returned None")
                    continue

                roll, pitch, yaw = rpy
                ax, ay, az = accel

                log_msg(f"{t:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f} {ax:.4f} {ay:.4f} {az:.4f} {wl:.4f} {wr:.4f} {pl:.4f} {pr:.4f}")
        finally:
            log_msg(f"Controller finished at t={robot.getTime():.3f}s")

    return 0

if __name__ == "__main__":
    sys.exit(main())
