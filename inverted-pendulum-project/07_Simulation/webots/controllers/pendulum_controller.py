#!/usr/bin/env python3
"""Pendulum robot controller: read IMU + wheel position sensors."""

from controller import Robot
import sys
import os

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Get sensor devices
    imu = robot.getDevice("imu")
    wheel_left_sensor = robot.getDevice("wheel_left_joint_sensor")
    wheel_right_sensor = robot.getDevice("wheel_right_joint_sensor")

    if not all([imu, wheel_left_sensor, wheel_right_sensor]):
        print("ERROR: One or more sensors not found", file=sys.stderr)
        return 1

    # Enable sensors
    imu.enable(timestep)
    wheel_left_sensor.enable(timestep)
    wheel_right_sensor.enable(timestep)

    # Log output (dual-sink: stdout + file)
    log_file = open("/tmp/pendulum_controller.log", "w")

    def log_msg(msg):
        print(msg)
        log_file.write(msg + "\n")
        log_file.flush()

    log_msg("Pendulum controller started. Reading IMU + wheel sensors.")
    log_msg("Time(s) Roll(rad) Pitch(rad) Yaw(rad) Ax(m/s2) Ay(m/s2) Az(m/s2) WheelL(rad) WheelR(rad)")

    # Main loop: run for up to 10 seconds
    max_time = 10.0

    while robot.step(timestep) != -1:
        t = robot.getTime()

        if t >= max_time:
            break

        # Read sensors
        rpy = imu.getRollPitchYaw()
        accel = imu.getAcceleration()
        wl = wheel_left_sensor.getValue()
        wr = wheel_right_sensor.getValue()

        # Log (check for NaN)
        if None in [rpy, accel, wl, wr] or any(x is None for x in rpy) or any(x is None for x in accel):
            log_msg(f"{t:.3f} READING_ERROR: sensor returned None")
            continue

        roll, pitch, yaw = rpy
        ax, ay, az = accel

        log_msg(f"{t:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f} {ax:.4f} {ay:.4f} {az:.4f} {wl:.4f} {wr:.4f}")

    log_msg(f"Controller finished at t={robot.getTime():.3f}s")
    log_file.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
