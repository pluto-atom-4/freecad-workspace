#!/usr/bin/env python3
"""Test controller: validate InertialUnit sensor access."""

from controller import Robot, Motor
import sys
import os

def main():
    # Write output to both stdout and a log file
    log_file = "/tmp/test_imu_controller.log"

    def log_msg(msg):
        print(msg, file=sys.stdout, flush=True)
        with open(log_file, 'a') as f:
            f.write(msg + '\n')

    # Clear log file at start
    with open(log_file, 'w') as f:
        f.write('')

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    log_msg(f"[test_imu] Robot initialized with timestep={timestep}")

    # Attempt to access InertialUnit by name
    try:
        imu = robot.getDevice("imu")
        if imu is None:
            log_msg("ERROR: InertialUnit 'imu' not found via getDevice()")
            return 1

        log_msg("SUCCESS: InertialUnit 'imu' accessible")
        log_msg(f"  Device type: {imu.getType()}")
        log_msg(f"  Device name: {imu.getName()}")

        # Enable sensor sampling
        imu.enable(timestep)
        log_msg("  Sensor enabled")

        # Run one timestep to read sensor data
        robot.step(timestep)
        log_msg("  One timestep executed")

        # Read IMU data
        try:
            roll_pitch_yaw = imu.getRollPitchYaw()
            accel = imu.getAcceleration()
            log_msg(f"  Roll/Pitch/Yaw: {roll_pitch_yaw}")
            log_msg(f"  Acceleration: {accel}")
            log_msg("SUCCESS: Sensor data readable")
            return 0
        except Exception as e:
            log_msg(f"ERROR: Could not read sensor data: {e}")
            import traceback
            log_msg(traceback.format_exc())
            return 1

    except Exception as e:
        log_msg(f"ERROR: {e}")
        import traceback
        log_msg(traceback.format_exc())
        return 1

if __name__ == "__main__":
    exit(main())
