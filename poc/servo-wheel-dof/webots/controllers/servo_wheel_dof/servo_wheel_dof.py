#!/usr/bin/env python3
"""
servo_wheel_dof -- Webots controller (issue #265, sub-issue of #258).

Drives the "wheel_motor" RotationalMotor with a velocity command from the pure
servo_sim.motion_profile() and reads back "wheel_sensor" via the pure
sensor_read.read_sensor(), printing (t, angle_rad, velocity_rad_s) each step.

Only Webots-specific import is `controller`; motion-profile math and
sensor-sample validation live in servo_sim.py / sensor_read.py so they stay
importable and unit-testable outside Webots (see this repo's esp32-dof
convention: poc/esp32-dof/CLAUDE.md "Conventions").

Device names (must match servo_wheel_dof.wbt exactly, case-sensitive):
    RotationalMotor: "wheel_motor"
    PositionSensor:  "wheel_sensor"
"""

import sys
from pathlib import Path

from controller import Robot

sys.path.insert(0, str(Path(__file__).resolve().parent))

from servo_sim import motion_profile  # noqa: E402
from sensor_read import read_sensor  # noqa: E402

MOTOR_NAME = "wheel_motor"
SENSOR_NAME = "wheel_sensor"
TAG = "servo_wheel_dof"


def _log(t_s: float, angle_rad: float, velocity_rad_s) -> None:
    # flush=True: Webots pipes stdout; without it lines can appear late/never.
    vel_str = "nan" if velocity_rad_s is None else f"{velocity_rad_s:.6f}"
    print(f"{TAG}: t={t_s:.4f} angle_rad={angle_rad:.6f} velocity_rad_s={vel_str}",
          flush=True)


def main() -> int:
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    motor = robot.getDevice(MOTOR_NAME)
    if motor is None:
        print(f"{TAG}: ERROR device '{MOTOR_NAME}' not found", file=sys.stderr, flush=True)
        return 1
    sensor = robot.getDevice(SENSOR_NAME)
    if sensor is None:
        print(f"{TAG}: ERROR device '{SENSOR_NAME}' not found", file=sys.stderr, flush=True)
        return 1

    # RotationalMotor defaults to position control (target 0 rad). Switch to
    # velocity-control mode: setPosition(inf) first, then setVelocity() works.
    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)

    # PositionSensor must be enabled at the sim timestep, or getValue() -> nan.
    sensor.enable(timestep)

    angle_prev_rad = None
    t_prev_s = None

    while robot.step(timestep) != -1:
        t_s = robot.getTime()

        _target_angle_rad, target_velocity_rad_s = motion_profile(t_s)
        motor.setVelocity(target_velocity_rad_s)

        reading = read_sensor(
            t_s=t_s,
            angle_source=sensor.getValue,
            angle_prev_rad=angle_prev_rad,
            t_prev_s=t_prev_s,
        )
        _log(reading.t_s, reading.angle_rad, reading.velocity_rad_s)

        angle_prev_rad = reading.angle_rad
        t_prev_s = reading.t_s

    return 0


if __name__ == "__main__":
    sys.exit(main())
