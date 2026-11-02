#!/usr/bin/env python3
"""
ctrl_wheel -- Webots controller for the ctrl-wheel relay simulation.

SIM ONLY. Nothing here is validated on hardware, and it makes no balance claim.
The sign of the wheel against the firmware is UNVERIFIED.

State after the strip step (issue #455, sub-issue of #446): this file is the
copied servo_wheel_dof controller with the telemetry, overlay, odometry and
upright-pose code removed. It binds a UDP socket and prints the wheel sensor
reading each step. It does NOT yet drive the motor and does NOT yet send
anything back; those come in the next tasks (#456 setVelocity and the stale
stop, #457 the sensor echo).

Only Webots-specific import is `controller`; the sensor maths lives in
sensor_read.py so it stays importable outside Webots.

Device names (must match ctrl_wheel.wbt, case-sensitive):
    RotationalMotor: "wheel_motor"
    PositionSensor:  "wheel_sensor"
"""

import os
import socket
import sys
from pathlib import Path

from controller import Robot

sys.path.insert(0, str(Path(__file__).resolve().parent))

from sensor_read import read_sensor  # noqa: E402

HOST = "127.0.0.1"
DEFAULT_PORT = 5006  # to be changed to 5008 in #456; 5006 belongs to servo_wheel_dof
PORT_ENV = "SWD_UDP_PORT"
MOTOR_NAME = "wheel_motor"
SENSOR_NAME = "wheel_sensor"
TAG = "ctrl_wheel"


def _say(msg: str, err: bool = False) -> None:
    # flush=True: Webots pipes stdout; without it lines can appear late/never.
    print(f"{TAG}: {msg}", file=sys.stderr if err else sys.stdout, flush=True)


def _log(t_s: float, angle_rad: float, velocity_rad_s) -> None:
    # flush=True: Webots pipes stdout; without it lines can appear late/never.
    vel_str = "nan" if velocity_rad_s is None else f"{velocity_rad_s:.6f}"
    print(f"{TAG}: t={t_s:.4f} angle_rad={angle_rad:.6f} velocity_rad_s={vel_str}",
          flush=True)


def _read_port() -> int:
    """Return the UDP port from SWD_UDP_PORT (default DEFAULT_PORT); ValueError if bad."""
    raw = os.environ.get(PORT_ENV)
    if raw is None or raw == "":
        return DEFAULT_PORT
    try:
        port = int(raw)
    except ValueError:
        raise ValueError(f"{PORT_ENV}={raw!r} is not an integer") from None
    if not 1 <= port <= 65535:
        raise ValueError(f"{PORT_ENV}={port} is outside 1..65535")
    return port


def _open_socket(port: int) -> socket.socket:
    """Bind a non-blocking UDP socket. Deliberately NO SO_REUSEADDR: on Linux it
    would let two listeners share the port and silently split datagrams."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((HOST, port))
    except OSError:
        sock.close()
        raise
    sock.setblocking(False)
    return sock


def main() -> int:
    try:
        port = _read_port()
    except ValueError as exc:
        _say(f"ERROR: {exc}", err=True)
        return 1

    # Plain Robot: nothing here needs Supervisor powers (works with the world's
    # supervisor flag either way).
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    motor = robot.getDevice(MOTOR_NAME)
    if motor is None:
        _say(f"ERROR device '{MOTOR_NAME}' not found", err=True)
        return 1
    sensor = robot.getDevice(SENSOR_NAME)
    if sensor is None:
        _say(f"ERROR device '{SENSOR_NAME}' not found", err=True)
        return 1

    try:
        sock = _open_socket(port)
    except OSError as exc:
        _say(
            f"ERROR: cannot bind UDP {HOST}:{port} ({exc}). "
            "Is another ctrl_wheel / Webots instance already running?",
            err=True,
        )
        return 1
    _say(f"listening on {HOST}:{port}")

    # PositionSensor must be enabled at the sim timestep, or getValue() -> nan.
    sensor.enable(timestep)

    angle_prev_rad = None
    t_prev_s = None

    try:
        while robot.step(timestep) != -1:
            t_s = robot.getTime()

            reading = read_sensor(
                t_s=t_s,
                angle_source=sensor.getValue,
                angle_prev_rad=angle_prev_rad,
                t_prev_s=t_prev_s,
            )
            _log(reading.t_s, reading.angle_rad, reading.velocity_rad_s)

            angle_prev_rad = reading.angle_rad
            t_prev_s = reading.t_s
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
