#!/usr/bin/env python3
"""
servo_wheel_dof -- Webots supervisor controller (issue #286, sub-issue of #258).

Follows UDP datagrams from the host monitor (127.0.0.1, default port 5006,
env SWD_UDP_PORT; JSON {"seq","roll","pitch","yaw","wheel_angle"}, radians).

Behavior:
- Every Webots step: drain all pending datagrams and keep the LATEST valid one.
- On a message: teleport the SERVO_WHEEL_DOF Robot to the upright pose
  upright_pose(pitch, yaw) (roll is ignored; the wheel centre stays fixed and the
  rim stays on the floor) and command "wheel_motor" to position wheel_angle.
- No message yet: translation and rotation are NOT touched, so the world keeps
  the pose from servo_wheel_dof.wbt.
- Each step prints "t=... angle_rad=... velocity_rad_s=..." read back from
  "wheel_sensor" via sensor_read.read_sensor().
- The simulation must be RUNNING (not paused) for the controller to step.

Only Webots-specific import is `controller`; message parsing, UDP draining and
the pose maths live in imu_wheel_msg.py, imu_udp_latest.py and upright_pose.py
so they stay importable and unit-testable outside Webots.

Device names (must match servo_wheel_dof.wbt exactly, case-sensitive):
    RotationalMotor: "wheel_motor"
    PositionSensor:  "wheel_sensor"
"""

import os
import socket
import sys
from pathlib import Path

from controller import Supervisor

sys.path.insert(0, str(Path(__file__).resolve().parent))

from imu_udp_latest import drain_latest  # noqa: E402
from sensor_read import read_sensor  # noqa: E402
from upright_pose import upright_pose  # noqa: E402

HOST = "127.0.0.1"
DEFAULT_PORT = 5006
PORT_ENV = "SWD_UDP_PORT"
ROBOT_DEF = "SERVO_WHEEL_DOF"
MOTOR_NAME = "wheel_motor"
SENSOR_NAME = "wheel_sensor"
TAG = "servo_wheel_dof"


def _say(msg: str, err: bool = False) -> None:
    # flush=True: Webots pipes stdout; without it lines can appear late/never.
    print(f"{TAG}: {msg}", file=sys.stderr if err else sys.stdout, flush=True)


def _log(t_s: float, angle_rad: float, velocity_rad_s) -> None:
    # flush=True: Webots pipes stdout; without it lines can appear late/never.
    vel_str = "nan" if velocity_rad_s is None else f"{velocity_rad_s:.6f}"
    print(f"{TAG}: t={t_s:.4f} angle_rad={angle_rad:.6f} velocity_rad_s={vel_str}",
          flush=True)


def _read_port() -> int:
    """Return the UDP port from SWD_UDP_PORT (default 5006); ValueError if bad."""
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

    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    motor = robot.getDevice(MOTOR_NAME)
    if motor is None:
        _say(f"ERROR device '{MOTOR_NAME}' not found", err=True)
        return 1
    sensor = robot.getDevice(SENSOR_NAME)
    if sensor is None:
        _say(f"ERROR device '{SENSOR_NAME}' not found", err=True)
        return 1

    node = robot.getFromDef(ROBOT_DEF)
    if node is None:
        node = robot.getSelf()
    if node is None:
        _say(f"ERROR: node DEF '{ROBOT_DEF}' not found and getSelf() failed",
             err=True)
        return 1
    translation_field = node.getField("translation")
    rotation_field = node.getField("rotation")
    if translation_field is None or rotation_field is None:
        _say("ERROR: node has no 'translation'/'rotation' field", err=True)
        return 1

    try:
        sock = _open_socket(port)
    except OSError as exc:
        _say(
            f"ERROR: cannot bind UDP {HOST}:{port} ({exc}). "
            "Is another servo_wheel_dof / Webots instance already running?",
            err=True,
        )
        return 1
    _say(f"listening on {HOST}:{port}")

    # PositionSensor must be enabled at the sim timestep, or getValue() -> nan.
    # The motor stays in its default position-control mode (no setPosition(inf)).
    sensor.enable(timestep)

    got_first = False
    invalid_total = 0
    angle_prev_rad = None
    t_prev_s = None

    try:
        while robot.step(timestep) != -1:
            t_s = robot.getTime()
            result = drain_latest(sock)

            if result.invalid:
                before = invalid_total
                invalid_total += result.invalid
                if before < 3 or invalid_total // 100 > before // 100:
                    _say(f"ignored {invalid_total} invalid datagram(s) so far")

            msg = result.latest
            if msg is not None:
                if not got_first:
                    got_first = True
                    _say("first message received")
                # Roll is ignored by design (issue #286): upright pose only.
                translation, rotation = upright_pose(msg.pitch, msg.yaw)
                translation_field.setSFVec3f(list(translation))
                rotation_field.setSFRotation(list(rotation))
                motor.setPosition(msg.wheel_angle)

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
