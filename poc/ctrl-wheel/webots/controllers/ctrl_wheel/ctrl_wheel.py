#!/usr/bin/env python3
"""
ctrl_wheel -- Webots controller for the ctrl-wheel relay simulation.

SIM ONLY. Nothing here is validated on hardware, and it makes no balance claim.
The sign of the wheel against the firmware is UNVERIFIED.

State after #456 (sub-issue of #446): this file is the copied servo_wheel_dof
controller with the telemetry, overlay, odometry and upright-pose code removed.
It binds UDP CMD_PORT, drains the command socket each step, keeps the latest
valid command and drives the motor by setVelocity. It sends 0 rad/s when no
command has arrived, when the latest command carries stale=1, or when the latest
command is STALE_SIM_S or more old in simulated time. A non-finite sensor read
also stops the motor (velocity 0) and skips that step's log line. The echo back to the bridge is still NOT implemented (next
task #457). Runtime behaviour in Webots is NOT CHECKED.

Only Webots-specific import is `controller`; the sensor maths lives in
sensor_read.py so it stays importable outside Webots.

Device names (must match ctrl_wheel.wbt, case-sensitive):
    RotationalMotor: "wheel_motor"
    PositionSensor:  "wheel_sensor"
"""

import json
import math
import socket
import sys
from pathlib import Path

from controller import Robot

sys.path.insert(0, str(Path(__file__).resolve().parent))

from sensor_read import SensorReadError, read_sensor  # noqa: E402

HOST = "127.0.0.1"
CMD_PORT = 5008  # fixed command port; 5006 belongs to servo_wheel_dof, never use it
MOTOR_NAME = "wheel_motor"
SENSOR_NAME = "wheel_sensor"
TAG = "ctrl_wheel"
CLAMP_RAD_S = 3.0  # UNVERIFIED: symmetric clamp on the commanded wheel velocity
STALE_SIM_S = 0.2  # simulated seconds without a fresh command before v = 0
SIGN = 1  # UNVERIFIED sign against the firmware; the flip lives ONLY in the bridge --sign flag, never flip both


def _say(msg: str, err: bool = False) -> None:
    # flush=True: Webots pipes stdout; without it lines can appear late/never.
    print(f"{TAG}: {msg}", file=sys.stderr if err else sys.stdout, flush=True)


def _log(t_s: float, angle_rad: float, velocity_rad_s) -> None:
    # flush=True: Webots pipes stdout; without it lines can appear late/never.
    vel_str = "nan" if velocity_rad_s is None else f"{velocity_rad_s:.6f}"
    print(f"{TAG}: t={t_s:.4f} angle_rad={angle_rad:.6f} velocity_rad_s={vel_str}",
          flush=True)


def _reject_constant(name: str):
    raise ValueError(f"non-finite JSON constant {name}")


def _no_duplicate_keys(pairs):
    # Same strictness as the bridge decoder (ctrl_wheel_udp.py): a repeated key is invalid.
    obj = {}
    for key, value in pairs:
        if key in obj:
            raise ValueError(f"duplicate key {key!r}")
        obj[key] = value
    return obj


def _parse_cmd(data: bytes):
    """Return (cmd_seq, wheel_rad_s, stale) for a valid datagram, else None."""
    try:
        obj = json.loads(data.decode("utf-8"), parse_constant=_reject_constant,
                         object_pairs_hook=_no_duplicate_keys)
    except (ValueError, RecursionError):
        return None
    if not isinstance(obj, dict) or set(obj.keys()) != {"cmd_seq", "wheel_rad_s", "stale"}:
        return None
    seq = obj["cmd_seq"]
    wheel = obj["wheel_rad_s"]
    stale = obj["stale"]
    if type(seq) is not int or seq < 0:
        return None
    if type(wheel) not in (int, float):
        return None
    if type(stale) is not int or stale not in (0, 1):
        return None
    try:
        if not math.isfinite(float(wheel)):
            return None
    except OverflowError:
        return None
    return seq, wheel, stale


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
    # Plain Robot: nothing here needs Supervisor powers (expected to work with the
    # world's supervisor flag either way; NOT CHECKED in Webots).
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
        sock = _open_socket(CMD_PORT)
    except OSError as exc:
        _say(
            f"ERROR: cannot bind UDP {HOST}:{CMD_PORT} ({exc}). "
            "Is another ctrl_wheel / Webots instance already running?",
            err=True,
        )
        return 1
    _say(f"listening on {HOST}:{CMD_PORT}")

    # PositionSensor must be enabled at the sim timestep, or getValue() -> nan.
    sensor.enable(timestep)

    # Velocity mode. NOT CHECKED in Webots: the position-infinity then
    # setVelocity sequence is the standard RotationalMotor idiom, not run here.
    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)

    angle_prev_rad = None
    t_prev_s = None
    last_cmd_seq = None
    last_cmd_wheel = None
    last_cmd_stale = None
    last_msg_sim_t = None
    ignored_count = 0

    try:
        while robot.step(timestep) != -1:
            t_s = robot.getTime()

            # Drain every pending datagram; keep only the latest valid one.
            while True:
                try:
                    data, _addr = sock.recvfrom(4096)
                except OSError:  # includes BlockingIOError: nothing more pending
                    break
                parsed = _parse_cmd(data)
                if parsed is None:
                    ignored_count += 1
                    if ignored_count <= 3:
                        _say(f"ignored invalid datagram #{ignored_count} "
                             f"({len(data)} bytes)")
                    continue
                last_cmd_seq, last_cmd_wheel, last_cmd_stale = parsed
                last_msg_sim_t = t_s

            # Stale or missing command -> stop. Sim time, not wall time.
            if (last_cmd_wheel is None or last_cmd_stale == 1
                    or (t_s - last_msg_sim_t) >= STALE_SIM_S):
                v = 0.0
            else:
                v = max(-CLAMP_RAD_S, min(CLAMP_RAD_S, SIGN * last_cmd_wheel))
            motor.setVelocity(v)

            try:
                reading = read_sensor(
                    t_s=t_s,
                    angle_source=sensor.getValue,
                    angle_prev_rad=angle_prev_rad,
                    t_prev_s=t_prev_s,
                )
            except SensorReadError as exc:
                # Never leave the last velocity latched on a bad read.
                motor.setVelocity(0.0)
                _say(f"ERROR: sensor read failed ({exc}); velocity 0, step skipped",
                     err=True)
                continue
            _log(reading.t_s, reading.angle_rad, reading.velocity_rad_s)

            angle_prev_rad = reading.angle_rad
            t_prev_s = reading.t_s
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
