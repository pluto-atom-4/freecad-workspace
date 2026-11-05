#!/usr/bin/env python3
"""
ctrl_wheels -- Webots controller for the twin-wheel ctrl-wheels relay simulation.

SIM ONLY. Nothing here is validated on hardware, and it makes no balance claim.
The sign of each wheel against the firmware is UNVERIFIED. This file has not been
run in Webots by its author; runtime behaviour in Webots is NOT CHECKED.

Webots controller for the twin-wheel ctrl-wheels simulation. It speaks the v2 wire
format of poc/ctrl-wheels/CONTRACT.md (sections S3 and S4).
It binds the UDP command port CMD_PORT (127.0.0.1:5018) and drains it every step,
keeps the newest valid v2 command, and drives the two wheel motors with
setVelocity. It reads at most 1024 datagrams per step (MAX_DATAGRAMS_PER_STEP),
processes each in arrival order, and the newest valid one wins. A backlog deeper
than the bound is read on later steps (not discarded), so a sim-time watchdog
cannot see the wall-clock age of queued commands. The bound is a chosen value, not
measured; not verified in Webots. The watchdog age is measured in simulated
time from the step in which the newest command was processed. This differs from
CONTRACT.md S5 (host clock, bridge side) on purpose, same as the single-wheel
controller; not verified in Webots at faster-than-real-time.
Known limits (documented only, no code change, owner decision):
(a) A Webots pause stops simulated time, so the 0.2 s sim-time watchdog does not
fire during a pause.
(b) The command socket is a plain Linux UDP socket with no SO_RCVBUF tuning and no
timestamps. If its receive buffer fills, the kernel drops the NEWEST datagrams, so
a stop (stale=1) sent during a stall can be lost, and the controller may apply a
command from before the stall for up to STALE_SIM_S of simulated time after resume.
(c) The intended guard during a pause is the bridge's host-clock stale=1
(CONTRACT.md S5), not this controller. The single-wheel controller behaves the
same. Sim-only, the bound is unmeasured, not verified in Webots.
Both wheels get 0 rad/s when no valid command has arrived, when the
newest command carries stale=1, or when the newest valid command is STALE_SIM_S
or more old in simulated time. A failed or non-finite sensor read also stops both
motors and skips that step's log line, echo and angle update. Each normal step it
sends ONE echo datagram to 127.0.0.1:5017 (ECHO_ADDR). Send failures are ignored
and counted, never raised. The wire format is contract v2 in
poc/ctrl-wheels/CONTRACT.md (sections S3 and S4).

Only the Webots-specific import is `controller`; the sensor maths lives in
sensor_read.py so it stays importable outside Webots.

Device names (must match the twin world ctrl_wheels.wbt, case-sensitive, #510).
The worktree used for this change does not contain that world file, so the names
are NOT CHECKED against it:
    RotationalMotor: "wheel_motor_left", "wheel_motor_right"
    PositionSensor:  "wheel_sensor_left", "wheel_sensor_right"
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
CMD_PORT = 5018  # UDP command port, host -> sim. Never 5006; 5017 is the echo port.
ECHO_ADDR = ("127.0.0.1", 5017)  # sensor echo destination (bridge side); send only, no bind
MOTOR_LEFT_NAME = "wheel_motor_left"
MOTOR_RIGHT_NAME = "wheel_motor_right"
SENSOR_LEFT_NAME = "wheel_sensor_left"
SENSOR_RIGHT_NAME = "wheel_sensor_right"
TAG = "ctrl_wheels"
PROTOCOL_VERSION = 2  # the "v" value of both the command and the echo (CONTRACT.md S3)
CLAMP_RAD_S = 3.0  # UNVERIFIED: symmetric clamp applied to each commanded wheel velocity
STALE_SIM_S = 0.2  # simulated seconds without a fresh valid command before both wheels stop
# UNVERIFIED sign for each wheel. The world is intended so that a positive command
# rolls both wheels forward (discussion 266, world design). The firmware-side mirror
# for the two servo chips is a separate sign path and is not checked against this.
# The bridge applies its own per-wheel sign to the model before it sends a command;
# do not flip the same wheel on both sides, because the two flips would cancel.
SIGN_LEFT = 1
SIGN_RIGHT = 1
RECV_BUFSIZE = 65535
MAX_DATAGRAMS_PER_STEP = 1024  # read bound per simulation step; a deeper backlog is read on later steps

# Wire key names. These are a contract with ctrl_wheels_udp.py in poc/ctrl-wheels,
# which this controller cannot import (the controller directory is outside the
# Python path in Webots). Change a key name in both files together or not at all.
CMD_KEYS = frozenset({"v", "cmd_seq", "wheels", "stale"})
WHEELS_KEYS = frozenset({"left", "right"})


def _say(msg: str, err: bool = False) -> None:
    # flush=True: Webots pipes stdout; without it lines can appear late or never.
    print(f"{TAG}: {msg}", file=sys.stderr if err else sys.stdout, flush=True)


def _fmt(value) -> str:
    return "nan" if value is None else f"{value:.6f}"


def _log(t_s: float, angle_l: float, vel_l, angle_r: float, vel_r) -> None:
    # flush=True: Webots pipes stdout; without it lines can appear late or never.
    print(f"{TAG}: t={t_s:.4f} angle_rad_l={angle_l:.6f} velocity_rad_s_l={_fmt(vel_l)} "
          f"angle_rad_r={angle_r:.6f} velocity_rad_s_r={_fmt(vel_r)}", flush=True)


def _reject_constant(name: str):
    raise ValueError(f"non-finite JSON constant {name}")


def _no_duplicate_keys(pairs):
    # Same strictness as ctrl_wheels_udp.py: a repeated key at any depth is invalid.
    obj = {}
    for key, value in pairs:
        if key in obj:
            raise ValueError(f"duplicate key {key!r}")
        obj[key] = value
    return obj


def _is_int(value) -> bool:
    # type() is used on purpose: bool is a subclass of int and must not pass.
    return type(value) is int


def _is_real(value) -> bool:
    return type(value) in (int, float)


def _parse_cmd(data: bytes):
    """Return (cmd_seq, left_rad_s, right_rad_s, stale) for a valid v2 command, else None.

    Checks match ctrl_wheels_udp.decode_cmd: exact key sets at the top level and
    inside "wheels", "v" must be the JSON integer 2, booleans are refused where
    numbers are expected, NaN and Infinity tokens are refused, ints must be JSON
    ints, duplicate keys are refused. The wheel values are not clamped here; the
    clamp is applied when the command is applied. Any malformed input returns None
    and never raises.
    """
    try:
        obj = json.loads(data.decode("utf-8"), parse_constant=_reject_constant,
                         object_pairs_hook=_no_duplicate_keys)
        if not isinstance(obj, dict) or set(obj) != CMD_KEYS:
            return None
        if not _is_int(obj["v"]) or obj["v"] != PROTOCOL_VERSION:
            return None
        seq = obj["cmd_seq"]
        if not _is_int(seq) or seq < 0:
            return None
        wheels = obj["wheels"]
        if not isinstance(wheels, dict) or set(wheels) != WHEELS_KEYS:
            return None
        left_raw = wheels["left"]
        right_raw = wheels["right"]
        if not _is_real(left_raw) or not _is_real(right_raw):
            return None
        stale = obj["stale"]
        if not _is_int(stale) or stale not in (0, 1):
            return None
        # float() raises OverflowError for a JSON int too large for a float; that is
        # caught by the except clause below. A JSON float literal such as 1e400
        # parses to inf without an error, and the isfinite check below refuses it.
        left = float(left_raw)
        right = float(right_raw)
        if not (math.isfinite(left) and math.isfinite(right)):
            return None
    except (ValueError, TypeError, OverflowError, RecursionError):
        return None
    return seq, left, right, stale


def _clamp(value: float) -> float:
    return max(-CLAMP_RAD_S, min(CLAMP_RAD_S, value))


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
    # Plain Robot: nothing here needs Supervisor powers (NOT CHECKED in Webots).
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    motor_left = robot.getDevice(MOTOR_LEFT_NAME)
    if motor_left is None:
        _say(f"ERROR device '{MOTOR_LEFT_NAME}' not found", err=True)
        return 1
    motor_right = robot.getDevice(MOTOR_RIGHT_NAME)
    if motor_right is None:
        _say(f"ERROR device '{MOTOR_RIGHT_NAME}' not found", err=True)
        return 1
    sensor_left = robot.getDevice(SENSOR_LEFT_NAME)
    if sensor_left is None:
        _say(f"ERROR device '{SENSOR_LEFT_NAME}' not found", err=True)
        return 1
    sensor_right = robot.getDevice(SENSOR_RIGHT_NAME)
    if sensor_right is None:
        _say(f"ERROR device '{SENSOR_RIGHT_NAME}' not found", err=True)
        return 1

    try:
        sock = _open_socket(CMD_PORT)
    except OSError as exc:
        _say(
            f"ERROR: cannot bind UDP {HOST}:{CMD_PORT} ({exc}). "
            "Is another ctrl_wheels / Webots instance already running?",
            err=True,
        )
        return 1
    _say(f"listening on {HOST}:{CMD_PORT} (protocol v{PROTOCOL_VERSION}), "
         f"echo to {ECHO_ADDR[0]}:{ECHO_ADDR[1]}, "
         f"motors {MOTOR_LEFT_NAME} and {MOTOR_RIGHT_NAME}, "
         f"sensors {SENSOR_LEFT_NAME} and {SENSOR_RIGHT_NAME}")

    # PositionSensor must be enabled at the sim timestep, or getValue() -> nan.
    sensor_left.enable(timestep)
    sensor_right.enable(timestep)

    # Velocity mode. NOT CHECKED in Webots: the position-infinity then
    # setVelocity sequence is the standard RotationalMotor idiom, not run here.
    for motor in (motor_left, motor_right):
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)

    # Newest valid command. All four stay None until the first valid datagram.
    last_cmd_seq = None
    last_cmd_left = None
    last_cmd_right = None
    last_cmd_stale = None
    last_msg_sim_t = None
    ignored_count = 0

    # Previous sample per wheel and the shared previous timestamp for the velocity.
    angle_prev_left = None
    angle_prev_right = None
    t_prev_s = None

    # Plain UDP socket for the sensor echo. No bind, non-blocking, send only.
    echo_sock = None
    echo_fail_count = 0

    try:
        echo_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        echo_sock.setblocking(False)
        while robot.step(timestep) != -1:
            t_s = robot.getTime()

            # Read at most MAX_DATAGRAMS_PER_STEP (1024) datagrams per step and process each
            # in arrival order, so the newest valid command wins and a stop in the tail is
            # honoured. Invalid datagrams count toward the bound. A backlog deeper than the
            # bound is read on later steps (not discarded), so a sim-time watchdog cannot
            # see the wall-clock age of queued commands. The bound is a chosen value, not
            # measured; not verified in Webots. There
            # is no sender authentication: any local sender that reaches 127.0.0.1:5018
            # can send commands (single-wheel behaviour, kept on purpose).
            # Invalid datagrams are counted and ignored, and they do not reset the watchdog.
            drained = 0
            while drained < MAX_DATAGRAMS_PER_STEP:
                try:
                    data, _addr = sock.recvfrom(RECV_BUFSIZE)
                except OSError:  # includes BlockingIOError: nothing more pending
                    break
                drained += 1
                parsed = _parse_cmd(data)
                if parsed is None:
                    ignored_count += 1
                    if ignored_count <= 3:
                        _say(f"ignored invalid datagram #{ignored_count} "
                             f"({len(data)} bytes)")
                    continue
                last_cmd_seq, last_cmd_left, last_cmd_right, last_cmd_stale = parsed
                last_msg_sim_t = t_s

            # Stale or missing command: both wheels stop. Watchdog time is simulation
            # time, not wall time.
            if (last_cmd_left is None or last_cmd_stale == 1
                    or (t_s - last_msg_sim_t) >= STALE_SIM_S):
                v_left = 0.0
                v_right = 0.0
            else:
                v_left = _clamp(SIGN_LEFT * last_cmd_left)
                v_right = _clamp(SIGN_RIGHT * last_cmd_right)
            motor_left.setVelocity(v_left)
            motor_right.setVelocity(v_right)

            try:
                reading_l = read_sensor(
                    t_s=t_s,
                    angle_source=sensor_left.getValue,
                    angle_prev_rad=angle_prev_left,
                    t_prev_s=t_prev_s,
                )
                reading_r = read_sensor(
                    t_s=t_s,
                    angle_source=sensor_right.getValue,
                    angle_prev_rad=angle_prev_right,
                    t_prev_s=t_prev_s,
                )
            except SensorReadError as exc:
                # Never leave the last velocities latched on a bad read.
                motor_left.setVelocity(0.0)
                motor_right.setVelocity(0.0)
                _say(f"ERROR: sensor read failed ({exc}); both velocities 0, step skipped",
                     err=True)
                continue
            _log(reading_l.t_s, reading_l.angle_rad, reading_l.velocity_rad_s,
                 reading_r.angle_rad, reading_r.velocity_rad_s)

            # ONE echo datagram per step. KEY NAMES ARE A CONTRACT with
            # ctrl_wheels_udp.py in poc/ctrl-wheels (cannot be imported here):
            # v, cmd_seq_echo, sim_t_s, left/right with angle_rad and vel_rad_s.
            # cmd_seq_echo is the seq of the newest valid command accepted so far
            # (null before the first one), even if the watchdog has since stopped the
            # wheels. Failures (ValueError from non-finite floats, OSError from send)
            # are counted and never raised, so Webots is never stalled.
            try:
                payload = json.dumps(
                    {"v": PROTOCOL_VERSION,
                     "cmd_seq_echo": last_cmd_seq,
                     "sim_t_s": reading_l.t_s,
                     "left": {"angle_rad": reading_l.angle_rad,
                              "vel_rad_s": reading_l.velocity_rad_s},
                     "right": {"angle_rad": reading_r.angle_rad,
                               "vel_rad_s": reading_r.velocity_rad_s}},
                    allow_nan=False, separators=(",", ":")).encode("utf-8")
                echo_sock.sendto(payload, ECHO_ADDR)
            except (ValueError, OSError):
                echo_fail_count += 1
                if echo_fail_count <= 3:
                    _say(f"echo send failed #{echo_fail_count}", err=True)

            angle_prev_left = reading_l.angle_rad
            angle_prev_right = reading_r.angle_rad
            t_prev_s = reading_l.t_s
    finally:
        # Best effort: never leave the last velocities latched on exit or on an exception.
        for name, motor in (("left", motor_left), ("right", motor_right)):
            if motor is None:
                continue
            try:
                motor.setVelocity(0.0)
            except Exception as exc:  # noqa: BLE001 - shutdown path, must not raise
                _say(f"ERROR: could not zero {name} motor on exit ({exc})", err=True)
        if echo_sock is not None:
            echo_sock.close()
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
