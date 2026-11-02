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
- Opt-in telemetry (issue #306): env SWD_TELEMETRY=1 emits one CSV row per step
  (columns in telemetry.COLUMNS; pose columns need the wheel node, else nan). With
  SWD_TELEMETRY_FILE the rows go to that file (relative paths land in the
  controller directory), else to stdout with the prefix "telemetry: ". Unset
  SWD_TELEMETRY leaves all output unchanged.
- Opt-in overlay (issue #313): env SWD_OVERLAY=1 (needs SWD_TELEMETRY=1 and the
  wheel node) draws the alignment status over the 3D view with
  Supervisor.setLabel, about 4 times per second. GUI only; unset SWD_OVERLAY
  leaves all output unchanged.
- Opt-in odometry (issue #301): env SWD_ODOMETRY=1 adds a no-slip rolling offset,
  from the change of the COMMANDED wheel_angle (never the sensor), to the x, y of
  the upright translation (z untouched; clamped to a +-0.9 m box, ONE stderr WARN
  on the first clamp). Positive wheel_angle rolls toward -X at yaw 0; the first
  message never moves the robot. On any odometry error: ONE stderr ERROR line and
  odometry stays off for the rest of the run. Unset SWD_ODOMETRY leaves all
  output unchanged.
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

from alignment_metrics import anchor_metrics, relative_position  # noqa: E402
from imu_udp_latest import drain_latest  # noqa: E402
from odometry import (  # noqa: E402
    OFFSET_LIMIT_M,
    OdomState,
    apply_offset,
    parse_odometry_env,
    step_odometry,
)
from overlay_text import (  # noqa: E402
    format_overlay,
    overlay_color,
    overlay_every_steps,
    parse_overlay_env,
)
from sensor_read import read_sensor  # noqa: E402
from telemetry import (  # noqa: E402
    TelemetryConfig,
    TelemetryWriter,
    parse_telemetry_env,
)
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


def _open_telemetry(cfg: TelemetryConfig) -> "TelemetryWriter | None":
    """Return the telemetry writer for cfg, or None (off or file unusable).

    File mode opens the path with "w"; stdout mode never owns sys.stdout.
    Never raises: an unusable file prints ONE stderr line and returns None.
    """
    if not cfg.enabled:
        return None
    if not cfg.path:
        _say("telemetry -> stdout")
        return TelemetryWriter(sys.stdout, prefix="telemetry: ", owns_file=False)
    path = os.path.abspath(cfg.path)
    try:
        fileobj = open(path, "w", encoding="utf-8", newline="")
    except (OSError, ValueError) as exc:
        _say(f"ERROR: cannot open telemetry file {path} ({exc}); telemetry off",
             err=True)
        return None
    writer = TelemetryWriter(fileobj)
    if writer.failed:
        writer.close()
        _say(f"ERROR: cannot write telemetry file {path}; telemetry off",
             err=True)
        return None
    _say(f"telemetry -> {path}")
    return writer


def _find_wheel(node):
    """Return the endPoint Solid of the first HingeJoint child of node, or None.

    Read-only Supervisor calls only. May raise (caller wraps it).
    """
    children = node.getField("children")
    if children is None:
        return None
    for i in range(children.getCount()):
        child = children.getMFNode(i)
        if child is not None and child.getTypeName() == "HingeJoint":
            end_point = child.getField("endPoint")
            return None if end_point is None else end_point.getSFNode()
    return None


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

    cfg = parse_telemetry_env(os.environ)
    for warning in cfg.warnings:
        _say(warning, err=True)
    telemetry = _open_telemetry(cfg)

    # Wheel lookup happens only with telemetry on, so default runs touch no
    # extra nodes. Missing wheel: ONE warning, pose columns stay nan.
    wheel = None
    if telemetry is not None:
        try:
            wheel = _find_wheel(node)
        except Exception:  # best effort; never stop the sim
            wheel = None
        if wheel is None:
            _say("WARN: wheel node not found, alignment metrics off", err=True)

    # Overlay (issue #313): needs telemetry AND the wheel node (it reuses the
    # per-step metrics). Read-only; setLabel is GUI-only and harmless headless.
    overlay_cfg = parse_overlay_env(os.environ)
    for warning in overlay_cfg.warnings:
        _say(warning, err=True)
    overlay_on = (
        overlay_cfg.enabled and telemetry is not None and wheel is not None
    )
    if overlay_cfg.enabled and not overlay_on:
        _say("WARN: SWD_OVERLAY needs SWD_TELEMETRY=1 and the wheel node; "
             "overlay off", err=True)
    overlay_every = overlay_every_steps(timestep)
    overlay_tick = 0
    overlay_shown = False

    # Odometry (issue #301): opt-in, pure arithmetic on the COMMANDED wheel_angle.
    # Unset SWD_ODOMETRY: no output and no extra Supervisor calls.
    odo_cfg = parse_odometry_env(os.environ)
    for warning in odo_cfg.warnings:
        _say(warning, err=True)
    odo_on = odo_cfg.enabled
    if odo_on:
        _say("odometry on")
    odo_state = OdomState()
    odo_clamp_warned = False

    # PositionSensor must be enabled at the sim timestep, or getValue() -> nan.
    # The motor stays in its default position-control mode (no setPosition(inf)).
    sensor.enable(timestep)

    got_first = False
    invalid_total = 0
    angle_prev_rad = None
    t_prev_s = None
    msgs_total = 0  # valid datagrams so far (telemetry only)
    t_last_msg = None  # sim time of the last valid datagram
    cmd = None  # last commanded wheel_angle (rad)
    warn_total = 0  # steps flagged by anchor_metrics (telemetry only)

    try:
        while robot.step(timestep) != -1:
            t_s = robot.getTime()

            # Pose columns (telemetry only). Read BEFORE the apply block below,
            # so a row shows the effect of the PREVIOUS step's command.
            # Read-only API: no set*, no resetPhysics.
            pose = (None,) * 9
            warn = 0
            am = None
            rel = None
            if telemetry is not None and wheel is not None:
                try:
                    rp = node.getPosition()
                    ro = node.getOrientation()
                    wp = wheel.getPosition()
                    wo = wheel.getOrientation()
                    am = anchor_metrics(rp, ro, wp, wo)
                    rel = relative_position(rp, ro, wp)
                    pose = (
                        rp[0], rp[1], rp[2], am.wheel_z,
                        rel[0], rel[1], rel[2], am.deviation_m, am.axis_dot,
                    )
                    warn = 1 if am.warn else 0
                    if warn:
                        warn_total += 1
                        if warn_total <= 3 or warn_total % 100 == 0:
                            _say(
                                f"WARN t={t_s:.4f} dev_m={am.deviation_m:.6f} "
                                f"dot={am.axis_dot:.6f}",
                                err=True,
                            )
                except Exception as exc:  # telemetry must never stop the sim
                    _say(f"ERROR: pose read failed ({exc!r}); telemetry off",
                         err=True)
                    telemetry.close()
                    telemetry = None

            if overlay_on and am is not None and rel is not None:
                try:  # overlay must never stop the sim
                    if overlay_tick % overlay_every == 0:
                        robot.setLabel(
                            0,
                            format_overlay(am.deviation_m, am.axis_dot, rel,
                                           am.warn),
                            0.01, 0.01, 0.05, overlay_color(am.warn), 0.0,
                        )
                        overlay_shown = True
                    overlay_tick += 1
                except Exception as exc:
                    _say(f"ERROR: overlay failed ({exc!r}); overlay off",
                         err=True)
                    overlay_on = False

            result = drain_latest(sock)
            msgs_total += result.valid

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
                if odo_on:
                    try:  # odometry must never stop the sim
                        new_state = step_odometry(
                            odo_state, msg.wheel_angle, msg.yaw,
                            limit=OFFSET_LIMIT_M,
                        )
                        shifted = apply_offset(translation, new_state)
                        if new_state.clamped and not odo_clamp_warned:
                            odo_clamp_warned = True
                            _say("WARN: odometry clamped to the floor limit",
                                 err=True)
                        odo_state = new_state
                        translation = shifted
                    except Exception as exc:
                        _say(f"ERROR: odometry failed ({exc!r}); odometry off",
                             err=True)
                        odo_on = False
                translation_field.setSFVec3f(list(translation))
                rotation_field.setSFRotation(list(rotation))
                motor.setPosition(msg.wheel_angle)
                t_last_msg = t_s
                cmd = msg.wheel_angle

            reading = read_sensor(
                t_s=t_s,
                angle_source=sensor.getValue,
                angle_prev_rad=angle_prev_rad,
                t_prev_s=t_prev_s,
            )
            _log(reading.t_s, reading.angle_rad, reading.velocity_rad_s)

            if telemetry is not None:
                try:
                    # COLUMNS order; pose and warn come from the read above.
                    kinematics = (
                        t_s,
                        msgs_total,
                        None if t_last_msg is None else t_s - t_last_msg,
                        cmd,
                        reading.angle_rad,
                        None if cmd is None else cmd - reading.angle_rad,
                        reading.velocity_rad_s,
                    )
                    telemetry.write_row(kinematics + pose + (warn,))
                except Exception as exc:  # telemetry must never stop the sim
                    _say(f"ERROR: telemetry row failed ({exc!r}); telemetry off",
                         err=True)
                    telemetry.close()
                    telemetry = None

            angle_prev_rad = reading.angle_rad
            t_prev_s = reading.t_s
    finally:
        if overlay_shown:
            try:
                robot.setLabel(0, "", 0.01, 0.01, 0.05, 0, 1.0)  # clear
            except Exception:  # sim may already be shutting down
                pass
        if telemetry is not None:
            telemetry.close()  # never raises; flushes the last rows
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
