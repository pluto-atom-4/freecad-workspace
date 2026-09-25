#!/usr/bin/env python3
"""
esp32_dof_follower -- Webots supervisor controller (issue #236, sub-issue of #227).

Rotates the ESP32_BODY Robot node to follow orientation datagrams sent by the
host monitor over UDP (127.0.0.1:5005, JSON {"seq","roll","pitch","yaw"}, radians,
ZYX convention). See poc/esp32-dof/README.md "Webots IPC contract".

Behavior:
- Every Webots step: drain all pending datagrams, keep the LATEST valid one,
  apply it via Node "rotation" field. No message yet -> node left unchanged.
- The simulation must be RUNNING (not paused) for the box to move.
- Set DOF_FOLLOWER_DEBUG=1 to print the applied rotation (every 10th apply).

Only Webots-specific import is `controller`; the rest is stdlib + local helpers,
so all parsing/socket logic lives in testable modules (dof_webots_math,
dof_udp_latest).
"""

import os
import socket
import sys
from pathlib import Path

from controller import Supervisor

sys.path.insert(0, str(Path(__file__).resolve().parent))

from dof_webots_math import euler_to_axis_angle  # noqa: E402
from dof_udp_latest import drain_latest  # noqa: E402

HOST = "127.0.0.1"
PORT = 5005
BODY_DEF = "ESP32_BODY"
TAG = "esp32_dof_follower"
DEBUG = os.environ.get("DOF_FOLLOWER_DEBUG", "") == "1"


def _log(msg, err=False):
    # flush=True: Webots pipes stdout; without it lines can appear late/never.
    print(f"{TAG}: {msg}", file=sys.stderr if err else sys.stdout, flush=True)


def _open_socket():
    """Bind a non-blocking UDP socket. Deliberately NO SO_REUSEADDR: on Linux it
    would let two listeners share the port and silently split datagrams."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((HOST, PORT))
    except OSError:
        sock.close()
        raise
    sock.setblocking(False)
    return sock


def main() -> int:
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    node = robot.getFromDef(BODY_DEF)
    if node is None:
        node = robot.getSelf()
    if node is None:
        _log(f"ERROR: node DEF '{BODY_DEF}' not found and getSelf() failed", err=True)
        return 1
    rot = node.getField("rotation")
    if rot is None:
        _log("ERROR: node has no 'rotation' field", err=True)
        return 1

    try:
        sock = _open_socket()
    except OSError as exc:
        _log(
            f"ERROR: cannot bind UDP {HOST}:{PORT} ({exc}). "
            "Is another esp32_dof_follower / Webots instance already running?",
            err=True,
        )
        return 1

    _log(f"listening on {HOST}:{PORT}")

    got_first = False
    applied = 0
    invalid_total = 0
    try:
        while robot.step(timestep) != -1:
            result = drain_latest(sock)

            if result.invalid:
                before = invalid_total
                invalid_total += result.invalid
                if before < 3 or invalid_total // 100 > before // 100:
                    _log(f"ignored {invalid_total} invalid datagram(s) so far")

            if result.latest is None:
                continue

            if not got_first:
                got_first = True
                _log("first orientation message received")

            roll, pitch, yaw = result.latest
            axis_angle = list(euler_to_axis_angle(roll, pitch, yaw))
            rot.setSFRotation(axis_angle)
            applied += 1
            if DEBUG and applied % 10 == 1:
                _log(
                    "rotation %.6f %.6f %.6f %.6f" % tuple(axis_angle)
                )
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
