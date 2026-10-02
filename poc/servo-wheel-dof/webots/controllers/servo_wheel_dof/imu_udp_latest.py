#!/usr/bin/env python3
"""
Drain a non-blocking UDP socket and keep only the LATEST valid IMU + wheel_angle
message for the servo-wheel-dof POC (issue #285, sub-issue of #258).

Stdlib + imu_wheel_msg only. Deliberately does NOT import `controller`, `serial`
or numpy, so it is unit-testable with pytest outside Webots.

COPIED (not imported) from esp32-dof. Fixes must be mirrored by hand:
  - poc/esp32-dof/webots/controllers/esp32_dof_follower/dof_udp_latest.py
    (drain_latest, DrainResult)
Differences: parses with parse_imu_wheel_msg and returns an ImuWheel (four
required keys, including wheel_angle), so a three-key esp32-style datagram
counts as invalid here.

Used by the follower controller (issue #286) once per Webots step: the monitor
sends at 50 Hz while Webots steps at basicTimeStep (16 ms), so several (or zero)
datagrams may be queued per step; only the newest valid sample matters.

Usage:
    from imu_udp_latest import drain_latest

    result = drain_latest(sock)          # sock must be non-blocking
    if result.latest is not None:
        apply(result.latest)
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import NamedTuple

sys.path.insert(0, str(Path(__file__).resolve().parent))

from imu_wheel_msg import ImuWheel, parse_imu_wheel_msg  # noqa: E402

RECV_BUFSIZE = 65535  # max UDP payload; no truncation possible
DEFAULT_MAX_DATAGRAMS = 1024  # per call; bounds time spent in one Webots step


class DrainResult(NamedTuple):
    """Outcome of one drain_latest call."""

    latest: ImuWheel | None  # newest valid sample, or None
    valid: int  # valid datagrams consumed this call
    invalid: int  # unparseable datagrams consumed this call


def drain_latest(sock, max_datagrams: int = DEFAULT_MAX_DATAGRAMS) -> DrainResult:
    """
    Read pending datagrams from `sock` until none are left or `max_datagrams`
    were read.

    Args:
        sock: A NON-BLOCKING socket (or anything with `recvfrom(n)`).
        max_datagrams: Upper bound on datagrams read in one call.

    Returns:
        DrainResult(latest, valid, invalid). `latest` is the last VALID
        ImuWheel; an invalid datagram never overrides an earlier valid one.

    Any OSError (BlockingIOError = queue empty, or a stray socket error such as
    ConnectionResetError) ends the drain quietly.
    """
    latest = None
    valid = 0
    invalid = 0
    for _ in range(max_datagrams):
        try:
            data, _addr = sock.recvfrom(RECV_BUFSIZE)
        except OSError:
            break
        parsed = parse_imu_wheel_msg(data)
        if parsed is None:
            invalid += 1
        else:
            latest = parsed
            valid += 1
    return DrainResult(latest, valid, invalid)