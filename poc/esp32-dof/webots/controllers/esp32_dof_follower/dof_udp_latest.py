#!/usr/bin/env python3
"""
Drain a non-blocking UDP socket and keep only the LATEST valid orientation.

Stdlib + dof_webots_math only. Deliberately does NOT import `controller`, so it
is unit-testable with pytest outside Webots.

Used by esp32_dof_follower.py once per Webots step: the monitor sends at 50 Hz
while Webots steps at basicTimeStep (16 ms), so several (or zero) datagrams may
be queued per step; only the newest valid pose matters.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import NamedTuple, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))

from dof_webots_math import parse_orientation_msg  # noqa: E402

RECV_BUFSIZE = 65535  # max UDP payload; no truncation possible
DEFAULT_MAX_DATAGRAMS = 1024  # per call; bounds time spent in one Webots step


class DrainResult(NamedTuple):
    latest: Optional[tuple[float, float, float]]  # (roll, pitch, yaw) or None
    valid: int  # valid datagrams consumed this call
    invalid: int  # unparseable datagrams consumed this call


def drain_latest(sock, max_datagrams: int = DEFAULT_MAX_DATAGRAMS) -> DrainResult:
    """
    Read pending datagrams from `sock` (must be non-blocking) until none are
    left or `max_datagrams` were read. Returns the last VALID (roll, pitch, yaw)
    (an invalid datagram never overrides an earlier valid one) plus counts.

    Any OSError (BlockingIOError = queue empty, or a stray socket error such as
    ConnectionResetError) ends the drain quietly.
    """
    latest = None
    valid = 0
    invalid = 0
    for _i in range(max_datagrams):
        try:
            data, _addr = sock.recvfrom(RECV_BUFSIZE)
        except OSError:
            break
        parsed = parse_orientation_msg(data)
        if parsed is None:
            invalid += 1
        else:
            latest = parsed
            valid += 1
    return DrainResult(latest, valid, invalid)
