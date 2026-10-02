#!/usr/bin/env python3
"""
Tests for imu_udp_latest.drain_latest using real localhost UDP sockets
(127.0.0.1, ephemeral port 0). Issue #285, sub-issue of #258. No Webots, no
hardware, must NOT import `controller` or `serial`.

Usage:
    cd poc/servo-wheel-dof
    mamba run -n servo-wheel-dof python3 -m pytest -q \
        webots/controllers/servo_wheel_dof/test_swd_imu_udp_latest.py
"""

from __future__ import annotations

import select
import socket
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from imu_udp_latest import drain_latest  # noqa: E402
from imu_wheel_msg import ImuWheel, format_imu_wheel_msg  # noqa: E402


@pytest.fixture
def pair():
    """A non-blocking receiver on an OS-chosen port, plus a sender socket."""
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = rx.getsockname()
    try:
        yield rx, tx, addr
    finally:
        tx.close()
        rx.close()


def _wait_readable(rx, timeout=2.0):
    """Loopback delivery is synchronous once sendto returns; this is a safety net."""
    ready, _, _ = select.select([rx], [], [], timeout)
    assert ready, "no datagram arrived on loopback within timeout"


def _sample(roll, pitch=0.0, yaw=0.0, wheel_angle=0.0) -> ImuWheel:
    return ImuWheel(roll, pitch, yaw, wheel_angle)


def _msg(sample: ImuWheel, seq: int = 1) -> bytes:
    """A valid four-key datagram (note: includes wheel_angle)."""
    return format_imu_wheel_msg(seq, sample)


class TestEmpty:
    def test_empty_socket(self, pair):
        """Nothing queued: latest is None and both counts are zero."""
        rx, _tx, _addr = pair
        r = drain_latest(rx)
        assert r.latest is None
        assert (r.valid, r.invalid) == (0, 0)

    def test_second_drain_is_empty(self, pair):
        """Draining consumes the queue; the next drain sees nothing."""
        rx, tx, addr = pair
        tx.sendto(_msg(_sample(0.2)), addr)
        _wait_readable(rx)
        assert drain_latest(rx).latest is not None
        r = drain_latest(rx)
        assert r.latest is None
        assert (r.valid, r.invalid) == (0, 0)


class TestLatestWins:
    def test_latest_valid_wins_and_garbage_counted(self, pair):
        """Valid A, garbage, valid B: B is latest, counts are (2 valid, 1 invalid)."""
        rx, tx, addr = pair
        tx.sendto(_msg(_sample(0.1)), addr)
        tx.sendto(b"not json", addr)
        tx.sendto(_msg(_sample(0.3, 0.2, 0.1, 1.5)), addr)
        _wait_readable(rx)
        r = drain_latest(rx)
        assert r.latest == _sample(0.3, 0.2, 0.1, 1.5)
        assert (r.valid, r.invalid) == (2, 1)

    def test_trailing_garbage_does_not_override_valid(self, pair):
        """An invalid datagram after a valid one never replaces it."""
        rx, tx, addr = pair
        tx.sendto(_msg(_sample(0.5)), addr)
        tx.sendto(
            b'{"roll": NaN, "pitch": 0, "yaw": 0, "wheel_angle": 0}', addr
        )
        _wait_readable(rx)
        r = drain_latest(rx)
        assert r.latest == _sample(0.5)
        assert (r.valid, r.invalid) == (1, 1)

    def test_missing_wheel_angle_is_invalid(self, pair):
        """A three-key (esp32-style) datagram is invalid here: wheel_angle required."""
        rx, tx, addr = pair
        tx.sendto(b'{"seq":1,"roll":0.1,"pitch":0.2,"yaw":0.3}', addr)
        _wait_readable(rx)
        r = drain_latest(rx)
        assert r.latest is None
        assert (r.valid, r.invalid) == (0, 1)


class TestCap:
    def test_max_datagrams_cap_leaves_remainder(self, pair):
        """The cap stops the drain; the remainder is picked up next call."""
        rx, tx, addr = pair
        for v in (0.1, 0.2, 0.3):
            tx.sendto(_msg(_sample(v)), addr)
        _wait_readable(rx)
        first = drain_latest(rx, max_datagrams=2)
        assert first.valid == 2
        assert first.latest == _sample(0.2)
        second = drain_latest(rx)
        assert second.valid == 1
        assert second.latest == _sample(0.3)


class TestSocketErrors:
    def test_oserror_ends_drain_quietly(self):
        """Any OSError after a valid datagram ends the drain without raising."""

        class FakeSock:
            def __init__(self):
                self.calls = 0

            def recvfrom(self, _n):
                self.calls += 1
                if self.calls == 1:
                    return _msg(_sample(0.4)), ("127.0.0.1", 1)
                raise ConnectionResetError("boom")

        r = drain_latest(FakeSock())
        assert r.latest == _sample(0.4)
        assert (r.valid, r.invalid) == (1, 0)