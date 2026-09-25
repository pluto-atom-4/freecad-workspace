#!/usr/bin/env python3
"""
Tests for dof_udp_latest.drain_latest using real localhost UDP sockets
(127.0.0.1, ephemeral port 0). No Webots needed.

Usage:
    cd poc/esp32-dof/webots/controllers/esp32_dof_follower
    mamba run -n esp32-dof python3 -m pytest test_esp32dof_udp_latest.py -v
"""

import select
import socket
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from dof_udp_latest import drain_latest  # noqa: E402


@pytest.fixture
def pair():
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


def _msg(roll, pitch=0.0, yaw=0.0):
    return ('{"seq":1,"roll":%r,"pitch":%r,"yaw":%r}' % (roll, pitch, yaw)).encode()


def test_empty_socket(pair):
    rx, _tx, _addr = pair
    r = drain_latest(rx)
    assert r.latest is None
    assert (r.valid, r.invalid) == (0, 0)


def test_latest_valid_wins_and_garbage_counted(pair):
    rx, tx, addr = pair
    tx.sendto(_msg(0.1), addr)
    tx.sendto(b"not json", addr)
    tx.sendto(_msg(0.3, 0.2, 0.1), addr)
    _wait_readable(rx)
    r = drain_latest(rx)
    assert r.latest == pytest.approx((0.3, 0.2, 0.1))
    assert (r.valid, r.invalid) == (2, 1)


def test_trailing_garbage_does_not_override_valid(pair):
    rx, tx, addr = pair
    tx.sendto(_msg(0.5), addr)
    tx.sendto(b'{"roll": NaN, "pitch": 0, "yaw": 0}', addr)
    _wait_readable(rx)
    r = drain_latest(rx)
    assert r.latest == pytest.approx((0.5, 0.0, 0.0))
    assert (r.valid, r.invalid) == (1, 1)


def test_second_drain_is_empty(pair):
    rx, tx, addr = pair
    tx.sendto(_msg(0.2), addr)
    _wait_readable(rx)
    assert drain_latest(rx).latest is not None
    r = drain_latest(rx)
    assert r.latest is None
    assert (r.valid, r.invalid) == (0, 0)


def test_max_datagrams_cap_leaves_remainder(pair):
    rx, tx, addr = pair
    for v in (0.1, 0.2, 0.3):
        tx.sendto(_msg(v), addr)
    _wait_readable(rx)
    first = drain_latest(rx, max_datagrams=2)
    assert first.valid == 2
    assert first.latest == pytest.approx((0.2, 0.0, 0.0))
    second = drain_latest(rx)
    assert second.valid == 1
    assert second.latest == pytest.approx((0.3, 0.0, 0.0))


def test_oserror_other_than_would_block_ends_drain_quietly():
    class FakeSock:
        def __init__(self):
            self.calls = 0

        def recvfrom(self, _n):
            self.calls += 1
            if self.calls == 1:
                return _msg(0.4), ("127.0.0.1", 1)
            raise ConnectionResetError("boom")

    r = drain_latest(FakeSock())
    assert r.latest == pytest.approx((0.4, 0.0, 0.0))
    assert (r.valid, r.invalid) == (1, 0)
