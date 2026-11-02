#!/usr/bin/env python3
"""
Unit tests for fake_wheel.FakeWheel, the headless stand-in for the Webots
ctrl_wheel controller (issue FW #451, test first). The module is written in
the next run; these tests pin its API and behaviour.

SIM ONLY. The sign convention and the model values (tau, dt, clamp) are
UNVERIFIED. Nothing here validates a real wheel or a balance loop.

Ephemeral ports only (port 0). Ports 5005-5008 are never used.

Usage (run from poc/ctrl-wheel):
    python3 -m pytest test_ctrl_wheel_fake.py -v
"""

from __future__ import annotations

import json
import socket
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ctrl_wheel_udp import decode_echo, drain_all, encode_cmd  # noqa: E402
from fake_wheel import FakeWheel  # noqa: E402


RESERVED_PORTS = frozenset(range(5005, 5009))
DT_S = 0.016
TOL = 0.1
# Lag factor dt / (tau + dt) with the default tau_s = 0.05, dt_s = 0.016.
LAG = DT_S / (0.05 + DT_S)


@pytest.fixture
def echo_rx():
    """Test-side echo receiver on an ephemeral port, non-blocking."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.setblocking(False)
    yield sock
    sock.close()


@pytest.fixture
def tx():
    """Test-side sender for command datagrams."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    yield sock
    sock.close()


@pytest.fixture
def make_wheel(echo_rx):
    """Factory for FakeWheel instances aimed at echo_rx; closes them at teardown."""
    made: list[FakeWheel] = []

    def factory(**kwargs) -> FakeWheel:
        wheel = FakeWheel(echo_rx.getsockname(), **kwargs)
        made.append(wheel)
        return wheel

    yield factory
    for wheel in made:
        wheel.close()


def _echoes(rx: socket.socket):
    return [decode_echo(data) for data in drain_all(rx)]


def _step_one(wheel: FakeWheel, rx: socket.socket):
    """Step once and return the single echo that step must send."""
    wheel.step()
    echoes = _echoes(rx)
    assert len(echoes) == 1, f"expected one echo per step, got {len(echoes)}"
    return echoes[0]


def _send(tx: socket.socket, wheel: FakeWheel, data: bytes) -> None:
    tx.sendto(data, wheel.cmd_addr)


def _run_steps_with_cmd(
    wheel: FakeWheel,
    tx: socket.socket,
    echo_rx: socket.socket,
    n: int,
    cmd_seq: int,
    wheel_rad_s: float,
    stale: int = 0,
) -> list:
    """Model the real bridge: re-send the same command before every step.

    Returns the list of decoded echoes, one per step.
    """
    echoes = []
    for _ in range(n):
        _send(tx, wheel, encode_cmd(cmd_seq, wheel_rad_s, stale))
        wheel.step()
        echoes.extend(_echoes(echo_rx))
    assert len(echoes) == n, f"expected {n} echoes, got {len(echoes)}"
    return echoes


def test_cmd_addr_is_ephemeral_and_not_reserved(make_wheel):
    wheel = make_wheel()
    host, port = wheel.cmd_addr
    assert host == "127.0.0.1"
    assert port != 0
    assert port not in RESERVED_PORTS


def test_first_step_echo_has_no_cmd_and_no_velocity(make_wheel, echo_rx):
    wheel = make_wheel()
    echo = _step_one(wheel, echo_rx)
    assert echo.cmd_seq_echo is None
    assert echo.vel_rad_s is None


def test_no_cmd_holds_zero_velocity(make_wheel, echo_rx):
    wheel = make_wheel()
    _step_one(wheel, echo_rx)
    for _ in range(10):
        echo = _step_one(wheel, echo_rx)
        assert echo.vel_rad_s == pytest.approx(0.0, abs=1e-12)


def test_cmd_1_rad_s_reaches_about_1_rad_s(make_wheel, echo_rx, tx):
    wheel = make_wheel()
    echoes = _run_steps_with_cmd(wheel, tx, echo_rx, 62, 1, 1.0)
    echo = echoes[-1]
    assert echo.cmd_seq_echo == 1
    assert echo.vel_rad_s == pytest.approx(1.0, abs=TOL)


def test_sign_minus_one_gives_about_minus_one(make_wheel, echo_rx, tx):
    wheel = make_wheel(sign=-1)
    echoes = _run_steps_with_cmd(wheel, tx, echo_rx, 62, 1, 1.0)
    assert echoes[-1].vel_rad_s == pytest.approx(-1.0, abs=TOL)


def test_command_above_clamp_is_limited_to_3(make_wheel, echo_rx, tx):
    wheel = make_wheel()
    raw = json.dumps({"cmd_seq": 1, "wheel_rad_s": 5.0, "stale": 0}).encode("utf-8")
    vels = []
    for _ in range(120):
        # Raw bytes, not encode_cmd: encode_cmd clamps on the sender side and
        # would skip the decoder clamp this test is about.
        _send(tx, wheel, raw)
        vels.append(_step_one(wheel, echo_rx).vel_rad_s)
    numeric = [v for v in vels if v is not None]
    assert max(numeric) <= 3.0 + 1e-9
    assert numeric[-1] == pytest.approx(3.0, abs=TOL)


def test_single_cmd_goes_stale_after_0_2_s_sim(make_wheel, echo_rx, tx):
    wheel = make_wheel()
    _send(tx, wheel, encode_cmd(1, 1.0, 0))
    # 15 steps = 0.24 s sim. Stale age 0.2 s is reached on step 14.
    vels = [_step_one(wheel, echo_rx).vel_rad_s for _ in range(15)]
    assert vels[13] < vels[12]
    assert vels[14] < vels[13]
    tail = [_step_one(wheel, echo_rx).vel_rad_s for _ in range(66)]
    assert tail[-1] == pytest.approx(0.0, abs=0.05)


def test_stale_flag_forces_zero_target_immediately(make_wheel, echo_rx, tx):
    wheel = make_wheel()
    _send(tx, wheel, encode_cmd(1, 1.0, 0))
    before = None
    for _ in range(62):
        before = _step_one(wheel, echo_rx).vel_rad_s
    _send(tx, wheel, encode_cmd(2, 1.0, 1))
    after = _step_one(wheel, echo_rx).vel_rad_s
    assert after == pytest.approx(before * (1.0 - LAG), abs=0.01)


def test_invalid_datagrams_are_ignored(make_wheel, echo_rx, tx):
    wheel = make_wheel()
    _send(tx, wheel, b"\xff\xfe not a datagram")
    _send(tx, wheel, b'{"cmd_seq":1}')
    first = _step_one(wheel, echo_rx)
    assert first.cmd_seq_echo is None

    valid = encode_cmd(7, 1.0, 0)
    echoes = []
    for i in range(62):
        if i == 30:
            _send(tx, wheel, b"\xff\xfe garbage")
        _send(tx, wheel, valid)
        wheel.step()
        echoes.extend(_echoes(echo_rx))
    assert len(echoes) == 62
    assert all(e.cmd_seq_echo == 7 for e in echoes)
    assert echoes[-1].vel_rad_s == pytest.approx(1.0, abs=TOL)


def test_echo_carries_latest_accepted_cmd_seq(make_wheel, echo_rx, tx):
    wheel = make_wheel()
    _send(tx, wheel, encode_cmd(3, 0.0, 0))
    assert _step_one(wheel, echo_rx).cmd_seq_echo == 3
    _send(tx, wheel, encode_cmd(4, 0.0, 0))
    _send(tx, wheel, encode_cmd(5, 0.0, 0))
    assert _step_one(wheel, echo_rx).cmd_seq_echo == 5


def test_sim_time_advances_by_dt_per_step(make_wheel, echo_rx):
    wheel = make_wheel()
    times = [_step_one(wheel, echo_rx).sim_t_s for _ in range(5)]
    for earlier, later in zip(times, times[1:]):
        assert later - earlier == pytest.approx(DT_S, abs=1e-9)


def test_close_is_idempotent(echo_rx):
    wheel = FakeWheel(echo_rx.getsockname())
    wheel.close()
    wheel.close()


def test_run_background_starts_and_stops_within_1s(make_wheel):
    wheel = make_wheel()
    stop = threading.Event()
    thread = wheel.run_background(stop)
    assert isinstance(thread, threading.Thread)
    assert thread.is_alive()
    time.sleep(0.05)
    stop.set()
    thread.join(timeout=1.0)
    assert not thread.is_alive()
