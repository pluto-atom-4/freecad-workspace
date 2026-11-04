#!/usr/bin/env python3
"""
Unit tests for fake_wheels.FakeWheel, the headless twin stand-in for the Webots
ctrl_wheels controller (freecad-workspace FW#495, task F11, umbrella FW#482).

TEST FIRST, RED UNTIL F12. Task F12 (FW#496) rewrites fake_wheels.py so that it
matches the API and the behaviour pinned here. F11 and F12 are one PR. Until F12
lands, these tests are expected to fail: the module still holds the single-wheel
FakeWheel (one wheel, a sign argument, the old reserved-port range).

Pinned API (F12 implements exactly this):
    FakeWheel(echo_addr, cmd_port=0, tau_s=0.05, dt_s=0.016, stale_s=0.2,
              clamp=3.0, host="127.0.0.1")
    .cmd_addr, .step(), .close(), .run_background(stop_event) -> Thread

Contract: poc/ctrl-wheels/CONTRACT.md S3 (UDP v2 command and echo shapes) and
S4 (the reserved port set is the literal set {5005, 5006, 5007, 5008, 5017, 5018},
never a range). Tests bind only ephemeral ports (0) on the loopback address.

SIM ONLY. The sign convention is UNCONFIRMED (CONTRACT.md S2). The fake applies
each wheel's command as given, so any sign is the bridge's job, not the fake's.
The model values (tau, dt, clamp) are guesses. Nothing here validates a real
wheel or a balance loop.

Usage (run from poc/ctrl-wheels):
    python3 -m pytest test_ctrl_wheels_fake.py -v
"""

from __future__ import annotations

import json
import math
import socket
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ctrl_wheels_udp import (  # noqa: E402
    WHEEL_CLAMP_RAD_S,
    decode_echo,
    drain_all,
    encode_cmd,
)
from fake_wheels import FakeWheel  # noqa: E402


# CONTRACT.md S4, written out as a literal set on purpose. Do not derive it from a range.
RESERVED_PORTS = frozenset({5005, 5006, 5007, 5008, 5017, 5018})
DT_S = 0.016
TAU_S = 0.05
# First-order lag factor, written here from the documented model: dt / (tau + dt).
LAG = DT_S / (TAU_S + DT_S)
# Stale age of CONTRACT.md S5, in sim seconds.
STALE_AGE_S = 0.200
TOL = 0.1


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


def _run(
    wheel: FakeWheel,
    tx: socket.socket,
    echo_rx: socket.socket,
    n: int,
    cmd_seq: int,
    left_rad_s: float,
    right_rad_s: float,
    stale: int = 0,
) -> list:
    """Model the real bridge: re-send the same v2 command before every step.

    Returns the decoded echoes, one per step.
    """
    echoes = []
    for _ in range(n):
        _send(tx, wheel, encode_cmd(cmd_seq, left_rad_s, right_rad_s, stale))
        wheel.step()
        echoes.extend(_echoes(echo_rx))
    assert len(echoes) == n, f"expected {n} echoes, got {len(echoes)}"
    return echoes


def _expected_path(targets):
    """Per-step velocity and angle for a list of per-step targets.

    Written from the documented model, not from fake_wheels.py: velocity starts at
    0 and angle at 0; each step v += (target - v) * LAG, then angle += v * DT_S.
    """
    vel = 0.0
    angle = 0.0
    vels = []
    angles = []
    for target in targets:
        vel = vel + (target - vel) * LAG
        angle = angle + vel * DT_S
        vels.append(vel)
        angles.append(angle)
    return vels, angles


def _leg(echo, side: str):
    """Return (angle_rad, vel_rad_s) of one wheel of an echo."""
    if side == "left":
        return echo.angle_rad_l, echo.vel_rad_s_l
    return echo.angle_rad_r, echo.vel_rad_s_r


def _assert_leg_path(echoes, side: str, targets) -> None:
    """Check one wheel of every echo against the first-order path.

    The velocity on the first step is None (no previous angle, CONTRACT.md S3
    note and S6), so index 0 checks for None and the rest compare numbers.
    """
    assert len(echoes) == len(targets), f"expected {len(targets)} echoes"
    vels, angles = _expected_path(targets)
    for k, echo in enumerate(echoes):
        angle, vel = _leg(echo, side)
        assert angle == pytest.approx(angles[k], abs=1e-9), f"{side} angle, step {k + 1}"
        if k == 0:
            assert vel is None, f"{side} velocity must be None on the first step"
        else:
            assert vel == pytest.approx(vels[k], abs=1e-9), f"{side} velocity, step {k + 1}"


def test_cmd_addr_is_ephemeral_and_not_reserved(make_wheel):
    wheel = make_wheel()
    host, port = wheel.cmd_addr
    assert host == "127.0.0.1"
    assert port != 0
    assert port not in RESERVED_PORTS


def test_first_step_without_command_has_zero_angles_and_no_seq_or_velocity(
    make_wheel, echo_rx
):
    wheel = make_wheel()
    echo = _step_one(wheel, echo_rx)
    assert echo.cmd_seq_echo is None
    assert echo.angle_rad_l == 0.0
    assert echo.angle_rad_r == 0.0
    assert echo.vel_rad_s_l is None
    assert echo.vel_rad_s_r is None


def test_no_command_holds_both_wheels_at_zero(make_wheel, echo_rx):
    wheel = make_wheel()
    _step_one(wheel, echo_rx)
    for _ in range(10):
        echo = _step_one(wheel, echo_rx)
        assert echo.vel_rad_s_l == pytest.approx(0.0, abs=1e-12)
        assert echo.vel_rad_s_r == pytest.approx(0.0, abs=1e-12)
        assert echo.angle_rad_l == 0.0
        assert echo.angle_rad_r == 0.0


def test_first_step_velocity_is_none_then_finite(make_wheel, echo_rx, tx):
    wheel = make_wheel()
    echoes = _run(wheel, tx, echo_rx, 11, 1, 1.0, -0.5)
    assert echoes[0].cmd_seq_echo == 1
    assert echoes[0].vel_rad_s_l is None
    assert echoes[0].vel_rad_s_r is None
    for echo in echoes[1:]:
        assert isinstance(echo.vel_rad_s_l, float) and math.isfinite(echo.vel_rad_s_l)
        assert isinstance(echo.vel_rad_s_r, float) and math.isfinite(echo.vel_rad_s_r)
        assert math.isfinite(echo.angle_rad_l)
        assert math.isfinite(echo.angle_rad_r)


def test_left_and_right_follow_the_first_order_lag(make_wheel, echo_rx, tx):
    wheel = make_wheel()
    echoes = _run(wheel, tx, echo_rx, 30, 1, 1.0, -0.5)
    _assert_leg_path(echoes, "left", [1.0] * 30)
    _assert_leg_path(echoes, "right", [-0.5] * 30)
    for k, echo in enumerate(echoes):
        assert echo.sim_t_s == pytest.approx((k + 1) * DT_S, abs=1e-9)


def test_left_and_right_commanded_differently_give_different_angles(
    make_wheel, echo_rx, tx
):
    wheel = make_wheel()
    last = _run(wheel, tx, echo_rx, 30, 1, 1.0, -0.5)[-1]
    assert last.angle_rad_l > 0.0
    assert last.angle_rad_r < 0.0
    # Both wheels obey the same linear lag, so the angles scale with the commands.
    assert last.angle_rad_r == pytest.approx(-0.5 * last.angle_rad_l, rel=1e-9)


def test_cmd_1_rad_s_reaches_about_1_rad_s_on_both_wheels(make_wheel, echo_rx, tx):
    wheel = make_wheel()
    echoes = _run(wheel, tx, echo_rx, 62, 1, 1.0, 1.0)
    assert all(e.cmd_seq_echo == 1 for e in echoes)
    assert echoes[-1].vel_rad_s_l == pytest.approx(1.0, abs=TOL)
    assert echoes[-1].vel_rad_s_r == pytest.approx(1.0, abs=TOL)


def test_left_command_does_not_move_right_wheel(make_wheel, echo_rx, tx):
    wheel = make_wheel()
    echoes = _run(wheel, tx, echo_rx, 62, 1, 1.0, 0.0)
    _assert_leg_path(echoes, "left", [1.0] * 62)
    _assert_leg_path(echoes, "right", [0.0] * 62)
    assert all(e.angle_rad_r == 0.0 for e in echoes)


def test_right_command_does_not_move_left_wheel(make_wheel, echo_rx, tx):
    wheel = make_wheel()
    echoes = _run(wheel, tx, echo_rx, 62, 1, 0.0, -1.0)
    _assert_leg_path(echoes, "left", [0.0] * 62)
    _assert_leg_path(echoes, "right", [-1.0] * 62)
    assert all(e.angle_rad_l == 0.0 for e in echoes)


def test_command_above_clamp_is_limited_to_3_per_wheel(make_wheel, echo_rx, tx):
    wheel = make_wheel()
    # Raw bytes, not encode_cmd: encode_cmd clamps on the sender side and would
    # skip the decoder clamp this test is about.
    raw = json.dumps(
        {"v": 2, "cmd_seq": 1, "wheels": {"left": 5.0, "right": -5.0}, "stale": 0}
    ).encode("utf-8")
    echoes = []
    for _ in range(120):
        _send(tx, wheel, raw)
        echoes.append(_step_one(wheel, echo_rx))
    _assert_leg_path(echoes, "left", [3.0] * 120)
    _assert_leg_path(echoes, "right", [-3.0] * 120)


def test_single_cmd_goes_stale_after_0_2_s_sim_on_both_wheels(make_wheel, echo_rx, tx):
    wheel = make_wheel()
    _send(tx, wheel, encode_cmd(1, 1.0, 1.0, 0))
    echoes = [_step_one(wheel, echo_rx) for _ in range(80)]
    # A command is accepted at the start of a step. Step k (1-based) sees an age
    # of (k - 1) * DT_S, so the age first reaches 0.2 s on step 14:
    # 12 * 0.016 = 0.192 (not stale), 13 * 0.016 = 0.208 (stale).
    targets = [1.0] * 13 + [0.0] * 67
    _assert_leg_path(echoes, "left", targets)
    _assert_leg_path(echoes, "right", targets)
    assert all(e.cmd_seq_echo == 1 for e in echoes)


def test_stale_flag_zeroes_both_targets_on_the_next_step(make_wheel, echo_rx, tx):
    wheel = make_wheel()
    before = _run(wheel, tx, echo_rx, 62, 1, 1.0, 1.0)[-1]
    _send(tx, wheel, encode_cmd(2, 1.0, 1.0, 1))
    after = _step_one(wheel, echo_rx)
    assert after.cmd_seq_echo == 2
    assert after.vel_rad_s_l == pytest.approx(before.vel_rad_s_l * (1.0 - LAG), abs=1e-9)
    assert after.vel_rad_s_r == pytest.approx(before.vel_rad_s_r * (1.0 - LAG), abs=1e-9)


def test_stale_zeroes_both_wheels_when_they_were_commanded_differently(
    make_wheel, echo_rx, tx
):
    wheel = make_wheel()
    moving = _run(wheel, tx, echo_rx, 62, 1, 1.0, -1.0)
    stopped = _run(wheel, tx, echo_rx, 62, 2, 1.0, -1.0, stale=1)
    echoes = moving + stopped
    _assert_leg_path(echoes, "left", [1.0] * 62 + [0.0] * 62)
    _assert_leg_path(echoes, "right", [-1.0] * 62 + [0.0] * 62)
    assert all(e.cmd_seq_echo == 2 for e in stopped)
    assert stopped[-1].vel_rad_s_l == pytest.approx(0.0, abs=1e-6)
    assert stopped[-1].vel_rad_s_r == pytest.approx(0.0, abs=1e-6)


# Datagrams the fake must ignore: the same refusals as the module decoder
# (ctrl_wheels_udp.decode_cmd raises UdpMsgError, and the fake skips them).
BAD_DATAGRAMS = [
    b"\xff\xfe not a datagram",
    b'{"cmd_seq":1}',
    # A single-wheel v1 command: no "v", keys cmd_seq, wheel_rad_s, stale.
    json.dumps({"cmd_seq": 1, "wheel_rad_s": 1.0, "stale": 0}).encode("utf-8"),
    b'{"v":2.0,"cmd_seq":1,"wheels":{"left":1.0,"right":1.0},"stale":0}',
    b'{"v":1,"cmd_seq":1,"wheels":{"left":1.0,"right":1.0},"stale":0}',
    b'{"v":2,"cmd_seq":1,"wheels":{"left":1.0,"right":1.0,"up":0.0},"stale":0}',
    b'{"v":2,"cmd_seq":1,"wheels":{"left":NaN,"right":1.0},"stale":0}',
    b'{"v":2,"cmd_seq":1,"wheels":{"left":1.0,"right":1.0},"stale":true}',
]


@pytest.mark.parametrize("bad", BAD_DATAGRAMS)
def test_bad_datagram_is_ignored(make_wheel, echo_rx, tx, bad):
    wheel = make_wheel()
    _send(tx, wheel, bad)
    first = _step_one(wheel, echo_rx)
    assert first.cmd_seq_echo is None
    assert first.angle_rad_l == 0.0
    assert first.angle_rad_r == 0.0
    second = _step_one(wheel, echo_rx)
    assert second.cmd_seq_echo is None
    assert second.vel_rad_s_l == pytest.approx(0.0, abs=1e-12)
    assert second.vel_rad_s_r == pytest.approx(0.0, abs=1e-12)


def test_v1_and_garbage_mid_stream_do_not_change_the_applied_command(
    make_wheel, echo_rx, tx
):
    wheel = make_wheel()
    v1 = json.dumps({"cmd_seq": 9, "wheel_rad_s": -3.0, "stale": 0}).encode("utf-8")
    echoes = []
    for i in range(62):
        if i == 30:
            _send(tx, wheel, v1)
            _send(tx, wheel, b"\xff\xfe garbage")
        _send(tx, wheel, encode_cmd(7, 1.0, 1.0, 0))
        echoes.append(_step_one(wheel, echo_rx))
    assert all(e.cmd_seq_echo == 7 for e in echoes)
    _assert_leg_path(echoes, "left", [1.0] * 62)
    _assert_leg_path(echoes, "right", [1.0] * 62)


def test_echo_carries_latest_accepted_cmd_seq(make_wheel, echo_rx, tx):
    wheel = make_wheel()
    _send(tx, wheel, encode_cmd(3, 0.0, 0.0, 0))
    assert _step_one(wheel, echo_rx).cmd_seq_echo == 3
    _send(tx, wheel, encode_cmd(4, 0.0, 0.0, 0))
    _send(tx, wheel, encode_cmd(5, 0.0, 0.0, 0))
    assert _step_one(wheel, echo_rx).cmd_seq_echo == 5


def test_sim_time_advances_by_dt_per_step(make_wheel, echo_rx):
    wheel = make_wheel()
    times = [_step_one(wheel, echo_rx).sim_t_s for _ in range(5)]
    assert times[0] == pytest.approx(DT_S, abs=1e-9)
    for earlier, later in zip(times, times[1:]):
        assert later - earlier == pytest.approx(DT_S, abs=1e-9)


def test_close_is_idempotent(echo_rx):
    wheel = FakeWheel(echo_rx.getsockname())
    wheel.close()
    wheel.close()


@pytest.mark.parametrize("clamp", [WHEEL_CLAMP_RAD_S + 0.5, 10.0])
def test_clamp_above_wheel_clamp_is_rejected(echo_rx, clamp):
    with pytest.raises(ValueError):
        FakeWheel(echo_rx.getsockname(), clamp=clamp)


@pytest.mark.parametrize("clamp", [0.0, -1.0, float("nan")])
def test_clamp_not_in_zero_to_wheel_clamp_is_rejected(echo_rx, clamp):
    with pytest.raises(ValueError):
        FakeWheel(echo_rx.getsockname(), clamp=clamp)


def test_clamp_at_wheel_clamp_is_accepted(make_wheel):
    make_wheel(clamp=WHEEL_CLAMP_RAD_S)


def test_clamp_limits_both_wheel_targets(make_wheel, echo_rx, tx):
    wheel = make_wheel(clamp=1.0)
    echoes = _run(wheel, tx, echo_rx, 120, 1, 2.0, -2.0)
    _assert_leg_path(echoes, "left", [1.0] * 120)
    _assert_leg_path(echoes, "right", [-1.0] * 120)


def test_run_background_starts_and_stops_within_1s(make_wheel, echo_rx, tx):
    wheel = make_wheel()
    _send(tx, wheel, encode_cmd(1, 1.0, 1.0, 0))
    stop = threading.Event()
    thread = wheel.run_background(stop)
    assert isinstance(thread, threading.Thread)
    assert thread.name == "fake_wheels"
    assert thread.daemon
    assert thread.is_alive()
    time.sleep(0.2)
    echoes = _echoes(echo_rx)
    assert len(echoes) >= 2
    assert any(e.cmd_seq_echo == 1 for e in echoes)
    t0 = time.monotonic()
    stop.set()
    thread.join(timeout=1.0)
    assert not thread.is_alive()
    assert time.monotonic() - t0 < 1.0


def test_stop_event_can_be_set_again_after_the_thread_ends(make_wheel):
    wheel = make_wheel()
    stop = threading.Event()
    thread = wheel.run_background(stop)
    stop.set()
    thread.join(timeout=1.0)
    stop.set()
    thread.join(timeout=1.0)
    assert not thread.is_alive()


@pytest.mark.parametrize(
    "kwargs",
    [
        {"dt_s": 0.0},
        {"dt_s": -0.016},
        {"dt_s": float("nan")},
        {"dt_s": float("inf")},
        {"tau_s": 0.0},
        {"tau_s": -0.016},
        {"tau_s": float("nan")},
        {"tau_s": float("inf")},
    ],
)
def test_bad_dynamics_arguments_rejected_before_binding(kwargs):
    with pytest.raises(ValueError):
        FakeWheel(echo_addr=("127.0.0.1", 40123), **kwargs)


@pytest.mark.parametrize("port", sorted(RESERVED_PORTS))
def test_reserved_command_port_is_refused(port):
    with pytest.raises(ValueError):
        FakeWheel(echo_addr=("127.0.0.1", 40123), cmd_port=port)


@pytest.mark.parametrize("port", sorted(RESERVED_PORTS))
def test_reserved_echo_port_is_refused(port):
    with pytest.raises(ValueError):
        FakeWheel(echo_addr=("127.0.0.1", port))


@pytest.mark.parametrize("port", [5009, 5016, 5019])
def test_ports_next_to_the_reserved_set_are_accepted_as_echo_addr(port):
    # Guards against deriving the reserved set from a range such as 5005..5018.
    # Only the echo address is given here; nothing binds it.
    wheel = FakeWheel(echo_addr=("127.0.0.1", port))
    wheel.close()
