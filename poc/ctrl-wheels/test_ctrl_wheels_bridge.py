#!/usr/bin/env python3
"""
Tests for the twin ctrl-wheels bridge (ctrl_wheels_bridge). The bridge module is
in this directory; these tests pin its API.

The bridge reads WHL2 lines, splits each into a left and a right single-wheel
sample (ctrl_wheels_msg.split), feeds one Relay per wheel, and sends one UDP v2
command per accepted line. It uses the model velocity only, forward-positive on
both wheels; the raw columns of the line are logged but never drive a command.
If either relay is stale, or the state is not 0, both wheels get (0.0, 0.0) with
stale 1 (they stop together). That stop is not a latch: a fresh state-0 sample
resumes. Each wheel is clamped to +-3.0 silently.

Integration tests use real loopback UDP sockets, a real FakeWheel in a thread
and the real clock, each well under 3 s. Every socket uses port 0 (ephemeral).
Ports 5005-5008, 5017 and 5018 are never bound here.

SIM ONLY. Webots-side +1/+1 direction owner-observed (2026-11-06, sim only);
chip-side signs UNCONFIRMED. The model values are UNVERIFIED. Nothing here
validates a real wheel, a balance loop or a serial device. open_serial_lines is
tested only for paths that need no hardware.

Usage (run from poc/ctrl-wheels):
    python3 -m pytest test_ctrl_wheels_bridge.py -v
"""

from __future__ import annotations

import csv
import dataclasses
import inspect
import io
import socket
import sys
import threading
import time
from pathlib import Path

import pytest
import serial

sys.path.insert(0, str(Path(__file__).resolve().parent))

import ctrl_wheels_bridge  # noqa: E402
import ctrl_wheels_relay  # noqa: E402
from ctrl_wheels_bridge import (  # noqa: E402
    BridgeStats,
    DofSerialError,
    _is_url,
    open_serial_lines,
    run_bridge,
)
from ctrl_wheels_log import COLUMNS, CsvLog  # noqa: E402
from ctrl_wheels_msg import Whl, Whl2, format_whl, format_whl2, step_lines  # noqa: E402
from ctrl_wheels_relay import Relay, RelayOut  # noqa: E402
from ctrl_wheels_udp import (  # noqa: E402
    UdpMsgError,
    decode_cmd,
    decode_echo,
    drain_all,
    encode_echo,
    open_listener,
)
from fake_wheels import FakeWheel  # noqa: E402


def _line(seq, model=0.5, state=0, raw_l=10, raw_r=-10, t_us=None):
    """One stripped WHL2 line (no newline), as the bridge's reader passes it.

    t_us defaults to seq * 20000 (no restart, no wrap); pass it explicitly for restart and wrap tests.
    """
    if t_us is None:
        t_us = (seq * 20000) % (2**32)
    return format_whl2(
        Whl2(seq=seq, t_us=t_us, model_rad_s=model, raw_l=raw_l, raw_r=raw_r, state=state)
    ).rstrip("\n")


def _v1_line(seq):
    """One stripped single-wheel WHL v1 line (6 fields), which the twin refuses."""
    return format_whl(
        Whl(seq=seq, t_us=seq * 20000, model_rad_s=0.5, raw=10, state=0)
    ).rstrip("\n")


def _bad_checksum(line):
    """Same body, checksum replaced by a value that does not match."""
    body, _, hh = line.rpartition("*")
    return f"{body}*{'00' if hh != '00' else '01'}"


def _endless(done):
    """Fresh state-0 WHL2 lines at about 50 Hz until done is set (real time)."""
    seq = 0
    while not done.is_set():
        yield _line(seq)
        seq += 1
        done.wait(0.02)


def _blocked(release):
    """Iterator that never yields until release is set (or 5 s pass)."""
    release.wait(timeout=5.0)
    yield from ()


class _StaleRelay(Relay):
    """Relay whose poll() is deterministic: stale after the first sample, whatever the clock."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._seen = False

    def on_whl(self, whl):
        super().on_whl(whl)
        self._seen = True

    def poll(self):
        if not self._seen:
            return None
        return RelayOut(wheel_rad_s=0.0, stale=1)


def _rows(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def _by_seq(rows):
    return {int(r["whl_seq"]): r for r in rows if r["whl_seq"] != ""}


def _wire_cmds(cmd_rx):
    return [decode_cmd(data) for data in drain_all(cmd_rx)]


def _pairs(cmds):
    """(left, right, stale) of each wire command."""
    return [(c.left_rad_s, c.right_rad_s, c.stale) for c in cmds]


def _run(tmp_path, lines, cmd_addr, echo_rx, *, relay_l=None, relay_r=None, **kwargs):
    """Run the bridge into a fresh CSV under tmp_path. Returns (stats, csv path)."""
    path = tmp_path / "bridge.csv"
    if relay_l is None:
        relay_l = Relay()
    if relay_r is None:
        relay_r = Relay()
    log = CsvLog(path)
    try:
        stats = run_bridge(iter(lines), relay_l, relay_r, cmd_addr, echo_rx, log, **kwargs)
    finally:
        log.close()
    return stats, path


@pytest.fixture
def cmd_rx():
    """Test-side command receiver on an ephemeral port, non-blocking."""
    sock = open_listener(0)
    yield sock
    sock.close()


@pytest.fixture
def echo_rx():
    """Bridge-side echo socket on an ephemeral port, non-blocking."""
    sock = open_listener(0)
    yield sock
    sock.close()


@pytest.fixture
def live_wheel(echo_rx):
    """Real twin FakeWheel stepping in a thread and echoing to echo_rx."""
    wheel = FakeWheel(echo_rx.getsockname())
    stop = threading.Event()
    thread = wheel.run_background(stop)
    yield wheel
    stop.set()
    thread.join(timeout=1.0)
    wheel.close()


# ---------------------------------------------------------------------------
# open_serial_lines helpers (no hardware)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("port", ["rfc2217://localhost:4002", "loop://"])
def test_is_url_true_for_scheme_ports(port):
    assert _is_url(port) is True


@pytest.mark.parametrize("port", ["/dev/ttyACM0", "COM3"])
def test_is_url_false_for_device_paths(port):
    assert _is_url(port) is False


def test_missing_device_path_raises_dof_serial_error():
    with pytest.raises(DofSerialError):
        next(open_serial_lines("/dev/definitely-not-a-port"))


def test_unreachable_url_ends_at_max_duration_without_long_wait(monkeypatch):
    """Stub pyserial so the URL wait loop runs; the 0.05 s deadline must end it."""

    def _refuse(url, baud, timeout=None):
        raise serial.SerialException(f"unreachable test stub: {url}")

    monkeypatch.setattr(serial, "serial_for_url", _refuse)
    t0 = time.monotonic()
    lines = list(open_serial_lines("unreachable://", max_duration_s=0.05))
    elapsed = time.monotonic() - t0
    assert lines == []
    assert elapsed < 2.0


# ---------------------------------------------------------------------------
# Pinned API
# ---------------------------------------------------------------------------


def test_run_bridge_signature_pinned():
    params = inspect.signature(run_bridge).parameters
    assert list(params) == [
        "lines",
        "relay_l",
        "relay_r",
        "cmd_addr",
        "echo_sock",
        "log",
        "duration_s",
        "poll_s",
        "clock",
        "sleep",
        "echo_timeout_s",
        "stop_event",
    ]
    for name in ("duration_s", "poll_s", "clock", "sleep", "echo_timeout_s", "stop_event"):
        assert params[name].kind is inspect.Parameter.KEYWORD_ONLY
    assert params["duration_s"].default is None
    assert params["poll_s"].default == 0.02
    assert ctrl_wheels_bridge.POLL_S is ctrl_wheels_relay.POLL_S
    assert params["poll_s"].default is ctrl_wheels_relay.POLL_S
    assert params["clock"].default is time.monotonic
    assert params["sleep"].default is time.sleep
    assert params["echo_timeout_s"].default == 0.5
    assert params["stop_event"].default is None


def test_bridge_stats_fields_pinned():
    names = [f.name for f in dataclasses.fields(BridgeStats)]
    assert names == [
        "whl_lines",
        "bad_lines",
        "cmds_sent",
        "echoes_matched",
        "unmatched_echoes",
        "bad_echoes",
        "missed_whl_total",
        "stale_cmds",
        "restarts",
    ]


# ---------------------------------------------------------------------------
# Integration: real FakeWheel, real loopback sockets, real clock
# ---------------------------------------------------------------------------


def test_fifty_hz_step_logs_rows_with_echoes(tmp_path, echo_rx, live_wheel):
    # step_lines writes raw_l positive and raw_r negative; both wheels must still
    # follow the model forward-positive.
    lines = step_lines(0.5, 1.2, rate_hz=50.0, realtime=True)
    stats, path = _run(tmp_path, lines, live_wheel.cmd_addr, echo_rx)
    rows = _rows(path)

    assert stats.whl_lines == 60
    assert len(rows) >= 55
    assert stats.cmds_sent >= 55
    assert all(r["missed_whl"] == "0" for r in rows)

    echoed = [r for r in rows if r["echo_cmd_seq"] != ""]
    assert len(echoed) >= 0.8 * len(rows)
    for r in echoed:
        assert r["echo_cmd_seq"] == r["cmd_seq"]
        assert float(r["host_rtt_ms"]) >= 0.0

    fresh = [r for r in echoed if r["stale"] == "0"]
    assert float(fresh[-1]["vel_rad_s_l"]) == pytest.approx(0.5, abs=0.1)
    assert float(fresh[-1]["vel_rad_s_r"]) == pytest.approx(0.5, abs=0.1)


def test_echo_match_with_real_fake_wheel_fills_both_wheels(tmp_path, echo_rx, live_wheel):
    stats, path = _run(
        tmp_path, step_lines(0.5, 0.4, rate_hz=50.0, realtime=True), live_wheel.cmd_addr, echo_rx
    )
    echoed = [r for r in _rows(path) if r["echo_cmd_seq"] != ""]
    assert echoed
    assert stats.echoes_matched > 0
    for r in echoed:
        assert r["echo_cmd_seq"] == r["cmd_seq"]
        assert r["angle_rad_l"] != ""
        assert r["angle_rad_r"] != ""
        assert r["sim_t_s"] != ""
        assert float(r["host_rtt_ms"]) >= 0.0
    # The fake wheel lags first-order, so only some rows reach the model value.
    assert any(
        r["vel_rad_s_l"] != "" and r["vel_rad_s_r"] != ""
        and float(r["vel_rad_s_l"]) > 0.3 and float(r["vel_rad_s_r"]) > 0.3
        for r in echoed
    )


# ---------------------------------------------------------------------------
# Command content (deterministic lines, no echoes)
# ---------------------------------------------------------------------------


def test_state_zero_line_sends_model_to_both_wheels_and_logs_raws(tmp_path, cmd_rx, echo_rx):
    _stats, path = _run(
        tmp_path,
        [_line(0, model=0.5, state=0, raw_l=10, raw_r=-10)],
        cmd_rx.getsockname(),
        echo_rx,
        echo_timeout_s=0.05,
    )
    cmds = _wire_cmds(cmd_rx)
    assert _pairs(cmds)[0] == (0.5, 0.5, 0)

    rows = _rows(path)
    assert len(rows) == 1
    row = rows[0]
    assert row["whl_raw_l"] == "10"
    assert row["whl_raw_r"] == "-10"
    assert row["whl_state"] == "0"
    assert row["stale"] == "0"
    assert float(row["whl_model_rad_s"]) == pytest.approx(0.5)
    assert float(row["cmd_left_rad_s"]) == pytest.approx(0.5)
    assert float(row["cmd_right_rad_s"]) == pytest.approx(0.5)


@pytest.mark.parametrize("state", [1, 2])
def test_nonzero_state_stops_both_wheels(tmp_path, cmd_rx, echo_rx, state):
    _stats, path = _run(
        tmp_path,
        [_line(0, model=0.5, state=state)],
        cmd_rx.getsockname(),
        echo_rx,
        echo_timeout_s=0.05,
    )
    assert _pairs(_wire_cmds(cmd_rx)) == [(0.0, 0.0, 1)]

    rows = _rows(path)
    assert len(rows) == 1
    assert rows[0]["stale"] == "1"
    assert rows[0]["whl_state"] == str(state)
    assert float(rows[0]["cmd_left_rad_s"]) == 0.0
    assert float(rows[0]["cmd_right_rad_s"]) == 0.0


@pytest.mark.parametrize("raw_l, raw_r", [(125, -125), (-125, 125), (-7, -7), (0, 0)])
def test_wire_raws_never_change_the_command(tmp_path, cmd_rx, echo_rx, raw_l, raw_r):
    _stats, path = _run(
        tmp_path,
        [_line(0, model=0.5, raw_l=raw_l, raw_r=raw_r)],
        cmd_rx.getsockname(),
        echo_rx,
        echo_timeout_s=0.05,
    )
    assert _pairs(_wire_cmds(cmd_rx))[0] == (0.5, 0.5, 0)
    row = _rows(path)[0]
    assert row["whl_raw_l"] == str(raw_l)
    assert row["whl_raw_r"] == str(raw_r)
    assert float(row["cmd_right_rad_s"]) == pytest.approx(0.5)


def test_relay_signs_apply_per_wheel(tmp_path, cmd_rx, echo_rx):
    _stats, path = _run(
        tmp_path,
        [_line(0, model=0.5)],
        cmd_rx.getsockname(),
        echo_rx,
        relay_l=Relay(sign=-1),
        relay_r=Relay(sign=1),
        echo_timeout_s=0.05,
    )
    assert _pairs(_wire_cmds(cmd_rx))[0] == (-0.5, 0.5, 0)
    row = _rows(path)[0]
    assert float(row["cmd_left_rad_s"]) == pytest.approx(-0.5)
    assert float(row["cmd_right_rad_s"]) == pytest.approx(0.5)


@pytest.mark.parametrize("model, expected", [(5.0, 3.0), (-5.0, -3.0)])
def test_each_wheel_is_clamped_to_three(tmp_path, cmd_rx, echo_rx, model, expected):
    # The relays are given a wide clamp so the bridge itself must limit the command.
    _stats, path = _run(
        tmp_path,
        [_line(0, model=model)],
        cmd_rx.getsockname(),
        echo_rx,
        relay_l=Relay(clamp=10.0),
        relay_r=Relay(clamp=10.0),
        echo_timeout_s=0.05,
    )
    assert _pairs(_wire_cmds(cmd_rx))[0] == (expected, expected, 0)
    row = _rows(path)[0]
    assert float(row["cmd_left_rad_s"]) == pytest.approx(expected)
    assert float(row["cmd_right_rad_s"]) == pytest.approx(expected)
    assert row["stale"] == "0"


@pytest.mark.parametrize("stale_side", ["left", "right"])
def test_either_relay_stale_stops_both_wheels(tmp_path, cmd_rx, echo_rx, stale_side):
    relay_l = _StaleRelay() if stale_side == "left" else Relay()
    relay_r = _StaleRelay() if stale_side == "right" else Relay()
    _stats, path = _run(
        tmp_path,
        [_line(0, model=0.5)],
        cmd_rx.getsockname(),
        echo_rx,
        relay_l=relay_l,
        relay_r=relay_r,
        echo_timeout_s=0.05,
    )
    assert _pairs(_wire_cmds(cmd_rx)) == [(0.0, 0.0, 1)]
    row = _rows(path)[0]
    assert row["stale"] == "1"
    assert float(row["cmd_left_rad_s"]) == 0.0
    assert float(row["cmd_right_rad_s"]) == 0.0


# ---------------------------------------------------------------------------
# Sequence tracking (deterministic lines, no echoes)
# ---------------------------------------------------------------------------


def test_seq_gap_is_counted_as_missed_whl(tmp_path, cmd_rx, echo_rx):
    seqs = [0, 1, 2, 6, 7]
    stats, path = _run(
        tmp_path, [_line(s) for s in seqs], cmd_rx.getsockname(), echo_rx
    )
    by_seq = _by_seq(_rows(path))
    assert {s: int(by_seq[s]["missed_whl"]) for s in seqs} == {0: 0, 1: 0, 2: 0, 6: 3, 7: 0}
    assert stats.missed_whl_total == 3
    assert stats.restarts == 0


def test_seq_wrap_is_not_a_gap(tmp_path, cmd_rx, echo_rx):
    # t_us keeps counting across the seq wrap, as the device does.
    seqs = [65534, 65535, 1]
    stats, path = _run(
        tmp_path, [_line(s, t_us=i * 20000) for i, s in enumerate(seqs)], cmd_rx.getsockname(), echo_rx
    )
    by_seq = _by_seq(_rows(path))
    assert {s: int(by_seq[s]["missed_whl"]) for s in seqs} == {65534: 0, 65535: 0, 1: 1}
    assert stats.missed_whl_total == 1


def test_seq_reorder_is_not_a_gap(tmp_path, cmd_rx, echo_rx):
    # 7 arrives before 6: the late 6 is a reorder (0), and 8 is measured from
    # the highest seq seen (7), so it is not a gap either.
    seqs = [5, 7, 6, 8]
    stats, path = _run(
        tmp_path, [_line(s) for s in seqs], cmd_rx.getsockname(), echo_rx
    )
    by_seq = _by_seq(_rows(path))
    assert {s: int(by_seq[s]["missed_whl"]) for s in seqs} == {5: 0, 6: 0, 7: 1, 8: 0}
    assert stats.missed_whl_total == 1
    assert stats.restarts == 0


def test_seq_pure_repeat_is_not_a_gap(tmp_path, cmd_rx, echo_rx):
    seqs = [5, 5, 6]
    stats, path = _run(
        tmp_path, [_line(s) for s in seqs], cmd_rx.getsockname(), echo_rx
    )
    rows = _rows(path)
    assert [int(r["whl_seq"]) for r in rows] == seqs
    assert all(r["missed_whl"] == "0" for r in rows)
    assert stats.missed_whl_total == 0
    assert stats.restarts == 0


def test_restart_with_high_old_seq_is_not_a_gap(tmp_path, cmd_rx, echo_rx):
    seq_t = [
        (49998, 1_000_000_000),
        (49999, 1_000_020_000),
        (50000, 1_000_040_000),
        (0, 2_900_000),
        (1, 2_920_000),
    ]
    stats, path = _run(
        tmp_path, [_line(s, t_us=t) for s, t in seq_t], cmd_rx.getsockname(), echo_rx
    )
    by_seq = _by_seq(_rows(path))
    assert int(by_seq[0]["missed_whl"]) == 0
    assert int(by_seq[1]["missed_whl"]) == 0
    assert stats.missed_whl_total == 0
    assert stats.restarts == 1


def test_restart_with_low_old_seq_is_detected_and_later_gap_counted(tmp_path, cmd_rx, echo_rx):
    lines = [_line(n, t_us=n * 20000) for n in range(1001)]
    lines += [
        _line(0, t_us=2_900_000),
        _line(1, t_us=2_920_000),
        _line(2, t_us=2_940_000),
        _line(10, t_us=3_100_000),
    ]
    stats, path = _run(tmp_path, lines, cmd_rx.getsockname(), echo_rx, echo_timeout_s=0.05)
    # _by_seq keeps the last row per seq, so seq 10 is the one after the restart.
    by_seq = _by_seq(_rows(path))
    assert int(by_seq[10]["missed_whl"]) == 7
    assert stats.restarts == 1
    assert stats.missed_whl_total == 7


def test_legit_seq_wrap_with_continuing_t_us_is_not_a_restart(tmp_path, cmd_rx, echo_rx):
    # Kept separate from test_seq_wrap_is_not_a_gap: that test uses seqs [65534, 65535, 1]
    # (seq 0 skipped, missed 1), so its inputs differ from this one.
    seqs = [65534, 65535, 0]
    stats, path = _run(
        tmp_path, [_line(s, t_us=i * 20000) for i, s in enumerate(seqs)], cmd_rx.getsockname(), echo_rx
    )
    by_seq = _by_seq(_rows(path))
    assert {s: int(by_seq[s]["missed_whl"]) for s in seqs} == {65534: 0, 65535: 0, 0: 0}
    assert stats.missed_whl_total == 0
    assert stats.restarts == 0


def test_t_us_wrap_with_continuing_seq_is_not_a_restart(tmp_path, cmd_rx, echo_rx):
    # 4294947296 + 20000 == 2**32, so the next t_us is 0 after a wrap.
    stats, path = _run(
        tmp_path,
        [_line(5, t_us=4_294_947_296), _line(6, t_us=0)],
        cmd_rx.getsockname(),
        echo_rx,
    )
    by_seq = _by_seq(_rows(path))
    assert int(by_seq[6]["missed_whl"]) == 0
    assert stats.missed_whl_total == 0
    assert stats.restarts == 0


def test_real_gap_after_restart_is_counted(tmp_path, cmd_rx, echo_rx):
    seq_t = [
        (50000, 1_000_000_000),
        (0, 2_900_000),
        (1, 2_920_000),
        (2, 2_940_000),
        (6, 3_020_000),
    ]
    stats, path = _run(
        tmp_path, [_line(s, t_us=t) for s, t in seq_t], cmd_rx.getsockname(), echo_rx
    )
    by_seq = _by_seq(_rows(path))
    assert int(by_seq[6]["missed_whl"]) == 3
    assert stats.missed_whl_total == 3
    assert stats.restarts == 1


def test_large_back_step_in_t_us_is_a_restart(tmp_path, cmd_rx, echo_rx):
    # 2.4e9 us is a back step of about 40 minutes (past 2**31 us). The rule has no 36-minute threshold.
    seq_t = [(100, 2_400_000_000), (0, 2_900_000), (1, 2_920_000)]
    stats, _path = _run(
        tmp_path, [_line(s, t_us=t) for s, t in seq_t], cmd_rx.getsockname(), echo_rx
    )
    assert stats.restarts == 1
    assert stats.missed_whl_total == 0


def test_restart_is_reported_in_summary_line(tmp_path, cmd_rx, capsys, monkeypatch):
    text = "\n".join([_line(50000, t_us=1_000_000_000), _line(0, t_us=2_900_000)]) + "\n"
    monkeypatch.setattr(sys, "stdin", io.StringIO(text))
    real_open_listener = ctrl_wheels_bridge.open_listener
    # main() binds the echo socket on --echo-port; this wrapper binds port 0 instead, so no fixed port is used.
    monkeypatch.setattr(ctrl_wheels_bridge, "open_listener", lambda _port: real_open_listener(0))
    rc = ctrl_wheels_bridge.main([
        "--stdin",
        "--record", str(tmp_path / "restart.csv"),
        "--cmd-port", str(cmd_rx.getsockname()[1]),
        "--echo-port", "5019",
        "--echo-timeout", "0.05",
    ])
    err = capsys.readouterr().err
    assert rc == 0
    assert "firmware restart detected (seq 50000 -> 0)" in err
    assert "restarts=1" in err


@pytest.mark.parametrize("gap", [0, 5])
def test_forward_bound_is_inclusive(tmp_path, cmd_rx, echo_rx, gap):
    # Forward bound for a gap: (gap + 1) * 40_000 + 1_000_000 us. Equal to the bound is forward.
    bound = (gap + 1) * 40_000 + 1_000_000
    stats, path = _run(
        tmp_path, [_line(0, t_us=0), _line(gap + 1, t_us=bound)], cmd_rx.getsockname(), echo_rx,
        echo_timeout_s=0.05,
    )
    by_seq = _by_seq(_rows(path))
    assert int(by_seq[gap + 1]["missed_whl"]) == gap
    assert stats.missed_whl_total == gap
    assert stats.restarts == 0


@pytest.mark.parametrize("gap", [0, 5])
def test_forward_bound_plus_one_is_a_restart(tmp_path, cmd_rx, echo_rx, gap):
    # One microsecond over the forward bound is not forward, so the line is a restart.
    bound = (gap + 1) * 40_000 + 1_000_000 + 1
    stats, _path = _run(
        tmp_path, [_line(0, t_us=0), _line(gap + 1, t_us=bound)], cmd_rx.getsockname(), echo_rx,
        echo_timeout_s=0.05,
    )
    assert stats.restarts == 1
    assert stats.missed_whl_total == 0


def test_forward_gap_32767_is_counted_at_the_bound(tmp_path, cmd_rx, echo_rx):
    # Largest forward gap (32767 < half of 65536), with the t_us step at the bound 32768 * 40_000 + 1_000_000.
    bound = 32768 * 40_000 + 1_000_000
    stats, path = _run(
        tmp_path, [_line(0, t_us=0), _line(32768, t_us=bound)], cmd_rx.getsockname(), echo_rx,
        echo_timeout_s=0.05,
    )
    by_seq = _by_seq(_rows(path))
    assert int(by_seq[32768]["missed_whl"]) == 32767
    assert stats.missed_whl_total == 32767
    assert stats.restarts == 0


def test_gap_32768_is_not_a_forward_gap(tmp_path, cmd_rx, echo_rx):
    # Gap 32768 is not below half, so the forward branch is closed. t_us goes forward, so the
    # back step is about 2**32 and the line is a restart.
    stats, path = _run(
        tmp_path, [_line(0, t_us=0), _line(32769, t_us=32769 * 20_000)], cmd_rx.getsockname(), echo_rx,
        echo_timeout_s=0.05,
    )
    by_seq = _by_seq(_rows(path))
    assert int(by_seq[32769]["missed_whl"]) == 0
    assert stats.missed_whl_total == 0
    assert stats.restarts == 1


@pytest.mark.parametrize("back_us, restarts", [(999_999, 0), (1_000_000, 1)])
def test_back_step_threshold_is_one_second(tmp_path, cmd_rx, echo_rx, back_us, restarts):
    # A seq going back is a reorder while t_us steps back by less than 1 s (999_999 us), and a restart at 1_000_000 us.
    stats, _path = _run(
        tmp_path, [_line(100, t_us=2_000_000), _line(99, t_us=2_000_000 - back_us)],
        cmd_rx.getsockname(), echo_rx, echo_timeout_s=0.05,
    )
    assert stats.restarts == restarts
    assert stats.missed_whl_total == 0


def test_seq_back_with_t_us_forward_is_a_restart(tmp_path, cmd_rx, echo_rx):
    # Known limit (i) in the bridge docstring: a seq going back while t_us goes forward is a restart.
    stats, path = _run(
        tmp_path, [_line(100, t_us=2_000_000), _line(99, t_us=2_020_000)], cmd_rx.getsockname(), echo_rx,
        echo_timeout_s=0.05,
    )
    by_seq = _by_seq(_rows(path))
    assert int(by_seq[99]["missed_whl"]) == 0
    assert stats.missed_whl_total == 0
    assert stats.restarts == 1


def test_restart_within_1s_below_old_t_us_is_read_as_reorder_until_t_passes(tmp_path, cmd_rx, echo_rx):
    # Pins known limits (ii) and (iii): a restart to a seq 0..32767 behind the baseline whose t_us is
    # within 1 s BELOW the old one is read as a reorder (no restart yet); the next line, whose t_us is
    # ABOVE the old baseline, is then read as the restart. Both lines count missed 0.
    stats, path = _run(
        tmp_path,
        [_line(100, t_us=2_000_000), _line(0, t_us=1_990_000), _line(1, t_us=2_010_000)],
        cmd_rx.getsockname(), echo_rx, echo_timeout_s=0.05,
    )
    by_seq = _by_seq(_rows(path))
    assert int(by_seq[0]["missed_whl"]) == 0
    assert int(by_seq[1]["missed_whl"]) == 0
    assert stats.missed_whl_total == 0
    assert stats.restarts == 1


def test_reorder_window_upper_edge_32767_behind_is_a_reorder(tmp_path, cmd_rx, echo_rx):
    # Pins the upper edge of known limit (ii): a seq 32767 behind the baseline (gap 32768) with t_us
    # 10 ms below is read as a reorder, so no restart.
    stats, path = _run(
        tmp_path,
        [_line(32768, t_us=2_000_000), _line(1, t_us=1_990_000)],
        cmd_rx.getsockname(), echo_rx, echo_timeout_s=0.05,
    )
    assert stats.missed_whl_total == 0
    assert stats.restarts == 0


def test_seq_32768_behind_is_a_restart_not_a_reorder(tmp_path, cmd_rx, echo_rx):
    # Pins the edge just outside known limit (ii): a seq 32768 behind the baseline (gap 32767) is not
    # in the reorder window; the t_us step fails the forward bound, so it is a restart at once.
    stats, path = _run(
        tmp_path,
        [_line(32768, t_us=2_000_000), _line(0, t_us=1_990_000)],
        cmd_rx.getsockname(), echo_rx, echo_timeout_s=0.05,
    )
    assert stats.missed_whl_total == 0
    assert stats.restarts == 1


def test_restart_read_as_reorder_is_noticed_when_new_seq_passes_old_baseline(tmp_path, cmd_rx, echo_rx):
    # Pins limit (iii), second route: baseline seq 100, restart line seq 0 (read as reorder), then
    # seq 101 with t_us still BELOW the old baseline: forward gap 0 with a backward t_us fails the
    # forward bound, so it is a restart. Missed stays 0.
    stats, path = _run(
        tmp_path,
        [_line(100, t_us=2_000_000), _line(0, t_us=1_900_000), _line(101, t_us=1_950_000)],
        cmd_rx.getsockname(), echo_rx, echo_timeout_s=0.05,
    )
    assert stats.missed_whl_total == 0
    assert stats.restarts == 1


def test_restart_from_high_old_seq_with_t_us_slightly_below_is_still_a_restart(tmp_path, cmd_rx, echo_rx):
    # Known limit (ii): the reorder reading needs the new seq to be 0..32767 behind the baseline. From
    # a high old seq (50000 -> 0 looks like a forward wrap, gap 15535) the t_us step fails the forward
    # bound, so it is a restart at once even though t_us is only 0.5 s below.
    stats, path = _run(
        tmp_path,
        [_line(50000, t_us=1_000_000_000), _line(0, t_us=999_500_000)],
        cmd_rx.getsockname(), echo_rx, echo_timeout_s=0.05,
    )
    by_seq = _by_seq(_rows(path))
    assert int(by_seq[0]["missed_whl"]) == 0
    assert stats.missed_whl_total == 0
    assert stats.restarts == 1


def test_late_frame_after_forward_jump_is_a_known_false_restart(tmp_path, cmd_rx, echo_rx):
    # Pins a KNOWN LIMIT (bridge docstring, known limit iv), not a goal. A late frame more than 1 s
    # behind the baseline, after a forward jump, is read as a restart; the next live frame then
    # counts a false missed 61. If the rule changes, this test is expected to change with it.
    lines = [_line(n, t_us=n * 20000) for n in range(101)]
    lines += [
        _line(161, t_us=3_200_000),
        _line(100, t_us=2_000_000),
        _line(162, t_us=3_220_000),
    ]
    stats, path = _run(tmp_path, lines, cmd_rx.getsockname(), echo_rx, echo_timeout_s=0.05)
    by_seq = _by_seq(_rows(path))
    assert int(by_seq[162]["missed_whl"]) == 61
    assert stats.restarts == 1
    assert stats.missed_whl_total == 60 + 61


def test_bad_lines_counted_and_skipped(tmp_path, cmd_rx, echo_rx):
    good2 = _line(2)
    lines = [_line(0), "garbage", _line(1), _bad_checksum(good2), good2]
    stats, path = _run(tmp_path, lines, cmd_rx.getsockname(), echo_rx)
    rows = _rows(path)

    assert stats.bad_lines == 2
    assert stats.whl_lines == 3
    assert sorted(int(r["whl_seq"]) for r in rows) == [0, 1, 2]
    assert all(r["missed_whl"] == "0" for r in rows)


def test_v1_whl_line_is_a_bad_line_and_sends_no_motion_command(tmp_path, cmd_rx, echo_rx):
    stats, path = _run(tmp_path, [_v1_line(0)], cmd_rx.getsockname(), echo_rx)
    assert stats.bad_lines == 1
    assert stats.whl_lines == 0
    assert _rows(path) == []
    # Only the closing stop reaches the wire.
    assert _pairs(_wire_cmds(cmd_rx)) == [(0.0, 0.0, 1)]


# ---------------------------------------------------------------------------
# Stale and state handling
# ---------------------------------------------------------------------------


def test_state_one_line_sends_zero_stale_command(tmp_path, cmd_rx, echo_rx):
    lines = [_line(0, model=0.5, state=0), _line(1, model=0.5, state=1)]
    _run(tmp_path, lines, cmd_rx.getsockname(), echo_rx)
    cmds = _wire_cmds(cmd_rx)
    assert [(c.cmd_seq, c.left_rad_s, c.right_rad_s, c.stale) for c in cmds] == [
        (0, 0.5, 0.5, 0),
        (1, 0.0, 0.0, 1),
    ]


def test_state_one_row_shows_stale_and_zero_command(tmp_path, cmd_rx, echo_rx):
    lines = [_line(0, model=0.5, state=0), _line(1, model=0.5, state=1)]
    _stats, path = _run(tmp_path, lines, cmd_rx.getsockname(), echo_rx)
    by_seq = _by_seq(_rows(path))
    assert by_seq[1]["whl_state"] == "1"
    assert by_seq[1]["stale"] == "1"
    assert float(by_seq[1]["cmd_left_rad_s"]) == 0.0
    assert float(by_seq[1]["cmd_right_rad_s"]) == 0.0


def test_fresh_state_zero_line_resumes_after_a_state_one_stop(tmp_path, cmd_rx, echo_rx):
    lines = [_line(0, state=0), _line(1, state=1), _line(2, state=0)]
    _run(tmp_path, lines, cmd_rx.getsockname(), echo_rx)
    assert _pairs(_wire_cmds(cmd_rx))[:3] == [(0.5, 0.5, 0), (0.0, 0.0, 1), (0.5, 0.5, 0)]


def test_silence_gives_one_stale_command_then_fresh_resumes(tmp_path, echo_rx):
    gate = threading.Event()
    resumed_at: list[float] = []

    def _gapped():
        yield _line(0, model=0.5)
        gate.wait(timeout=5.0)
        resumed_at.append(time.monotonic())
        yield _line(1, model=0.5)

    # Echo tap: FakeWheel sends its echoes here; the tap records each echoed
    # (vel_rad_s_l, vel_rad_s_r) (public decode_echo) and forwards the bytes
    # unchanged to the bridge's echo socket. The CSV keeps only one echo per
    # command, so it cannot show the decay during the silence.
    tap = open_listener(0)
    tap_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    samples: list[tuple[float, float, float]] = []
    tap_done = threading.Event()

    def _tap():
        while not tap_done.is_set():
            for data in drain_all(tap):
                tap_tx.sendto(data, echo_rx.getsockname())
                try:
                    echo = decode_echo(data)
                except (UdpMsgError, RecursionError):
                    continue
                if echo.vel_rad_s_l is not None and echo.vel_rad_s_r is not None:
                    samples.append((time.monotonic(), echo.vel_rad_s_l, echo.vel_rad_s_r))
            tap_done.wait(0.005)

    wheel = FakeWheel(tap.getsockname())
    wheel_stop = threading.Event()
    wheel_thread = wheel.run_background(wheel_stop)
    tap_thread = threading.Thread(target=_tap, daemon=True)
    tap_thread.start()
    gap_timer = threading.Timer(0.5, gate.set)
    gap_timer.start()
    try:
        stats, path = _run(tmp_path, _gapped(), wheel.cmd_addr, echo_rx)
    finally:
        gap_timer.cancel()
        gate.set()
        wheel_stop.set()
        wheel_thread.join(timeout=1.0)
        wheel.close()
        tap_done.set()
        tap_thread.join(timeout=1.0)
        tap_tx.close()
        tap.close()

    assert len(resumed_at) == 1
    rows = _rows(path)
    stale_rows = [r for r in rows if r["whl_seq"] == ""]
    assert len(stale_rows) == 1
    assert stale_rows[0]["stale"] == "1"
    assert float(stale_rows[0]["cmd_left_rad_s"]) == 0.0
    assert float(stale_rows[0]["cmd_right_rad_s"]) == 0.0

    by_seq = _by_seq(rows)
    assert by_seq[1]["stale"] == "0"
    assert float(by_seq[1]["cmd_left_rad_s"]) == pytest.approx(0.5)
    assert float(by_seq[1]["cmd_right_rad_s"]) == pytest.approx(0.5)
    assert int(stale_rows[0]["cmd_seq"]) == int(by_seq[0]["cmd_seq"]) + 1
    assert int(by_seq[1]["cmd_seq"]) == int(stale_rows[0]["cmd_seq"]) + 1
    assert stats.stale_cmds >= 1

    assert max(vl for _, vl, _ in samples) > 0.3
    assert max(vr for _, _, vr in samples) > 0.3
    before_resume = [(vl, vr) for t, vl, vr in samples if t < resumed_at[0]]
    assert before_resume
    assert before_resume[-1][0] < 0.1
    assert before_resume[-1][1] < 0.1


# ---------------------------------------------------------------------------
# Stop conditions
# ---------------------------------------------------------------------------


def test_duration_stops_loop_and_sends_final_stale(tmp_path, cmd_rx, echo_rx):
    done = threading.Event()
    t0 = time.monotonic()
    try:
        stats, _path = _run(
            tmp_path, _endless(done), cmd_rx.getsockname(), echo_rx, duration_s=0.4
        )
    finally:
        done.set()
    elapsed = time.monotonic() - t0

    assert 0.35 <= elapsed < 2.0
    cmds = _wire_cmds(cmd_rx)
    assert cmds[-1].stale == 1
    assert cmds[-1].left_rad_s == 0.0
    assert cmds[-1].right_rad_s == 0.0
    assert stats.cmds_sent == len(cmds)


def test_stop_event_ends_run_and_sends_final_stale(tmp_path, cmd_rx, echo_rx):
    done = threading.Event()
    stop = threading.Event()
    timer = threading.Timer(0.3, stop.set)
    timer.start()
    t0 = time.monotonic()
    try:
        stats, _path = _run(
            tmp_path, _endless(done), cmd_rx.getsockname(), echo_rx, stop_event=stop
        )
    finally:
        timer.cancel()
        done.set()
    elapsed = time.monotonic() - t0

    assert elapsed < 2.0
    cmds = _wire_cmds(cmd_rx)
    assert cmds[-1].stale == 1
    assert cmds[-1].left_rad_s == 0.0
    assert cmds[-1].right_rad_s == 0.0
    assert stats.cmds_sent == len(cmds)


def test_final_stop_has_no_csv_row(tmp_path, cmd_rx, echo_rx):
    stats, path = _run(
        tmp_path, [_line(0, state=0)], cmd_rx.getsockname(), echo_rx, echo_timeout_s=0.05
    )
    assert len(_rows(path)) == 1
    assert stats.cmds_sent == 2
    assert _pairs(_wire_cmds(cmd_rx)) == [(0.5, 0.5, 0), (0.0, 0.0, 1)]


def test_failed_send_has_no_row_and_final_stop_is_still_attempted(
    tmp_path, cmd_rx, echo_rx, monkeypatch
):
    class _FlakyTx:
        """Wraps the real command socket; the first sendto raises OSError."""

        def __init__(self, real):
            self._real = real
            self.calls = 0

        def sendto(self, data, addr):
            self.calls += 1
            if self.calls == 1:
                raise OSError("injected send failure")
            return self._real.sendto(data, addr)

        def close(self):
            self._real.close()

    real_run = ctrl_wheels_bridge._Run

    class _FlakyRun(real_run):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.cmd_tx = _FlakyTx(self.cmd_tx)

    monkeypatch.setattr(ctrl_wheels_bridge, "_Run", _FlakyRun)
    stats, path = _run(
        tmp_path, [_line(0, state=0)], cmd_rx.getsockname(), echo_rx, echo_timeout_s=0.05
    )
    assert stats.send_errors == 1
    assert _rows(path) == []
    assert stats.cmds_sent == 1
    assert _pairs(_wire_cmds(cmd_rx)) == [(0.0, 0.0, 1)]


def test_blocked_iterator_does_not_hang_bridge(tmp_path, cmd_rx, echo_rx):
    release = threading.Event()
    t0 = time.monotonic()
    try:
        stats, path = _run(
            tmp_path, _blocked(release), cmd_rx.getsockname(), echo_rx, duration_s=0.3
        )
    finally:
        release.set()
    assert time.monotonic() - t0 < 2.0
    assert stats.whl_lines == 0
    assert _rows(path) == []


def test_source_error_after_run_ended_is_not_logged_or_recorded(tmp_path, cmd_rx, echo_rx, capsys):
    release = threading.Event()
    about_to_raise = threading.Event()

    def _late_failure():
        yield _line(0)
        release.wait(timeout=5.0)
        about_to_raise.set()
        raise RuntimeError("late failure")

    try:
        stats, _path = _run(
            tmp_path, _late_failure(), cmd_rx.getsockname(), echo_rx, duration_s=0.3
        )
        readers = [t for t in threading.enumerate() if t.name == "ctrl_wheels_lines"]
        release.set()
        assert about_to_raise.wait(timeout=2.0)
        for t in readers:
            t.join(timeout=2.0)
        assert not any(t.is_alive() for t in readers)
    finally:
        release.set()
    err = capsys.readouterr().err
    assert stats.source_error is None
    assert "line source stopped" not in err


def test_source_error_before_the_run_ended_is_still_recorded(tmp_path, cmd_rx, echo_rx, capsys):
    def _early_failure():
        yield _line(0)
        raise RuntimeError("early failure")

    stats, _path = _run(
        tmp_path, _early_failure(), cmd_rx.getsockname(), echo_rx, duration_s=5.0
    )
    err = capsys.readouterr().err
    assert stats.source_error is not None
    assert "early failure" in stats.source_error
    assert "line source stopped: early failure" in err


# ---------------------------------------------------------------------------
# Echo handling
# ---------------------------------------------------------------------------


def test_bad_and_unmatched_echoes_counted_and_rows_flush_empty(tmp_path, cmd_rx, echo_rx):
    addr = echo_rx.getsockname()
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        tx.sendto(b"\xff\xfe junk", addr)
        tx.sendto(encode_echo(999, 0.1, 0.0, 0.0, 0.0, 0.0), addr)
        tx.sendto(encode_echo(None, 0.1, 0.0, None, 0.0, None), addr)
    finally:
        tx.close()

    def _linger():
        # Keep the run alive long enough for the bridge to drain echo_rx.
        yield _line(0)
        time.sleep(0.1)

    stats, path = _run(
        tmp_path, _linger(), cmd_rx.getsockname(), echo_rx, echo_timeout_s=0.05
    )
    assert stats.bad_echoes == 1
    assert stats.unmatched_echoes == 2
    assert stats.echoes_matched == 0

    rows = _rows(path)
    assert len(rows) == 1
    assert rows[0]["echo_cmd_seq"] == ""
    assert rows[0]["sim_t_s"] == ""
    assert rows[0]["angle_rad_l"] == ""
    assert rows[0]["angle_rad_r"] == ""
    assert rows[0]["vel_rad_s_l"] == ""
    assert rows[0]["vel_rad_s_r"] == ""
    assert rows[0]["host_rtt_ms"] == ""


# ---------------------------------------------------------------------------
# CSV contract
# ---------------------------------------------------------------------------


def test_csv_header_is_ctrl_wheels_log_columns(tmp_path, cmd_rx, echo_rx):
    stats, path = _run(tmp_path, iter([]), cmd_rx.getsockname(), echo_rx)
    with open(path, newline="", encoding="utf-8") as fh:
        header = next(csv.reader(fh))
    assert header == list(COLUMNS)
    assert len(header) == 19
    assert _rows(path) == []
    assert stats.whl_lines == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
