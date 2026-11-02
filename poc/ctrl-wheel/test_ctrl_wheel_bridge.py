#!/usr/bin/env python3
"""
Tests for the ctrl-wheel bridge (ctrl_wheel_bridge), issue FW #453, part 1
(tests first). The module is written in the next run; these tests pin its API.

Integration tests use real loopback UDP sockets, a real FakeWheel in a thread
and the real clock, each well under 3 s. Every socket uses port 0 (ephemeral).
Ports 5005-5008 are never used.

SIM ONLY. The sign convention and the model values are UNVERIFIED. Nothing here
validates a real wheel, a balance loop or a serial device. open_serial_lines is
tested only for paths that need no hardware.

Usage (run from poc/ctrl-wheel):
    python3 -m pytest test_ctrl_wheel_bridge.py -v
"""

from __future__ import annotations

import csv
import dataclasses
import inspect
import socket
import sys
import threading
import time
from pathlib import Path

import pytest
import serial

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ctrl_wheel_bridge import (  # noqa: E402
    BridgeStats,
    DofSerialError,
    _is_url,
    open_serial_lines,
    run_bridge,
)
from ctrl_wheel_log import COLUMNS, CsvLog  # noqa: E402
from ctrl_wheel_msg import Whl, format_whl, step_lines  # noqa: E402
from ctrl_wheel_relay import Relay  # noqa: E402
from ctrl_wheel_udp import decode_cmd, drain_all, encode_echo, open_listener  # noqa: E402
from fake_wheel import FakeWheel  # noqa: E402


def _line(seq, model=0.5, state=0):
    """One stripped WHL line (no newline), as the bridge's reader passes it."""
    t_us = (seq * 20000) % (2**32)
    return format_whl(Whl(seq=seq, t_us=t_us, model_rad_s=model, raw=0, state=state)).rstrip("\n")


def _bad_checksum(line):
    """Same body, checksum replaced by a value that does not match."""
    body, _, hh = line.rpartition("*")
    return f"{body}*{'00' if hh != '00' else '01'}"


def _endless(done):
    """Fresh state-0 WHL lines at about 50 Hz until done is set (real time)."""
    seq = 0
    while not done.is_set():
        yield _line(seq)
        seq += 1
        done.wait(0.02)


def _blocked(release):
    """Iterator that never yields until release is set (or 5 s pass)."""
    release.wait(timeout=5.0)
    yield from ()


def _rows(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def _by_seq(rows):
    return {int(r["whl_seq"]): r for r in rows if r["whl_seq"] != ""}


def _wire_cmds(cmd_rx):
    return [decode_cmd(data) for data in drain_all(cmd_rx)]


def _run(tmp_path, lines, cmd_addr, echo_rx, *, relay=None, **kwargs):
    """Run the bridge into a fresh CSV under tmp_path. Returns (stats, csv path)."""
    path = tmp_path / "bridge.csv"
    if relay is None:
        relay = Relay()
    log = CsvLog(path)
    try:
        stats = run_bridge(iter(lines), relay, cmd_addr, echo_rx, log, **kwargs)
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
    """Real FakeWheel stepping in a thread and echoing to echo_rx."""
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


@pytest.mark.parametrize("port", ["rfc2217://localhost:4000", "loop://"])
def test_is_url_true_for_scheme_ports(port):
    assert _is_url(port) is True


@pytest.mark.parametrize("port", ["/dev/ttyACM0", "COM3", ""])
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
        "relay",
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
    ]


# ---------------------------------------------------------------------------
# Integration: real FakeWheel, real loopback sockets, real clock
# ---------------------------------------------------------------------------


def test_a_fifty_hz_step_logs_rows_with_echoes(tmp_path, echo_rx, live_wheel):
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
    assert float(fresh[-1]["vel_rad_s"]) == pytest.approx(0.5, abs=0.1)


# ---------------------------------------------------------------------------
# Sequence tracking (deterministic lines, no echoes)
# ---------------------------------------------------------------------------


def test_b_seq_gap_is_counted_as_missed_whl(tmp_path, cmd_rx, echo_rx):
    seqs = [0, 1, 2, 6, 7]
    stats, path = _run(
        tmp_path, [_line(s) for s in seqs], cmd_rx.getsockname(), echo_rx
    )
    by_seq = _by_seq(_rows(path))
    assert {s: int(by_seq[s]["missed_whl"]) for s in seqs} == {0: 0, 1: 0, 2: 0, 6: 3, 7: 0}
    assert stats.missed_whl_total == 3


def test_b_seq_wrap_is_not_a_gap(tmp_path, cmd_rx, echo_rx):
    seqs = [65534, 65535, 1]
    stats, path = _run(
        tmp_path, [_line(s) for s in seqs], cmd_rx.getsockname(), echo_rx
    )
    by_seq = _by_seq(_rows(path))
    assert {s: int(by_seq[s]["missed_whl"]) for s in seqs} == {65534: 0, 65535: 0, 1: 1}
    assert stats.missed_whl_total == 1


def test_c_bad_lines_counted_and_skipped(tmp_path, cmd_rx, echo_rx):
    good2 = _line(2)
    lines = [_line(0), "garbage", _line(1), _bad_checksum(good2), good2]
    stats, path = _run(tmp_path, lines, cmd_rx.getsockname(), echo_rx)
    rows = _rows(path)

    assert stats.bad_lines == 2
    assert stats.whl_lines == 3
    assert sorted(int(r["whl_seq"]) for r in rows) == [0, 1, 2]
    assert all(r["missed_whl"] == "0" for r in rows)


# ---------------------------------------------------------------------------
# Stale and state handling
# ---------------------------------------------------------------------------


def test_d_state_one_line_sends_zero_stale_command(tmp_path, cmd_rx, echo_rx):
    lines = [_line(0, model=0.5, state=0), _line(1, model=0.5, state=1)]
    _run(tmp_path, lines, cmd_rx.getsockname(), echo_rx)
    cmds = _wire_cmds(cmd_rx)
    assert [(c.cmd_seq, c.wheel_rad_s, c.stale) for c in cmds] == [(0, 0.5, 0), (1, 0.0, 1)]


def test_d_state_one_row_shows_stale_and_zero_command(tmp_path, cmd_rx, echo_rx):
    lines = [_line(0, model=0.5, state=0), _line(1, model=0.5, state=1)]
    _stats, path = _run(tmp_path, lines, cmd_rx.getsockname(), echo_rx)
    by_seq = _by_seq(_rows(path))
    assert by_seq[1]["whl_state"] == "1"
    assert by_seq[1]["stale"] == "1"
    assert float(by_seq[1]["cmd_rad_s"]) == 0.0


def test_e_silence_gives_one_stale_command_then_fresh_resumes(tmp_path, echo_rx, live_wheel):
    gate = threading.Event()
    resumed_at: list[float] = []

    def _gapped():
        yield _line(0, model=0.5)
        gate.wait(timeout=5.0)
        resumed_at.append(time.monotonic())
        yield _line(1, model=0.5)

    samples: list[tuple[float, float]] = []
    sampling_done = threading.Event()

    def _sample():
        # Test-only read of FakeWheel._velocity to see the wheel decay.
        while not sampling_done.is_set():
            samples.append((time.monotonic(), live_wheel._velocity))
            sampling_done.wait(0.01)

    sampler = threading.Thread(target=_sample, daemon=True)
    gap_timer = threading.Timer(0.5, gate.set)
    sampler.start()
    gap_timer.start()
    try:
        stats, path = _run(tmp_path, _gapped(), live_wheel.cmd_addr, echo_rx)
    finally:
        gap_timer.cancel()
        gate.set()
        sampling_done.set()
        sampler.join(timeout=1.0)

    assert len(resumed_at) == 1
    rows = _rows(path)
    stale_rows = [r for r in rows if r["whl_seq"] == ""]
    assert len(stale_rows) == 1
    assert stale_rows[0]["stale"] == "1"
    assert float(stale_rows[0]["cmd_rad_s"]) == 0.0

    by_seq = _by_seq(rows)
    assert by_seq[1]["stale"] == "0"
    assert float(by_seq[1]["cmd_rad_s"]) == pytest.approx(0.5)
    assert int(stale_rows[0]["cmd_seq"]) == int(by_seq[0]["cmd_seq"]) + 1
    assert int(by_seq[1]["cmd_seq"]) == int(stale_rows[0]["cmd_seq"]) + 1
    assert stats.stale_cmds >= 1

    assert max(v for _, v in samples) > 0.3
    before_resume = [v for t, v in samples if t < resumed_at[0]]
    assert before_resume
    assert before_resume[-1] < 0.1


# ---------------------------------------------------------------------------
# Stop conditions
# ---------------------------------------------------------------------------


def test_f_duration_stops_loop_and_sends_final_stale(tmp_path, cmd_rx, echo_rx):
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
    assert cmds[-1].wheel_rad_s == 0.0
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
    assert cmds[-1].wheel_rad_s == 0.0
    assert stats.cmds_sent == len(cmds)


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


# ---------------------------------------------------------------------------
# Echo handling
# ---------------------------------------------------------------------------


def test_bad_and_unmatched_echoes_counted_and_rows_flush_empty(tmp_path, cmd_rx, echo_rx):
    addr = echo_rx.getsockname()
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        tx.sendto(b"\xff\xfe junk", addr)
        tx.sendto(encode_echo(999, 0.1, 0.0, 0.0), addr)
        tx.sendto(encode_echo(None, 0.1, 0.0, None), addr)
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
    assert rows[0]["host_rtt_ms"] == ""


# ---------------------------------------------------------------------------
# CSV contract
# ---------------------------------------------------------------------------


def test_g_csv_header_is_ctrl_wheel_log_columns(tmp_path, cmd_rx, echo_rx):
    stats, path = _run(tmp_path, iter([]), cmd_rx.getsockname(), echo_rx)
    with open(path, newline="", encoding="utf-8") as fh:
        header = next(csv.reader(fh))
    assert header == list(COLUMNS)
    assert _rows(path) == []
    assert stats.whl_lines == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
