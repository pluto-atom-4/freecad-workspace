#!/usr/bin/env python3
"""
Tests for monitor/mock_publisher.py (issue #284, sub-issue of #258).

Covers CSV parsing, timelines, pacing, the never-raise Publisher, one real UDP
loopback send on 127.0.0.1 (ephemeral port) and the CLI. No Webots, no
hardware, no real sleeping; must NOT import `controller` or `serial`.

Usage:
    cd poc/servo-wheel-dof
    mamba run -n servo-wheel-dof python3 -m pytest -q monitor
"""

from __future__ import annotations

import itertools
import json
import re
import select
import socket
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import mock_publisher as mp  # noqa: E402

# mock_publisher put the controller dir on sys.path, so this import works.
from imu_wheel_msg import ImuWheel, mock_imu_wheel, parse_imu_wheel_msg  # noqa: E402

HEADER = "roll,pitch,yaw,wheel_angle"
SAMPLE = ImuWheel(0.1, 0.2, 0.3, 0.4)


class _FakeSock:
    """Stands in for the UDP socket: records sends or raises on demand."""

    def __init__(self, exc=None):
        self.sent = []
        self.exc = exc

    def sendto(self, data, addr):
        if self.exc is not None:
            raise self.exc
        self.sent.append((data, addr))

    def close(self):
        pass


class _Rec:
    """Stands in for Publisher in run_timeline tests."""

    def __init__(self):
        self.calls = []

    def send(self, seq, sample):
        self.calls.append((seq, sample))


def _fake_publisher(exc=None):
    pub = mp.Publisher("127.0.0.1", 5006)
    pub._sock.close()
    fake = _FakeSock(exc)
    pub._sock = fake
    return pub, fake


def _no_sleep(_seconds):
    pass


def _raise_interrupt(_seconds):
    raise KeyboardInterrupt


@pytest.fixture
def rx():
    """Non-blocking loopback receiver on an OS-chosen port."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.setblocking(False)
    try:
        yield sock
    finally:
        sock.close()


def _port(sock) -> int:
    return sock.getsockname()[1]


def _drain(sock) -> list[bytes]:
    """All queued datagrams (loopback delivery is synchronous)."""
    out = []
    while True:
        ready, _, _ = select.select([sock], [], [], 0)
        if not ready:
            return out
        out.append(sock.recvfrom(65535)[0])


# ---------------------------------------------------------------- CSV parsing


def test_parse_happy_path():
    lines = [HEADER, "0.1,0.2,0.3,0.4", "-1,2,3e-1,0"]
    samples, t_us = mp.parse_replay_csv(lines)
    assert samples == [ImuWheel(0.1, 0.2, 0.3, 0.4), ImuWheel(-1.0, 2.0, 0.3, 0.0)]
    assert t_us == []


def test_parse_any_order_extras_ignored():
    lines = ["wheel_angle, yaw ,pitch,roll,note", "4,3,2,1,hello", "", "8,7,6,5,x"]
    samples, _ = mp.parse_replay_csv(lines)
    assert samples == [ImuWheel(1.0, 2.0, 3.0, 4.0), ImuWheel(5.0, 6.0, 7.0, 8.0)]


def test_parse_strips_bom_in_header():
    samples, _ = mp.parse_replay_csv(["\ufeff" + HEADER, "1,2,3,4"])
    assert samples == [ImuWheel(1.0, 2.0, 3.0, 4.0)]


def test_load_replay_csv_file_with_bom_and_crlf(tmp_path):
    path = tmp_path / "bom.csv"
    path.write_bytes(b"\xef\xbb\xbf" + (HEADER + "\r\n1,2,3,4\r\n").encode())
    samples, t_us = mp.load_replay_csv(path)
    assert samples == [ImuWheel(1.0, 2.0, 3.0, 4.0)]
    assert t_us == []


def test_parse_missing_column():
    with pytest.raises(ValueError, match=r"x\.csv: missing columns .*wheel_angle"):
        mp.parse_replay_csv(["roll,pitch,yaw", "1,2,3"], source="x.csv")


def test_parse_empty_input_reports_missing_columns():
    with pytest.raises(ValueError, match="missing columns"):
        mp.parse_replay_csv([], source="x.csv")


def test_parse_bad_row_reports_line_number():
    lines = [HEADER, "1,2,3,4", "1,oops,3,4", "1,2,3,4"]
    with pytest.raises(ValueError, match=r"x\.csv:3: pitch"):
        mp.parse_replay_csv(lines, source="x.csv")


@pytest.mark.parametrize("bad", ["oops", "", "nan", "inf", "-inf", "1e999"])
def test_parse_rejects_bad_values(bad):
    lines = [HEADER, "1,2,3,4", f"1,{bad},3,4"]
    with pytest.raises(ValueError, match=r"x\.csv:3: pitch"):
        mp.parse_replay_csv(lines, source="x.csv")


def test_parse_short_row():
    with pytest.raises(ValueError, match=r"x\.csv:2: yaw: missing value"):
        mp.parse_replay_csv([HEADER, "1,2"], source="x.csv")


def test_parse_t_us_column():
    lines = [HEADER + ",t_us", "0,0,0,0,0", "1,1,1,1,20000"]
    samples, t_us = mp.parse_replay_csv(lines)
    assert len(samples) == 2
    assert t_us == [0, 20000]


@pytest.mark.parametrize("bad", ["1.5", "", "abc"])
def test_parse_bad_t_us(bad):
    lines = [HEADER + ",t_us", f"0,0,0,0,{bad}"]
    with pytest.raises(ValueError, match=r"x\.csv:2: t_us"):
        mp.parse_replay_csv(lines, source="x.csv")


# ------------------------------------------------------------------ timelines


def test_mock_timeline_values():
    pairs = list(itertools.islice(mp.mock_timeline(50.0), 51))
    assert len(pairs) == 51
    assert pairs[0] == (0.0, mock_imu_wheel(0.0))
    assert pairs[25] == (0.5, mock_imu_wheel(0.5))
    assert pairs[50][0] == 1.0


def test_replay_timeline_fixed_rate():
    samples = [SAMPLE, SAMPLE, SAMPLE]
    times = [t for t, _ in mp.replay_timeline(samples, [], 10.0)]
    assert times == pytest.approx([0.0, 0.1, 0.2])


def test_replay_timeline_t_us_pacing():
    samples = [SAMPLE] * 4
    t_us = [1_000_000, 1_020_000, 1_020_000, 9_000_000]
    times = [t for t, _ in mp.replay_timeline(samples, t_us, 1.0)]
    # first at 0; +20 ms; zero gap adds nothing; huge gap clamped to 1 s.
    assert times == pytest.approx([0.0, 0.02, 0.02, 1.02])


# ------------------------------------------------------------------ Publisher


def test_send_datagram_round_trip():
    pub, fake = _fake_publisher()
    sample = mock_imu_wheel(1.25)
    assert pub.send(7, sample) is True
    data, addr = fake.sent[0]
    assert addr == ("127.0.0.1", 5006)
    assert parse_imu_wheel_msg(data) == sample
    assert json.loads(data)["seq"] == 7
    assert (pub.sent, pub.errors, pub.skipped) == (1, 0, 0)


def test_send_real_udp_loopback(rx):
    pub = mp.Publisher("127.0.0.1", _port(rx))
    sample = mock_imu_wheel(0.5)
    try:
        assert pub.send(3, sample) is True
    finally:
        pub.close()
    ready, _, _ = select.select([rx], [], [], 2.0)
    assert ready, "no datagram arrived on loopback within timeout"
    data, _ = rx.recvfrom(65535)
    assert parse_imu_wheel_msg(data) == sample
    assert (pub.sent, pub.errors, pub.skipped) == (1, 0, 0)


def test_send_counts_oserror_never_raises():
    pub, _ = _fake_publisher(exc=OSError("boom"))
    for seq in range(3):
        assert pub.send(seq, SAMPLE) is False
    assert (pub.sent, pub.errors, pub.skipped) == (0, 3, 0)


def test_send_skips_invalid_inputs():
    pub, fake = _fake_publisher()
    nan, inf = float("nan"), float("inf")
    cases = [
        (0, ImuWheel(nan, 0.0, 0.0, 0.0)),
        (0, ImuWheel(0.0, inf, 0.0, 0.0)),
        (0, ImuWheel(0.0, 0.0, "1", 0.0)),
        (0, None),
        (-1, SAMPLE),
        (True, SAMPLE),
        (1.5, SAMPLE),
    ]
    for seq, sample in cases:
        assert pub.send(seq, sample) is False
    assert (pub.sent, pub.errors, pub.skipped) == (0, 0, len(cases))
    assert fake.sent == []


def test_send_after_close_is_noop():
    pub = mp.Publisher("127.0.0.1", 5006)
    pub.close()
    pub.close()
    assert pub.send(0, SAMPLE) is False
    assert (pub.sent, pub.errors, pub.skipped) == (0, 0, 0)


@pytest.mark.parametrize("port", [0, 65536, True, "5006"])
def test_publisher_rejects_bad_port(port):
    with pytest.raises(ValueError):
        mp.Publisher("127.0.0.1", port)


# --------------------------------------------------------------- run_timeline


def test_run_timeline_paces_with_injected_clock():
    now = [0.0]

    def clock():
        return now[0]

    def sleep(seconds):
        now[0] += seconds

    pub = _Rec()
    timeline = [(0.0, SAMPLE), (0.5, SAMPLE), (1.0, SAMPLE)]
    mp.run_timeline(pub, timeline, None, clock=clock, sleep=sleep)
    assert [seq for seq, _ in pub.calls] == [0, 1, 2]
    assert now[0] == pytest.approx(1.0)


def test_run_timeline_duration_cap():
    pub = _Rec()
    timeline = [(0.0, SAMPLE), (0.5, SAMPLE), (1.0, SAMPLE)]
    mp.run_timeline(pub, timeline, 0.75, clock=lambda: 0.0, sleep=_no_sleep)
    assert [seq for seq, _ in pub.calls] == [0, 1]


# ------------------------------------------------------------------------ CLI


def test_main_mock_publishes_and_prints_summary(rx, capsys):
    argv = ["--mock", "--duration", "1", "--port", str(_port(rx))]
    assert mp.main(argv, sleep=_no_sleep) == 0
    out = capsys.readouterr().out
    assert re.search(r"^published : 50$", out, re.M)
    datagrams = _drain(rx)
    assert len(datagrams) == 50
    assert parse_imu_wheel_msg(datagrams[0]) == mock_imu_wheel(0.0)


def test_main_replay_file(rx, tmp_path, capsys):
    path = tmp_path / "run.csv"
    path.write_text("\n".join([HEADER, "1,2,3,4", "5,6,7,8", "0,0,0,0"]) + "\n")
    argv = ["--replay", str(path), "--port", str(_port(rx))]
    assert mp.main(argv, sleep=_no_sleep) == 0
    assert re.search(r"^published : 3$", capsys.readouterr().out, re.M)
    datagrams = _drain(rx)
    assert len(datagrams) == 3
    assert parse_imu_wheel_msg(datagrams[1]) == ImuWheel(5.0, 6.0, 7.0, 8.0)


def test_main_replay_duration_cap(rx, tmp_path, capsys):
    path = tmp_path / "run.csv"
    rows = [HEADER + ",t_us", "0,0,0,0,0", "0,0,0,0,500000", "0,0,0,0,1000000"]
    path.write_text("\n".join(rows) + "\n")
    argv = ["--replay", str(path), "--duration", "0.75", "--port"]
    argv.append(str(_port(rx)))
    assert mp.main(argv, sleep=_no_sleep) == 0
    assert re.search(r"^published : 2$", capsys.readouterr().out, re.M)


def test_main_port_from_env(rx, monkeypatch, capsys):
    monkeypatch.setenv("SWD_UDP_PORT", str(_port(rx)))
    assert mp.main(["--mock", "--duration", "0.1"], sleep=_no_sleep) == 0
    assert re.search(r"^published : 5$", capsys.readouterr().out, re.M)
    assert len(_drain(rx)) == 5


def test_main_keyboard_interrupt_exits_zero(rx, capsys):
    argv = ["--mock", "--port", str(_port(rx))]
    rc = mp.main(argv, clock=lambda: 0.0, sleep=_raise_interrupt)
    assert rc == 0
    captured = capsys.readouterr()
    assert re.search(r"^published : 1$", captured.out, re.M)
    assert "interrupted" in captured.err


@pytest.mark.parametrize(
    "argv",
    [
        [],
        ["--mock", "--replay", "x.csv"],
        ["--mock", "--port", "0"],
        ["--mock", "--port", "70000"],
        ["--mock", "--port", "abc"],
        ["--mock", "--rate", "0"],
        ["--mock", "--rate", "nan"],
        ["--mock", "--duration", "-1"],
    ],
)
def test_main_bad_args_exit_2(argv):
    with pytest.raises(SystemExit) as exc:
        mp.main(argv, sleep=_no_sleep)
    assert exc.value.code == 2


def test_main_bad_env_port_exits_2(monkeypatch):
    monkeypatch.setenv("SWD_UDP_PORT", "abc")
    with pytest.raises(SystemExit) as exc:
        mp.main(["--mock", "--duration", "0.1"], sleep=_no_sleep)
    assert exc.value.code == 2


def test_main_missing_csv_exits_2(tmp_path):
    with pytest.raises(SystemExit) as exc:
        mp.main(["--replay", str(tmp_path / "nope.csv")], sleep=_no_sleep)
    assert exc.value.code == 2


def test_main_bad_csv_exits_2_with_line_number(tmp_path, capsys):
    path = tmp_path / "bad.csv"
    path.write_text("\n".join([HEADER, "1,2,3,4", "1,oops,3,4"]) + "\n")
    with pytest.raises(SystemExit) as exc:
        mp.main(["--replay", str(path)], sleep=_no_sleep)
    assert exc.value.code == 2
    assert "bad.csv:3" in capsys.readouterr().err
