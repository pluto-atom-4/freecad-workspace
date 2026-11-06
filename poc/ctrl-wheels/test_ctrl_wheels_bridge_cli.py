#!/usr/bin/env python3
"""
Tests for the ctrl-wheels bridge command line (ctrl_wheels_bridge.main and
build_parser), WHL2 twin form. These tests pin its contract.

CLI contract pinned here:
    exactly one of --port PORT / --stdin; --record CSV (required);
    --overwrite; --sign-left {1,-1} (default 1); --sign-right {1,-1} (default 1);
    --duration SECONDS (> 0);
    --cmd-port N (default 5018); --echo-port N (default 5017);
    --baud N (default 115200); --echo-timeout SECONDS (default 0.5).
--sign alone is refused: it is an ambiguous prefix of --sign-left and --sign-right.
--sign-left and --sign-right are Webots-side signs applied to the model velocity
(Relay sign). They are not the chip raw signs of the generator ctrl_wheels_msg
(--sign-left/--sign-right there default +1/-1). The bridge ignores the raw columns.
The CLI never latches: a fresh state-0 line resumes after a stop.
Parsed attribute names: port, stdin, record, overwrite, sign_left, sign_right,
duration, cmd_port, echo_port, baud, echo_timeout.

Ports: 5005-5008, 5017 and 5018 are never bound by a test. The CLI refuses only
5006; the defaults 5017/5018 are accepted by its checks. Ports used in tests are
ephemeral (bound to port 0, read back, closed). Refusal tests pass fixed
non-reserved numbers because usage errors happen before any socket is bound.

SIM ONLY. Webots-side +1/+1 direction owner-observed (2026-11-06, sim only); chip-side signs UNCONFIRMED; the model values are UNVERIFIED. Nothing
here validates a real wheel, a balance loop or a serial device.

Usage (run from poc/ctrl-wheels):
    python3 -m pytest test_ctrl_wheels_bridge_cli.py -v
"""

from __future__ import annotations

import csv
import io
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from dataclasses import fields
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import ctrl_wheels_bridge as cwb  # noqa: E402
from ctrl_wheels_bridge import BridgeStats, DofSerialError, build_parser, main  # noqa: E402
from ctrl_wheels_log import COLUMNS, CsvLog  # noqa: E402
from ctrl_wheels_msg import Whl, Whl2, format_whl, format_whl2, step_lines  # noqa: E402
from ctrl_wheels_udp import open_listener  # noqa: E402
from fake_wheels import FakeWheel  # noqa: E402

_RESERVED_PORTS = frozenset({5005, 5006, 5007, 5008, 5017, 5018})
_RESERVED_ECHO_PORT = 5006
_LOOPBACK = "127.0.0.1"


def _free_port() -> int:
    """Ephemeral loopback UDP port, released before return. Never a reserved port."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((_LOOPBACK, 0))
        port = sock.getsockname()[1]
    finally:
        sock.close()
    assert port not in _RESERVED_PORTS and port != _RESERVED_ECHO_PORT
    return port


def _two_free_ports() -> tuple[int, int]:
    cmd_port = _free_port()
    echo_port = _free_port()
    while echo_port == cmd_port:
        echo_port = _free_port()
    return cmd_port, echo_port


def _whl_line(seq: int, model: float = 0.5) -> str:
    return format_whl2(
        Whl2(seq=seq, t_us=seq * 20000, model_rad_s=model, raw_l=0, raw_r=0, state=0))


def _rows(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def _argv(record, cmd_port=40001, echo_port=40002, *extra):
    """Valid stdin-source argv. Default ports are fixed, for refusal tests only."""
    return [
        "--stdin",
        "--record", str(record),
        "--cmd-port", str(cmd_port),
        "--echo-port", str(echo_port),
        *extra,
    ]


def _assert_usage_error(argv, record):
    """argparse-level refusal: SystemExit(2), and no CSV is created."""
    with pytest.raises(SystemExit) as exc:
        main(argv)
    assert exc.value.code == 2
    assert not record.exists()


@pytest.fixture(autouse=True)
def _no_real_stdin(monkeypatch):
    """Default stdin is empty, so a test can never block on the real terminal."""
    monkeypatch.setattr(sys, "stdin", io.StringIO(""))


@pytest.fixture
def pipe_stdin(monkeypatch):
    """Replace sys.stdin with a pipe fed in real time by a writer thread."""
    handles = []

    def _feed(lines):
        read_fd, write_fd = os.pipe()

        def _writer():
            with os.fdopen(write_fd, "w", encoding="ascii") as wf:
                for line in lines:
                    wf.write(line)
                    wf.flush()

        thread = threading.Thread(target=_writer, name="cli_stdin_feeder", daemon=True)
        thread.start()
        reader = os.fdopen(read_fd, "r", encoding="ascii")
        handles.append((reader, thread))
        monkeypatch.setattr(sys, "stdin", reader)
        return reader

    yield _feed
    for reader, thread in handles:
        reader.close()
        thread.join(timeout=2.0)


# ---------------------------------------------------------------------------
# Parser defaults and accepted values
# ---------------------------------------------------------------------------


def test_defaults_via_parse_args():
    args = build_parser().parse_args(["--stdin", "--record", "out.csv"])
    assert args.stdin is True
    assert args.port is None
    assert args.record == "out.csv"
    assert args.overwrite is False
    assert args.sign_left == 1
    assert args.sign_right == 1
    assert not hasattr(args, "sign")
    assert args.duration is None
    assert args.cmd_port == 5018
    assert args.echo_port == 5017
    assert args.baud == 115200
    assert args.echo_timeout == 0.5


def test_default_ports_pass_cli_checks_without_refusal():
    """5017/5018 are in the reserved set but the CLI refuses only 5006. No socket is bound."""
    parser = build_parser()
    args = parser.parse_args(["--stdin", "--record", "out.csv"])
    cwb._check_args(parser, args)
    assert (args.cmd_port, args.echo_port) == (5018, 5017)


def test_port_source_and_options_parse():
    args = build_parser().parse_args([
        "--port", "rfc2217://localhost:4002",
        "--record", "out.csv",
        "--overwrite",
        "--duration", "2.5",
        "--baud", "57600",
        "--echo-timeout", "0.25",
    ])
    assert args.port == "rfc2217://localhost:4002"
    assert args.stdin is False
    assert args.overwrite is True
    assert args.duration == 2.5
    assert args.baud == 57600
    assert args.echo_timeout == 0.25


@pytest.mark.parametrize("right_text", ["1", "-1"])
@pytest.mark.parametrize("left_text", ["1", "-1"])
def test_sign_left_and_right_accept_plus_and_minus_one(left_text, right_text):
    args = build_parser().parse_args([
        "--stdin", "--record", "out.csv",
        "--sign-left", left_text, "--sign-right", right_text,
    ])
    assert args.sign_left == int(left_text)
    assert args.sign_right == int(right_text)
    assert type(args.sign_left) is int
    assert type(args.sign_right) is int


# ---------------------------------------------------------------------------
# Usage errors (SystemExit 2, no CSV created)
# ---------------------------------------------------------------------------


def test_neither_port_nor_stdin_refused(tmp_path):
    record = tmp_path / "out.csv"
    _assert_usage_error(["--record", str(record)], record)


def test_port_and_stdin_together_refused(tmp_path):
    record = tmp_path / "out.csv"
    _assert_usage_error(
        ["--port", "rfc2217://localhost:4002", "--stdin", "--record", str(record)], record)


def test_record_is_required(tmp_path):
    with pytest.raises(SystemExit) as exc:
        main(["--stdin"])
    assert exc.value.code == 2


@pytest.mark.parametrize("text", ["2", "0", "-2", "1.0", "x"])
@pytest.mark.parametrize("flag", ["--sign-left", "--sign-right"])
def test_sign_other_than_plus_or_minus_one_refused(tmp_path, flag, text):
    record = tmp_path / "out.csv"
    _assert_usage_error(_argv(record, 40001, 40002, flag, text), record)


def test_old_sign_flag_refused_as_ambiguous_prefix(tmp_path):
    """--sign is a prefix of both --sign-left and --sign-right, so argparse abbreviation
    matching refuses it as ambiguous; it is not a silent alias."""
    record = tmp_path / "out.csv"
    _assert_usage_error(_argv(record, 40001, 40002, "--sign", "1"), record)


@pytest.mark.parametrize(("flag", "value"), [
    ("--cmd-port", "0"),
    ("--cmd-port", "65536"),
    ("--cmd-port", "-1"),
    ("--echo-port", "0"),
    ("--echo-port", "70000"),
    ("--echo-port", "-1"),
])
def test_port_out_of_range_refused(tmp_path, flag, value):
    record = tmp_path / "out.csv"
    cmd_port, echo_port = 40001, 40002
    if flag == "--cmd-port":
        cmd_port = value
    else:
        echo_port = value
    _assert_usage_error(_argv(record, cmd_port, echo_port), record)


def test_cmd_port_5006_refused(tmp_path):
    record = tmp_path / "out.csv"
    _assert_usage_error(_argv(record, 5006, 40002), record)


def test_echo_port_5006_refused(tmp_path):
    record = tmp_path / "out.csv"
    _assert_usage_error(_argv(record, 40001, 5006), record)


def test_equal_ports_refused(tmp_path):
    record = tmp_path / "out.csv"
    _assert_usage_error(_argv(record, 40001, 40001), record)


@pytest.mark.parametrize("text", ["0", "-1", "nan"])
def test_duration_must_be_positive(tmp_path, text):
    record = tmp_path / "out.csv"
    _assert_usage_error(_argv(record, 40001, 40002, "--duration", text), record)


def test_echo_timeout_10_seconds_accepted():
    args = build_parser().parse_args(["--stdin", "--record", "out.csv", "--echo-timeout", "10"])
    assert args.echo_timeout == 10.0


@pytest.mark.parametrize("text", ["10.5", "1e9"])
def test_echo_timeout_above_10_seconds_refused(tmp_path, text):
    record = tmp_path / "out.csv"
    with pytest.raises(SystemExit) as exc:
        build_parser().parse_args(["--stdin", "--record", str(record), "--echo-timeout", text])
    assert exc.value.code == 2
    assert not record.exists()


# ---------------------------------------------------------------------------
# --record and --overwrite
# ---------------------------------------------------------------------------


def test_existing_csv_refused_without_overwrite(tmp_path, capsys):
    record = tmp_path / "out.csv"
    original = b"sentinel,do-not-touch\n"
    record.write_bytes(original)
    cmd_port, echo_port = _two_free_ports()

    rc = main(_argv(record, cmd_port, echo_port))

    assert rc == 2
    assert record.read_bytes() == original
    assert capsys.readouterr().err.strip() != ""


def test_overwrite_replaces_existing_csv(tmp_path):
    record = tmp_path / "out.csv"
    record.write_bytes(b"old,content\n")
    cmd_port, echo_port = _two_free_ports()

    rc = main(_argv(record, cmd_port, echo_port, "--overwrite"))

    assert rc == 0
    with open(record, newline="", encoding="utf-8") as fh:
        header = next(csv.reader(fh))
    assert header == list(COLUMNS)
    assert b"old,content" not in record.read_bytes()


# ---------------------------------------------------------------------------
# Runtime errors returning 2, and KeyboardInterrupt returning 0
# ---------------------------------------------------------------------------


def test_missing_device_path_returns_2_with_message(tmp_path, capsys):
    record = tmp_path / "out.csv"
    cmd_port, echo_port = _two_free_ports()
    argv = [
        "--port", "/dev/definitely-not-a-port",
        "--record", str(record),
        "--cmd-port", str(cmd_port),
        "--echo-port", str(echo_port),
    ]

    rc = main(argv)

    assert rc == 2
    assert "/dev/definitely-not-a-port" in capsys.readouterr().err


def test_busy_echo_port_returns_2_with_message(tmp_path, capsys):
    record = tmp_path / "out.csv"
    cmd_port = _free_port()
    held = open_listener(0)
    try:
        rc = main(_argv(record, cmd_port, held.getsockname()[1]))
    finally:
        held.close()

    assert rc == 2
    assert capsys.readouterr().err.strip() != ""


class _InterruptedStdin:
    """One good line, then KeyboardInterrupt, as Ctrl-C during a stdin read would be."""

    def __init__(self):
        self._gen = self._lines()

    def _lines(self):
        yield _whl_line(0) + "\n"
        raise KeyboardInterrupt

    def __iter__(self):
        return self

    def __next__(self):
        return next(self._gen)


def test_keyboard_interrupt_is_clean_shutdown_returns_0(tmp_path, monkeypatch):
    record = tmp_path / "out.csv"
    cmd_port, echo_port = _two_free_ports()
    monkeypatch.setattr(sys, "stdin", _InterruptedStdin())

    rc = main(_argv(record, cmd_port, echo_port))

    assert rc == 0
    with open(record, newline="", encoding="utf-8") as fh:
        assert next(csv.reader(fh)) == list(COLUMNS)


def test_source_error_mid_run_returns_2_with_summary_and_error(tmp_path, monkeypatch, capsys):
    record = tmp_path / "source_error.csv"
    cmd_port, echo_port = _two_free_ports()

    def _failing_source():
        yield _whl_line(0)
        raise DofSerialError("'/dev/fake-wheel': read failed: injected")

    monkeypatch.setattr(cwb, "_stdin_lines", _failing_source)

    rc = main(_argv(record, cmd_port, echo_port, "--echo-timeout", "0.05"))

    assert rc == 2
    err_lines = [ln for ln in capsys.readouterr().err.splitlines() if ln.strip()]
    assert any("read failed: injected" in ln for ln in err_lines)
    summaries = [ln for ln in err_lines if ln.startswith("[ctrl_wheels_bridge] summary ")]
    assert len(summaries) == 1
    assert re.search(r"whl_lines=1\b", summaries[0])
    with open(record, newline="", encoding="utf-8") as fh:
        reader = list(csv.reader(fh))
    assert reader[0] == list(COLUMNS)
    assert len(reader) - 1 == 1


def test_csv_write_error_returns_2_and_closes_run(tmp_path, monkeypatch, capsys):
    record = tmp_path / "disk_full.csv"
    cmd_port, echo_port = _two_free_ports()

    def _disk_full(self, row):
        raise OSError(28, "No space left on device")

    monkeypatch.setattr(CsvLog, "write_row", _disk_full)
    monkeypatch.setattr(cwb, "_stdin_lines", lambda: iter([_whl_line(0)]))

    rc = main(_argv(record, cmd_port, echo_port, "--echo-timeout", "0.05"))

    assert rc == 2
    assert "No space left on device" in capsys.readouterr().err
    with open(record, newline="", encoding="utf-8") as fh:
        assert next(csv.reader(fh)) == list(COLUMNS)


# ---------------------------------------------------------------------------
# End to end: real FakeWheel, real loopback UDP, real clock
# ---------------------------------------------------------------------------


def test_stdin_end_to_end_logs_rows_and_summary(tmp_path, monkeypatch, capsys):
    record = tmp_path / "e2e.csv"
    cmd_port, echo_port = _two_free_ports()
    wheel = FakeWheel(("127.0.0.1", echo_port), cmd_port=cmd_port)
    stop = threading.Event()
    thread = wheel.run_background(stop)
    lines = step_lines(0.5, 0.6, realtime=False)
    monkeypatch.setattr(sys, "stdin", io.StringIO("\n".join(lines)))
    try:
        rc = main([
            "--stdin",
            "--record", str(record),
            "--cmd-port", str(cmd_port),
            "--echo-port", str(echo_port),
        ])
    finally:
        stop.set()
        thread.join(timeout=1.0)
        wheel.close()

    assert rc == 0
    with open(record, newline="", encoding="utf-8") as fh:
        assert next(csv.reader(fh)) == list(COLUMNS)
    assert len(_rows(record)) >= 30

    err_lines = [ln for ln in capsys.readouterr().err.splitlines() if ln.strip()]
    assert len(err_lines) == 1
    summary = err_lines[0]
    for field in (f.name for f in fields(BridgeStats)):
        assert field in summary
    assert re.search(r"whl_lines\D*30\b", summary)
    assert re.search(r"bad_lines\D*0\b", summary)


@pytest.mark.parametrize("sign_right", [1, -1])
@pytest.mark.parametrize("sign_left", [1, -1])
def test_sign_flags_flip_each_echoed_wheel_velocity(tmp_path, pipe_stdin, sign_left, sign_right):
    """--sign-left/--sign-right are Webots-side signs on the model velocity, SIM ONLY and
    Chip-side signs UNCONFIRMED; Webots +1/+1 owner-observed. The input uses the generator's default raws (+1 left, -1 right); the bridge
    ignores raw, so each echoed wheel follows its own sign flag alone."""
    record = tmp_path / "sign.csv"
    cmd_port, echo_port = _two_free_ports()
    wheel = FakeWheel(("127.0.0.1", echo_port), cmd_port=cmd_port)
    stop = threading.Event()
    thread = wheel.run_background(stop)
    pipe_stdin(step_lines(0.5, 1.0, realtime=True))
    try:
        rc = main([
            "--stdin",
            "--record", str(record),
            "--sign-left", str(sign_left),
            "--sign-right", str(sign_right),
            "--cmd-port", str(cmd_port),
            "--echo-port", str(echo_port),
        ])
    finally:
        stop.set()
        thread.join(timeout=1.0)
        wheel.close()

    assert rc == 0
    rows = _rows(record)
    for column, sign in (("vel_rad_s_l", sign_left), ("vel_rad_s_r", sign_right)):
        vel = [float(r[column]) for r in rows if r[column] != ""]
        assert len(vel) >= 3, f"{column}: only {len(vel)} echoed velocity rows, too few to check"
        if sign == 1:
            assert max(vel) > 0.05, column
            assert min(vel) >= -1e-6, column
        else:
            assert min(vel) < -0.05, column
            assert max(vel) <= 1e-6, column
    live = [r for r in rows if r["stale"] == "0"]
    assert live
    for r in live:
        assert float(r["cmd_left_rad_s"]) == pytest.approx(sign_left * 0.5)
        assert float(r["cmd_right_rad_s"]) == pytest.approx(sign_right * 0.5)


def _run_stdin_with_wheel(tmp_path, monkeypatch, lines, name):
    record = tmp_path / name
    cmd_port, echo_port = _two_free_ports()
    wheel = FakeWheel(("127.0.0.1", echo_port), cmd_port=cmd_port)
    stop = threading.Event()
    thread = wheel.run_background(stop)
    monkeypatch.setattr(sys, "stdin", io.StringIO("".join(lines)))
    try:
        rc = main([
            "--stdin",
            "--record", str(record),
            "--cmd-port", str(cmd_port),
            "--echo-port", str(echo_port),
        ])
    finally:
        stop.set()
        thread.join(timeout=1.0)
        wheel.close()
    assert rc == 0
    return _rows(record)


def test_state_1_line_stops_both_wheels(tmp_path, monkeypatch):
    rows = _run_stdin_with_wheel(
        tmp_path, monkeypatch, step_lines(0.5, 0.4, realtime=False, state=1), "state1.csv")
    held = [r for r in rows if r["whl_state"] == "1"]
    assert held
    for r in held:
        assert float(r["cmd_left_rad_s"]) == 0.0
        assert float(r["cmd_right_rad_s"]) == 0.0
        assert r["stale"] == "1"


def test_state_0_line_after_state_1_resumes_no_latch(tmp_path, monkeypatch):
    lines = list(step_lines(0.5, 0.4, realtime=False, state=1, start_seq=0))
    lines += list(step_lines(0.5, 0.4, realtime=False, state=0, start_seq=len(lines)))
    rows = _run_stdin_with_wheel(tmp_path, monkeypatch, lines, "resume.csv")
    by_line = [r for r in rows if r["whl_state"] != ""]
    held = [r for r in by_line if r["whl_state"] == "1"]
    run = [r for r in by_line if r["whl_state"] == "0"]
    assert held and run
    assert all(r["stale"] == "1" for r in held)
    assert run[-1]["stale"] == "0"
    assert float(run[-1]["cmd_left_rad_s"]) == pytest.approx(0.5)
    assert float(run[-1]["cmd_right_rad_s"]) == pytest.approx(0.5)


def test_v1_whl_line_on_stdin_is_a_bad_line(tmp_path, monkeypatch, capsys):
    record = tmp_path / "v1.csv"
    cmd_port, echo_port = _two_free_ports()
    v1 = format_whl(Whl(seq=0, t_us=0, model_rad_s=0.5, raw=0, state=0))
    monkeypatch.setattr(sys, "stdin", io.StringIO(v1))

    rc = main(_argv(record, cmd_port, echo_port, "--echo-timeout", "0.05"))

    assert rc == 0
    err = capsys.readouterr().err
    assert re.search(r"bad_lines=1\b", err)
    assert re.search(r"whl_lines=0\b", err)


# ---------------------------------------------------------------------------
# Script entry point
# ---------------------------------------------------------------------------


def test_script_help_runs_as_program():
    script = Path(__file__).resolve().parent / "ctrl_wheels_bridge.py"
    proc = subprocess.run(
        [sys.executable, str(script), "--help"],
        capture_output=True,
        text=True,
        timeout=30,
        cwd=str(script.parent),
    )
    assert proc.returncode == 0, proc.stderr
    for token in ("--stdin", "--record", "--sign-left", "--sign-right", "4002"):
        assert token in proc.stdout


def test_csv_close_error_returns_2_after_summary(tmp_path, monkeypatch, capsys):
    """log.close() can fail on flush (disk full): rc 2, summary still printed, no traceback."""
    orig = CsvLog._release

    def bad(self):
        orig(self)
        raise OSError(28, "No space left on device (injected at close)")

    monkeypatch.setattr(CsvLog, "_release", bad)
    out = tmp_path / "close.csv"
    cmd_port, echo_port = _two_free_ports()
    rc = cwb.main(["--stdin", "--record", str(out), "--cmd-port", str(cmd_port),
                   "--echo-port", str(echo_port), "--echo-timeout", "0.05"])
    err = capsys.readouterr().err
    assert rc == 2
    assert "summary" in err
    assert "cannot finish --record" in err


class _HeldStdin:
    """One good line, then blocks until released (or 5 s), then ends: a stdin that does not end."""

    def __init__(self, release):
        self._release = release
        self.yielded = threading.Event()  # set once the first line has been handed out
        self._sent = False

    def __iter__(self):
        return self

    def __next__(self):
        if not self._sent:
            self._sent = True
            self.yielded.set()
            return _whl_line(0) + "\n"
        self._release.wait(timeout=5.0)
        raise StopIteration


def test_stdin_with_duration_ends_rc_0_with_summary(tmp_path, monkeypatch, capsys):
    """--duration stops a stdin source that has not ended: rc 0 and a summary."""
    record = tmp_path / "dur.csv"
    cmd_port, echo_port = _two_free_ports()
    release = threading.Event()
    held = _HeldStdin(release)
    monkeypatch.setattr(sys, "stdin", held)
    try:
        rc = main(_argv(record, cmd_port, echo_port, "--duration", "1.5", "--echo-timeout", "0.05"))
    finally:
        release.set()

    assert held.yielded.wait(timeout=1.0), "the stdin source was never read"
    assert rc == 0
    err = capsys.readouterr().err
    summaries = [ln for ln in err.splitlines() if ln.startswith("[ctrl_wheels_bridge] summary ")]
    assert len(summaries) == 1
    assert re.search(r"whl_lines=1\b", summaries[0])


def test_unused_rfc2217_url_with_duration_returns_2_without_rows(tmp_path, capsys):
    """Nothing listens on the URL: the source never opens, so the run is a failed source."""
    record = tmp_path / "url.csv"
    cmd_port, echo_port = _two_free_ports()
    url_port = _free_port()

    rc = main([
        "--port", f"rfc2217://127.0.0.1:{url_port}",
        "--record", str(record),
        "--cmd-port", str(cmd_port),
        "--echo-port", str(echo_port),
        "--duration", "0.2",
    ])

    assert rc == 2
    assert "did not open" in capsys.readouterr().err
    assert not record.exists() or _rows(record) == []


def _reader_threads():
    return [t for t in threading.enumerate() if t.name == "ctrl_wheels_lines" and t.is_alive()]


def _sigint_when_reader_runs(cancel: threading.Event, seen: threading.Event) -> None:
    """Wait (bounded, 3 s) until the bridge's line reader thread runs, set seen, pause a fixed
    0.05 s and send SIGINT unless cancel is set. If the reader thread never appears, no SIGINT
    is sent and seen stays unset, so the test fails (main then ends on its --duration)."""
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not cancel.is_set():
        if _reader_threads():
            seen.set()
            break
        time.sleep(0.01)
    if not seen.is_set():
        return
    time.sleep(0.05)
    if not cancel.is_set():
        os.kill(os.getpid(), signal.SIGINT)


def test_clean_ctrl_c_prints_nothing_after_the_summary(tmp_path, capsys):
    """After a clean stop on a pyserial loop:// loopback source, the reader
    thread's failed read must not log 'line source stopped' after the summary. The SIGINT lands
    while the reader thread is live, so the port is closed mid-read."""
    for i in range(8):
        for _ in range(300):  # a reader left over from the previous run must be gone first
            if not _reader_threads():
                break
            time.sleep(0.01)
        assert not _reader_threads(), f"run {i}: a reader from an earlier run is still alive"
        cmd_port, echo_port = _two_free_ports()
        cancel = threading.Event()
        seen =threading.Event()
        helper = threading.Thread(
            target=_sigint_when_reader_runs, args=(cancel, seen), daemon=True)
        helper.start()
        try:
            rc = cwb.main(["--port", "loop://", "--record", str(tmp_path / f"clean{i}.csv"),
                           "--cmd-port", str(cmd_port), "--echo-port", str(echo_port),
                           "--echo-timeout", "0.05", "--duration", "10"])
        finally:
            cancel.set()
            helper.join(timeout=5.0)
        assert seen.is_set(), f"run {i}: the reader thread never appeared, no SIGINT was sent"
        assert rc == 0, f"run {i}"
        for _ in range(500):  # the reader must be gone, so a late 'line source stopped' is seen
            if not _reader_threads():
                break
            time.sleep(0.01)
        assert not _reader_threads(), f"run {i}: the reader thread is still alive after 5 s"
        err = capsys.readouterr().err
        lines = err.splitlines()
        summaries = [ln for ln in lines if ln.startswith("[ctrl_wheels_bridge] summary ")]
        assert len(summaries) == 1, f"run {i}: {err!r}"
        assert "line source stopped" not in err, f"run {i}: {err!r}"
        idx = lines.index(summaries[0])
        assert not [ln for ln in lines[idx + 1:] if ln.strip()], f"run {i}: {err!r}"


def test_midrun_source_failure_line_comes_before_the_summary(tmp_path, monkeypatch, capsys):
    """A real mid-run source failure is still logged,
    before the summary, and returns 2."""
    record = tmp_path / "midrun_fail.csv"
    cmd_port, echo_port = _two_free_ports()

    def _failing_source():
        yield _whl_line(0)
        raise DofSerialError("'/dev/fake-wheel': read failed: injected")

    monkeypatch.setattr(cwb, "_stdin_lines", _failing_source)

    rc = main(_argv(record, cmd_port, echo_port, "--echo-timeout", "0.05"))

    assert rc == 2
    time.sleep(0.2)
    err = capsys.readouterr().err
    lines = err.splitlines()
    summaries = [i for i, ln in enumerate(lines) if ln.startswith("[ctrl_wheels_bridge] summary ")]
    stopped = [i for i, ln in enumerate(lines) if "line source stopped" in ln]
    assert len(summaries) == 1
    assert len(stopped) == 1
    assert "read failed: injected" in lines[stopped[0]]
    assert stopped[0] < summaries[0]
    assert not [ln for ln in lines[summaries[0] + 1:] if ln.strip()]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
