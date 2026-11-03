#!/usr/bin/env python3
"""
Tests for the ctrl-wheel bridge command line (ctrl_wheel_bridge.main and
build_parser), issue FW #454. These tests pin its contract.

CLI contract pinned here:
    exactly one of --port PORT / --stdin; --record CSV (required);
    --overwrite; --sign {1,-1} (default 1); --duration SECONDS (> 0);
    --cmd-port N (default 5008); --echo-port N (default 5007);
    --baud N (default 115200); --echo-timeout SECONDS (default 0.5).
Parsed attribute names: port, stdin, record, overwrite, sign, duration,
cmd_port, echo_port, baud, echo_timeout.

Ports: 5005-5008 are never bound. Ports used in tests are ephemeral (bound
to port 0, read back, closed). Refusal tests pass fixed non-reserved numbers
because usage errors happen before any socket is bound.

SIM ONLY. The sign convention and the model values are UNVERIFIED. Nothing
here validates a real wheel, a balance loop or a serial device.

Usage (run from poc/ctrl-wheel):
    python3 -m pytest test_ctrl_wheel_bridge_cli.py -v
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

import ctrl_wheel_bridge as cwb  # noqa: E402
from ctrl_wheel_bridge import BridgeStats, DofSerialError, build_parser, main  # noqa: E402
from ctrl_wheel_log import COLUMNS, CsvLog  # noqa: E402
from ctrl_wheel_msg import Whl, format_whl, step_lines  # noqa: E402
from ctrl_wheel_udp import open_listener  # noqa: E402
from fake_wheel import FakeWheel  # noqa: E402

_RESERVED_PORTS = frozenset(range(5005, 5009))
_RESERVED_ECHO_PORT = 5006
_LOOPBACK = "127.0.0.1"


def _free_port() -> int:
    """Ephemeral loopback UDP port, released before return. Never 5005-5008 or 5006."""
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
    return format_whl(Whl(seq=seq, t_us=seq * 20000, model_rad_s=model, raw=0, state=0))


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
    assert args.sign == 1
    assert args.duration is None
    assert args.cmd_port == 5008
    assert args.echo_port == 5007
    assert args.baud == 115200
    assert args.echo_timeout == 0.5


def test_port_source_and_options_parse():
    args = build_parser().parse_args([
        "--port", "rfc2217://localhost:4001",
        "--record", "out.csv",
        "--overwrite",
        "--duration", "2.5",
        "--baud", "57600",
        "--echo-timeout", "0.25",
    ])
    assert args.port == "rfc2217://localhost:4001"
    assert args.stdin is False
    assert args.overwrite is True
    assert args.duration == 2.5
    assert args.baud == 57600
    assert args.echo_timeout == 0.25


@pytest.mark.parametrize(("text", "value"), [("1", 1), ("-1", -1)])
def test_sign_accepts_plus_and_minus_one(text, value):
    args = build_parser().parse_args(["--stdin", "--record", "out.csv", "--sign", text])
    assert args.sign == value
    assert type(args.sign) is int


# ---------------------------------------------------------------------------
# Usage errors (SystemExit 2, no CSV created)
# ---------------------------------------------------------------------------


def test_neither_port_nor_stdin_refused(tmp_path):
    record = tmp_path / "out.csv"
    _assert_usage_error(["--record", str(record)], record)


def test_port_and_stdin_together_refused(tmp_path):
    record = tmp_path / "out.csv"
    _assert_usage_error(
        ["--port", "rfc2217://localhost:4001", "--stdin", "--record", str(record)], record)


def test_record_is_required(tmp_path):
    with pytest.raises(SystemExit) as exc:
        main(["--stdin"])
    assert exc.value.code == 2


@pytest.mark.parametrize("text", ["2", "0", "-2"])
def test_sign_other_than_plus_or_minus_one_refused(tmp_path, text):
    record = tmp_path / "out.csv"
    _assert_usage_error(_argv(record, 40001, 40002, "--sign", text), record)


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
    summaries = [ln for ln in err_lines if ln.startswith("[ctrl_wheel_bridge] summary ")]
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
    echo_port = _free_port()
    wheel = FakeWheel(("127.0.0.1", echo_port))  # cmd_port 0: ephemeral, read back below
    stop = threading.Event()
    thread = wheel.run_background(stop)
    lines = step_lines(0.5, 0.6, realtime=False)
    monkeypatch.setattr(sys, "stdin", io.StringIO("\n".join(lines)))
    try:
        rc = main([
            "--stdin",
            "--record", str(record),
            "--cmd-port", str(wheel.cmd_addr[1]),
            "--echo-port", str(echo_port),
        ])
    finally:
        stop.set()
        thread.join(timeout=1.0)
        wheel.close()

    assert rc == 0
    with open(record, newline="", encoding="utf-8") as fh:
        assert next(csv.reader(fh)) == list(COLUMNS)
    assert len(_rows(record)) > 20

    err_lines = [ln for ln in capsys.readouterr().err.splitlines() if ln.strip()]
    assert len(err_lines) == 1
    summary = err_lines[0]
    for field in (f.name for f in fields(BridgeStats)):
        assert field in summary
    assert re.search(r"whl_lines\D*30\b", summary)
    assert re.search(r"bad_lines\D*0\b", summary)


@pytest.mark.parametrize(("sign_text", "expect_positive"), [("1", True), ("-1", False)])
def test_sign_flag_flips_echoed_velocity(tmp_path, pipe_stdin, sign_text, expect_positive):
    record = tmp_path / "sign.csv"
    echo_port = _free_port()
    wheel = FakeWheel(("127.0.0.1", echo_port))
    stop = threading.Event()
    thread = wheel.run_background(stop)
    pipe_stdin(step_lines(0.5, 1.0, realtime=True))
    try:
        rc = main([
            "--stdin",
            "--record", str(record),
            "--sign", sign_text,
            "--cmd-port", str(wheel.cmd_addr[1]),
            "--echo-port", str(echo_port),
        ])
    finally:
        stop.set()
        thread.join(timeout=1.0)
        wheel.close()

    assert rc == 0
    vel = [float(r["vel_rad_s"]) for r in _rows(record) if r["vel_rad_s"] != ""]
    assert vel
    if expect_positive:
        assert max(vel) > 0.3
        assert min(vel) >= -1e-6
    else:
        assert min(vel) < -0.3
        assert max(vel) <= 1e-6


# ---------------------------------------------------------------------------
# Script entry point
# ---------------------------------------------------------------------------


def test_script_help_runs_as_program():
    script = Path(__file__).resolve().parent / "ctrl_wheel_bridge.py"
    proc = subprocess.run(
        [sys.executable, str(script), "--help"],
        capture_output=True,
        text=True,
        timeout=30,
        cwd=str(script.parent),
    )
    assert proc.returncode == 0, proc.stderr
    for token in ("--stdin", "--record", "--sign"):
        assert token in proc.stdout


def test_csv_close_error_returns_2_after_summary(tmp_path, monkeypatch, capsys):
    """log.close() can fail on flush (disk full): rc 2, summary still printed, no traceback."""
    orig = CsvLog._release

    def bad(self):
        orig(self)
        raise OSError(28, "No space left on device (injected at close)")

    monkeypatch.setattr(CsvLog, "_release", bad)
    monkeypatch.setattr(sys, "stdin", io.StringIO(""))
    out = tmp_path / "close.csv"
    echo_port = _free_port()
    rc = cwb.main(["--stdin", "--record", str(out), "--cmd-port", str(_free_port()),
                   "--echo-port", str(echo_port), "--echo-timeout", "0.05"])
    err = capsys.readouterr().err
    assert rc == 2
    assert "summary" in err
    assert "cannot finish --record" in err


def test_ctrl_c_on_a_real_serial_source_exits_0(tmp_path):
    """Regression: closing the port at shutdown made the reader thread fail and flipped a clean
    Ctrl-C into rc 2 about 40% of the time. A pyserial loop:// source stands in for the port."""
    for i in range(5):
        timer = threading.Timer(0.3, lambda: os.kill(os.getpid(), signal.SIGINT))
        timer.start()
        try:
            rc = cwb.main(["--port", "loop://", "--record", str(tmp_path / f"c{i}.csv"),
                           "--cmd-port", str(_free_port()), "--echo-port", str(_free_port()),
                           "--echo-timeout", "0.05"])
        finally:
            timer.cancel()
        assert rc == 0, f"run {i}"


def _sigint_when_reader_runs(cancel: threading.Event) -> None:
    """Wait (bounded, 3 s) until the bridge's line reader thread runs, then pause a fixed
    0.05 s and send SIGINT unless cancel is set. SIGINT is sent at the deadline even if the
    reader thread was never seen, so main always stops (a loop:// source never ends alone)."""
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not cancel.is_set():
        if any(t.name == "ctrl_wheel_lines" and t.is_alive() for t in threading.enumerate()):
            break
        time.sleep(0.01)
    time.sleep(0.05)
    if not cancel.is_set():
        os.kill(os.getpid(), signal.SIGINT)


def test_clean_ctrl_c_prints_nothing_after_the_summary(tmp_path, capsys):
    """Regression (FW #477): after a clean stop on a pyserial loop:// loopback source, the reader
    thread's failed read must not log 'line source stopped' after the summary. The SIGINT lands
    while the reader thread is live, so the port is closed mid-read."""
    for i in range(8):
        cmd_port, echo_port = _two_free_ports()
        cancel = threading.Event()
        helper = threading.Thread(target=_sigint_when_reader_runs, args=(cancel,), daemon=True)
        helper.start()
        try:
            rc = cwb.main(["--port", "loop://", "--record", str(tmp_path / f"clean{i}.csv"),
                           "--cmd-port", str(cmd_port), "--echo-port", str(echo_port),
                           "--echo-timeout", "0.05", "--duration", "10"])
        finally:
            cancel.set()
            helper.join(timeout=5.0)
        assert rc == 0, f"run {i}"
        time.sleep(0.3)
        err = capsys.readouterr().err
        lines = err.splitlines()
        summaries = [ln for ln in lines if ln.startswith("[ctrl_wheel_bridge] summary ")]
        assert len(summaries) == 1, f"run {i}: {err!r}"
        assert "line source stopped" not in err, f"run {i}: {err!r}"
        idx = lines.index(summaries[0])
        assert not [ln for ln in lines[idx + 1:] if ln.strip()], f"run {i}: {err!r}"


def test_midrun_source_failure_line_comes_before_the_summary(tmp_path, monkeypatch, capsys):
    """Over-suppression guard (FW #477): a real mid-run source failure is still logged,
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
    summaries = [i for i, ln in enumerate(lines) if ln.startswith("[ctrl_wheel_bridge] summary ")]
    stopped = [i for i, ln in enumerate(lines) if "line source stopped" in ln]
    assert len(summaries) == 1
    assert len(stopped) == 1
    assert "read failed: injected" in lines[stopped[0]]
    assert stopped[0] < summaries[0]
    assert not [ln for ln in lines[summaries[0] + 1:] if ln.strip()]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
