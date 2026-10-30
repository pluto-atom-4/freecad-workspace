"""Tests for tools/wokwi-stream.sh (no Wokwi, localhost only).

Run on its own. `tools/` is outside the POC's main command, but a bare
`python3 -m pytest` from poc/servo-wheel-dof also collects this file:
    cd poc/servo-wheel-dof
    mamba run -n servo-wheel-dof python3 -m pytest -q tools/test_swd_wokwi_stream.py

Three kinds of test:
- argument tests: run anywhere, no mamba needed;
- refused-port tests: real connection attempt on localhost; skip without mamba
  and the esp32-dof env;
- fake-serial tests: the REAL script runs, so mamba still launches the python,
  but a `serial` shim (written under tmp_path, first on PYTHONPATH) replaces
  pyserial. The shim prints the URL and baud the script built and feeds canned
  bytes. Skip without mamba and the esp32-dof env, like the refused-port tests.
"""

import json
import os
import select
import shutil
import signal
import socket
import subprocess
from functools import lru_cache
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parent / "wokwi-stream.sh"
STRIP_VARS = ("WOKWI_HOST", "WOKWI_PORT", "BAUD", "PYTHONPATH", "FAKE_SCRIPT")

# Stand-in for pyserial. serial_for_url() reports what the script asked for on
# stderr, then returns canned chunks from the JSON file named by FAKE_SCRIPT:
#   chunks: list of hex strings, or {"hex": ..., "gate": path} where readline
#           waits until the gate file exists before returning that chunk;
#   end: "idle" (empty reads forever), "error" (raise SerialException after the
#        chunks), "repeat" (cycle the chunks forever, for a closed-pipe test).
SHIM_SRC = r'''
import binascii
import json
import os
import sys
import time


class SerialException(Exception):
    pass


class _Fake(object):
    def __init__(self, url, baudrate):
        sys.stderr.write("fake-serial: open %s baud=%s\n" % (url, baudrate))
        sys.stderr.flush()
        with open(os.environ["FAKE_SCRIPT"], "r", encoding="utf-8") as f:
            cfg = json.load(f)
        self._all = list(cfg.get("chunks", []))
        self._items = list(self._all)
        self._end = cfg.get("end", "idle")
        self.is_open = True

    def readline(self):
        if not self._items and self._end == "repeat" and self._all:
            time.sleep(0.001)
            self._items = list(self._all)
        if self._items:
            item = self._items.pop(0)
            if isinstance(item, dict):
                gate = item.get("gate")
                deadline = time.time() + 15
                while gate and not os.path.exists(gate) and time.time() < deadline:
                    time.sleep(0.02)
                item = item["hex"]
            return binascii.unhexlify(item)
        if self._end == "error":
            raise SerialException("fake end of stream")
        time.sleep(0.01)
        return b""

    def close(self):
        self.is_open = False
        sys.stderr.write("fake-serial: closed\n")
        sys.stderr.flush()


def serial_for_url(url, baudrate=9600, timeout=None, **kwargs):
    return _Fake(url, baudrate)
'''


def _clean_env(**extra):
    env = {k: v for k, v in os.environ.items() if k not in STRIP_VARS}
    env.update(extra)
    return env


def _argv(args):
    return ["bash", str(SCRIPT), *args]


def _run(args, cwd, **env):
    return subprocess.run(
        _argv(args),
        cwd=str(cwd),
        env=_clean_env(**env),
        capture_output=True,
        timeout=60,
    )


def _popen(args, **env):
    return subprocess.Popen(
        _argv(args),
        env=_clean_env(**env),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )


def _free_port():
    """Return a localhost port that nothing listens on (bind, read, close)."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@lru_cache(maxsize=1)
def _pyserial_env_problem():
    """Return None if mamba and the esp32-dof env can run python, else a reason."""
    if shutil.which("mamba") is None:
        return "mamba not on PATH"
    try:
        probe = subprocess.run(
            ["mamba", "run", "-n", "esp32-dof", "python3", "-c", "import serial"],
            capture_output=True,
            text=True,
            timeout=120,
        )
    except (OSError, subprocess.TimeoutExpired) as e:
        return "mamba probe failed: %s" % e
    if probe.returncode != 0:
        return "mamba env esp32-dof with pyserial is not available"
    return None


def _need_pyserial_env():
    reason = _pyserial_env_problem()
    if reason:
        pytest.skip(reason)


def _shim(tmp_path, chunks=(), end="error"):
    """Write the serial shim and its config under tmp_path; return env vars."""
    shim_dir = tmp_path / "shim"
    shim_dir.mkdir(exist_ok=True)
    (shim_dir / "serial.py").write_text(SHIM_SRC, encoding="utf-8")
    cfg = tmp_path / "fake.json"
    cfg.write_text(json.dumps({"chunks": list(chunks), "end": end}), encoding="utf-8")
    return {"PYTHONPATH": str(shim_dir), "FAKE_SCRIPT": str(cfg)}


def _hex(data):
    return data.hex()


# ---------------------------------------------------------------- argument tests


def test_script_is_executable():
    assert SCRIPT.is_file()
    assert os.access(SCRIPT, os.X_OK), "wokwi-stream.sh must have the executable bit"


def test_help_exits_0_and_documents_flags_and_empty_env(tmp_path):
    r = _run(["--help"], tmp_path)
    out = r.stdout.decode("utf-8")
    assert r.returncode == 0, r.stderr
    for flag in ("--host", "--port", "--baud", "--help"):
        assert flag in out
    assert "WOKWI_HOST, WOKWI_PORT, BAUD" in out
    assert "empty value" in out


def test_unknown_flag_exits_2(tmp_path):
    r = _run(["--no-such-flag"], tmp_path)
    assert r.returncode == 2
    assert b"unknown argument" in r.stderr


@pytest.mark.parametrize(
    "args",
    [
        ["--port", "0"],
        ["--port", "65536"],
        ["--port=70000"],
        ["--port", "abc"],
        ["--port", "-1"],
        ["--port", "99999999999999999999"],
    ],
)
def test_bad_port_exits_2(tmp_path, args):
    r = _run(args, tmp_path)
    assert r.returncode == 2
    assert b"invalid port" in r.stderr


@pytest.mark.parametrize("args", [["--baud", "0"], ["--baud=fast"], ["--baud", "1000000000"]])
def test_bad_baud_exits_2(tmp_path, args):
    r = _run(args, tmp_path)
    assert r.returncode == 2
    assert b"invalid baud" in r.stderr


def test_bad_env_port_exits_2(tmp_path):
    r = _run([], tmp_path, WOKWI_PORT="abc")
    assert r.returncode == 2
    assert b"invalid port" in r.stderr


@pytest.mark.parametrize("args", [["--port"], ["--host"], ["--baud"]])
def test_missing_value_exits_2(tmp_path, args):
    r = _run(args, tmp_path)
    assert r.returncode == 2
    assert b"needs a value" in r.stderr


@pytest.mark.parametrize("args", [["--host", ""], ["--host="]])
def test_empty_host_exits_2(tmp_path, args):
    r = _run(args, tmp_path)
    assert r.returncode == 2
    assert b"host must not be empty" in r.stderr


# ------------------------------------------------- real refused-port connections


def test_refused_port_one_line_from_other_cwd(tmp_path):
    _need_pyserial_env()
    port = _free_port()
    r = _run(["--port", str(port)], tmp_path)  # cwd is tmp_path, not tools/
    err = r.stderr.decode("utf-8")
    assert r.returncode == 1, err
    assert "Traceback" not in err
    lines = [ln for ln in err.splitlines() if ln.strip()]
    assert len(lines) == 1, err
    assert "localhost:%d" % port in lines[0]
    assert r.stdout == b""


def test_refused_ipv6_names_bracketed_host(tmp_path):
    _need_pyserial_env()
    port = _free_port()
    r = _run(["--host", "::1", "--port", str(port)], tmp_path)
    err = r.stderr.decode("utf-8")
    assert r.returncode == 1, err
    assert "Traceback" not in err
    assert "[::1]:%d" % port in err


# ------------------------------------------------ fake serial: env, flags, URL


@pytest.mark.parametrize(
    "args",
    [
        ["--host=127.0.0.1", "--port=4321", "--baud=9600"],
        ["--host", "127.0.0.1", "--port", "4321", "--baud", "9600"],
    ],
    ids=["equals-form", "space-form"],
)
def test_flag_forms_build_the_url(tmp_path, args):
    _need_pyserial_env()
    r = _run(args, tmp_path, **_shim(tmp_path))
    assert b"fake-serial: open rfc2217://127.0.0.1:4321 baud=9600" in r.stderr


def test_env_host_port_baud_are_honoured(tmp_path):
    _need_pyserial_env()
    env = _shim(tmp_path)
    env.update(WOKWI_HOST="127.0.0.1", WOKWI_PORT="4321", BAUD="9600")
    r = _run([], tmp_path, **env)
    assert b"fake-serial: open rfc2217://127.0.0.1:4321 baud=9600" in r.stderr


def test_flags_beat_env(tmp_path):
    _need_pyserial_env()
    env = _shim(tmp_path)
    env.update(WOKWI_HOST="10.0.0.9", WOKWI_PORT="5555", BAUD="4800")
    r = _run(["--host", "127.0.0.1", "--port=4321", "--baud=9600"], tmp_path, **env)
    assert b"fake-serial: open rfc2217://127.0.0.1:4321 baud=9600" in r.stderr


def test_empty_env_counts_as_unset(tmp_path):
    _need_pyserial_env()
    env = _shim(tmp_path)
    env.update(WOKWI_HOST="", WOKWI_PORT="", BAUD="")
    r = _run([], tmp_path, **env)
    assert b"fake-serial: open rfc2217://localhost:4000 baud=115200" in r.stderr


def test_ipv6_host_is_bracketed_in_url(tmp_path):
    _need_pyserial_env()
    r = _run(["--host", "::1", "--port", "4321"], tmp_path, **_shim(tmp_path))
    assert b"fake-serial: open rfc2217://[::1]:4321 baud=115200" in r.stderr


# ------------------------------------------------- fake serial: stream contract


def test_partial_line_is_held_until_its_newline(tmp_path):
    _need_pyserial_env()
    gate = tmp_path / "gate"
    chunks = ["616263", {"hex": _hex(b"def\n"), "gate": str(gate)}]  # "abc", then "def\n"
    proc = _popen([], **_shim(tmp_path, chunks=chunks, end="error"))
    try:
        ready, _, _ = select.select([proc.stdout.fileno()], [], [], 1.5)
        assert not ready, "partial line 'abc' was written before its newline arrived"
        gate.write_text("go", encoding="utf-8")
        out, err = proc.communicate(timeout=30)
    finally:
        if proc.poll() is None:
            proc.kill()
    assert out == b"abcdef\n"
    assert proc.returncode == 1


def test_bytes_are_copied_faithfully_and_pending_flushed_at_exit(tmp_path):
    _need_pyserial_env()
    chunks = [
        _hex(b"abc"),
        _hex(b"def\n"),
        _hex(b"\xff\xfe\n"),  # non-ASCII bytes pass through unchanged
        _hex(b"partial"),  # no newline: flushed with one at exit
    ]
    r = _run([], tmp_path, **_shim(tmp_path, chunks=chunks, end="error"))
    assert r.returncode == 1, r.stderr
    assert r.stdout == b"abcdef\n\xff\xfe\npartial\n"
    assert b"serial error" in r.stderr


def test_overlong_line_is_split_with_one_note(tmp_path):
    _need_pyserial_env()
    big = _hex(b"x" * 70000)
    r = _run([], tmp_path, **_shim(tmp_path, chunks=[big, big], end="error"))
    assert r.stdout == b"x" * 70000 + b"\n" + b"x" * 70000 + b"\n"
    assert r.stderr.count(b"line longer than") == 1


def test_ctrl_c_exits_130_with_one_stop_line(tmp_path):
    _need_pyserial_env()
    proc = _popen([], **_shim(tmp_path, chunks=[_hex(b"hello\n")], end="idle"))
    try:
        first = proc.stdout.readline()
        assert first == b"hello\n"
        os.killpg(proc.pid, signal.SIGINT)
        out, err = proc.communicate(timeout=30)
    finally:
        if proc.poll() is None:
            proc.kill()
    assert proc.returncode == 130, err
    assert err.count(b"stopped by user") == 1
    assert b"Traceback" not in err


def test_closed_downstream_exits_0_quietly(tmp_path):
    _need_pyserial_env()
    proc = _popen([], **_shim(tmp_path, chunks=[_hex(b"line\n")], end="repeat"))
    try:
        assert proc.stdout.readline() == b"line\n"
        proc.stdout.close()  # downstream goes away while data keeps coming
        err = proc.stderr.read()
        proc.wait(timeout=30)
    finally:
        if proc.poll() is None:
            proc.kill()
    assert proc.returncode == 0, err
    assert b"Exception ignored" not in err
    assert b"Traceback" not in err
    assert b"BrokenPipeError" not in err
