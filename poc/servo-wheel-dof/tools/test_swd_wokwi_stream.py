"""Tests for tools/wokwi-stream.sh (no Wokwi, localhost only).

Run on its own. `tools/` is outside the POC's main command, but a bare
`python3 -m pytest` from poc/servo-wheel-dof also collects this file:
    cd poc/servo-wheel-dof
    mamba run -n servo-wheel-dof python3 -m pytest -q tools/test_swd_wokwi_stream.py

Three kinds of test:
- argument and host-validation tests: run anywhere, no mamba needed;
- refused-port tests: a real connection attempt on localhost; skip without
  mamba and the esp32-dof env;
- fake-serial tests: the REAL script runs, so mamba still launches the python,
  but a `serial` shim (written under tmp_path, first on PYTHONPATH) replaces
  pyserial. Skip without mamba and the esp32-dof env.

Every stdout/stderr read in the streaming tests is guarded by select with a
deadline, so a regression fails the test instead of hanging the run.
"""

import json
import os
import select
import shutil
import signal
import socket
import subprocess
import time
from functools import lru_cache
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parent / "wokwi-stream.sh"
STRIP_VARS = ("WOKWI_HOST", "WOKWI_PORT", "BAUD", "PYTHONPATH", "FAKE_SCRIPT")
CAP = 64 * 1024

# Stand-in for pyserial, loaded through PYTHONPATH. It models
# SerialBase.read_until(expected, size): return at `expected`, at `size` bytes,
# or with the bytes read so far after a gap (b"" if none). A gap is what
# happens when the next chunk is not there yet (end of the list, or a gate file
# still missing). readline() is modelled as read_until(b"\n") that also returns
# at a gap; real pyserial IOBase.readline would not return on a newline-less
# stream without a gap, so this shim is more lenient there.
#
# Config (JSON file named by env FAKE_SCRIPT):
#   chunks:   list of hex strings, or {"hex": ..., "gate": path}; a gated chunk
#             is not delivered (a gap happens instead) until the gate exists;
#   end:      "idle" (gaps forever), "error" (SerialException once the chunks
#             are used up), "repeat" (cycle the chunks forever);
#   pid_file: write os.getpid() here when the port opens;
#   die_file: once this file exists, the next gap raises SerialException.
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
        if cfg.get("pid_file"):
            with open(cfg["pid_file"], "w", encoding="utf-8") as f:
                f.write(str(os.getpid()))
        self._all = list(cfg.get("chunks", []))
        self._items = list(self._all)
        self._end = cfg.get("end", "error")
        self._die = cfg.get("die_file")
        self._buf = b""
        self._pos = 0
        self.is_open = True

    def _pull(self):
        if not self._items and self._end == "repeat" and self._all:
            self._items = list(self._all)
            time.sleep(0.001)
        if not self._items:
            return False
        item = self._items[0]
        if isinstance(item, dict):
            gate = item.get("gate")
            if gate and not os.path.exists(gate):
                return False
            item = item["hex"]
        self._items.pop(0)
        self._buf = binascii.unhexlify(item)
        self._pos = 0
        return True

    def read_until(self, expected=b"\n", size=None):
        out = bytearray()
        while True:
            if self._pos >= len(self._buf) and not self._pull():
                break
            want = len(self._buf) - self._pos
            if size is not None:
                want = min(want, size - len(out))
            piece = self._buf[self._pos:self._pos + want]
            idx = piece.find(expected)
            if idx >= 0:
                cut = idx + len(expected)
                out += piece[:cut]
                self._pos += cut
                return bytes(out)
            out += piece
            self._pos += len(piece)
            if size is not None and len(out) >= size:
                return bytes(out)
        if not out:
            if self._die and os.path.exists(self._die):
                raise SerialException("fake die")
            if self._end == "error" and not self._items:
                raise SerialException("fake end of stream")
            time.sleep(0.01)
        return bytes(out)

    def readline(self):
        return self.read_until(b"\n", None)

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


def _shim(tmp_path, chunks=(), end="error", **extra):
    """Write the serial shim and its config under tmp_path; return env vars."""
    shim_dir = tmp_path / "shim"
    shim_dir.mkdir(exist_ok=True)
    (shim_dir / "serial.py").write_text(SHIM_SRC, encoding="utf-8")
    cfg = dict(extra)
    cfg.update({"chunks": list(chunks), "end": end})
    cfg_path = tmp_path / "fake.json"
    cfg_path.write_text(json.dumps(cfg), encoding="utf-8")
    return {"PYTHONPATH": str(shim_dir), "FAKE_SCRIPT": str(cfg_path)}


def _hex(data):
    return data.hex()


class _Run(object):
    """A running script with guarded reads. Output is accumulated in out/err."""

    def __init__(self, args, **env):
        self.proc = subprocess.Popen(
            _argv(args),
            env=_clean_env(**env),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
        )
        self.out = bytearray()
        self.err = bytearray()

    def read_line(self, timeout=15):
        """Next stdout line; fails (does not hang) if none arrives in time."""
        fd = self.proc.stdout.fileno()
        deadline = time.time() + timeout
        line = bytearray()
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise AssertionError(
                    "no complete stdout line within %ss; got %r" % (timeout, bytes(line)))
            if not select.select([fd], [], [], remaining)[0]:
                continue
            b = os.read(fd, 1)
            if not b:
                raise AssertionError("stdout closed; got %r" % bytes(line))
            line += b
            self.out += b
            if b == b"\n":
                return bytes(line)

    def quiet(self, seconds):
        """Assert that no stdout byte arrives for `seconds`."""
        fd = self.proc.stdout.fileno()
        ready = select.select([fd], [], [], seconds)[0]
        if ready:
            got = os.read(fd, 4096)
            self.out += got
            raise AssertionError("unexpected stdout within %ss: %r" % (seconds, got))

    def wait_err(self, marker, timeout=30):
        """Read stderr until `marker` appears (the shim's open line, say)."""
        fd = self.proc.stderr.fileno()
        deadline = time.time() + timeout
        while marker not in self.err:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise AssertionError("stderr marker %r not seen; got %r" % (marker, bytes(self.err)))
            if select.select([fd], [], [], remaining)[0]:
                chunk = os.read(fd, 4096)
                if not chunk:
                    raise AssertionError("stderr closed before %r; got %r" % (marker, bytes(self.err)))
                self.err += chunk

    def finish(self, timeout=30):
        """Wait for exit, collect the rest of both streams, return the exit code."""
        out_rest, err_rest = self.proc.communicate(timeout=timeout)
        self.out += out_rest
        self.err += err_rest
        return self.proc.returncode

    def kill(self):
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.proc.wait(timeout=10)
        self.proc.stdout.close()
        self.proc.stderr.close()


# ------------------------------------------------------------ argument tests


def test_script_is_executable():
    assert SCRIPT.is_file()
    assert os.access(SCRIPT, os.X_OK), "wokwi-stream.sh must have the executable bit"


def test_help_exits_0_and_documents_flags_env_and_output(tmp_path):
    r = _run(["--help"], tmp_path)
    out = r.stdout.decode("utf-8")
    assert r.returncode == 0, r.stderr
    for flag in ("--host", "--port", "--baud", "--help"):
        assert flag in out
    assert "WOKWI_HOST, WOKWI_PORT, BAUD" in out
    assert "empty value" in out
    assert "64 KiB" in out


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


@pytest.mark.parametrize(
    "host, reason",
    [
        ("localhost:4000", b"use --port for the port"),
        ("1.2.3.4:4000", b"use --port for the port"),
        ("fe80:1", b"use --port for the port"),
        ("[::1", b"unmatched"),
        ("[localhost]", b"invalid host"),
        ("[::1]x", b"invalid host"),
    ],
)
def test_host_with_port_or_broken_brackets_exits_2(tmp_path, host, reason):
    r = _run(["--host", host], tmp_path)
    assert r.returncode == 2, r.stderr
    assert reason in r.stderr
    assert b"Traceback" not in r.stderr


# ------------------------------------------- real refused-port connections


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


def test_refused_bracketed_ipv6_is_not_wrapped_twice(tmp_path):
    _need_pyserial_env()
    port = _free_port()
    r = _run(["--host", "[::1]", "--port", str(port)], tmp_path)
    err = r.stderr.decode("utf-8")
    assert r.returncode == 1, err
    assert "[::1]:%d" % port in err
    assert "[[" not in err


def test_refused_leading_dash_host_reaches_the_message(tmp_path):
    _need_pyserial_env()
    port = _free_port()
    r = _run(["--host=-x", "--port", str(port)], tmp_path)
    err = r.stderr.decode("utf-8")
    assert r.returncode == 1, err
    assert "-x:%d" % port in err


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


def test_bare_ipv6_host_is_bracketed_in_url(tmp_path):
    _need_pyserial_env()
    r = _run(["--host", "::1", "--port", "4321"], tmp_path, **_shim(tmp_path))
    assert b"fake-serial: open rfc2217://[::1]:4321 baud=115200" in r.stderr


def test_bracketed_ipv6_host_is_used_as_is(tmp_path):
    _need_pyserial_env()
    r = _run(["--host", "[::1]", "--port", "4321"], tmp_path, **_shim(tmp_path))
    assert b"fake-serial: open rfc2217://[::1]:4321 baud=115200" in r.stderr


# ------------------------------------------------- fake serial: stream contract


def test_partial_line_is_held_until_its_newline(tmp_path):
    _need_pyserial_env()
    gate = tmp_path / "gate"
    chunks = ["616263", {"hex": _hex(b"def\n"), "gate": str(gate)}]  # "abc", then "def\n"
    run = _Run([], **_shim(tmp_path, chunks=chunks, end="error"))
    try:
        run.wait_err(b"fake-serial: open")
        run.quiet(1.5)  # "abc" arrived with no newline: nothing may be written
        gate.write_text("go", encoding="utf-8")
        assert run.read_line() == b"abcdef\n"
        assert run.finish() == 1
    finally:
        run.kill()
    assert bytes(run.out) == b"abcdef\n"


def test_empty_read_with_pending_bytes_does_not_emit(tmp_path):
    _need_pyserial_env()
    run = _Run([], **_shim(tmp_path, chunks=[_hex(b"abc")], end="idle"))
    try:
        run.wait_err(b"fake-serial: open")
        run.quiet(1.5)  # gaps (empty reads) after "abc": still nothing written
    finally:
        run.kill()
    assert bytes(run.out) == b""


def test_each_line_is_flushed_before_the_next_arrives(tmp_path):
    _need_pyserial_env()
    gate = tmp_path / "gate"
    chunks = [_hex(b"abc\n"), {"hex": _hex(b"defg\n"), "gate": str(gate)}]
    run = _Run([], **_shim(tmp_path, chunks=chunks, end="error"))
    try:
        run.wait_err(b"fake-serial: open")
        # The first line must come out of the live pipe while the second is
        # still gated (proves -a "" streams and the write is flushed).
        assert run.read_line() == b"abc\n"
        gate.write_text("go", encoding="utf-8")
        assert run.read_line() == b"defg\n"
        assert run.finish() == 1
    finally:
        run.kill()


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


def test_complete_line_below_the_cap_is_whole(tmp_path):
    _need_pyserial_env()
    line = b"x" * 60000 + b"\n"
    r = _run([], tmp_path, **_shim(tmp_path, chunks=[_hex(line)], end="error"))
    assert r.stdout == line
    assert b"split" not in r.stderr


def test_newline_less_run_at_the_cap_is_split(tmp_path):
    _need_pyserial_env()
    r = _run([], tmp_path, **_shim(tmp_path, chunks=[_hex(b"x" * CAP)], end="error"))
    assert r.stdout == b"x" * CAP + b"\n"
    assert r.stderr.count(b"split") == 1


def test_newline_less_run_one_past_the_cap_is_split_once_more(tmp_path):
    _need_pyserial_env()
    r = _run([], tmp_path, **_shim(tmp_path, chunks=[_hex(b"x" * (CAP + 1))], end="error"))
    assert r.stdout == b"x" * CAP + b"\n" + b"x" + b"\n"
    assert r.stderr.count(b"split") == 1


def test_line_longer_than_the_cap_is_split_and_the_rest_follows(tmp_path):
    _need_pyserial_env()
    line = b"x" * (CAP + 4464) + b"\n"
    r = _run([], tmp_path, **_shim(tmp_path, chunks=[_hex(line)], end="error"))
    assert r.stdout == b"x" * CAP + b"\n" + b"x" * 4464 + b"\n"


def test_stdout_write_failure_has_its_own_message(tmp_path):
    _need_pyserial_env()
    if not os.path.exists("/dev/full"):
        pytest.skip("needs /dev/full (Linux)")
    env = _shim(tmp_path, chunks=[_hex(b"x\n")], end="error")
    with open("/dev/full", "wb") as full:
        r = subprocess.run(
            _argv([]),
            env=_clean_env(**env),
            stdout=full,
            stderr=subprocess.PIPE,
            timeout=60,
        )
    assert r.returncode == 1, r.stderr
    assert b"cannot write to stdout" in r.stderr
    assert b"serial error" not in r.stderr


# ---------------------------------------------------------- Ctrl+C and pipes


def test_ctrl_c_python_layer_returns_130(tmp_path):
    """SIGINT to the python process only: Python's own KeyboardInterrupt path."""
    _need_pyserial_env()
    pid_file = tmp_path / "pid"
    env = _shim(tmp_path, chunks=[_hex(b"hello\n")], end="idle", pid_file=str(pid_file))
    run = _Run([], **env)
    try:
        assert run.read_line() == b"hello\n"
        pid = int(pid_file.read_text(encoding="utf-8"))
        os.kill(pid, signal.SIGINT)
        rc = run.finish()
    finally:
        run.kill()
    assert rc == 130, bytes(run.err)
    assert run.err.count(b"stopped by user") == 1
    assert b"Traceback" not in run.err


def test_ctrl_c_bash_trap_maps_to_130(tmp_path):
    """SIGINT to the bash pid only: the trap must turn the exit into 130."""
    _need_pyserial_env()
    die = tmp_path / "die"
    env = _shim(tmp_path, chunks=[_hex(b"hello\n")], end="idle", die_file=str(die))
    run = _Run([], **env)
    try:
        assert run.read_line() == b"hello\n"
        run.proc.send_signal(signal.SIGINT)  # bash only; python keeps streaming
        die.write_text("stop", encoding="utf-8")  # python now fails with rc 1
        rc = run.finish()
    finally:
        run.kill()
    assert rc == 130, bytes(run.err)
    assert b"fake die" in run.err


def test_ctrl_c_on_the_group_exits_130(tmp_path):
    """SIGINT to the whole process group, as a terminal Ctrl+C does."""
    _need_pyserial_env()
    env = _shim(tmp_path, chunks=[_hex(b"hello\n")], end="idle")
    run = _Run([], **env)
    try:
        assert run.read_line() == b"hello\n"
        os.killpg(run.proc.pid, signal.SIGINT)
        rc = run.finish()
    finally:
        run.kill()
    assert rc == 130, bytes(run.err)
    assert run.err.count(b"stopped by user") == 1


def test_closed_downstream_exits_0_quietly(tmp_path):
    _need_pyserial_env()
    run = _Run([], **_shim(tmp_path, chunks=[_hex(b"line\n")], end="repeat"))
    try:
        assert run.read_line() == b"line\n"
        run.proc.stdout.close()  # downstream goes away while data keeps coming
        rc = run.proc.wait(timeout=30)
        run.err += run.proc.stderr.read()
    finally:
        run.kill()
    assert rc == 0, bytes(run.err)
    assert b"Exception ignored" not in run.err
    assert b"Traceback" not in run.err
    assert b"BrokenPipeError" not in run.err
