"""Tests for tools/wokwi-stream.sh (no Wokwi, localhost only).

Run on its own (tools/ is not in the POC's main pytest command):
    cd poc/servo-wheel-dof
    mamba run -n servo-wheel-dof python3 -m pytest -q tools/test_swd_wokwi_stream.py

Tests that need a real connection attempt through mamba and the esp32-dof env
skip with a reason when mamba or that env is missing. Nothing here opens a
Wokwi session; a refused port on localhost is used for the connection path.
"""

import os
import shutil
import socket
import subprocess
from functools import lru_cache
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parent / "wokwi-stream.sh"
ENV_VARS = ("WOKWI_HOST", "WOKWI_PORT", "BAUD")


def _clean_env(**extra):
    env = {k: v for k, v in os.environ.items() if k not in ENV_VARS}
    env.update(extra)
    return env


def _run(args, cwd, **env):
    return subprocess.run(
        ["bash", str(SCRIPT), *args],
        cwd=str(cwd),
        env=_clean_env(**env),
        capture_output=True,
        text=True,
        timeout=60,
    )


def _free_port():
    """Return a localhost port that nothing listens on (bind, read, close)."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@lru_cache(maxsize=1)
def _pyserial_env_problem():
    """Return None if mamba and the esp32-dof env can import serial, else a reason."""
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


def test_script_is_executable():
    assert SCRIPT.is_file()
    assert os.access(SCRIPT, os.X_OK), "wokwi-stream.sh must have the executable bit"


def test_help_exits_0_and_lists_flags(tmp_path):
    r = _run(["--help"], tmp_path)
    assert r.returncode == 0, r.stderr
    for flag in ("--host", "--port", "--baud", "--help"):
        assert flag in r.stdout
    assert "WOKWI_PORT" in r.stdout


def test_unknown_flag_exits_2(tmp_path):
    r = _run(["--no-such-flag"], tmp_path)
    assert r.returncode == 2
    assert "unknown argument" in r.stderr


@pytest.mark.parametrize("bad_port", ["0", "65536", "70000", "abc", "-1", "99999999999999999999"])
def test_bad_port_exits_2(tmp_path, bad_port):
    r = _run(["--port", bad_port], tmp_path)
    assert r.returncode == 2
    assert "invalid port" in r.stderr


@pytest.mark.parametrize("bad_baud", ["0", "fast"])
def test_bad_baud_exits_2(tmp_path, bad_baud):
    r = _run(["--baud", bad_baud], tmp_path)
    assert r.returncode == 2
    assert "invalid baud" in r.stderr


def test_bad_env_port_exits_2(tmp_path):
    r = _run([], tmp_path, WOKWI_PORT="abc")
    assert r.returncode == 2
    assert "invalid port" in r.stderr


def test_refused_port_exits_1_with_one_line(tmp_path):
    _need_pyserial_env()
    port = _free_port()
    r = _run(["--port", str(port)], tmp_path)
    assert r.returncode == 1, r.stderr
    assert "Traceback" not in r.stderr
    lines = [ln for ln in r.stderr.splitlines() if ln.strip()]
    assert len(lines) == 1, r.stderr
    assert "localhost:%d" % port in lines[0]
    assert r.stdout == ""


def test_env_port_is_honoured(tmp_path):
    _need_pyserial_env()
    port = _free_port()
    r = _run([], tmp_path, WOKWI_PORT=str(port))
    assert r.returncode == 1, r.stderr
    assert "localhost:%d" % port in r.stderr


def test_flag_port_overrides_env_port(tmp_path):
    _need_pyserial_env()
    env_port = _free_port()
    flag_port = _free_port()
    r = _run(["--port", str(flag_port)], tmp_path, WOKWI_PORT=str(env_port))
    assert r.returncode == 1, r.stderr
    assert "localhost:%d" % flag_port in r.stderr
    assert "localhost:%d" % env_port not in r.stderr


def test_works_from_other_cwd(tmp_path):
    # Same help output from an unrelated directory: no cwd-relative paths.
    r = _run(["-h"], tmp_path)
    assert r.returncode == 0
    assert "Usage: wokwi-stream.sh" in r.stdout
