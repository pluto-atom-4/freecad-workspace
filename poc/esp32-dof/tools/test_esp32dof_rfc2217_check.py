"""Shell-launcher tests for tools/wokwi-rfc2217-check.sh.

These are shell-launcher tests only, no simulator: nothing here starts a real
bridge, a real mamba env or a Wokwi simulation. Where a run must reach the
mamba call, a stub `mamba` script on PATH records its argv and PWD and exits 37.
"""

import os
import shutil
import subprocess
from pathlib import Path

import pytest

SCRIPT = Path(__file__).parent / "wokwi-rfc2217-check.sh"
BASH = shutil.which("bash") or "/bin/bash"
STUB_EXIT = 37
TOKEN = "SECRETVALUE123"


def run(args, *, env_update=None, path=None):
    env = dict(os.environ)
    if path is not None:
        env["PATH"] = path
    if env_update:
        env.update(env_update)
    return subprocess.run(
        [BASH, str(SCRIPT), *args],
        capture_output=True,
        text=True,
        env=env,
        timeout=60,
    )


def make_stub_mamba(tmp_path):
    bindir = tmp_path / "stubbin"
    bindir.mkdir()
    out = tmp_path / "stub_argv.txt"
    stub = bindir / "mamba"
    stub.write_text(
        "#!/bin/bash\n"
        "{ printf 'PWD=%s\\n' \"$PWD\"; printf '%s\\n' \"$@\"; } > \"$STUB_OUT\"\n"
        f"exit {STUB_EXIT}\n"
    )
    stub.chmod(0o755)
    path = str(bindir) + os.pathsep + os.environ.get("PATH", "")
    return path, out


def make_fw(tmp_path, with_monitor=True):
    fw = tmp_path / "fw"
    if with_monitor:
        (fw / "poc" / "esp32-dof" / "monitor").mkdir(parents=True)
    else:
        fw.mkdir()
    return fw


def read_stub(out):
    lines = out.read_text().splitlines()
    pwd = lines[0][len("PWD="):]
    return pwd, lines[1:]


def test_help_exit0_mentions_guide():
    r = run(["--help"])
    assert r.returncode == 0
    assert "--guide" in r.stdout


def test_guide_exit0_and_content():
    r = run(["--guide"], env_update={"WOKWI_CLI_TOKEN": TOKEN})
    assert r.returncode == 0
    assert "TTL_SERVO_DOF_FRAMES" in r.stdout
    assert "SIM-ONLY" in r.stdout
    assert "\x1b" not in r.stdout
    assert "\x1b" not in r.stderr


def test_guide_no_escape_with_no_color():
    r = run(["--guide"], env_update={"NO_COLOR": "1"})
    assert r.returncode == 0
    assert "\x1b" not in r.stdout


def test_guide_never_prints_token():
    r = run(["--guide"], env_update={"WOKWI_CLI_TOKEN": TOKEN})
    assert r.returncode == 0
    assert TOKEN not in r.stdout
    assert TOKEN not in r.stderr


def test_guide_runs_nothing(tmp_path):
    path, out = make_stub_mamba(tmp_path)
    r = run(["--guide"], path=path, env_update={"STUB_OUT": str(out)})
    assert r.returncode == 0
    assert not out.exists()


def test_dry_run_prints_command_and_writes_nothing(tmp_path):
    path, out = make_stub_mamba(tmp_path)
    fw = make_fw(tmp_path)
    rec = tmp_path / "rec.csv"
    r = run(
        ["--dry-run", "--record", str(rec)],
        path=path,
        env_update={"FW_DIR": str(fw), "STUB_OUT": str(out)},
    )
    assert r.returncode == 0
    assert "mamba run -a" in r.stdout
    assert "dof_wheel_bridge.py" in r.stdout
    assert "--echo-text" in r.stdout
    assert "\x1b" not in r.stdout
    assert not rec.exists()
    assert not out.exists()


def test_dry_run_passthrough_after_dashdash(tmp_path):
    fw = make_fw(tmp_path)
    r = run(
        ["--dry-run", "--", "--foo", "bar baz"],
        env_update={"FW_DIR": str(fw)},
    )
    assert r.returncode == 0
    assert "--foo" in r.stdout
    assert "bar\\ baz" in r.stdout  # printed shell-quoted (printf %q)


def test_dry_run_record_equals_form(tmp_path):
    fw = make_fw(tmp_path)
    rec = tmp_path / "eq.csv"
    r = run(
        ["--dry-run", f"--record={rec}"],
        env_update={"FW_DIR": str(fw)},
    )
    assert r.returncode == 0
    assert str(rec) in r.stdout
    assert not rec.exists()


@pytest.mark.parametrize(
    "args",
    [
        ["--bogus"],
        ["stray"],
        ["--core", "xyz"],
        ["--core=xyz"],
        ["--duration", "0"],
        ["--duration", "abc"],
        ["--duration=0"],
        ["--port", ""],
        ["--port"],
        ["--record", ""],
    ],
)
def test_bad_arguments_exit2(args, tmp_path):
    path, out = make_stub_mamba(tmp_path)
    r = run(args, path=path, env_update={"STUB_OUT": str(out)})
    assert r.returncode == 2
    assert "Usage" in r.stderr
    assert not out.exists()


def test_record_exists_without_overwrite_exits1(tmp_path):
    path, out = make_stub_mamba(tmp_path)
    fw = make_fw(tmp_path)
    rec = tmp_path / "exists.csv"
    rec.write_text("keep me\n")
    r = run(
        ["--record", str(rec)],
        path=path,
        env_update={"FW_DIR": str(fw), "STUB_OUT": str(out)},
    )
    assert r.returncode == 1
    assert "FATAL" in r.stderr
    assert not out.exists()
    assert rec.read_text() == "keep me\n"


def test_overwrite_reaches_bridge_with_passthrough(tmp_path):
    path, out = make_stub_mamba(tmp_path)
    fw = make_fw(tmp_path)
    rec = tmp_path / "exists.csv"
    rec.write_text("old\n")
    r = run(
        ["--overwrite", "--record", str(rec), "--", "--extra-x", "y"],
        path=path,
        env_update={"FW_DIR": str(fw), "STUB_OUT": str(out)},
    )
    assert r.returncode == STUB_EXIT
    pwd, argv = read_stub(out)
    assert Path(pwd).resolve() == (fw / "poc" / "esp32-dof" / "monitor").resolve()
    assert argv == [
        "run",
        "-a",
        "",
        "-n",
        "esp32-dof",
        "python3",
        "dof_wheel_bridge.py",
        "--port",
        "rfc2217://localhost:4000",
        "--core",
        "lqr",
        "--duration",
        "120",
        "--record",
        str(rec),
        "--echo-text",
        "--extra-x",
        "y",
    ]


def test_missing_monitor_dir_exits1(tmp_path):
    path, out = make_stub_mamba(tmp_path)
    fw = make_fw(tmp_path, with_monitor=False)
    r = run(
        ["--record", str(tmp_path / "new.csv")],
        path=path,
        env_update={"FW_DIR": str(fw), "STUB_OUT": str(out)},
    )
    assert r.returncode == 1
    assert "FATAL" in r.stderr
    assert not out.exists()


def test_mamba_missing_exits1(tmp_path):
    # Empty PATH: the script before the mamba check uses bash builtins only.
    empty = tmp_path / "empty"
    empty.mkdir()
    fw = make_fw(tmp_path)
    rec = tmp_path / "new.csv"
    r = run(
        ["--record", str(rec)],
        path=str(empty),
        env_update={"FW_DIR": str(fw)},
    )
    assert r.returncode == 1
    assert "FATAL" in r.stderr
    assert "mamba" in r.stderr
    assert not rec.exists()
