"""Shell-launcher tests for tools/wokwi-rfc2217-check.sh.

These are shell-launcher tests only, no simulator: nothing here starts a real
bridge, a real mamba env or a Wokwi simulation. Where a run must reach the
mamba call, a stub `mamba` script on PATH records its argv and PWD and exits 37.
Every stub PATH also carries a stub `ss`, silent unless a test asks for a
listener, so the host's real port 4000 is never read. Colour is checked on a
real pseudo-terminal (stdlib pty) where available, and on a pipe otherwise.
"""

import os
import select
import shlex
import shutil
import subprocess
import time
from pathlib import Path

import pytest

try:
    import pty
except ImportError:  # pragma: no cover - platform without pty
    pty = None

SCRIPT = Path(__file__).parent / "wokwi-rfc2217-check.sh"
REPO_FW = Path(__file__).resolve().parents[3]
BASH = shutil.which("bash") or "/bin/bash"
STUB_EXIT = 37
PTY_TIMEOUT_S = 30
TOKEN = "SECRETVALUE123"
LISTEN_HEADER = "State Recv-Q Send-Q Local Address:Port Peer Address:Port"


def run(args, *, env_update=None, path=None, cwd=None, drop=()):
    env = dict(os.environ)
    for key in drop:
        env.pop(key, None)
    if path is not None:
        env["PATH"] = path
    if env_update:
        env.update(env_update)
    return subprocess.run(
        [BASH, str(SCRIPT), *args],
        capture_output=True,
        text=True,
        env=env,
        cwd=cwd,
        timeout=60,
    )


def run_on_pty(args, *, env_update=None, drop=()):
    if pty is None:
        pytest.skip("pty module unavailable")
    try:
        master, slave = pty.openpty()
    except OSError as exc:
        pytest.skip(f"cannot open a pty: {exc}")
    env = dict(os.environ)
    for key in drop:
        env.pop(key, None)
    if env_update:
        env.update(env_update)
    proc = None
    slave_open = True
    try:
        proc = subprocess.Popen(
            [BASH, str(SCRIPT), *args],
            stdin=subprocess.DEVNULL,
            stdout=slave,
            stderr=subprocess.PIPE,
            env=env,
        )
        os.close(slave)
        slave_open = False
        chunks = []
        deadline = time.monotonic() + PTY_TIMEOUT_S
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                proc.kill()
                proc.wait()
                pytest.fail(f"pty run did not finish within {PTY_TIMEOUT_S} s")
            ready, _, _ = select.select([master], [], [], min(remaining, 1.0))
            if not ready:
                continue
            try:
                data = os.read(master, 4096)
            except OSError:  # EIO once the child closed the slave
                break
            if not data:
                break
            chunks.append(data)
        proc.wait(timeout=PTY_TIMEOUT_S)
        out = b"".join(chunks).decode("utf-8", errors="replace")
        err = proc.stderr.read().decode("utf-8", errors="replace")
        return proc.returncode, out, err
    finally:
        if slave_open:
            os.close(slave)
        os.close(master)
        if proc is not None and proc.poll() is None:
            proc.kill()
            proc.wait()


def _write_stub(path_obj, text):
    path_obj.write_text(text)
    path_obj.chmod(0o755)


def make_stub_mamba(tmp_path):
    """Return (PATH, argv_file). The bindir also holds a silent stub ss."""
    bindir = tmp_path / "stubbin"
    bindir.mkdir()
    out = tmp_path / "stub_argv.txt"
    _write_stub(
        bindir / "mamba",
        "#!/bin/bash\n"
        "{ printf 'PWD=%s\\n' \"$PWD\"; printf '%s\\n' \"$@\"; } > \"$STUB_OUT\"\n"
        f"exit {STUB_EXIT}\n",
    )
    _write_stub(bindir / "ss", "#!/bin/bash\nexit 0\n")
    path = str(bindir) + os.pathsep + os.environ.get("PATH", "")
    return path, out


def make_stub_ss(tmp_path, path, listening):
    """Prefix PATH with a stub ss that prints a LISTEN line for *:4000."""
    ssdir = tmp_path / "ssbin"
    ssdir.mkdir(exist_ok=True)
    body = LISTEN_HEADER + "\n"
    if listening:
        body += "LISTEN 0 4096 *:4000 *:*\n"
    _write_stub(ssdir / "ss", "#!/bin/bash\ncat <<'SSEOF'\n" + body + "SSEOF\n")
    return str(ssdir) + os.pathsep + path


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
    assert "--opt=value" in r.stdout
    assert "--stdin" in r.stdout
    assert "--text-log" in r.stdout


def test_guide_exit0_and_content():
    r = run(["--guide"], env_update={"WOKWI_CLI_TOKEN": TOKEN})
    assert r.returncode == 0
    assert "TTL_SERVO_DOF_FRAMES" in r.stdout
    assert "SIM-ONLY" in r.stdout
    assert "--overwrite" in r.stdout
    assert "reads BRC_DIR and FW_DIR" in r.stdout
    assert "\x1b" not in r.stdout
    assert "\x1b" not in r.stderr


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


def test_guide_wins_over_dry_run():
    r = run(["--dry-run", "--guide"])
    assert r.returncode == 0
    assert "1 Env" in r.stdout
    assert "Dry run:" not in r.stdout


def test_guide_colour_on_pty_when_no_color_unset():
    rc, out, _ = run_on_pty(["--guide"], drop=("NO_COLOR",))
    assert rc == 0
    assert "\x1b[1;36m" in out
    assert "\x1b[32m" in out


def test_guide_no_escape_on_pty_when_no_color_set():
    rc, out, _ = run_on_pty(["--guide"], env_update={"NO_COLOR": "1"})
    assert rc == 0
    assert "\x1b" not in out


def test_guide_colour_on_pty_when_no_color_empty():
    rc, out, _ = run_on_pty(["--guide"], env_update={"NO_COLOR": ""})
    assert rc == 0
    assert "\x1b[1;36m" in out


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
    assert "performs none of them" in r.stdout
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
    assert shlex.quote(str(rec)) in r.stdout
    assert not rec.exists()


def test_dry_run_relative_record_resolves_against_caller_cwd(tmp_path):
    fw = make_fw(tmp_path)
    r = run(
        ["--dry-run", "--record", "rel.csv"],
        env_update={"FW_DIR": str(fw)},
        cwd=str(tmp_path),
    )
    assert r.returncode == 0
    expected = Path(tmp_path).resolve() / "rel.csv"
    assert shlex.quote(str(expected)) in r.stdout


def test_dry_run_default_fw_dir_derived_from_script():
    r = run(["--dry-run"], drop=("FW_DIR",))
    assert r.returncode == 0
    expected = REPO_FW / "poc" / "esp32-dof" / "monitor"
    assert shlex.quote(str(expected)) in r.stdout


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
        ["--port", "--dry-run"],
        ["--record", "--overwrite"],
    ],
)
def test_bad_arguments_exit2(args, tmp_path):
    path, out = make_stub_mamba(tmp_path)
    r = run(args, path=path, env_update={"STUB_OUT": str(out)})
    assert r.returncode == 2
    assert "Usage" in r.stderr
    assert not out.exists()


@pytest.mark.parametrize(
    "passthrough",
    [
        ["--record", "x"],
        ["--record=x"],
        ["--port", "y"],
        ["--port=y"],
        ["--stdin"],
        ["--core", "pid"],
        ["--replay", "x.csv"],
        ["--mock"],
        ["--rec", "x"],
        ["--r", "x"],
        ["--p", "4001"],
        ["--s"],
        ["--text-log", "/existing"],
    ],
)
def test_reserved_passthrough_refused(passthrough, tmp_path):
    path, out = make_stub_mamba(tmp_path)
    fw = make_fw(tmp_path)
    r = run(
        ["--", *passthrough],
        path=path,
        env_update={"FW_DIR": str(fw), "STUB_OUT": str(out)},
    )
    assert r.returncode == 2
    assert "reserved" in r.stderr
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


def test_relative_record_exists_in_caller_cwd_exits1(tmp_path):
    path, out = make_stub_mamba(tmp_path)
    fw = make_fw(tmp_path)
    caller = tmp_path / "caller"
    caller.mkdir()
    rel = caller / "rel.csv"
    rel.write_text("keep me\n")
    r = run(
        ["--record", "rel.csv"],
        path=path,
        cwd=str(caller),
        env_update={"FW_DIR": str(fw), "STUB_OUT": str(out)},
    )
    assert r.returncode == 1
    assert "FATAL" in r.stderr
    assert not out.exists()
    assert rel.read_text() == "keep me\n"


def test_dangling_symlink_record_exits1_without_overwrite(tmp_path):
    path, out = make_stub_mamba(tmp_path)
    fw = make_fw(tmp_path)
    rec = tmp_path / "dangling.csv"
    os.symlink(tmp_path / "nowhere.csv", rec)
    r = run(
        ["--record", str(rec)],
        path=path,
        env_update={"FW_DIR": str(fw), "STUB_OUT": str(out)},
    )
    assert r.returncode == 1
    assert "FATAL" in r.stderr
    assert not out.exists()
    assert rec.is_symlink()


def test_dangling_symlink_record_with_overwrite_reaches_bridge(tmp_path):
    path, out = make_stub_mamba(tmp_path)
    fw = make_fw(tmp_path)
    rec = tmp_path / "dangling.csv"
    os.symlink(tmp_path / "nowhere.csv", rec)
    r = run(
        ["--overwrite", "--record", str(rec)],
        path=path,
        env_update={"FW_DIR": str(fw), "STUB_OUT": str(out)},
    )
    assert r.returncode == STUB_EXIT
    assert out.exists()


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


def test_relative_record_real_run_argv_is_absolute(tmp_path):
    path, out = make_stub_mamba(tmp_path)
    fw = make_fw(tmp_path)
    caller = tmp_path / "caller"
    caller.mkdir()
    r = run(
        ["--record", "rel.csv"],
        path=path,
        cwd=str(caller),
        env_update={"FW_DIR": str(fw), "STUB_OUT": str(out)},
    )
    assert r.returncode == STUB_EXIT
    _, argv = read_stub(out)
    i = argv.index("--record")
    assert argv[i + 1] == str(caller.resolve() / "rel.csv")


@pytest.mark.parametrize(
    "flag, value",
    [
        ("--port", "rfc2217://127.0.0.1:4001"),
        ("--core", "pid"),
        ("--duration", "30"),
    ],
)
@pytest.mark.parametrize("form", ["space", "equals"])
def test_value_flags_reach_bridge_argv(flag, value, form, tmp_path):
    path, out = make_stub_mamba(tmp_path)
    fw = make_fw(tmp_path)
    rec = tmp_path / "new.csv"
    if form == "space":
        opt = [flag, value]
    else:
        opt = [f"{flag}={value}"]
    r = run(
        [*opt, "--record", str(rec)],
        path=path,
        env_update={"FW_DIR": str(fw), "STUB_OUT": str(out)},
    )
    assert r.returncode == STUB_EXIT
    _, argv = read_stub(out)
    i = argv.index(flag)
    assert argv[i + 1] == value


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


def test_ss_listener_warns_and_keeps_bridge_rc(tmp_path):
    path, out = make_stub_mamba(tmp_path)
    path = make_stub_ss(tmp_path, path, listening=True)
    fw = make_fw(tmp_path)
    r = run(
        ["--record", str(tmp_path / "new.csv")],
        path=path,
        env_update={"FW_DIR": str(fw), "STUB_OUT": str(out)},
    )
    assert "WARNING" in r.stderr
    assert "4000" in r.stderr
    assert r.returncode == STUB_EXIT
    assert out.exists()


def test_ss_no_listener_no_warning(tmp_path):
    path, out = make_stub_mamba(tmp_path)
    path = make_stub_ss(tmp_path, path, listening=False)
    fw = make_fw(tmp_path)
    r = run(
        ["--record", str(tmp_path / "new.csv")],
        path=path,
        env_update={"FW_DIR": str(fw), "STUB_OUT": str(out)},
    )
    assert "WARNING" not in r.stderr
    assert r.returncode == STUB_EXIT


@pytest.mark.parametrize(
    "url, expect_warning",
    [
        ("rfc2217://[::1]:4000", False),
        ("rfc2217://localhost", False),
        ("rfc2217://localhost:4000/path", True),
        ("rfc2217://127.0.0.1:4000", True),
    ],
)
def test_url_shapes_listener_check(url, expect_warning, tmp_path):
    path, out = make_stub_mamba(tmp_path)
    path = make_stub_ss(tmp_path, path, listening=True)
    fw = make_fw(tmp_path)
    r = run(
        ["--port", url, "--record", str(tmp_path / "new.csv")],
        path=path,
        env_update={"FW_DIR": str(fw), "STUB_OUT": str(out)},
    )
    assert r.returncode == STUB_EXIT
    if expect_warning:
        assert "WARNING" in r.stderr
        assert "4000" in r.stderr
    else:
        assert "WARNING" not in r.stderr
