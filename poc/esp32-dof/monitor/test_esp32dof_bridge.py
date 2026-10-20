#!/usr/bin/env python3
"""Tests for dof_wheel_bridge (frames -> SerialImuHal -> LQR/PID -> UDP wheel display).

No hardware, no Webots. UDP only on loopback ephemeral ports (never 5006).

Usage:
    cd poc/esp32-dof
    mamba run -n esp32-dof python3 -m pytest -q monitor/test_esp32dof_bridge.py
"""

import ast
import csv
import importlib.util
import json
import math
import socket
import subprocess
import sys
import threading
import time
from itertools import islice
from pathlib import Path

import pytest

MONITOR = Path(__file__).resolve().parent
REPO_POC = MONITOR.parents[1]
sys.path.insert(0, str(MONITOR))

import dof_wheel_bridge as b  # noqa: E402
from dof_frame import Frame, format_frame, parse_line  # noqa: E402
from dof_hal import SerialImuHal  # noqa: E402
from hal.control_core import ControlOutput, LqrBalance, PidBalance  # noqa: E402
from hal.hal import HalFault  # noqa: E402

SWD_MSG = REPO_POC / "servo-wheel-dof" / "webots" / "controllers" / "servo_wheel_dof" / "imu_wheel_msg.py"
SWD_MOCK = REPO_POC / "servo-wheel-dof" / "monitor" / "mock_publisher.py"
REF_LQR = -0.1086
DT_US = 20000


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod          # dataclasses needs the module registered before exec
    spec.loader.exec_module(mod)
    return mod


def tilt(pitch):
    return (-math.sin(pitch), 0.0, math.cos(pitch))


def line(seq, t_us, accel, gyro):
    return format_frame(Frame(seq=seq % 65536, t_us=t_us % 2**32, accel=accel, gyro=gyro)).rstrip("\r\n")


def still_lines(n):
    return list(islice(b.still_then_mock_lines(realtime=False), n))


def write_raw_csv(path, lines):
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["seq", "t_us", "ax", "ay", "az", "gx", "gy", "gz"])
        for ln in lines:
            f = parse_line(ln)
            w.writerow([f.seq, f.t_us, *f.accel, *f.gyro])


class Rec:
    """Stub publisher: records send() calls, no socket."""

    def __init__(self):
        self.msgs = []

    def send(self, seq, roll, pitch, yaw, wheel_angle, cmd_rad_s=None):
        self.msgs.append({"seq": seq, "roll": roll, "pitch": pitch, "yaw": yaw,
                          "wheel": wheel_angle, "cmd": cmd_rad_s})


class StubCore:
    def __init__(self, cmd):
        self.cmd = cmd
        self.resets = 0

    def step(self, imu, dt):
        return ControlOutput(float(self.cmd), float(self.cmd), (0.0, 0.0), False)

    def reset(self):
        self.resets += 1


def free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def drain(sock):
    out = []
    sock.settimeout(0.05)
    while True:
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            return out
        out.append(data)


# ----- 1. gains -------------------------------------------------------------------------------

def test_load_gains_values_match_parity_json():
    g = b.load_gains()
    assert g["lqr"]["K"] == (-23.818429946899414, -3.4940226078033447)
    assert g["lqr"]["theta_ref_rad"] == -0.10859999805688858
    assert g["pid"]["ki"] == 0.10000000149011612


def test_load_gains_missing_key_raises(tmp_path):
    p = tmp_path / "bad.json"
    p.write_text(json.dumps({"lqr": {"theta_ref_rad": -0.1, "out_limit": 1.0},
                             "pid": {"kp": 1, "ki": 0, "kd": 0, "out_limit": 1}}))
    with pytest.raises(ValueError, match="lqr.K"):
        b.load_gains(p)


def test_load_gains_missing_pid_key_raises(tmp_path):
    p = tmp_path / "bad2.json"
    p.write_text(json.dumps({"lqr": {"K": [1, 2], "theta_ref_rad": 0.0, "out_limit": 1.0},
                             "pid": {"kp": 1, "kd": 0, "out_limit": 1}}))
    with pytest.raises(ValueError, match="pid.ki"):
        b.load_gains(p)


# ----- 2. build_core --------------------------------------------------------------------------

def test_build_core_lqr():
    core, ref = b.build_core("lqr", b.load_gains())
    assert isinstance(core, LqrBalance)
    assert core.K == (-23.818429946899414, -3.4940226078033447)
    assert ref == pytest.approx(REF_LQR, abs=1e-6)   # JSON is -0.10859999805688858 (plan's -0.1086 rounded)


def test_build_core_pid():
    core, ref = b.build_core("pid", b.load_gains())
    assert isinstance(core, PidBalance)
    assert ref == 0.0


def test_build_core_none():
    assert b.build_core("none", None) == (None, 0.0)


# ----- 3. start pose gives ~0 command ---------------------------------------------------------

@pytest.mark.parametrize("name", ["lqr", "pid"])
def test_start_pose_gives_zero_command(name):
    core, ref = b.build_core(name, b.load_gains())
    lines = [line(i, i * DT_US, tilt(0.3), (2.0, -1.0, 0.5)) for i in range(53)]
    hal = SerialImuHal(lines, reference_pitch_rad=ref)
    rec = Rec()
    res = b.run_bridge(hal, core, ref, rec, sleep=lambda s: None)
    assert res.ticks == 4                # 53 lines: 50 consumed by the arm tick, then 3 frame ticks
    for m in rec.msgs:
        assert abs(m["cmd"]) < 1e-9
    if name == "lqr":
        assert all(abs(m["pitch"]) < 1e-9 for m in rec.msgs)
        assert all(m["pitch"] != pytest.approx(REF_LQR, abs=1e-3) for m in rec.msgs)


# ----- 4. integration of the wheel angle ------------------------------------------------------

def test_constant_command_integrates_wheel_angle():
    hal = SerialImuHal(still_lines(60), reference_pitch_rad=0.0)
    rec = Rec()
    res = b.run_bridge(hal, StubCore(0.5), 0.0, rec, sleep=lambda s: None)
    # 50 lines arm on the first tick (that tick also integrates), then 10 frame ticks: 11 ticks.
    assert res.ticks == 11
    assert res.final_wheel_angle == pytest.approx(0.11, abs=1e-12)
    assert rec.msgs[-1]["wheel"] == pytest.approx(0.11, abs=1e-12)


def test_command_is_clamped_by_hal_to_one_rad_s():
    hal = SerialImuHal(still_lines(60), reference_pitch_rad=0.0)
    rec = Rec()
    res = b.run_bridge(hal, StubCore(5.0), 0.0, rec, sleep=lambda s: None)
    assert res.final_wheel_angle == pytest.approx(0.22, abs=1e-12)


# ----- 5. UDP datagrams parse with servo-wheel-dof's parser -----------------------------------

def test_datagrams_parse_with_servo_wheel_dof_parser():
    parse_mod = load_module("swd_imu_wheel_msg", SWD_MSG)
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.bind(("127.0.0.1", 0))
    pub = b.WheelPublisher(port=listener.getsockname()[1])
    try:
        hal = SerialImuHal(still_lines(60), reference_pitch_rad=0.0)
        res = b.run_bridge(hal, StubCore(0.5), 0.0, pub, sleep=lambda s: None)
        datagrams = drain(listener)
    finally:
        pub.close()
        listener.close()
    assert len(datagrams) == res.ticks == 11
    parsed = [parse_mod.parse_imu_wheel_msg(d) for d in datagrams]
    assert all(p is not None for p in parsed)
    for d in datagrams:
        assert "cmd_rad_s" in json.loads(d)
    assert parsed[-1].wheel_angle == pytest.approx(res.final_wheel_angle, abs=1e-12)


# ----- 6. --core none mirror ------------------------------------------------------------------

def test_core_none_mirrors_pitch_and_hal_integral_stays_zero():
    lines = list(islice(b.still_then_mock_lines(realtime=False), 90))
    hal = SerialImuHal(lines, reference_pitch_rad=0.0)
    rec = Rec()
    res = b.run_bridge(hal, None, 0.0, rec, sleep=lambda s: None)
    assert res.ticks > 0
    assert max(abs(m["pitch"]) for m in rec.msgs) > 0.01
    for m in rec.msgs:
        assert m["wheel"] == m["pitch"]
        assert m["cmd"] == 0.0
    assert hal.wheel_angle_rad == 0.0


# ----- 7. HalFault hold -----------------------------------------------------------------------

def test_hal_fault_holds_wheel_and_zero_command():
    lines = still_lines(50)
    lines.append(line(50, 50 * DT_US, tilt(0.0), (0.0, 0.0, 0.0)))        # normal
    lines.append(line(51, 50 * DT_US + 250000, tilt(0.0), (0.0, 0.0, 0.0)))  # 0.25 s gap: stale
    lines.append(line(52, 50 * DT_US + 270000, tilt(0.0), (0.0, 0.0, 0.0)))
    lines.append(line(53, 50 * DT_US + 290000, tilt(0.0), (0.0, 0.0, 0.0)))
    hal = SerialImuHal(lines, reference_pitch_rad=0.0)
    rec = Rec()
    core = StubCore(0.5)
    res = b.run_bridge(hal, core, 0.0, rec, sleep=lambda s: None)
    assert res.faults == 1
    assert core.resets == 1
    assert len(rec.msgs) == 5
    stale = rec.msgs[2]
    assert stale["wheel"] == rec.msgs[1]["wheel"]
    assert stale["cmd"] == 0.0
    assert rec.msgs[3]["wheel"] == pytest.approx(rec.msgs[1]["wheel"] + 0.01, abs=1e-12)


# ----- 8. short finite replay -----------------------------------------------------------------

def test_short_replay_returns_2_and_names_the_window(tmp_path, capsys):
    src = tmp_path / "short.csv"
    write_raw_csv(src, still_lines(30))
    rc = b.main(["--replay", str(src), "--udp-port", str(free_udp_port())])
    assert rc == 2
    err = capsys.readouterr().err
    assert "never armed" in err
    assert "window incomplete" in err


# ----- 9. --record round-trips through servo-wheel-dof's replay CSV reader --------------------

def test_record_round_trips_through_mock_publisher_reader(tmp_path):
    src = tmp_path / "raw.csv"
    write_raw_csv(src, still_lines(60))
    rec_path = tmp_path / "rec.csv"
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.bind(("127.0.0.1", 0))
    try:
        rc = b.main(["--replay", str(src), "--record", str(rec_path),
                     "--udp-port", str(listener.getsockname()[1])])
    finally:
        listener.close()
    assert rc == 0
    swd = load_module("swd_mock_publisher", SWD_MOCK)
    samples, t_us = swd.parse_replay_csv(rec_path.read_text().splitlines(), str(rec_path))
    assert len(samples) == 11
    assert len(t_us) == len(samples)
    assert all(t_us[i] <= t_us[i + 1] for i in range(len(t_us) - 1))
    header = rec_path.read_text().splitlines()[0]
    assert header == ",".join(b.RECORD_COLUMNS)


# ----- 10. WheelPublisher ---------------------------------------------------------------------

def test_wheel_publisher_swallows_oserror_and_refuses_bad_values():
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.bind(("127.0.0.1", 0))
    pub = b.WheelPublisher(port=listener.getsockname()[1])
    try:
        class _BoomSock:
            def sendto(self, *a, **k):
                raise OSError("no route")

            def close(self):
                pass

        pub._sock = _BoomSock()
        pub.send(1, 0.0, 0.0, 0.0, 0.0, cmd_rad_s=0.0)
        assert pub.errors == 1 and pub.sent == 0

        pub.send(2, float("nan"), 0.0, 0.0, 0.0)
        pub.send(3, 0.0, float("inf"), 0.0, 0.0)
        pub.send(4, True, 0.0, 0.0, 0.0)
        pub.send(5, 0.0, 0.0, 0.0, 0.0, cmd_rad_s=float("nan"))
        assert pub.skipped == 4
        pub.close()
        pub.close()
        pub.send(6, 0.0, 0.0, 0.0, 0.0)   # closed: silent no-op
    finally:
        listener.close()


# ----- 11. still lead-in arms the HAL and is continuous ---------------------------------------

def test_still_lead_in_arms_hal_and_is_continuous_at_join():
    lines = list(islice(b.still_then_mock_lines(realtime=False), 100))
    for ln in lines[:75]:
        f = parse_line(ln)
        assert f.accel == (0.0, 0.0, 1.0)
        assert f.gyro == (0.0, 0.0, 0.0)
    hal = SerialImuHal(lines[:75], reference_pitch_rad=0.0)
    hal.wait_next_tick()
    assert hal.armed
    a, c = parse_line(lines[74]), parse_line(lines[75])
    assert c.t_us - a.t_us == DT_US
    assert c.seq - a.seq == 1


# ----- 12. numpy/scipy/serial-free import and run ---------------------------------------------

def test_subprocess_isolated_import_and_mock_run_without_numpy_scipy_serial():
    port = free_udp_port()
    code = textwrap_code(port)
    proc = subprocess.run([sys.executable, "-I", "-c", code], capture_output=True, text=True,
                          timeout=60)
    assert proc.returncode == 0, proc.stderr
    assert "NOSERIAL_OK" in proc.stdout


def textwrap_code(port):
    return (
        "import sys\n"
        f"sys.path.insert(0, {str(MONITOR)!r})\n"
        "for m in ('numpy', 'scipy', 'serial'):\n"
        "    sys.modules[m] = None\n"
        "import dof_wheel_bridge as b\n"
        f"rc = b.main(['--mock', '--duration', '2.2', '--udp-port', '{port}'])\n"
        "assert rc == 0, rc\n"
        "assert 'dof_serial' not in sys.modules\n"
        "print('NOSERIAL_OK')\n"
    )


def test_ast_no_forbidden_top_level_imports():
    tree = ast.parse((MONITOR / "dof_wheel_bridge.py").read_text(encoding="utf-8"))
    forbidden = {"controller", "numpy", "scipy", "serial", "dof_serial", "dof_monitor",
                 "dof_publisher"}
    top = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            top += [a.name.split(".")[0] for a in node.names]
        elif isinstance(node, ast.ImportFrom) and node.module:
            top.append(node.module.split(".")[0])
    assert not forbidden.intersection(top), top


# ----- 13. CLI validation ---------------------------------------------------------------------

def test_cli_rejects_bad_core():
    with pytest.raises(SystemExit) as e:
        b.main(["--mock", "--core", "bogus"])
    assert e.value.code == 2


def test_cli_rejects_udp_port_zero():
    with pytest.raises(SystemExit) as e:
        b.main(["--mock", "--udp-port", "0"])
    assert e.value.code == 2


def test_cli_rejects_two_sources():
    with pytest.raises(SystemExit) as e:
        b.main(["--mock", "--replay", "x.csv"])
    assert e.value.code == 2


def test_cli_rejects_no_source():
    with pytest.raises(SystemExit) as e:
        b.main([])
    assert e.value.code == 2


# ----- 14. PolledSource -----------------------------------------------------------------------

class _Blocker:
    def __init__(self):
        self.release = threading.Event()

    def __iter__(self):
        return self

    def __next__(self):
        self.release.wait(5.0)
        raise StopIteration


class _Boom:
    def __iter__(self):
        return self

    def __next__(self):
        raise RuntimeError("boom")


def test_polled_source_yields_none_on_stall():
    blk = _Blocker()
    src = b.PolledSource(blk, poll_s=0.1)
    t0 = time.monotonic()
    got = next(src)
    elapsed = time.monotonic() - t0
    assert got is None
    assert elapsed < 0.4
    blk.release.set()
    src.close()


def test_polled_source_reraises_iterator_exception():
    src = b.PolledSource(_Boom(), poll_s=0.05)
    with pytest.raises(RuntimeError, match="boom"):
        for _ in range(100):
            next(src)


def test_polled_source_items_then_sticky_end():
    src = b.PolledSource(["a", "b"], poll_s=0.5)
    got = []
    while True:
        try:
            v = next(src)
        except StopIteration:
            break
        if v is not None:
            got.append(v)
    assert got == ["a", "b"]
    with pytest.raises(StopIteration):
        next(src)
