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
import io
import itertools
import json
import math
import socket
import subprocess
import sys
import threading
import time
import types
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

SWD_MSG = REPO_POC / "servo-wheel-dof" / "webots" / "controllers" / "servo_wheel_dof" / "imu_wheel_msg.py"
SWD_MOCK = REPO_POC / "servo-wheel-dof" / "monitor" / "mock_publisher.py"
REF_LQR = -0.1086
DT = 0.02
DT_US = 20000
JSON_GAINS = json.loads((b.GAINS_PATH).read_text(encoding="utf-8"))


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod          # dataclasses needs the module registered before exec
    spec.loader.exec_module(mod)
    return mod


def tilt(pitch):
    return (-math.sin(pitch), 0.0, math.cos(pitch))


def tilt_rp(roll, pitch):
    """Accelerometer gravity vector for roll/pitch (ZYX, radians)."""
    return (-math.sin(pitch), math.sin(roll) * math.cos(pitch), math.cos(roll) * math.cos(pitch))


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
    """Stub publisher: records send() calls. With hal=, also snapshots the HAL at send time."""

    def __init__(self, hal=None):
        self.hal = hal
        self.msgs = []
        self.sent = 0
        self.errors = 0
        self.skipped = 0

    def send(self, seq, roll, pitch, yaw, wheel_angle, cmd_rad_s=None):
        m = {"seq": seq, "roll": roll, "pitch": pitch, "yaw": yaw, "wheel": wheel_angle,
             "cmd": cmd_rad_s}
        if self.hal is not None:
            m["hal_pitch"] = self.hal.tared_pitch_rad
            m["hal_t_us"] = int(self.hal.now_s() * 1e6)
        self.msgs.append(m)
        self.sent += 1


class StubCore:
    """Constant command; records the dt it was called with; optional fault flag."""

    def __init__(self, cmd, fault=False):
        self.cmd = cmd
        self.fault = fault
        self.resets = 0
        self.dts = []

    def step(self, imu, dt):
        self.dts.append(dt)
        return ControlOutput(float(self.cmd), float(self.cmd), (0.0, 0.0), self.fault)

    def reset(self):
        self.resets += 1


class FakeClock:
    """Injected clock: sleep() advances time and records the requested duration."""

    def __init__(self):
        self.t = 0.0
        self.sleeps = []

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.sleeps.append(s)
        self.t += s


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


def arm_then(frames_after, still=50):
    """still arm lines followed by the given lines."""
    return still_lines(still) + list(frames_after)


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


@pytest.mark.parametrize("bad_k", [[1, 2, 3], [1], "ab", [1, "x"], [1, True], [1, float("nan")]])
def test_load_gains_rejects_bad_k(tmp_path, bad_k):
    base = dict(JSON_GAINS)
    base["lqr"] = dict(JSON_GAINS["lqr"], K=bad_k)
    p = tmp_path / "k.json"
    p.write_text(json.dumps(base))
    with pytest.raises(ValueError):
        b.load_gains(p)


def test_load_gains_invalid_json_message(tmp_path):
    p = tmp_path / "broken.json"
    p.write_text("{nope")
    with pytest.raises(ValueError, match="invalid JSON"):
        b.load_gains(p)


def test_load_gains_non_object_top_level(tmp_path):
    p = tmp_path / "list.json"
    p.write_text("[1, 2]")
    with pytest.raises(ValueError, match="JSON object"):
        b.load_gains(p)


# ----- 2. build_core and gain wiring (e) ------------------------------------------------------

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


def test_pid_and_lqr_wiring_matches_json():
    g = b.load_gains()
    pid, _ = b.build_core("pid", g)
    assert (pid.kp, pid.ki, pid.kd, pid.out_limit) == (
        JSON_GAINS["pid"]["kp"], JSON_GAINS["pid"]["ki"], JSON_GAINS["pid"]["kd"],
        JSON_GAINS["pid"]["out_limit"])
    lqr, _ = b.build_core("lqr", g)
    assert lqr.out_limit == JSON_GAINS["lqr"]["out_limit"]
    assert lqr.theta_ref_rad == JSON_GAINS["lqr"]["theta_ref_rad"]


# ----- 3. start pose gives ~0 command ---------------------------------------------------------

@pytest.mark.parametrize("name", ["lqr", "pid"])
def test_start_pose_gives_zero_command(name):
    core, ref = b.build_core(name, b.load_gains())
    lines = [line(i, i * DT_US, tilt(0.3), (2.0, -1.0, 0.5)) for i in range(53)]
    hal = SerialImuHal(lines, reference_pitch_rad=ref)
    rec = Rec(hal)
    res = b.run_bridge(hal, core, rec, sleep=lambda s: None)
    assert res.ticks == 4                # 53 lines: 50 consumed by the arm tick, then 3 frame ticks
    assert len(rec.msgs) == 4
    for m in rec.msgs:
        assert abs(m["cmd"]) < 1e-9
    if name == "lqr":
        assert all(abs(m["pitch"]) < 1e-9 for m in rec.msgs)
        assert all(m["pitch"] != pytest.approx(REF_LQR, abs=1e-3) for m in rec.msgs)


# ----- 4. integration of the wheel angle ------------------------------------------------------

def test_constant_command_integrates_wheel_angle():
    hal = SerialImuHal(still_lines(60), reference_pitch_rad=0.0)
    rec = Rec(hal)
    res = b.run_bridge(hal, StubCore(0.5), rec, sleep=lambda s: None)
    # 50 lines arm on the first tick (that tick also integrates), then 10 frame ticks: 11 ticks.
    assert res.ticks == 11
    assert res.final_wheel_angle == pytest.approx(0.11, abs=1e-12)
    assert rec.msgs[-1]["wheel"] == pytest.approx(0.11, abs=1e-12)


def test_command_is_clamped_by_hal_to_one_rad_s():
    hal = SerialImuHal(still_lines(60), reference_pitch_rad=0.0)
    res = b.run_bridge(hal, StubCore(5.0), Rec(hal), sleep=lambda s: None)
    assert res.final_wheel_angle == pytest.approx(0.22, abs=1e-12)


# ----- a. UDP payload values ------------------------------------------------------------------

def test_udp_payload_values_match_hal_state():
    parse_mod = load_module("swd_imu_wheel_msg_a", SWD_MSG)
    lines = [line(i, i * DT_US, tilt_rp(0.2, 0.0), (0.0, 0.0, 0.0)) for i in range(50)]
    lines += [line(50 + j, (50 + j) * DT_US, tilt_rp(0.2, 0.1), (0.0, 0.0, 0.0)) for j in range(10)]
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.bind(("127.0.0.1", 0))
    pub = b.WheelPublisher(port=listener.getsockname()[1])
    hal = SerialImuHal(lines, reference_pitch_rad=0.0)
    rec = Rec(hal)
    try:
        res = b.run_bridge(hal, StubCore(0.5), _Tee(pub, rec), sleep=lambda s: None)
        datagrams = drain(listener)
    finally:
        pub.close()
        listener.close()
    assert res.ticks == 11 and len(datagrams) == 11
    for k, d in enumerate(datagrams):
        raw = json.loads(d)
        assert set(raw) == {"seq", "roll", "pitch", "yaw", "wheel_angle", "cmd_rad_s"}
        assert raw["seq"] == k
        assert raw["roll"] == pytest.approx(0.2, abs=1e-5)           # untared, radians
        assert raw["pitch"] == pytest.approx(rec.msgs[k]["hal_pitch"], abs=1e-12)
        assert raw["yaw"] == 0.0
        assert raw["cmd_rad_s"] == 0.5
        assert raw["wheel_angle"] == pytest.approx(0.01 * (k + 1), abs=1e-12)
        assert parse_mod.parse_imu_wheel_msg(d) is not None


class _Tee:
    """Forward send() to a real publisher and to a recording stub, so both see the same call."""

    def __init__(self, pub, rec):
        self._pub = pub
        self._rec = rec

    @property
    def sent(self):
        return self._pub.sent

    @property
    def errors(self):
        return self._pub.errors

    @property
    def skipped(self):
        return self._pub.skipped

    def send(self, *a, **k):
        self._rec.send(*a, **k)
        self._pub.send(*a, **k)


# ----- 5. UDP datagrams parse with servo-wheel-dof's parser -----------------------------------

def test_datagrams_parse_with_servo_wheel_dof_parser():
    parse_mod = load_module("swd_imu_wheel_msg", SWD_MSG)
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.bind(("127.0.0.1", 0))
    pub = b.WheelPublisher(port=listener.getsockname()[1])
    try:
        hal = SerialImuHal(still_lines(60), reference_pitch_rad=0.0)
        res = b.run_bridge(hal, StubCore(0.5), pub, sleep=lambda s: None)
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


# ----- 6. --core none mirror (c, m) -----------------------------------------------------------

def test_core_none_mirrors_actual_pitch_values():
    lines = list(islice(b.still_then_mock_lines(realtime=False), 90))
    hal = SerialImuHal(lines, reference_pitch_rad=0.0)
    rec = Rec(hal)
    res = b.run_bridge(hal, None, rec, sleep=lambda s: None)
    assert res.ticks == 41 and len(rec.msgs) == 41
    pitches = [m["pitch"] for m in rec.msgs]
    assert max(abs(p) for p in pitches) > 0.01              # real tilt, not a flat run
    for m in rec.msgs:
        assert m["pitch"] == pytest.approx(m["hal_pitch"], abs=1e-12)
        assert m["wheel"] == m["pitch"]
        assert m["cmd"] == 0.0
    assert hal.wheel_angle_rad == 0.0


# ----- 7. HalFault hold -----------------------------------------------------------------------

def _stale_tilt_lines():
    # Arm on a tilted roll (0.2) so the filter is seeded there; complementary filter moves ~2%/frame.
    lines = [line(i, i * DT_US, tilt_rp(0.2, 0.0), (0.0, 0.0, 0.0)) for i in range(50)]
    lines.append(line(50, 50 * DT_US, tilt_rp(0.2, 0.1), (0.0, 0.0, 0.0)))     # normal, tilted
    lines.append(line(51, 50 * DT_US + 250000, tilt_rp(0.3, 0.15), (0.0, 0.0, 0.0)))  # 0.25 s gap
    lines.append(line(52, 50 * DT_US + 270000, tilt_rp(0.3, 0.15), (0.0, 0.0, 0.0)))
    lines.append(line(53, 50 * DT_US + 290000, tilt_rp(0.3, 0.15), (0.0, 0.0, 0.0)))
    return lines


def test_hal_fault_holds_tilted_pose_wheel_and_zero_command():
    hal = SerialImuHal(_stale_tilt_lines(), reference_pitch_rad=0.0)
    rec = Rec(hal)
    core = StubCore(0.5)
    res = b.run_bridge(hal, core, rec, sleep=lambda s: None)
    assert res.faults == 1
    assert core.resets == 1
    assert len(rec.msgs) == 5
    prev, stale, nxt = rec.msgs[1], rec.msgs[2], rec.msgs[3]
    assert stale["roll"] == prev["roll"] and stale["pitch"] == prev["pitch"]   # held pose
    assert prev["roll"] == pytest.approx(0.2, abs=1e-4) and prev["pitch"] != 0.0  # tilted, not zero
    assert stale["wheel"] == prev["wheel"]
    assert stale["cmd"] == 0.0
    assert nxt["roll"] != prev["roll"]                                       # fresh values resume
    assert nxt["wheel"] == pytest.approx(prev["wheel"] + 0.01, abs=1e-12)


def test_hal_fault_hold_core_none_publishes_last_pitch():
    hal = SerialImuHal(_stale_tilt_lines(), reference_pitch_rad=0.0)
    rec = Rec(hal)
    res = b.run_bridge(hal, None, rec, sleep=lambda s: None)
    assert res.faults == 1
    prev, stale = rec.msgs[1], rec.msgs[2]
    assert stale["pitch"] == prev["pitch"]
    assert stale["wheel"] == prev["pitch"]                                   # mirror keeps pitch
    assert stale["cmd"] == 0.0


def test_unarmed_ticks_are_not_published_or_faults(tmp_path):
    moving = [line(i, i * DT_US, tilt(0.0), (20.0 * (i % 2), 0.0, 0.0)) for i in range(50)]
    still = [line(50 + i, (50 + i) * DT_US, tilt(0.0), (0.0, 0.0, 0.0)) for i in range(53)]
    hal = SerialImuHal(moving + still, reference_pitch_rad=0.0)
    rec = Rec(hal)
    out = io.StringIO()
    rec_file = io.StringIO()
    res = b.run_bridge(hal, StubCore(0.5), rec, record=rec_file, sleep=lambda s: None, out=out)
    assert res.faults == 0
    assert res.ticks == 5                    # 1 unarmed (moving window), 4 armed
    assert len(rec.msgs) == res.published == 4
    assert len(rec_file.getvalue().strip().splitlines()) == 1 + 4   # header + armed rows only
    assert "not armed" in out.getvalue()


# ----- 8. short finite replay -----------------------------------------------------------------

def test_short_replay_returns_2_and_names_the_window(tmp_path, capsys):
    src = tmp_path / "short.csv"
    write_raw_csv(src, still_lines(30))
    rc = b.main(["--replay", str(src), "--udp-port", str(free_udp_port())])
    assert rc == 2
    err = capsys.readouterr().err
    assert "never armed" in err
    assert "window incomplete" in err


# ----- 9. --record round-trips through servo-wheel-dof's replay CSV reader (b) ----------------

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
    assert rec_path.read_text().splitlines()[0] == ",".join(b.RECORD_COLUMNS)


def test_record_columns_roll_pitch_cmd_wheel_and_hal_time():
    hal = SerialImuHal(_stale_tilt_lines()[:54], reference_pitch_rad=0.0)
    rec = Rec(hal)
    rec_file = io.StringIO()
    b.run_bridge(hal, StubCore(0.5), rec, record=rec_file, sleep=lambda s: None)
    rows = list(csv.reader(io.StringIO(rec_file.getvalue())))
    assert tuple(rows[0]) == b.RECORD_COLUMNS
    body = rows[1:]
    assert len(body) == len(rec.msgs) == 5     # arm tick + 3 frame ticks + the stale hold tick
    t_col = []
    expected_cmd = [0.5, 0.5, 0.0, 0.5, 0.5]    # the stale tick (index 2) is a hold: cmd 0
    for k, (row, m) in enumerate(zip(body, rec.msgs)):
        seq, t_us, roll, pitch, yaw, wheel, cmd = row
        assert int(seq) == m["seq"]
        assert float(roll) == pytest.approx(m["roll"], abs=1e-12)     # roll column is roll
        assert float(pitch) == pytest.approx(m["pitch"], abs=1e-12)   # pitch column is pitch
        assert float(yaw) == 0.0
        assert float(wheel) == pytest.approx(m["wheel"], abs=1e-12)
        assert float(cmd) == expected_cmd[k]                           # command, not the wheel
        assert int(t_us) == m["hal_t_us"]                              # HAL time, microseconds
        t_col.append(int(t_us))
    assert t_col[0] == 0                       # arm tick sits at HAL t=0 (dof_hal semantics)
    assert all(t_col[i] <= t_col[i + 1] for i in range(len(t_col) - 1))
    assert t_col[-1] > 0


class _TrackFile(io.StringIO):
    def __init__(self):
        super().__init__()
        self.flush_calls = 0

    def flush(self):
        self.flush_calls += 1
        super().flush()


def test_record_flushed_on_exception_path():
    class Boom(StubCore):
        def step(self, imu, dt):
            if len(self.dts) >= 2:
                raise RuntimeError("core fault")
            return super().step(imu, dt)

    hal = SerialImuHal(still_lines(60), reference_pitch_rad=0.0)
    f = _TrackFile()
    with pytest.raises(RuntimeError, match="core fault"):
        b.run_bridge(hal, Boom(0.5), Rec(hal), record=f, sleep=lambda s: None)
    assert f.flush_calls >= 1
    assert len(f.getvalue().strip().splitlines()) == 1 + 2    # header + the two completed ticks


# ----- 10. WheelPublisher ---------------------------------------------------------------------

class _BoomSock:
    def sendto(self, *a, **k):
        raise OSError("no route")

    def close(self):
        pass


def test_wheel_publisher_swallows_oserror_and_refuses_bad_values():
    pub = b.WheelPublisher(port=5)
    try:
        pub._sock.close()
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
    finally:
        pub.close()


def test_closed_publisher_send_changes_nothing():
    pub = b.WheelPublisher(port=5)
    pub.close()
    before = (pub.sent, pub.errors, pub.skipped)
    pub.send(6, 0.0, 0.0, 0.0, 0.0)
    assert (pub.sent, pub.errors, pub.skipped) == before


def test_published_counts_only_accepted_sends():
    pub = b.WheelPublisher(port=5)
    try:
        pub._sock.close()
        pub._sock = _BoomSock()
        hal = SerialImuHal(still_lines(60), reference_pitch_rad=0.0)
        res = b.run_bridge(hal, StubCore(0.5), pub, sleep=lambda s: None)
        assert res.ticks == 11
        assert res.published == 0
        assert pub.errors == 11
    finally:
        pub.close()


@pytest.mark.parametrize("port", [0, 65536, True, "5006"])
def test_wheel_publisher_rejects_bad_port(port):
    with pytest.raises(ValueError, match="port"):
        b.WheelPublisher(port=port)


@pytest.mark.parametrize("host", ["", None, 3])
def test_wheel_publisher_rejects_bad_host(host):
    with pytest.raises(ValueError, match="host"):
        b.WheelPublisher(host=host, port=5)


def test_wheel_publisher_refuses_bool_seq():
    pub = b.WheelPublisher(port=5)
    try:
        pub._sock.close()
        pub._sock = _BoomSock()
        pub.send(True, 0.0, 0.0, 0.0, 0.0)
        assert pub.skipped == 1 and pub.sent == 0 and pub.errors == 0
    finally:
        pub.close()


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

def _isolated_code(port):
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


def test_subprocess_isolated_import_and_mock_run_without_numpy_scipy_serial():
    proc = subprocess.run([sys.executable, "-I", "-c", _isolated_code(free_udp_port())],
                          capture_output=True, text=True, timeout=60)
    assert proc.returncode == 0, proc.stderr
    assert "NOSERIAL_OK" in proc.stdout


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


# ----- 13. CLI validation and defaults (f) ----------------------------------------------------

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


def test_cli_defaults_and_port_range():
    args = b.build_parser().parse_args(["--mock"])
    assert args.core == "lqr"
    assert args.alpha == 0.98
    assert args.udp_port == 5006
    assert b.build_parser().parse_args(["--mock", "--udp-port", "65535"]).udp_port == 65535
    with pytest.raises(SystemExit) as e:
        b.build_parser().parse_args(["--mock", "--udp-port", "65536"])
    assert e.value.code == 2


@pytest.mark.parametrize("alpha", ["1.5", "-0.1", "nan", "x"])
def test_cli_rejects_alpha_out_of_range(alpha):
    with pytest.raises(SystemExit) as e:
        b.build_parser().parse_args(["--mock", "--alpha", alpha])
    assert e.value.code == 2


def test_alpha_and_clock_threaded_to_hal(monkeypatch, tmp_path):
    captured = {}
    src = tmp_path / "ten.csv"
    write_raw_csv(src, still_lines(10))

    class Spy(SerialImuHal):
        def __init__(self, lines, **kw):
            captured.update(kw)
            super().__init__(lines, **kw)

    monkeypatch.setattr(b, "SerialImuHal", Spy)
    fake = FakeClock()
    rc = b.main(["--replay", str(src), "--alpha", "0.5", "--udp-port", str(free_udp_port())],
                clock=fake, sleep=fake.sleep)
    assert rc == 2                                   # 10 rows never arm; the alpha is what matters
    assert captured["alpha"] == 0.5
    assert captured["clock"] is fake
    assert captured["reference_pitch_rad"] == pytest.approx(REF_LQR, abs=1e-6)


def test_duration_under_arm_window_exits_2(tmp_path, capsys):
    rc = b.main(["--mock", "--duration", "0.5", "--udp-port", str(free_udp_port())])
    assert rc == 2
    assert "never armed" in capsys.readouterr().err


# ----- g. pacing and duration with an injected clock -----------------------------------------

def test_pacing_sleeps_on_polls_only():
    fake = FakeClock()
    lines = still_lines(50) + [None, None, line(50, 50 * DT_US, tilt(0.0), (0.0, 0.0, 0.0)), None]
    hal = SerialImuHal(lines, reference_pitch_rad=0.0, clock=fake)
    rec = Rec(hal)
    res = b.run_bridge(hal, StubCore(0.5), rec, clock=fake, sleep=fake.sleep)
    assert res.ticks == 5                       # arm tick, 2 polls, 1 frame, 1 poll
    assert len(fake.sleeps) == 3                # one per poll tick, none on frame ticks
    assert all(s == pytest.approx(DT, abs=1e-12) for s in fake.sleeps)


def test_stall_integrates_until_stale_then_freezes():
    fake = FakeClock()
    stream = itertools.chain(still_lines(50), itertools.repeat(None))
    hal = SerialImuHal(stream, reference_pitch_rad=0.0, clock=fake)
    rec = Rec(hal)
    res = b.run_bridge(hal, StubCore(0.5), rec, duration_s=1.0, clock=fake, sleep=fake.sleep)
    assert fake.t <= 1.0 + DT + 1e-9            # duration boundary: no tick past the boundary
    assert res.faults > 0                       # wall-stale after STALE_S -> HalFault hold
    first_fault = next(i for i, m in enumerate(rec.msgs) if m["cmd"] == 0.0)
    wheels_after = [m["wheel"] for m in rec.msgs[first_fault:]]
    assert len(set(wheels_after)) == 1          # frozen while stalled
    assert rec.msgs[1]["wheel"] > rec.msgs[0]["wheel"]   # integrates while polls are fresh


def test_duration_stops_loop_at_boundary_with_fake_clock():
    fake = FakeClock()
    hal = SerialImuHal(itertools.chain(still_lines(60), itertools.repeat(None)),
                       reference_pitch_rad=0.0, clock=fake)
    res = b.run_bridge(hal, StubCore(0.5), Rec(hal), duration_s=0.5, clock=fake, sleep=fake.sleep)
    assert 0.5 <= fake.t <= 0.5 + DT + 1e-9
    assert res.ticks >= 1


# ----- h. seq counter -------------------------------------------------------------------------

def test_seq_starts_at_zero_and_increments():
    hal = SerialImuHal(still_lines(60), reference_pitch_rad=0.0)
    rec = Rec(hal)
    b.run_bridge(hal, StubCore(0.0), rec, sleep=lambda s: None)
    assert [m["seq"] for m in rec.msgs] == list(range(11))


def test_seq_wraps_at_seq_mod(monkeypatch):
    monkeypatch.setattr(b, "SEQ_MOD", 4)
    hal = SerialImuHal(still_lines(56), reference_pitch_rad=0.0)
    rec = Rec(hal)
    b.run_bridge(hal, StubCore(0.0), rec, sleep=lambda s: None)
    assert [m["seq"] for m in rec.msgs] == [i % 4 for i in range(len(rec.msgs))]
    assert len(rec.msgs) == 7


# ----- d. core fault and dt -------------------------------------------------------------------

def test_core_fault_forces_zero_command_and_dt_is_nominal():
    hal = SerialImuHal(still_lines(60), reference_pitch_rad=0.0)
    core = StubCore(0.7, fault=True)
    rec = Rec(hal)
    res = b.run_bridge(hal, core, rec, sleep=lambda s: None)
    assert all(m["cmd"] == 0.0 for m in rec.msgs)
    assert hal.wheel_angle_rad == 0.0
    assert res.final_wheel_angle == 0.0
    assert len(core.dts) == 11
    assert all(d == pytest.approx(DT, abs=1e-12) for d in core.dts)


# ----- main() error paths and lifecycle (i, k, j) ---------------------------------------------

def test_bad_gains_path_returns_2(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(b, "GAINS_PATH", tmp_path / "missing.json")
    rc = b.main(["--mock", "--udp-port", str(free_udp_port())])
    assert rc == 2
    assert "error" in capsys.readouterr().err


def test_invalid_gains_json_message(monkeypatch, tmp_path, capsys):
    bad = tmp_path / "bad.json"
    bad.write_text("{nope")
    monkeypatch.setattr(b, "GAINS_PATH", bad)
    rc = b.main(["--mock", "--udp-port", str(free_udp_port())])
    assert rc == 2
    assert "invalid JSON" in capsys.readouterr().err


def test_run_bridge_oserror_returns_2(monkeypatch, capsys):
    def raiser(*a, **k):
        raise OSError("disk gone")
    monkeypatch.setattr(b, "run_bridge", raiser)
    rc = b.main(["--mock", "--udp-port", str(free_udp_port())])
    assert rc == 2
    assert "disk gone" in capsys.readouterr().err


def test_keyboard_interrupt_unarmed_returns_2(monkeypatch, capsys):
    monkeypatch.setattr(b, "_make_source", lambda args, clock=None: still_lines(60))
    monkeypatch.setattr(b, "run_bridge", lambda hal, core, pub, **kw: (_ for _ in ()).throw(KeyboardInterrupt()))
    rc = b.main(["--mock", "--udp-port", str(free_udp_port())])
    assert rc == 2
    assert "interrupted before armed" in capsys.readouterr().err


def test_keyboard_interrupt_armed_returns_0(monkeypatch):
    def fake_run(hal, core, pub, **kw):
        hal.wait_next_tick()
        raise KeyboardInterrupt
    monkeypatch.setattr(b, "_make_source", lambda args, clock=None: still_lines(60))
    monkeypatch.setattr(b, "run_bridge", fake_run)
    assert b.main(["--mock", "--udp-port", str(free_udp_port())]) == 0


def test_source_error_while_armed_returns_2(monkeypatch, capsys):
    def gen():
        yield from still_lines(50)
        raise RuntimeError("bad row")
    monkeypatch.setattr(b, "_make_source", lambda args, clock=None: gen())
    rc = b.main(["--mock", "--udp-port", str(free_udp_port())])
    assert rc == 2
    err = capsys.readouterr().err
    assert "source failed" in err and "bad row" in err


def test_pyserial_import_error_returns_2(monkeypatch, capsys):
    monkeypatch.delitem(sys.modules, "dof_serial", raising=False)
    monkeypatch.setitem(sys.modules, "serial", None)
    rc = b.main(["--port", "/dev/does-not-matter", "--udp-port", str(free_udp_port())])
    assert rc == 2
    assert "pyserial" in capsys.readouterr().err


def test_port_branch_end_to_end(monkeypatch):
    calls = {}
    fake = types.ModuleType("dof_serial")

    def serial_lines(port, baud=115200, timeout_s=1.0, reconnect=True, *, max_duration_s=None):
        calls["port"] = port
        calls["baud"] = baud
        return iter(still_lines(60))

    fake.serial_lines = serial_lines
    monkeypatch.setitem(sys.modules, "dof_serial", fake)
    made = []

    class SpyPoll(b.PolledSource):
        def __init__(self, it, **kw):
            made.append(self)
            super().__init__(it, **kw)

    monkeypatch.setattr(b, "PolledSource", SpyPoll)
    rc = b.main(["--port", "/dev/fake", "--baud", "9600", "--udp-port", str(free_udp_port())])
    assert rc == 0
    assert calls == {"port": "/dev/fake", "baud": 9600}
    assert len(made) == 1


def test_port_branch_never_armed_returns_2(monkeypatch):
    fake = types.ModuleType("dof_serial")
    fake.serial_lines = lambda port, baud=115200, **kw: iter(still_lines(10))
    monkeypatch.setitem(sys.modules, "dof_serial", fake)
    assert b.main(["--port", "/dev/fake", "--udp-port", str(free_udp_port())]) == 2


def test_hal_pub_and_record_closed_when_run_bridge_raises(monkeypatch, tmp_path):
    seen = {}

    class SpyHal(SerialImuHal):
        def close(self):
            seen["hal_closed"] = True
            super().close()

    class SpyPub(b.WheelPublisher):
        def close(self):
            seen["pub_closed"] = True
            super().close()

    def fake_run(hal, core, pub, **kw):
        seen["record"] = kw["record"]
        raise RuntimeError("boom")

    monkeypatch.setattr(b, "SerialImuHal", SpyHal)
    monkeypatch.setattr(b, "WheelPublisher", SpyPub)
    monkeypatch.setattr(b, "run_bridge", fake_run)
    monkeypatch.setattr(b, "_make_source", lambda args, clock=None: still_lines(60))
    rec_path = tmp_path / "r.csv"
    with pytest.raises(RuntimeError, match="boom"):
        b.main(["--mock", "--record", str(rec_path), "--udp-port", str(free_udp_port())])
    assert seen["hal_closed"] and seen["pub_closed"]
    assert seen["record"].closed


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
    assert elapsed < 1.0
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


def test_polled_source_thread_is_daemon_and_stops_after_close():
    src = b.PolledSource(itertools.repeat("x"), poll_s=0.05, maxsize=4)
    assert src._thread.daemon is True
    src.close()
    src._thread.join(2.0)
    assert not src._thread.is_alive()


@pytest.mark.parametrize("poll_s", [0, -1, float("nan"), True, "x"])
def test_polled_source_rejects_bad_poll_s(poll_s):
    with pytest.raises(ValueError, match="poll_s"):
        b.PolledSource([], poll_s=poll_s)
