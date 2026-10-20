#!/usr/bin/env python3
"""Tests for dof_hal.SerialImuHal (mock/CSV lines only, no hardware, no Webots).

Usage:
    cd poc/esp32-dof
    mamba run -n esp32-dof python3 -m pytest -q monitor/test_esp32dof_hal.py
"""

import ast
import csv
import math
import subprocess
import sys
import textwrap
from itertools import islice
from pathlib import Path

import pytest

MONITOR = Path(__file__).resolve().parent
sys.path.insert(0, str(MONITOR))

from dof_frame import Frame, format_frame  # noqa: E402
from dof_sources import csv_replay_lines, mock_lines  # noqa: E402
from dof_hal import SerialImuHal  # noqa: E402
from hal.hal import HAL_CONTRACT_VERSION, Hal, HalFault  # noqa: E402

ACC = (-math.sin(0.2), 0.0, math.cos(0.2))   # pitch 0.2 rad, roll 0
G = (1.0, -2.0, 0.5)                          # constant gyro bias used in still windows
REF_LQR = -0.1086
ABS = 1e-6                                    # %.6f formatting of the frame floats


def mk(seq, t_us, gyro=G, accel=ACC):
    """One protocol line; seq and t_us are reduced to their wire ranges."""
    return format_frame(Frame(seq=seq % 65536, t_us=t_us % 2**32, accel=accel, gyro=gyro)).rstrip("\n")


def still(n, seq0=0, t0_us=0, dt_us=20000, accel=ACC, gyro=G):
    return [mk(seq0 + i, t0_us + i * dt_us, gyro=gyro, accel=accel) for i in range(n)]


def gyro_run(gyros, seq0=0, t0_us=0, dt_us=20000, accel=ACC):
    return [mk(seq0 + i, t0_us + i * dt_us, gyro=g, accel=accel) for i, g in enumerate(gyros)]


def moving(n, seq0=0, t0_us=0):
    """Alternating gx 0 / 10 dps: span 10 > 5 dps, cannot arm."""
    return gyro_run([(0.0 if i % 2 == 0 else 10.0, 0.0, 0.0) for i in range(n)],
                    seq0=seq0, t0_us=t0_us)


class FakeClock:
    def __init__(self, now=0.0):
        self.now = now

    def __call__(self):
        return self.now


def window_hal(extra, **kw):
    """HAL over a 50-frame still window followed by extra lines."""
    return SerialImuHal(iter(still(50) + extra), **kw)


# 1. Mock motion cannot arm -------------------------------------------------------------------

def test_mock_motion_cannot_arm():
    lines = list(islice(mock_lines(realtime=False), 60))
    hal = SerialImuHal(iter(lines))
    assert hal.wait_next_tick() == pytest.approx(0.02)
    with pytest.raises(HalFault, match="moving"):
        hal.read_imu()
    assert not hal.armed
    assert "moving" in hal.arm_error


# 2. Constant offset: pitch equals the reference -----------------------------------------------

@pytest.mark.parametrize("ref", [0.0, REF_LQR])
def test_constant_offset_pitch_equals_reference(ref):
    hal = SerialImuHal(iter(still(53)), reference_pitch_rad=ref)
    samples, dts = [], []
    for _ in range(4):
        dts.append(hal.wait_next_tick())
        samples.append(hal.read_imu())
    assert dts == [pytest.approx(0.02)] * 4
    assert hal.wait_next_tick() == -1.0
    for s in samples:
        assert s.pitch_rad == pytest.approx(ref, abs=ABS)
        assert s.gyro_rad_s == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
        assert s.yaw_rad == 0.0
    # first returned sample is at t=0; the 3 extra frames advance by 0.02 each
    assert [s.t_s for s in samples] == pytest.approx([0.0, 0.02, 0.04, 0.06])


# 3. Arm rejection ----------------------------------------------------------------------------

def test_span_over_limit_rejected():
    lines = gyro_run([(0.0 if i % 2 == 0 else 5.1, 0.0, 0.0) for i in range(50)])
    hal = SerialImuHal(iter(lines))
    assert hal.wait_next_tick() == pytest.approx(0.02)
    with pytest.raises(HalFault, match="moving"):
        hal.read_imu()
    assert "gyro x" in hal.arm_error


def test_span_under_limit_arms():
    lines = gyro_run([(0.0 if i % 2 == 0 else 4.9, 0.0, 0.0) for i in range(50)])
    hal = SerialImuHal(iter(lines))
    hal.wait_next_tick()
    assert hal.armed
    hal.read_imu()


def test_short_window_incomplete():
    hal = SerialImuHal(iter(still(49)))
    assert hal.wait_next_tick() == -1.0
    assert "49/50" in hal.arm_error
    with pytest.raises(HalFault):
        hal.read_imu()


def test_explicit_arm_raises_on_moving():
    hal = SerialImuHal(iter(moving(50)))
    with pytest.raises(HalFault, match="moving"):
        hal.arm()
    assert not hal.armed


def test_poll_ticks_before_window_do_not_arm():
    hal = SerialImuHal(iter([None, None] + still(52)))
    for _ in range(2):
        assert hal.wait_next_tick() == pytest.approx(0.02)
        with pytest.raises(HalFault):
            hal.read_imu()
        assert not hal.armed
    assert hal.wait_next_tick() == pytest.approx(0.02)   # window completes on this tick
    assert hal.armed


# 4. Deg/s -> rad/s and the one sign constant ---------------------------------------------------

def _offset_then_gyro(**kw):
    lines = still(50) + gyro_run([(10.0, 16.0, 0.5)], seq0=50, t0_us=50 * 20000)
    hal = SerialImuHal(iter(lines), **kw)
    hal.wait_next_tick()          # arm, hold
    hal.wait_next_tick()          # frame 50
    return hal, hal.read_imu()


def test_gyro_deg_to_rad_and_tared_pitch():
    hal, s = _offset_then_gyro()
    assert s.gyro_rad_s == pytest.approx((math.radians(9), math.radians(18), 0.0))
    assert s.gyro_rad_s == pytest.approx((0.15707963, 0.31415927, 0.0), abs=1e-7)
    assert hal.tared_pitch_rad == pytest.approx(0.98 * 0.02 * math.radians(18), abs=ABS)
    assert hal.tared_pitch_rad == pytest.approx(0.0061575216, abs=ABS)
    assert s.pitch_rad == pytest.approx(0.0061575216, abs=ABS)


def test_pitch_sign_flips_pitch_and_gyro_y():
    hal, s = _offset_then_gyro(pitch_sign=-1.0)
    assert s.pitch_rad == pytest.approx(-0.0061575216, abs=ABS)
    assert s.gyro_rad_s[1] == pytest.approx(-0.31415927)
    assert s.gyro_rad_s[0] == pytest.approx(0.15707963)
    assert hal.tared_pitch_rad == pytest.approx(-0.0061575216, abs=ABS)


def test_roll_is_not_tared():
    hal = SerialImuHal(iter(still(53, accel=(0.0, math.sin(0.1), math.cos(0.1)))))
    hal.wait_next_tick()
    s = hal.read_imu()
    assert s.roll_rad == pytest.approx(0.1, abs=ABS)
    assert s.pitch_rad == pytest.approx(0.0, abs=ABS)


# 5. dt semantics -----------------------------------------------------------------------------

def test_dt_contiguous_is_0_02():
    hal = window_hal(still(3, seq0=50, t0_us=1000000))
    dts = [hal.wait_next_tick() for _ in range(4)]
    assert dts == [pytest.approx(0.02)] * 4


def test_dt_after_one_dropped_frame_is_0_04():
    lines = still(50) + [mk(51, 49 * 20000 + 40000)]
    hal = SerialImuHal(iter(lines))
    assert hal.wait_next_tick() == pytest.approx(0.02)       # hold
    assert hal.wait_next_tick() == pytest.approx(0.04)
    assert hal.stats.dropped == 1
    assert hal.last_seq == 51


# 6. Stale -------------------------------------------------------------------------------------

def test_device_gap_is_stale_for_one_tick():
    lines = still(50) + [mk(50, 49 * 20000 + 250000), mk(51, 49 * 20000 + 270000)]
    hal = SerialImuHal(iter(lines))
    hal.wait_next_tick()
    hal.read_imu()
    assert hal.wait_next_tick() == pytest.approx(0.02)
    with pytest.raises(HalFault, match="stale"):
        hal.read_imu()
    assert hal.wait_next_tick() == pytest.approx(0.02)
    s = hal.read_imu()
    assert s.pitch_rad == pytest.approx(0.0, abs=ABS)
    assert hal.stale_count == 1


def test_wall_clock_stale_on_poll_tick():
    clock = FakeClock(0.0)
    hal = SerialImuHal(iter(still(50) + [None] + still(1, seq0=50, t0_us=50 * 20000)), clock=clock)
    hal.wait_next_tick()
    hal.read_imu()
    clock.now = 0.3
    assert hal.wait_next_tick() == pytest.approx(0.02)   # poll tick: no frame
    with pytest.raises(HalFault, match="stale"):
        hal.read_imu()
    assert hal.wait_next_tick() == pytest.approx(0.02)   # next accepted frame clears it
    hal.read_imu()
    assert hal.stale_count == 1


def test_bad_arguments_raise_value_error():
    bad = [
        dict(reference_pitch_rad=float("nan")),
        dict(arm_frames=1),
        dict(arm_frames=True),
        dict(arm_span_dps=-1.0),
        dict(stale_s=0.0),
        dict(pitch_sign=0.5),
        dict(wheel_display_sign=0),
        dict(clock=1),
        dict(alpha=2.0),
    ]
    for kw in bad:
        with pytest.raises(ValueError):
            SerialImuHal(iter([]), **kw)
    with pytest.raises(ValueError):
        SerialImuHal(None)


# 7. Reorder / duplicates ----------------------------------------------------------------------

def test_reorder_and_duplicates_skipped():
    s50 = mk(50, 1000000)
    lines = still(50) + [s50, s50, mk(51, 1020000), mk(50, 1000000)]
    hal = SerialImuHal(iter(lines))
    assert hal.wait_next_tick() == pytest.approx(0.02)   # arm + hold
    assert hal.wait_next_tick() == pytest.approx(0.02)   # seq 50
    assert hal.wait_next_tick() == pytest.approx(0.02)   # seq 51
    assert hal.wait_next_tick() == -1.0                  # trailing backward frame is skipped
    assert hal.stats.out_of_order == 2
    assert hal.last_seq == 51


# 8. Reset -----------------------------------------------------------------------------------

TILT_B = (0.0, math.sin(0.1), math.cos(0.1))


def test_reset_rearms_at_new_tilt():
    lines = still(50, seq0=100, t0_us=0) + still(50, seq0=0, t0_us=0, accel=TILT_B)
    hal = SerialImuHal(iter(lines))
    hal.wait_next_tick()
    hal.read_imu()
    assert hal.wait_next_tick() == pytest.approx(0.02)   # reset frame re-arms on the new window
    assert hal.armed
    assert hal.rearm_count == 1
    s = hal.read_imu()
    assert s.pitch_rad == pytest.approx(0.0, abs=ABS)
    assert s.roll_rad == pytest.approx(0.1, abs=ABS)
    assert s.t_s > 0.0


def test_reset_during_moving_window_stays_unarmed():
    lines = (moving(30, seq0=100) + [mk(0, 0, gyro=(0.0, 0.0, 0.0))]
             + moving(19, seq0=1, t0_us=20000))
    hal = SerialImuHal(iter(lines))
    assert hal.wait_next_tick() == -1.0                  # window never fills after the reset
    with pytest.raises(HalFault):
        hal.read_imu()
    assert not hal.armed
    assert hal.rearm_count == 1


# 9. Wrap ------------------------------------------------------------------------------------

def test_t_us_and_seq_wrap_keep_dt_and_monotonic_time():
    t0 = 2**32 - 300000
    lines = still(60, seq0=65520, t0_us=t0)
    hal = SerialImuHal(iter(lines))
    assert hal.wait_next_tick() == pytest.approx(0.02)   # hold
    samples = [hal.read_imu().t_s]
    for _ in range(9):
        assert hal.wait_next_tick() == pytest.approx(0.02)
        samples.append(hal.read_imu().t_s)
    assert samples == pytest.approx([0.02 * i for i in range(10)])
    assert hal.stats.resets == 0
    assert hal.stats.out_of_order == 0


# 10. Wheel ----------------------------------------------------------------------------------

def test_wheel_clamp_and_non_finite():
    hal = SerialImuHal(iter(still(53)))
    hal.wait_next_tick()
    hal.write_wheel_velocity(5.0, 5.0)                   # clamped to 1.0, times dt 0.02
    assert hal.wheel_angle_rad == pytest.approx(0.02)
    for bad in (float("nan"), float("inf"), None, "x", -float("inf")):
        hal.write_wheel_velocity(bad, 0.0)               # becomes 0.0, never raises
    assert hal.wheel_angle_rad == pytest.approx(0.02)


def test_wheel_integrates_left_command_over_ticks():
    hal = SerialImuHal(iter(still(60)))
    for _ in range(10):
        dt = hal.wait_next_tick()
        assert dt == pytest.approx(0.02)
        hal.write_wheel_velocity(0.5, -0.9)              # right is not integrated
    assert hal.wheel_angle_rad == pytest.approx(0.10)


def test_wheel_sign_and_close_hold():
    hal = SerialImuHal(iter(still(60)), wheel_display_sign=-1.0)
    for _ in range(10):
        hal.wait_next_tick()
        hal.write_wheel_velocity(0.5, 0.0)
    assert hal.wheel_angle_rad == pytest.approx(-0.10)
    hal.close()
    hal.write_wheel_velocity(0.5, 0.0)
    assert hal.wheel_angle_rad == pytest.approx(-0.10)
    assert hal.wait_next_tick() == -1.0


# 11. Interface -----------------------------------------------------------------------------

class _NoReadSpy:
    def __iter__(self):
        return self

    def __next__(self):
        raise AssertionError("constructor must not read the source")


def test_constructor_does_not_read_and_is_hal():
    hal = SerialImuHal(_NoReadSpy())
    assert isinstance(hal, Hal)
    assert hal.now_s() == 0.0
    assert hal.tared_pitch_rad == 0.0


def test_read_encoders_raises():
    with pytest.raises(HalFault):
        SerialImuHal(iter([])).read_encoders()


def test_contract_version():
    assert SerialImuHal.CONTRACT_VERSION == HAL_CONTRACT_VERSION == 1


# 12. Import isolation ----------------------------------------------------------------------

def test_import_without_serial_numpy_scipy_controller():
    code = textwrap.dedent(f"""
        import sys
        for name in ("serial", "numpy", "scipy", "controller"):
            sys.modules[name] = None
        sys.path.insert(0, {str(MONITOR)!r})
        import dof_hal
        print(dof_hal.SerialImuHal.CONTRACT_VERSION)
    """)
    r = subprocess.run([sys.executable, "-I", "-c", code], capture_output=True, text=True,
                       cwd="/")
    assert r.returncode == 0, r.stderr
    assert r.stdout.strip() == "1"


def test_module_ast_has_no_forbidden_imports():
    tree = ast.parse((MONITOR / "dof_hal.py").read_text(encoding="utf-8"))
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names |= {a.name.split(".")[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            names.add(node.module.split(".")[0])
    forbidden = {"controller", "serial", "numpy", "scipy",
                 "dof_serial", "dof_monitor", "dof_publisher"}
    assert not (names & forbidden), names & forbidden
    allowed = {"math", "numbers", "sys", "time", "collections", "pathlib",
               "dof_frame", "dof_fusion", "dof_stats", "hal"}
    assert names <= allowed, names - allowed


# 13. Line intake ---------------------------------------------------------------------------

def test_junk_prefixed_frame_ignored_uncounted():
    hal = SerialImuHal(iter(["[imu] x" + mk(0, 0)] + still(50)))
    hal.wait_next_tick()
    assert hal.bad_frames == 0
    assert hal.stats.received == 50


def test_corrupt_imu_line_counts_bad():
    hal = SerialImuHal(iter(["IMU,"] + still(50)))
    hal.wait_next_tick()
    assert hal.bad_frames == 1
    assert hal.armed


def test_csv_replay_of_still_rows(tmp_path):
    path = tmp_path / "still.csv"
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["seq", "t_us", "ax", "ay", "az", "gx", "gy", "gz"])
        for i in range(53):
            w.writerow([i, i * 20000, ACC[0], ACC[1], ACC[2], G[0], G[1], G[2]])
    hal = SerialImuHal(csv_replay_lines(path, realtime=False))
    assert hal.wait_next_tick() == pytest.approx(0.02)
    s = hal.read_imu()
    assert s.pitch_rad == pytest.approx(0.0, abs=ABS)
    assert s.gyro_rad_s == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    dts = [hal.wait_next_tick() for _ in range(3)]
    assert dts == [pytest.approx(0.02)] * 3
    assert hal.wait_next_tick() == -1.0
