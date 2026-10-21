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

from dof_frame import Frame, format_frame, parse_line  # noqa: E402
from dof_fusion import ComplementaryFilter  # noqa: E402
from dof_sources import csv_replay_lines, mock_lines  # noqa: E402
from dof_hal import STALE_S, SerialImuHal  # noqa: E402
from hal.hal import HAL_CONTRACT_VERSION, Hal, HalFault  # noqa: E402

ACC = (-math.sin(0.2), 0.0, math.cos(0.2))   # pitch 0.2 rad, roll 0
G = (1.0, -2.0, 0.5)                          # constant gyro bias used in still windows
REF_LQR = -0.1086
ABS = 1e-6                                    # %.6f formatting of the frame floats


def tilt(pitch):
    return (-math.sin(pitch), 0.0, math.cos(pitch))


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


def _boom_after(items, exc):
    yield from items
    raise exc


class FakeClock:
    def __init__(self, now=0.0):
        self.now = now

    def __call__(self):
        return self.now


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
    # the arming tick samples the last window frame at t=0; the 3 extra frames advance 0.02 each
    assert [s.t_s for s in samples] == pytest.approx([0.0, 0.02, 0.04, 0.06])


# 3. Arm rejection and gate ----------------------------------------------------------------------

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


@pytest.mark.parametrize("axis", [0, 1, 2])
@pytest.mark.parametrize("span,arms", [(5.0, True), (5.1, False)])
def test_arm_span_boundary_per_axis(axis, span, arms):
    base = (0.0, -2.0, 0.5)
    gyros = []
    for i in range(50):
        g = list(base)
        if i % 2:
            g[axis] = base[axis] + span
        gyros.append(tuple(g))
    hal = SerialImuHal(iter(gyro_run(gyros)))
    hal.wait_next_tick()
    if arms:
        assert hal.armed
        hal.read_imu()
    else:
        assert not hal.armed
        assert f"gyro {'xyz'[axis]}" in hal.arm_error
        with pytest.raises(HalFault):
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


def test_explicit_arm_succeeds_then_wait_holds_without_consuming():
    lines = still(51)
    source = iter(lines)
    hal = SerialImuHal(source)
    hal.arm()
    assert hal.armed
    assert hal.wait_next_tick() == pytest.approx(0.02)
    assert list(source) == lines[50:]          # the hold tick consumed no frame
    assert hal.now_s() == 0.0


def test_explicit_arm_raises_on_poll_then_retry_arms():
    hal = SerialImuHal(iter([None] + still(50)))
    with pytest.raises(HalFault, match="retry"):
        hal.arm()
    assert not hal.armed
    hal.arm()
    assert hal.armed


def test_explicit_arm_then_wait_nominal_then_real_dt():
    lines = still(50) + [mk(50, 49 * 20000 + 40000)]
    hal = SerialImuHal(iter(lines))
    hal.arm()
    assert hal.wait_next_tick() == pytest.approx(0.02)   # hold: no frame consumed
    assert hal.last_seq == 49
    assert hal.wait_next_tick() == pytest.approx(0.04)   # real dt of frame 50, not nominal
    assert hal.last_seq == 50


def test_arm_twice_does_not_consume_stream():
    lines = still(51)
    source = iter(lines)
    hal = SerialImuHal(source)
    hal.arm()
    hal.arm()                                            # already armed: no-op
    assert list(source) == lines[50:]


def test_arm_window_gap_boundary_is_strict():
    stale_us = round(STALE_S * 1e6)                      # 200000 us: gap == stale_s
    lines = still(25) + [mk(25, 24 * 20000 + stale_us)]
    lines += [mk(26 + j, 24 * 20000 + stale_us + (j + 1) * 20000) for j in range(24)]
    hal = SerialImuHal(iter(lines))
    assert hal.wait_next_tick() == pytest.approx(0.02)   # accepted: 50-frame window in one call
    assert hal.armed
    over_us = round(1.05 * STALE_S * 1e6)                # 210000 us: 1.05 * stale_s
    lines = still(25) + [mk(25, 24 * 20000 + over_us)]
    lines += [mk(26 + j, 24 * 20000 + over_us + (j + 1) * 20000) for j in range(24)]
    hal = SerialImuHal(iter(lines))
    hal.wait_next_tick()
    assert not hal.armed
    assert "gap" in hal.arm_error


def test_run_frame_gap_of_1_05_stale_s_is_stale():
    over_us = round(1.05 * STALE_S * 1e6)
    hal = SerialImuHal(iter(still(50) + [mk(50, 980000 + over_us)]))
    hal.wait_next_tick()
    assert hal.wait_next_tick() == pytest.approx(0.02)
    with pytest.raises(HalFault, match="stale"):
        hal.read_imu()
    assert hal.stale_count == 1


def test_tared_pitch_is_zero_after_reset_before_rearm():
    # tare stays 0.2 from the old window; the unarmed property must still report 0.0
    hal = SerialImuHal(iter(still(50, seq0=100) + [mk(0, 0)]))
    hal.wait_next_tick()
    assert hal.armed
    assert hal.wait_next_tick() == -1.0                  # reset, window never refills
    assert not hal.armed
    assert hal.tared_pitch_rad == 0.0


def test_poll_ticks_before_window_do_not_arm():
    hal = SerialImuHal(iter([None, None] + still(52)))
    for _ in range(2):
        assert hal.wait_next_tick() == pytest.approx(0.02)
        with pytest.raises(HalFault):
            hal.read_imu()
        assert not hal.armed
    assert hal.wait_next_tick() == pytest.approx(0.02)   # window completes on this tick
    assert hal.armed


def test_in_window_device_gap_restarts_window():
    # frames 0..24 contiguous, then a 0.25 s gap at frame 25 (inside the arm window)
    lines = still(25) + [mk(25, 24 * 20000 + 250000)]
    lines += [mk(26 + j, 24 * 20000 + 250000 + (j + 1) * 20000) for j in range(49)]
    hal = SerialImuHal(iter(lines))
    assert hal.wait_next_tick() == pytest.approx(0.02)
    assert not hal.armed
    assert "gap" in hal.arm_error
    assert hal.wait_next_tick() == pytest.approx(0.02)   # the 50-frame window from the gap frame
    assert hal.armed
    assert hal.arm_error is None


# 4. Deg/s -> rad/s and the one sign constant ---------------------------------------------------

def _offset_then_gyro(**kw):
    lines = still(50) + gyro_run([(10.0, 16.0, 0.5)], seq0=50, t0_us=50 * 20000)
    hal = SerialImuHal(iter(lines), **kw)
    hal.wait_next_tick()          # arm (arming tick)
    hal.wait_next_tick()          # frame 50
    return hal, hal.read_imu()


def test_gyro_deg_to_rad_and_tared_pitch():
    hal, s = _offset_then_gyro()
    assert s.gyro_rad_s == pytest.approx((math.radians(9), math.radians(18), 0.0))
    expected = 0.98 * 0.02 * math.radians(18)     # alpha*(prev + gyro*dt) + (1-alpha)*accel, tared
    assert hal.tared_pitch_rad == pytest.approx(expected, abs=ABS)
    assert s.pitch_rad == pytest.approx(expected, abs=ABS)


def test_pitch_sign_flips_pitch_and_gyro_y():
    hal, s = _offset_then_gyro(pitch_sign=-1.0)
    assert s.pitch_rad == pytest.approx(-0.98 * 0.02 * math.radians(18), abs=ABS)
    assert s.gyro_rad_s[1] == pytest.approx(-math.radians(18))
    assert s.gyro_rad_s[0] == pytest.approx(math.radians(9))
    assert hal.tared_pitch_rad == pytest.approx(-0.98 * 0.02 * math.radians(18), abs=ABS)


def test_pitch_sign_leaves_gz_unchanged():
    # gz is 2.5 dps against a 0.5 bias: corrected 2.0 dps (0.0 would hide a negated gz)
    lines = still(50) + gyro_run([(10.0, 16.0, 2.5)], seq0=50, t0_us=50 * 20000)
    hal = SerialImuHal(iter(lines), pitch_sign=-1.0)
    hal.wait_next_tick()
    hal.wait_next_tick()
    s = hal.read_imu()
    assert s.gyro_rad_s[2] == pytest.approx(math.radians(2.0))


def test_filter_uses_real_dt_0_04_after_wide_frame():
    lines = still(50) + [mk(50, 49 * 20000 + 40000, gyro=(10.0, 16.0, 0.5))]
    hal = SerialImuHal(iter(lines))
    hal.wait_next_tick()
    assert hal.wait_next_tick() == pytest.approx(0.04)
    # pred = tare + rad(18)*0.04; pitch = 0.98*pred + 0.02*accel_pitch; tared = pitch - tare
    assert hal.tared_pitch_rad == pytest.approx(0.98 * math.radians(18) * 0.04, abs=1e-9)
    assert hal.tared_pitch_rad != pytest.approx(0.98 * math.radians(18) * 0.02, abs=1e-6)


def test_replay_bias_and_tare_match_independent_computation():
    gyros = [(1.0 + 0.01 * i, -2.0 + 0.02 * i, 0.5 - 0.01 * i) for i in range(50)]
    accels = [tilt(0.2 + 0.001 * i) for i in range(50)]
    ts, t = [], 0
    for i in range(50):
        ts.append(t)
        t += 40000 if i % 5 == 4 else 20000       # varied dt inside the window
    lines = [mk(i, ts[i], gyro=gyros[i], accel=accels[i]) for i in range(50)]
    hal = SerialImuHal(iter(lines))
    hal.wait_next_tick()
    s = hal.read_imu()

    frames = [parse_line(line) for line in lines]
    bias = [sum(f.gyro[k] for f in frames) / 50 for k in range(3)]
    flt = ComplementaryFilter(alpha=0.98)
    pitches = []
    for i, f in enumerate(frames):
        dt = 0.0 if i == 0 else (f.t_us - frames[i - 1].t_us) / 1e6
        corrected = tuple(f.gyro[k] - bias[k] for k in range(3))
        _, p, _ = flt.update(f.accel, corrected, dt)
        pitches.append(p)
    tare = sum(pitches) / 50
    last = frames[-1]
    assert s.pitch_rad == pytest.approx(pitches[-1] - tare, abs=1e-9)
    assert s.gyro_rad_s == pytest.approx(
        tuple(math.radians(last.gyro[k] - bias[k]) for k in range(3)), abs=1e-9)


def test_bias_is_mean_not_min_max_midpoint():
    # skewed x gyro: mean 1.18, midpoint of min/max 2.5; span 3.0 is within the gate
    gx = [4.0 if i < 3 else 1.0 for i in range(50)]
    hal = SerialImuHal(iter(gyro_run([(g, -2.0, 0.5) for g in gx])))
    hal.wait_next_tick()
    s = hal.read_imu()
    assert s.gyro_rad_s[0] == pytest.approx(math.radians(1.0 - 1.18))


def test_roll_neither_signed_nor_tared_with_pitch_sign_and_tare():
    r, p = 0.1, 0.2
    accel = (-math.sin(p), math.sin(r) * math.cos(p), math.cos(r) * math.cos(p))
    hal = SerialImuHal(iter(still(53, accel=accel)), pitch_sign=-1.0)
    hal.wait_next_tick()
    s = hal.read_imu()
    assert hal.tared_pitch_rad == pytest.approx(0.0, abs=ABS)   # tare 0.2 == fused pitch
    assert s.roll_rad == pytest.approx(r, abs=ABS)              # not negated, not tared
    assert s.yaw_rad == 0.0


# 5. dt semantics -----------------------------------------------------------------------------

def test_dt_contiguous_is_0_02():
    lines = still(50) + still(3, seq0=50, t0_us=1000000)
    hal = SerialImuHal(iter(lines))
    dts = [hal.wait_next_tick() for _ in range(4)]
    assert dts == [pytest.approx(0.02)] * 4


def test_dt_after_one_dropped_frame_is_0_04():
    lines = still(50) + [mk(51, 49 * 20000 + 40000)]
    hal = SerialImuHal(iter(lines))
    assert hal.wait_next_tick() == pytest.approx(0.02)       # arming tick
    assert hal.wait_next_tick() == pytest.approx(0.04)
    assert hal.stats.dropped == 1
    assert hal.last_seq == 51


# 6. Stale -------------------------------------------------------------------------------------

def test_device_gap_is_stale_for_one_tick_and_advances_time():
    lines = still(50) + [mk(50, 49 * 20000 + 250000), mk(51, 49 * 20000 + 270000)]
    hal = SerialImuHal(iter(lines))
    hal.wait_next_tick()
    hal.read_imu()
    assert hal.wait_next_tick() == pytest.approx(0.02)   # stale tick returns nominal dt
    assert hal.now_s() == pytest.approx(0.25)            # time advanced by the real gap
    with pytest.raises(HalFault, match="stale"):
        hal.read_imu()
    assert hal.wait_next_tick() == pytest.approx(0.02)
    assert hal.now_s() == pytest.approx(0.27)
    hal.read_imu()
    assert hal.stale_count == 1


def test_device_gap_reinitialises_pitch_from_accel_not_blended():
    acc_b = tilt(0.5)
    lines = still(50) + [mk(50, 980000 + 250000, accel=acc_b), mk(51, 1230000 + 20000, accel=acc_b)]
    hal = SerialImuHal(iter(lines))
    hal.wait_next_tick()
    hal.read_imu()
    assert hal.wait_next_tick() == pytest.approx(0.02)
    with pytest.raises(HalFault):
        hal.read_imu()
    assert hal.wait_next_tick() == pytest.approx(0.02)
    # re-initialised from accel at pitch 0.5; blending with the old 0.2 would give pitch_rad ~0.006
    assert hal.read_imu().pitch_rad == pytest.approx(0.5 - 0.2, abs=1e-5)


def test_two_consecutive_stale_polls_count_once():
    clock = FakeClock(0.0)
    lines = still(50) + [None, None] + still(1, seq0=50, t0_us=1000000)
    hal = SerialImuHal(iter(lines), clock=clock)
    hal.wait_next_tick()
    hal.read_imu()
    clock.now = 0.3
    assert hal.wait_next_tick() == pytest.approx(0.02)
    assert hal.stale_count == 1
    with pytest.raises(HalFault, match="stale"):
        hal.read_imu()
    assert hal.wait_next_tick() == pytest.approx(0.02)
    assert hal.stale_count == 1
    with pytest.raises(HalFault):
        hal.read_imu()
    assert hal.wait_next_tick() == pytest.approx(0.02)   # frame 50 clears the stale flag
    hal.read_imu()
    assert hal.stale_count == 1


def test_stale_boundaries_are_strict():
    # device: dt == stale_s (0.2) is not stale (rule is dt > stale_s)
    hal = SerialImuHal(iter(still(50) + [mk(50, 980000 + 200000)]))
    hal.wait_next_tick()
    assert hal.wait_next_tick() == pytest.approx(0.2)
    hal.read_imu()
    assert hal.stale_count == 0
    # wall clock: gap == stale_s is not stale; a larger gap is
    clock = FakeClock(0.0)
    hal = SerialImuHal(iter(still(50) + [None, None]), clock=clock)
    hal.wait_next_tick()
    clock.now = 0.2
    hal.wait_next_tick()
    hal.read_imu()
    assert hal.stale_count == 0
    clock.now = 0.4
    hal.wait_next_tick()
    with pytest.raises(HalFault):
        hal.read_imu()
    assert hal.stale_count == 1


def test_wall_clock_stale_on_poll_tick():
    clock = FakeClock(0.0)
    hal = SerialImuHal(iter(still(50) + [None] + still(1, seq0=50, t0_us=50 * 20000)), clock=clock)
    hal.wait_next_tick()
    hal.read_imu()
    now_before = hal.now_s()
    clock.now = 0.3
    assert hal.wait_next_tick() == pytest.approx(0.02)   # poll tick: nominal dt, not wall time
    assert hal.now_s() == now_before                     # poll does not advance device time
    with pytest.raises(HalFault, match="stale"):
        hal.read_imu()
    assert hal.wait_next_tick() == pytest.approx(0.02)   # next accepted frame clears it
    assert hal.now_s() == pytest.approx(now_before + 0.02)
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
    for text_lines in ("IMU,0,0", b"IMU,0,0"):
        with pytest.raises(ValueError, match="not str/bytes"):
            SerialImuHal(text_lines)


# 7. Reorder / duplicates ----------------------------------------------------------------------

def test_reorder_and_duplicates_skipped():
    s50 = mk(50, 1000000)
    lines = still(50) + [s50, s50, mk(51, 1020000), mk(50, 1000000)]
    hal = SerialImuHal(iter(lines))
    assert hal.wait_next_tick() == pytest.approx(0.02)   # arming tick
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
    # re-arm adds ONE nominal step to the previous sample (t=0), not the ~1 s window (documented)
    assert s.t_s == pytest.approx(0.02)
    assert hal.now_s() == pytest.approx(0.02)


def test_reset_during_moving_window_stays_unarmed():
    lines = (moving(30, seq0=100) + [mk(0, 0, gyro=(0.0, 0.0, 0.0))]
             + moving(50, seq0=1, t0_us=20000))
    hal = SerialImuHal(iter(lines))
    assert hal.wait_next_tick() == pytest.approx(0.02)   # re-armed window is moving: attempt fails
    with pytest.raises(HalFault, match="moving"):
        hal.read_imu()
    assert not hal.armed
    assert "moving" in hal.arm_error
    assert hal.rearm_count == 1


def test_device_reset_clears_stale_moving_arm_error():
    lines = moving(50, seq0=100) + [mk(0, 0)] + [None]
    hal = SerialImuHal(iter(lines))
    assert hal.wait_next_tick() == pytest.approx(0.02)   # first window fails "board moving"
    assert "moving" in hal.arm_error
    assert hal.wait_next_tick() == pytest.approx(0.02)   # reset frame consumed, then poll tick
    assert hal.arm_error is None
    assert hal.rearm_count == 1
    with pytest.raises(HalFault) as info:
        hal.read_imu()
    assert "moving" not in str(info.value)


# 9. Wrap ------------------------------------------------------------------------------------

def test_t_us_and_seq_wrap_keep_dt_and_monotonic_time():
    t0 = 2**32 - 300000
    lines = still(60, seq0=65520, t0_us=t0)
    hal = SerialImuHal(iter(lines))
    assert hal.wait_next_tick() == pytest.approx(0.02)   # arming tick
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
    for bad in (float("nan"), float("inf"), None, "x", "0.5", -float("inf")):
        hal.write_wheel_velocity(bad, 0.0)               # becomes 0.0, never raises
    assert hal.wheel_angle_rad == pytest.approx(0.02)


def test_wheel_huge_int_overflow_contributes_zero_and_does_not_raise():
    hal = SerialImuHal(iter(still(53)))
    hal.wait_next_tick()
    hal.write_wheel_velocity(10**400, 0)                 # float() OverflowError -> 0.0
    assert hal.wheel_angle_rad == 0.0


def test_wheel_write_after_end_of_stream_integrates_nothing():
    hal = SerialImuHal(iter(still(51)))
    assert hal.wait_next_tick() == pytest.approx(0.02)   # arming tick
    hal.write_wheel_velocity(0.5, 0.0)
    assert hal.wheel_angle_rad == pytest.approx(0.01)
    assert hal.wait_next_tick() == pytest.approx(0.02)   # frame 50
    assert hal.wait_next_tick() == -1.0                  # end of stream, no close()
    hal.write_wheel_velocity(0.5, 0.0)
    assert hal.wheel_angle_rad == pytest.approx(0.01)


def test_wheel_zero_dt_frame_integrates_nothing():
    hal = SerialImuHal(iter(still(50) + [mk(50, 49 * 20000)]))   # same t_us, next seq
    hal.wait_next_tick()
    assert hal.wait_next_tick() == 0.0
    hal.write_wheel_velocity(0.5, 0.0)
    assert hal.wheel_angle_rad == 0.0


def test_wheel_integrates_left_command_over_ticks():
    hal = SerialImuHal(iter(still(60)))
    for _ in range(10):
        dt = hal.wait_next_tick()
        assert dt == pytest.approx(0.02)
        hal.write_wheel_velocity(0.5, -0.9)              # right is not integrated
    assert hal.wheel_angle_rad == pytest.approx(0.10)


def test_wheel_uses_real_last_dt_after_wide_frame():
    hal = SerialImuHal(iter(still(50) + [mk(50, 49 * 20000 + 40000)]))
    hal.wait_next_tick()
    assert hal.wait_next_tick() == pytest.approx(0.04)
    hal.write_wheel_velocity(0.5, 0.0)
    assert hal.wheel_angle_rad == pytest.approx(0.5 * 0.04)


def test_wheel_after_stale_and_poll_ticks_uses_nominal_dt():
    # stale tick returns NOMINAL_DT_S, so a command written after it integrates 0.5 * 0.02
    hal = SerialImuHal(iter(still(50) + [mk(50, 980000 + 250000)]))
    hal.wait_next_tick()
    assert hal.wait_next_tick() == pytest.approx(0.02)
    hal.write_wheel_velocity(0.5, 0.0)                   # integrates regardless of stale state
    assert hal.wheel_angle_rad == pytest.approx(0.5 * 0.02)
    # poll tick returns NOMINAL_DT_S as well
    hal = SerialImuHal(iter(still(50) + [None]))
    hal.wait_next_tick()
    assert hal.wait_next_tick() == pytest.approx(0.02)
    hal.write_wheel_velocity(0.5, 0.0)
    assert hal.wheel_angle_rad == pytest.approx(0.5 * 0.02)


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


# 11. Interface and source errors -----------------------------------------------------------

class _NoReadSpy:
    def __iter__(self):
        return self

    def __next__(self):
        raise AssertionError("constructor must not read the source")


def test_constructor_does_not_read_source_and_no_encoders():
    hal = SerialImuHal(_NoReadSpy())
    assert isinstance(hal, Hal)
    assert hal.now_s() == 0.0
    assert hal.tared_pitch_rad == 0.0
    with pytest.raises(HalFault, match="no encoders"):
        hal.read_encoders()


def test_contract_version_tripwire():
    # Deliberate tripwire: a contract bump in hal.hal must be a conscious change here too.
    assert HAL_CONTRACT_VERSION == 1
    assert SerialImuHal.CONTRACT_VERSION == HAL_CONTRACT_VERSION


def test_roll_is_not_tared_and_reference_not_added_to_roll():
    accel = (0.0, math.sin(0.1), math.cos(0.1))
    hal = SerialImuHal(iter(still(53, accel=accel)), reference_pitch_rad=REF_LQR)
    hal.wait_next_tick()
    s = hal.read_imu()
    assert s.roll_rad == pytest.approx(0.1, abs=ABS)
    assert s.pitch_rad == pytest.approx(REF_LQR, abs=ABS)
    assert s.yaw_rad == 0.0


def test_read_imu_after_close_raises():
    hal = SerialImuHal(iter(still(53)))
    hal.wait_next_tick()
    hal.close()
    with pytest.raises(HalFault, match="closed"):
        hal.read_imu()


def test_source_error_while_unarmed_is_recorded():
    hal = SerialImuHal(_boom_after(still(10), OSError("port lost")))
    assert hal.wait_next_tick() == -1.0
    assert "port lost" in hal.source_error
    assert "window incomplete: 10/50" in hal.arm_error
    assert "source error" in hal.arm_error


def test_source_error_while_armed_is_recorded():
    hal = SerialImuHal(_boom_after(still(50), RuntimeError("boom")))
    assert hal.wait_next_tick() == pytest.approx(0.02)
    assert hal.source_error is None
    assert hal.wait_next_tick() == -1.0
    assert "boom" in hal.source_error
    assert hal.arm_error is None


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
    assert r.stdout.strip() == str(HAL_CONTRACT_VERSION)


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
    hal = SerialImuHal(iter(["[imu] x" + mk(0, 0), "IMUX,1,2"] + still(50)))
    hal.wait_next_tick()
    assert hal.bad_frames == 0
    assert hal.stats.received == 50


def test_corrupt_imu_line_counts_bad():
    hal = SerialImuHal(iter(["IMU,"] + still(50)))
    hal.wait_next_tick()
    assert hal.bad_frames == 1
    assert hal.armed


def test_non_str_items_skipped_silently():
    hal = SerialImuHal(iter([5, b"IMU,x", 3.5] + still(50)))
    assert hal.wait_next_tick() == pytest.approx(0.02)
    assert hal.armed
    assert hal.bad_frames == 0
    assert hal.stats.received == 50


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


class _SpySource:
    def __init__(self, items=(), raise_on_close=False):
        self._it = iter(items)
        self.closes = 0
        self._raise = raise_on_close

    def __iter__(self):
        return self

    def __next__(self):
        return next(self._it)

    def close(self):
        self.closes += 1
        if self._raise:
            raise RuntimeError("close failed")


def test_arm_after_end_of_stream_raises_with_window_reason():
    hal = SerialImuHal(iter(still(10)))
    assert hal.wait_next_tick() == -1.0
    with pytest.raises(HalFault, match="10/50"):
        hal.arm()
    assert not hal.armed


def test_arm_after_end_of_stream_when_armed_raises_source_exhausted():
    hal = SerialImuHal(iter(still(50)))
    assert hal.wait_next_tick() == pytest.approx(0.02)
    assert hal.wait_next_tick() == -1.0
    with pytest.raises(HalFault, match="source exhausted"):
        hal.arm()


def test_failed_moving_attempt_restarts_fresh_window():
    lines = moving(50) + still(60, seq0=50, t0_us=50 * 20000)
    hal = SerialImuHal(iter(lines))
    assert hal.wait_next_tick() == pytest.approx(0.02)
    assert not hal.armed
    assert "moving" in hal.arm_error
    assert hal.wait_next_tick() == pytest.approx(0.02)   # fresh window from the next frame
    assert hal.armed
    assert hal.arm_error is None


def test_wheel_negative_clamp_and_infinite_inputs_contribute_nothing():
    hal = SerialImuHal(iter(still(53)))
    hal.wait_next_tick()
    hal.write_wheel_velocity(-5.0, 0.0)                  # clamped to -1.0, times 0.02
    assert hal.wheel_angle_rad == pytest.approx(-0.02)
    for bad in (float("inf"), float("-inf")):
        h = SerialImuHal(iter(still(53)))
        h.wait_next_tick()
        h.write_wheel_velocity(bad, 0.0)
        assert h.wheel_angle_rad == 0.0


def test_close_forwards_once_and_is_idempotent():
    src = _SpySource(still(1))
    hal = SerialImuHal(src)
    hal.close()
    hal.close()
    assert src.closes == 1


def test_close_swallows_source_close_error():
    src = _SpySource(raise_on_close=True)
    hal = SerialImuHal(src)
    hal.close()                                          # must not raise
    assert src.closes == 1
    assert hal.wait_next_tick() == -1.0


def test_arm_after_close_raises_closed():
    hal = SerialImuHal(iter(still(50)))
    hal.close()
    with pytest.raises(HalFault, match="closed"):
        hal.arm()


def test_custom_arm_frames_changes_window_length():
    hal = SerialImuHal(iter(still(10)), arm_frames=10)
    assert hal.wait_next_tick() == pytest.approx(0.02)
    assert hal.armed
    hal = SerialImuHal(iter(still(10)))                  # default 50 frames: incomplete
    assert hal.wait_next_tick() == -1.0
    assert "10/50" in hal.arm_error


def test_arm_frames_two_is_accepted_and_arms():
    hal = SerialImuHal(iter(still(2)), arm_frames=2)
    assert hal.wait_next_tick() == pytest.approx(0.02)
    assert hal.armed
    hal.read_imu()
    hal = SerialImuHal(iter(still(1)), arm_frames=2)     # one frame: 1/2 incomplete
    assert hal.wait_next_tick() == -1.0
    assert "1/2" in hal.arm_error


def test_custom_arm_span_changes_gate():
    gyros = [(0.0 if i % 2 == 0 else 2.0, 0.0, 0.0) for i in range(50)]   # span 2.0
    hal = SerialImuHal(iter(gyro_run(gyros)), arm_span_dps=1.0)
    hal.wait_next_tick()
    assert not hal.armed
    assert "gyro x" in hal.arm_error
    hal = SerialImuHal(iter(gyro_run(gyros)))            # default 5.0 accepts span 2.0
    hal.wait_next_tick()
    assert hal.armed


def test_arm_span_zero_accepts_only_constant_gyro():
    hal = SerialImuHal(iter(still(50)), arm_span_dps=0.0)
    hal.wait_next_tick()
    assert hal.armed
    wobble = gyro_run([(0.0 if i % 2 == 0 else 0.001, 0.0, 0.0) for i in range(50)])
    hal = SerialImuHal(iter(wobble), arm_span_dps=0.0)
    hal.wait_next_tick()
    assert not hal.armed


def test_custom_stale_s_changes_stale_threshold():
    lines = still(50) + [mk(50, 980000 + 100000)]        # 0.1 s device gap
    hal = SerialImuHal(iter(lines), stale_s=0.05)
    hal.wait_next_tick()
    assert hal.wait_next_tick() == pytest.approx(0.02)   # stale under 0.05
    with pytest.raises(HalFault, match="stale"):
        hal.read_imu()
    hal = SerialImuHal(iter(lines))                      # default 0.2: not stale
    hal.wait_next_tick()
    assert hal.wait_next_tick() == pytest.approx(0.1)
    hal.read_imu()


def test_invalid_constructor_values_rejected():
    for kw in (dict(stale_s=float("inf")), dict(arm_span_dps=float("inf")),
               dict(pitch_sign=True)):
        with pytest.raises(ValueError):
            SerialImuHal(iter([]), **kw)


@pytest.mark.parametrize("ref", [True, "x", "0.1"])
def test_reference_pitch_bool_or_str_rejected(ref):
    with pytest.raises(ValueError, match="reference_pitch_rad"):
        SerialImuHal(iter([]), reference_pitch_rad=ref)


@pytest.mark.parametrize("span", ["x", "5"])
def test_arm_span_str_raises_value_error_not_type_error(span):
    with pytest.raises(ValueError, match="arm_span_dps"):
        SerialImuHal(iter([]), arm_span_dps=span)


@pytest.mark.parametrize("frames", [1, 5.0, 2.0])
def test_arm_frames_below_two_or_float_rejected(frames):
    with pytest.raises(ValueError, match="arm_frames"):
        SerialImuHal(iter([]), arm_frames=frames)


def test_pitch_sign_minus_one_with_reference_tared_excludes_reference():
    lines = still(50) + gyro_run([(10.0, 16.0, 0.5)], seq0=50, t0_us=50 * 20000)
    ref = -0.1086
    hal = SerialImuHal(iter(lines), reference_pitch_rad=ref, pitch_sign=-1.0)
    hal.wait_next_tick()
    hal.wait_next_tick()
    s = hal.read_imu()
    tared = -0.98 * 0.02 * math.radians(18)
    assert hal.tared_pitch_rad == pytest.approx(tared, abs=ABS)     # no reference
    assert s.pitch_rad == pytest.approx(tared + ref, abs=ABS)        # -(p - tare) + ref
    assert s.yaw_rad == 0.0


def test_reset_inside_initial_window_needs_full_window_after_reset():
    # reset frame = window frame 1; 48 more give 49 (not armed); the 50th arms
    lines = (still(30, seq0=100) + [mk(0, 0)] + still(48, seq0=1, t0_us=20000)
             + [None] + still(1, seq0=49, t0_us=49 * 20000))
    hal = SerialImuHal(iter(lines))
    assert hal.wait_next_tick() == pytest.approx(0.02)   # poll with 49 of 50 window frames
    assert not hal.armed
    assert hal.rearm_count == 1
    assert hal.wait_next_tick() == pytest.approx(0.02)   # 50th window frame arms
    assert hal.armed


def test_reset_then_failed_attempt_then_clean_window_arms_one_nominal_step():
    lines = (still(50, seq0=100) + [mk(0, 0)] + moving(49, seq0=1, t0_us=20000)
             + still(50, seq0=50, t0_us=1000000))
    hal = SerialImuHal(iter(lines))
    assert hal.wait_next_tick() == pytest.approx(0.02)
    assert hal.read_imu().t_s == 0.0
    assert hal.wait_next_tick() == pytest.approx(0.02)   # reset + 49 moving frames: fails
    assert not hal.armed
    assert "moving" in hal.arm_error
    with pytest.raises(HalFault):
        hal.read_imu()
    assert hal.wait_next_tick() == pytest.approx(0.02)   # clean window arms
    assert hal.armed
    assert hal.arm_error is None
    assert hal.read_imu().t_s == pytest.approx(0.02)     # one nominal step after re-arm
    assert hal.rearm_count == 1


def test_device_gap_stale_then_reset_while_armed():
    lines = (still(50, seq0=100) + [mk(150, 980000 + 250000)] + [mk(0, 0)]
             + still(49, seq0=1, t0_us=20000))
    hal = SerialImuHal(iter(lines))
    hal.wait_next_tick()
    hal.read_imu()
    assert hal.wait_next_tick() == pytest.approx(0.02)   # device gap: stale tick
    with pytest.raises(HalFault, match="stale"):
        hal.read_imu()
    assert hal.now_s() == pytest.approx(0.25)
    assert hal.wait_next_tick() == pytest.approx(0.02)   # reset re-arms from the reset frame
    assert hal.armed
    assert hal.rearm_count == 1
    assert hal.stale_count == 1
    assert hal.read_imu().t_s == pytest.approx(0.27)     # 0.25 real gap + one nominal step


def test_reset_during_explicit_arm_completes_then_holds():
    lines = still(30, seq0=100) + [mk(0, 0)] + still(49, seq0=1, t0_us=20000)
    hal = SerialImuHal(iter(lines))
    hal.arm()                                            # reset restarts the window inside arm()
    assert hal.armed
    assert hal.rearm_count == 1
    assert hal.now_s() == 0.0                            # reset before first successful arm: no step
    assert hal.wait_next_tick() == pytest.approx(0.02)   # hold: no frame consumed
    assert hal.read_imu().t_s == 0.0
