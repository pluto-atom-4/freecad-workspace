#!/usr/bin/env python3
"""Tests for the monitor line filter: only lines starting with "IMU," are frame candidates."""

import io
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from dof_monitor import is_frame_candidate, run
from test_esp32dof_monitor import fast, rows  # shared helpers; G0/G1 are not defined there

G1 = "IMU,12345,1234567890,0.500000,-9.806650,1.250000,10.500000,-5.500000,0.000000*52"
G0 = "IMU,0,0,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000*51"


def go(lines, quiet_every=1):
    out = io.StringIO()
    stats = run(iter(lines), out=out, quiet_every=quiet_every)
    return stats, out.getvalue()


def test_ignored_lines_do_not_change_bad():
    """Text, blank and whitespace lines between frames leave bad_frames at 0."""
    base = fast()
    noise = ["[imu] accel=0.1,0.2,0.3", "[safe] torque off", "INFO,gyro_cal_skipped",
             "ERR,imu_init", "", "   ", "[frames] max_emit_us=420"]
    lines = []
    for i, f in enumerate(base):
        lines.append(f)
        lines.append(noise[i % len(noise)])
    stats, _ = go(lines)
    assert stats.bad_frames == 0
    assert stats.received == len(base)


def test_corrupt_imu_counts_bad():
    """Each corrupt IMU, line counts exactly once as bad."""
    golden_flipped = G1[:-2] + "53"
    for bad_line in ["IMU,1,2*ZZ",
                     "IMU,0,0,0,0,0,0,0,0*00",
                     golden_flipped,
                     "IMU,"]:
        stats, _ = go([bad_line])
        assert stats.bad_frames == 1, bad_line
        assert stats.received == 0, bad_line


def test_filter_does_not_reject_golden_frames():
    """Golden frames pass the filter and parse with no bad frames."""
    for good in (G1, G0):
        stats, _ = go([good])
        assert stats.received == 1, good
        assert stats.bad_frames == 0, good


def test_interleaved_equals_clean():
    """Text lines are never printed to out, so a noisy stream matches the clean one."""
    clean = fast()
    interleaved = []
    for f in clean:
        interleaved.append(f)
        interleaved.append("[imu] heartbeat")
    s_clean, out_clean = go(clean)
    s_mixed, out_mixed = go(interleaved)
    assert s_mixed.received == s_clean.received == len(clean)
    assert s_mixed.dropped == s_clean.dropped
    assert s_mixed.out_of_order == s_clean.out_of_order
    assert s_mixed.resets == s_clean.resets
    assert s_mixed.bad_frames == s_clean.bad_frames == 0
    assert len(rows(out_mixed)) == len(rows(out_clean)) == len(clean)


def test_crlf_frame_ok():
    """CRLF-terminated golden frame parses; CRLF text line is ignored."""
    stats, _ = go([G1 + "\r\n"])
    assert stats.received == 1
    assert stats.bad_frames == 0

    stats, _ = go(["[imu] ready\r\n"])
    assert stats.received == 0
    assert stats.bad_frames == 0


def test_prefix_boundary():
    """Lines that only look like IMU frames are ignored and not counted."""
    for near in ["IMUX,1,2*00", "IMU 1,2", "imu,0,0,0,0,0,0,0,0*00"]:
        stats, _ = go([near])
        assert stats.bad_frames == 0, near
        assert stats.received == 0, near


def test_whitespace_or_nul_prefixed_frame_ignored():
    """Documented tradeoff: a frame with leading space or NUL is ignored, not counted bad."""
    for prefixed in [" " + G1, "\x00" + G1]:
        stats, _ = go([prefixed])
        assert stats.bad_frames == 0, repr(prefixed)
        assert stats.received == 0, repr(prefixed)


def test_junk_then_frame_on_one_line():
    """Documented tradeoff: junk glued before a frame loses that frame; next clean frame parses."""
    stats, _ = go(["[imu] x" + G1, G0])
    assert stats.bad_frames == 0
    assert stats.received == 1


def test_partial_tail():
    """Torn frame: kept head is bad; a tail without its head is ignored."""
    stats, _ = go(["IMU,12345,1234567890,0.500000"])
    assert stats.bad_frames == 1
    assert stats.received == 0

    tail = ",0.500000,-9.806650,1.250000,10.500000,-5.500000,0.000000*52"
    stats, _ = go([tail])
    assert stats.bad_frames == 0
    assert stats.received == 0


def test_is_frame_candidate():
    """Only the 'IMU,' prefix qualifies."""
    assert is_frame_candidate("IMU,x") is True
    assert is_frame_candidate("IMU") is False
    assert is_frame_candidate("") is False
    assert is_frame_candidate("[imu] IMU,1") is False
