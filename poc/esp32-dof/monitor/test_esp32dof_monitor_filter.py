#!/usr/bin/env python3
"""
Unit tests for the monitor's line filter (dof_monitor.is_frame_candidate / run skip rule).

Only lines starting with "IMU," are frame candidates. Other text lines are ignored and
never counted. A line starting with "IMU," that fails to parse counts as bad_frames.

Usage:
    cd poc/esp32-dof
    mamba run -n esp32-dof python3 -m pytest -q monitor/test_esp32dof_monitor_filter.py
"""

import io
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from dof_monitor import run, is_frame_candidate
from dof_sources import mock_lines as real_mock_lines

G1 = "IMU,12345,1234567890,0.500000,-9.806650,1.250000,10.500000,-5.500000,0.000000*52"
G0 = "IMU,0,0,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000*51"


def fast(n_s=1.0, drop_every=0):
    """Fast mock source (non-realtime)."""
    return list(real_mock_lines(duration_s=n_s, realtime=False, drop_every=drop_every))


def rows(text):
    """Extract data rows (lines starting with a digit after split)."""
    return [l for l in text.splitlines() if l.split() and l.split()[0].isdigit()]


def go(lines, quiet_every=1):
    out = io.StringIO()
    stats = run(iter(lines), out=out, quiet_every=quiet_every)
    return stats, out.getvalue()


def test_ignored_lines_do_not_change_bad():
    """Text, blank and whitespace lines are ignored: no bad, received unchanged."""
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


def test_good_frame_parsed():
    """Both golden frames parse with no bad frames."""
    for good in (G1, G0):
        stats, _ = go([good])
        assert stats.received == 1, good
        assert stats.bad_frames == 0, good


def test_interleaved_equals_clean():
    """Text lines between frames change nothing except the output of the text itself."""
    clean = fast()
    interleaved = []
    for f in clean:
        interleaved.append(f)
        interleaved.append("[imu] heartbeat")
    s_clean, out_clean = go(clean)
    s_mixed, out_mixed = go(interleaved)
    assert s_mixed.received == s_clean.received == 50
    assert s_mixed.dropped == s_clean.dropped
    assert s_mixed.out_of_order == s_clean.out_of_order
    assert s_mixed.resets == s_clean.resets
    assert s_mixed.bad_frames == s_clean.bad_frames == 0
    assert len(rows(out_mixed)) == len(rows(out_clean)) == 50


def test_crlf_frame_ok():
    """CRLF-terminated golden frame parses; CRLF text line is ignored."""
    stats, _ = go([G1 + "\r\n"])
    assert stats.received == 1
    assert stats.bad_frames == 0

    stats, _ = go(["[imu] ready\r\n"])
    assert stats.received == 0
    assert stats.bad_frames == 0


def test_prefix_boundary():
    """Lines that only look like IMU frames are ignored; 'IMU,' alone is bad."""
    for near in ["IMUX,1,2*00", "IMU 1,2", "imu,0,0,0,0,0,0,0,0*00"]:
        stats, _ = go([near])
        assert stats.bad_frames == 0, near
        assert stats.received == 0, near

    stats, _ = go(["IMU,"])
    assert stats.bad_frames == 1


def test_junk_then_frame_on_one_line():
    """Text glued before a frame: the frame is lost (intended), next clean frame parses."""
    stats, _ = go(["[imu] x" + G1, G0])
    assert stats.bad_frames == 0
    assert stats.received == 1


def test_partial_tail():
    """Torn frame: head kept -> bad; tail without head -> ignored."""
    stats, _ = go(["IMU,12345,1234567890,0.500000"])
    assert stats.bad_frames == 1
    assert stats.received == 0

    tail = ",0.500000,-9.806650,1.250000,10.500000,-5.500000,0.000000*52"
    stats, _ = go([tail])
    assert stats.bad_frames == 0
    assert stats.received == 0


def test_is_frame_candidate():
    """Unit: only the 'IMU,' prefix qualifies."""
    assert is_frame_candidate("IMU,x") is True
    assert is_frame_candidate("IMU") is False
    assert is_frame_candidate("") is False
    assert is_frame_candidate("[imu] IMU,1") is False
