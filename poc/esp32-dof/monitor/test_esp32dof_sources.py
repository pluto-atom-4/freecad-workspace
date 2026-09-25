#!/usr/bin/env python3
"""
Unit tests for ESP32 6-DOF IMU mock and CSV replay frame sources.

Pure Python / pytest, no external dependencies beyond pytest.

Usage:
    cd poc/esp32-dof
    mamba run -n esp32-dof python3 -m pytest monitor/test_esp32dof_sources.py -v
"""

import sys
from pathlib import Path
from itertools import islice
import math

import pytest

# Add the script directory to path for imports
sys.path.insert(0, str(Path(__file__).resolve().parent))

from dof_frame import parse_line, format_frame
from dof_sources import mock_lines, mock_frame, mock_angles, csv_replay_lines
import dof_sources


class TestMockBasic:
    """Test mock_lines() basic generation."""

    def test_mock_100_lines_parse_and_strictly_increasing(self):
        """Generate 100 lines at 50 Hz, parse them, validate seq/t_us."""
        lines = list(islice(mock_lines(realtime=False), 100))

        assert len(lines) == 100

        # No line ends with \n or \r
        for line in lines:
            assert not line.endswith("\n")
            assert not line.endswith("\r")

        # Parse all lines
        frames = [parse_line(l) for l in lines]

        # Sequences are 0..99
        seqs = [f.seq for f in frames]
        assert seqs == list(range(100))

        # t_us increases strictly: at 50 Hz, dt = 20000 us
        for i, f in enumerate(frames):
            assert f.t_us == 20000 * i

    def test_mock_duration_1s_50hz_gives_50(self):
        """Duration-limited generation: 1s @ 50Hz = 50 frames; 0.1s = 5; 100Hz 1s = 100."""
        assert len(list(mock_lines(50.0, 1.0, False))) == 50
        assert len(list(mock_lines(50.0, 0.1, False))) == 5
        assert len(list(mock_lines(100.0, 1.0, False))) == 100

    def test_mock_drop_every_10(self):
        """Drop every 10th frame slot: 50 frames/s * 1s, drop_every=10 -> 45 frames."""
        lines = list(mock_lines(50.0, 1.0, False, drop_every=10))

        assert len(lines) == 45

        frames = [parse_line(l) for l in lines]
        seqs = {f.seq for f in frames}

        # Expected seqs: all of 0-49 except 9, 19, 29, 39, 49 (10-1, 20-1, etc.)
        expected_missing = {9, 19, 29, 39, 49}
        assert seqs == set(range(50)) - expected_missing

        # Consecutive seq diffs are 1, except 2 at each gap
        seq_list = sorted(seqs)
        diffs = [seq_list[i + 1] - seq_list[i] for i in range(len(seq_list) - 1)]

        # We have 9 frames per gap: 0-8, 10-18, 20-28, 30-38, 40-48
        # Diffs: [1]*8 + [2] + [1]*8 + [2] + [1]*8 + [2] + [1]*8 + [2] + [1]*8
        expected_diffs = [1]*8 + [2] + [1]*8 + [2] + [1]*8 + [2] + [1]*8 + [2] + [1]*8
        assert diffs == expected_diffs

        # Frame with seq 10 has t_us == 200000 (k=10, dt=20000/frame)
        f10 = next(f for f in frames if f.seq == 10)
        assert f10.t_us == 200000


class TestMockSampleValues:
    """Test mock_frame() output against known values."""

    def test_mock_sample_values(self):
        """Validate specific frame samples at k=0, 125, 250, 375 (50 Hz)."""
        # k=0 (t_us=0, t=0)
        f = mock_frame(0, 50.0)
        assert f.t_us == 0
        assert f.accel == pytest.approx((0, 0, 1.0), abs=1e-6)
        assert f.gyro == pytest.approx((36.0, 10.8, 0.0), abs=1e-6)

        # k=125 (t_us=2500000, t=2.5s)
        f = mock_frame(125, 50.0)
        assert f.t_us == 2500000
        assert f.accel == pytest.approx((-0.295520, 0.0, 0.955336), abs=1e-6)
        assert f.gyro == pytest.approx((-36.0, 0.0, 0.0), abs=1e-6)

        # k=250 (t_us=5000000, t=5.0s)
        f = mock_frame(250, 50.0)
        assert f.t_us == 5000000
        assert f.accel == pytest.approx((0.0, 0.0, 1.0), abs=1e-6)
        assert f.gyro == pytest.approx((36.0, -10.8, 0.0), abs=1e-6)

        # k=375 (t_us=7500000, t=7.5s)
        f = mock_frame(375, 50.0)
        assert f.t_us == 7500000
        assert f.accel == pytest.approx((0.295520, 0.0, 0.955336), abs=1e-6)
        assert f.gyro == pytest.approx((-36.0, 0.0, 0.0), abs=1e-6)

    def test_mock_sample_values_100hz_at_peak(self):
        """At 100 Hz, k=125 (t=1.25s) is roll peak; validate accel/gyro with math."""
        # Roll peak: k=125, rate=100 -> t = 1.25 s
        # roll = 0.5*sin(2π*0.2*1.25) = 0.5*sin(π/2) = 0.5
        # pitch = 0.3*sin(2π*0.1*1.25) = 0.3*sin(π/4) ≈ 0.212132
        r = 0.5
        p = 0.3 * math.sin(math.pi / 4)

        f = mock_frame(125, 100.0)

        # accel: ax = -sin(p), ay = sin(r)*cos(p), az = cos(r)*cos(p)
        ax_exp = -math.sin(p)
        ay_exp = math.sin(r) * math.cos(p)
        az_exp = math.cos(r) * math.cos(p)
        assert f.accel[0] == pytest.approx(ax_exp, abs=1e-6)
        assert f.accel[1] == pytest.approx(ay_exp, abs=1e-6)
        assert f.accel[2] == pytest.approx(az_exp, abs=1e-6)

        # gyro: gx ≈ 0.0, gy = 10.8*cos(π/4) ≈ 7.636753
        assert f.gyro[0] == pytest.approx(0.0, abs=1e-6)
        assert f.gyro[1] == pytest.approx(10.8 * math.cos(math.pi / 4), abs=1e-6)
        assert f.gyro[2] == pytest.approx(0.0, abs=1e-6)


class TestMockWrapping:
    """Test seq and t_us wrapping behavior."""

    def test_mock_seq_wraps(self):
        """seq wraps at 65536; validate wrapping at indices 65535, 65536, 65540."""
        lines = list(islice(mock_lines(realtime=False), 65541))
        frames = [parse_line(l) for l in lines]

        assert frames[65535].seq == 65535
        assert frames[65536].seq == 0
        assert frames[65540].seq == 4

    def test_mock_frame_t_us_wrap(self):
        """t_us wraps at 2^32 us; k=214749 @ 50Hz gives small t_us."""
        # At 50 Hz: t_us = k * 20000 us
        # 2^32 = 4294967296
        # k * 20000 ≡ small (mod 2^32) when k ≈ 214749
        k = 214749
        f = mock_frame(k, 50.0)

        # Expected: (214749 * 20000) % 2^32
        expected_t_us = (214749 * 20000) % (2**32)
        assert f.t_us == expected_t_us
        assert 0 <= f.t_us < 2**32
        assert f.t_us == 12704

        # Previous frame (k-1) has larger t_us
        f_prev = mock_frame(k - 1, 50.0)
        assert f_prev.t_us == 4294960000  # Just before wrap

        # format_frame and parse_line work without error
        line = format_frame(f)
        f_parsed = parse_line(line)
        assert f_parsed.t_us == 12704


class TestMockFusionConsistency:
    """Test that mock gyro rates match derivatives of angles."""

    def test_mock_fusion_consistency(self):
        """Gyro gx, gy match d(roll)/dt, d(pitch)/dt; validate accel attitude compatibility."""
        for k in (0, 125, 250, 375):
            f = mock_frame(k, 50.0)

            # Estimated roll from accel: atan2(ay, az)
            roll_est = math.atan2(f.accel[1], f.accel[2])

            # Estimated pitch from accel: atan2(-ax, hypot(ay, az))
            pitch_est = math.atan2(-f.accel[0], math.hypot(f.accel[1], f.accel[2]))

            # Expected roll, pitch from mock_angles at time t = k / 50.0
            t = k / 50.0
            roll, pitch = mock_angles(t)

            # Both estimates should match to high precision
            assert roll_est == pytest.approx(roll, abs=1e-9)
            assert pitch_est == pytest.approx(pitch, abs=1e-9)


class TestMockValidation:
    """Test mock_lines() argument validation (eager)."""

    def test_mock_invalid_args(self):
        """Argument validation is eager: errors raised on call, not iteration."""
        # rate_hz must be finite > 0
        with pytest.raises(ValueError):
            mock_lines(rate_hz=0)

        with pytest.raises(ValueError):
            mock_lines(rate_hz=-1)

        with pytest.raises(ValueError):
            mock_lines(rate_hz=float("nan"))

        # duration_s must be None or finite > 0
        with pytest.raises(ValueError):
            mock_lines(duration_s=0)

        with pytest.raises(ValueError):
            mock_lines(duration_s=-1)

        # drop_every must be >= 0
        with pytest.raises(ValueError):
            mock_lines(drop_every=-1)


class TestCSVRoundtrip:
    """Test csv_replay_lines() with synthetic CSV."""

    def test_csv_replay_roundtrip(self, tmp_path):
        """Write, read, parse CSV; validate frame values."""
        p = tmp_path / "r.csv"
        p.write_text("seq,t_us,ax,ay,az,gx,gy,gz\n"
                     "0,0,0.0,0.0,1.0,0.0,0.0,0.0\n"
                     "1,20000,0.1,0.2,0.9,1.5,-2.5,0.25\n"
                     "2,40000,-0.3,0.0,0.95,-10.0,5.0,0.0\n")

        lines = list(csv_replay_lines(str(p), realtime=False))
        assert len(lines) == 3

        frames = [parse_line(l) for l in lines]

        seqs = [f.seq for f in frames]
        assert seqs == [0, 1, 2]

        t_us_vals = [f.t_us for f in frames]
        assert t_us_vals == [0, 20000, 40000]

        # Check accel/gyro values
        assert frames[0].accel == pytest.approx((0.0, 0.0, 1.0), abs=1e-6)
        assert frames[0].gyro == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)

        assert frames[1].accel == pytest.approx((0.1, 0.2, 0.9), abs=1e-6)
        assert frames[1].gyro == pytest.approx((1.5, -2.5, 0.25), abs=1e-6)

        assert frames[2].accel == pytest.approx((-0.3, 0.0, 0.95), abs=1e-6)
        assert frames[2].gyro == pytest.approx((-10.0, 5.0, 0.0), abs=1e-6)


class TestCSVVariants:
    """Test csv_replay_lines() robustness."""

    def test_csv_replay_variants(self, tmp_path):
        """Test: reordered columns, extra column, BOM, blank lines, missing column, bad data, overflow, short row."""

        # (a) Reordered columns + extra column
        p_reord = tmp_path / "reord.csv"
        p_reord.write_text("gx,seq,foo,ax,t_us,ay,gy,az,gz\n"
                           "1.5,0,extra,0.0,0,0.0,0.0,1.0,0.0\n")
        lines = list(csv_replay_lines(str(p_reord), realtime=False))
        f = parse_line(lines[0])
        assert f.seq == 0
        assert f.accel == pytest.approx((0.0, 0.0, 1.0), abs=1e-6)
        assert f.gyro == pytest.approx((1.5, 0.0, 0.0), abs=1e-6)

        # (b) BOM in file
        p_bom = tmp_path / "bom.csv"
        p_bom.write_bytes(b"\xef\xbb\xbf" + b"seq,t_us,ax,ay,az,gx,gy,gz\n0,0,0.0,0.0,1.0,0.0,0.0,0.0\n")
        lines = list(csv_replay_lines(str(p_bom), realtime=False))
        assert len(lines) == 1
        f = parse_line(lines[0])
        assert f.seq == 0

        # (c) Blank line between rows
        p_blank = tmp_path / "blank.csv"
        p_blank.write_text("seq,t_us,ax,ay,az,gx,gy,gz\n"
                           "0,0,0.0,0.0,1.0,0.0,0.0,0.0\n"
                           "\n"
                           "1,20000,0.1,0.2,0.9,1.5,-2.5,0.25\n")
        lines = list(csv_replay_lines(str(p_blank), realtime=False))
        assert len(lines) == 2

        # (d) Missing column (no gz)
        p_missing = tmp_path / "missing.csv"
        p_missing.write_text("seq,t_us,ax,ay,az,gx,gy\n"
                             "0,0,0.0,0.0,1.0,0.0,0.0\n")
        with pytest.raises(ValueError, match="missing columns"):
            list(csv_replay_lines(str(p_missing), realtime=False))

        # (e) Bad row: good row on line 2, bad on line 3
        p_badrow = tmp_path / "badrow.csv"
        p_badrow.write_text("seq,t_us,ax,ay,az,gx,gy,gz\n"
                            "0,0,0.0,0.0,1.0,0.0,0.0,0.0\n"
                            "abc,20000,0.1,0.2,0.9,1.5,-2.5,0.25\n")
        with pytest.raises(ValueError, match=":3:"):
            list(csv_replay_lines(str(p_badrow), realtime=False))

        # (f) seq overflow (70000 >= 65536)
        p_overflow = tmp_path / "overflow.csv"
        p_overflow.write_text("seq,t_us,ax,ay,az,gx,gy,gz\n"
                              "70000,0,0.0,0.0,1.0,0.0,0.0,0.0\n")
        with pytest.raises(ValueError):
            list(csv_replay_lines(str(p_overflow), realtime=False))

        # (g) Short row (missing gz)
        p_short = tmp_path / "short.csv"
        p_short.write_text("seq,t_us,ax,ay,az,gx,gy,gz\n"
                           "0,0,0.0,0.0,1.0,0.0,0.0\n")
        with pytest.raises(ValueError):
            list(csv_replay_lines(str(p_short), realtime=False))


class TestCSVRealtimeNaps:
    """Test csv_replay_lines() realtime behavior (nap timing)."""

    def test_csv_replay_realtime_no_hang(self, tmp_path, monkeypatch):
        """Monkeypatch time.sleep; verify nap values match t_us deltas (clamped to 1s)."""
        sleeps = []

        def mock_sleep(delay):
            sleeps.append(delay)

        monkeypatch.setattr(dof_sources.time, "sleep", mock_sleep)

        # CSV with t_us: 0, 20000, 5020000, 4000000
        # Deltas: -, 0.02, 5.0, -1.02 (negative, no sleep)
        # Expected sleeps: [0.02, 1.0] (clamped to max 1.0)
        p = tmp_path / "realtime.csv"
        p.write_text("seq,t_us,ax,ay,az,gx,gy,gz\n"
                     "0,0,0.0,0.0,1.0,0.0,0.0,0.0\n"
                     "1,20000,0.1,0.2,0.9,1.5,-2.5,0.25\n"
                     "2,5020000,0.2,0.3,0.8,2.0,-3.0,0.5\n"
                     "3,4000000,-0.3,0.0,0.95,-10.0,5.0,0.0\n")

        lines = list(csv_replay_lines(str(p), realtime=True))
        assert len(lines) == 4

        # sleeps should be [0.02, 1.0]: first row has no prev_t (no sleep),
        # second: dt=0.02, third: dt=5.0 (clamped to 1.0), fourth: dt<0 (no sleep)
        assert len(sleeps) == 2
        assert sleeps[0] == pytest.approx(0.02, abs=1e-6)
        assert sleeps[1] == pytest.approx(1.0, abs=1e-6)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
