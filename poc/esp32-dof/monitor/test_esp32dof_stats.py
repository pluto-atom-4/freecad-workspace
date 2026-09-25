#!/usr/bin/env python3
"""
Unit tests for StreamStats (dof_stats).

Usage:
    cd poc/esp32-dof
    mamba run -n esp32-dof python3 -m pytest -q monitor/test_esp32dof_stats.py
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from dof_stats import StreamStats


def feed(stats, pairs):
    """Helper: feed a list of (seq, t_us) pairs and return the list of booleans."""
    return [stats.update(s, t) for s, t in pairs]


class TestStreamStats:
    """Tests for StreamStats."""

    def test_sequential_no_drops(self):
        """Test: sequential frames with no drops."""
        stats = StreamStats()
        pairs = [(i, i * 20000) for i in range(10)]
        results = feed(stats, pairs)
        assert stats.received == 10
        assert stats.dropped == 0
        assert stats.out_of_order == 0
        assert stats.resets == 0
        assert stats.bad_frames == 0

    def test_single_skip_is_one_drop(self):
        """Test: single frame skip counts as one drop."""
        stats = StreamStats()
        pairs = [(i, i * 20000) for i in [0, 1, 2, 3, 4, 6, 7]]
        feed(stats, pairs)
        assert stats.dropped == 1
        assert stats.received == 7

    def test_multi_gap(self):
        """Test: multi-frame gap."""
        stats = StreamStats()
        pairs = [(i, i * 20000) for i in [0, 1, 5]]
        feed(stats, pairs)
        assert stats.dropped == 3

    def test_wrap_no_drops(self):
        """Test: seq wrapping with no drops."""
        stats = StreamStats()
        seqs = [65534, 65535, 0, 1]
        pairs = [(seqs[i], i * 20000) for i in range(4)]
        feed(stats, pairs)
        assert stats.dropped == 0
        assert stats.out_of_order == 0
        assert stats.resets == 0
        assert stats.received == 4

    def test_wrap_with_drop(self):
        """Test: seq wrapping with a drop."""
        stats = StreamStats()
        pairs = [(65534, 0), (0, 20000)]
        feed(stats, pairs)
        assert stats.dropped == 1

    def test_duplicate_is_out_of_order(self):
        """Test: duplicate frame is out_of_order."""
        stats = StreamStats()
        pairs = [(0, 0), (1, 20000), (1, 20000), (2, 40000)]
        results = feed(stats, pairs)
        assert results == [True, True, False, True]
        assert stats.out_of_order == 1
        assert stats.dropped == 0
        assert stats.received == 4

    def test_small_backward_is_out_of_order_and_prev_unchanged(self):
        """Test: small backward jump (<=50) is out_of_order."""
        stats = StreamStats()
        pairs = [(i, i * 20000) for i in [10, 11, 12, 11, 13]]
        feed(stats, pairs)
        assert stats.out_of_order == 1
        assert stats.dropped == 0
        assert stats.resets == 0

    def test_large_backward_is_reset(self):
        """Test: large backward jump (>50) is a reset."""
        stats = StreamStats()
        seqs = [100, 101, 102, 103, 104, 3, 4, 5]
        pairs = [(seqs[i], i * 20000) for i in range(8)]
        feed(stats, pairs)
        assert stats.resets == 1
        assert stats.out_of_order == 0
        assert stats.dropped == 0
        assert stats.received == 8
        assert stats.rate_hz == pytest.approx(50.0)

    def test_time_backwards_with_valid_seq_is_reset(self):
        """Test: device time going backwards is a reset."""
        stats = StreamStats()
        pairs = [(0, 1_000_000), (1, 0)]
        feed(stats, pairs)
        assert stats.resets == 1
        assert stats.dropped == 0
        assert stats.received == 2
        assert stats.rate_hz == 0.0

    def test_rate_50hz(self):
        """Test: rate_hz for 10 frames at 20000 us intervals."""
        stats = StreamStats()
        pairs = [(i, i * 20000) for i in range(10)]
        feed(stats, pairs)
        assert stats.rate_hz == pytest.approx(50.0)

    def test_rate_zero_when_fewer_than_2_frames(self):
        """Test: rate_hz is 0.0 when < 2 frames."""
        stats = StreamStats()
        assert stats.rate_hz == 0.0
        stats.update(0, 0)
        assert stats.rate_hz == 0.0

    def test_rate_window_is_last_50_frames(self):
        """Test: rate window captures last 50 frames."""
        stats = StreamStats()
        # First 10 frames at 100000 us intervals
        pairs = [(i, i * 100000) for i in range(10)]
        feed(stats, pairs)
        # Then 50 more frames at 20000 us intervals (from seq 10 to 59)
        pairs = [(i, 900000 + 20000 * (i - 9)) for i in range(10, 60)]
        feed(stats, pairs)
        assert stats.rate_hz == pytest.approx(50.0)
        assert len(stats._window) == 50

    def test_t_us_wrap_dt(self):
        """Test: t_us wrapping and dt computation."""
        stats = StreamStats()
        pairs = [(0, 2**32 - 20000), (1, 0), (2, 20000)]
        feed(stats, pairs)
        assert stats.last_dt_us == 20000
        assert stats.rate_hz == pytest.approx(50.0)
        assert stats.avg_rate_hz == pytest.approx(50.0)
        assert stats.resets == 0

    def test_duplicates_not_in_rate_window(self):
        """Test: duplicates don't affect rate window."""
        stats = StreamStats()
        pairs = [(0, 0), (1, 20000), (1, 20000), (2, 40000)]
        feed(stats, pairs)
        assert stats.rate_hz == pytest.approx(50.0)

    def test_last_dt_us(self):
        """Test: last_dt_us tracking."""
        stats = StreamStats()
        assert stats.last_dt_us is None
        stats.update(0, 0)
        assert stats.last_dt_us is None
        stats.update(1, 20000)
        assert stats.last_dt_us == 20000
        stats.update(1, 20000)  # duplicate
        assert stats.last_dt_us is None

    def test_note_bad(self):
        """Test: note_bad increments bad_frames."""
        stats = StreamStats()
        stats.note_bad()
        stats.note_bad()
        stats.note_bad()
        assert stats.bad_frames == 3
        assert stats.received == 0

    def test_avg_rate_and_elapsed(self):
        """Test: avg_rate_hz and elapsed_s."""
        stats = StreamStats()
        pairs = [(i, i * 20000) for i in range(10)]
        feed(stats, pairs)
        assert stats.avg_rate_hz == pytest.approx(50.0)
        assert stats.elapsed_s == pytest.approx(0.18)
        fresh = StreamStats()
        assert fresh.avg_rate_hz == 0.0

    def test_window_arg_validation(self):
        """Test: window parameter validation."""
        with pytest.raises(ValueError):
            StreamStats(window=1)
        with pytest.raises(ValueError):
            StreamStats(window=True)
