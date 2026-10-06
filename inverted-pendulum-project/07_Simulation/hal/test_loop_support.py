#!/usr/bin/env python3
"""
Loop support helpers test suite.

Verify is_saturated, should_log_row, time_limit_reached, require_value,
format_startup_rate_line, format_wheel_motors_line, format_data_row,
format_summary_lines, and static leak guard.

Usage:
    cd inverted-pendulum-project
    mamba run -n pendulum-tools python3 -m pytest -q 07_Simulation/hal/test_loop_support.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pytest
from hal.hal import HalFault, ImuSample, EncoderSample
from hal.loop_support import (
    is_saturated,
    should_log_row,
    time_limit_reached,
    require_value,
    format_startup_rate_line,
    format_wheel_motors_line,
    format_data_row,
    format_summary_lines,
)


class TestIsSaturated:
    """Test suite for is_saturated."""

    def test_is_saturated_default_limit_positive(self):
        """0.95 with default limit=1.0 is saturated."""
        assert is_saturated(0.95) is True

    def test_is_saturated_default_limit_negative(self):
        """0.9499 with default limit=1.0 is not saturated."""
        assert is_saturated(0.9499) is False

    def test_is_saturated_negative_value(self):
        """-1.0 with default limit=1.0 is saturated."""
        assert is_saturated(-1.0) is True

    def test_is_saturated_zero(self):
        """0.0 is not saturated."""
        assert is_saturated(0.0) is False

    def test_is_saturated_custom_limit_saturated(self):
        """1.9 with limit=2.0 is saturated."""
        assert is_saturated(1.9, limit=2.0) is True

    def test_is_saturated_custom_limit_not_saturated(self):
        """1.89 with limit=2.0 is not saturated."""
        assert is_saturated(1.89, limit=2.0) is False


class TestShouldLogRow:
    """Test suite for should_log_row."""

    def test_should_log_row_divisible_50_50(self):
        """50 % 50 == 0, should log."""
        assert should_log_row(50, 50) is True

    def test_should_log_row_not_divisible_49_50(self):
        """49 % 50 != 0, should not log."""
        assert should_log_row(49, 50) is False

    def test_should_log_row_divisible_100_50(self):
        """100 % 50 == 0, should log."""
        assert should_log_row(100, 50) is True

    def test_should_log_row_divisible_1_1(self):
        """1 % 1 == 0, should log."""
        assert should_log_row(1, 1) is True

    def test_should_log_row_zero_throttle_raises(self):
        """Throttle of 0 raises ZeroDivisionError."""
        with pytest.raises(ZeroDivisionError):
            should_log_row(1, 0)


class TestTimeLimitReached:
    """Test suite for time_limit_reached."""

    def test_time_limit_reached_no_limit(self):
        """t=1.0, max=0.0 (no limit), returns False."""
        assert time_limit_reached(1.0, 0.0) is False

    def test_time_limit_reached_at_limit(self):
        """t=30.0, max=30.0 (at limit), returns True."""
        assert time_limit_reached(30.0, 30.0) is True

    def test_time_limit_reached_before_limit(self):
        """t=29.999, max=30.0 (before limit), returns False."""
        assert time_limit_reached(29.999, 30.0) is False

    def test_time_limit_reached_negative_max(self):
        """t=5.0, max=-1.0 (negative max), returns False."""
        assert time_limit_reached(5.0, -1.0) is False


class TestRequireValue:
    """Test suite for require_value."""

    def test_require_value_none_raises_with_name(self):
        """None with name='pivot_left' raises HalFault with 'pivot_left' in message."""
        with pytest.raises(HalFault, match="pivot_left"):
            require_value(None, name="pivot_left")

    def test_require_value_float_returns(self):
        """0.25 returns 0.25 as float."""
        result = require_value(0.25, name="test")
        assert result == 0.25
        assert isinstance(result, float)


class TestFormatStartupRateLine:
    """Test suite for format_startup_rate_line."""

    def test_format_startup_rate_line_20ms(self):
        """20ms control rate formats to 'Control rate: 20ms (50.0Hz)'."""
        assert format_startup_rate_line(20) == "Control rate: 20ms (50.0Hz)"

    def test_format_startup_rate_line_25ms(self):
        """25ms control rate formats to 'Control rate: 25ms (40.0Hz)'."""
        assert format_startup_rate_line(25) == "Control rate: 25ms (40.0Hz)"


class TestFormatWheelMotorsLine:
    """Test suite for format_wheel_motors_line."""

    def test_format_wheel_motors_line_ones(self):
        """(1,1,1,1) formats with 2 decimal places."""
        result = format_wheel_motors_line(1, 1, 1, 1)
        assert result == "Wheel motors: mode=velocity, wheel_left maxVelocity=1.00 maxTorque=1.00, wheel_right maxVelocity=1.00 maxTorque=1.00"

    def test_format_wheel_motors_line_mixed(self):
        """(0.5, 2.0, 3.14159, 0.1) rounds to 2 decimal places."""
        result = format_wheel_motors_line(0.5, 2.0, 3.14159, 0.1)
        assert result == "Wheel motors: mode=velocity, wheel_left maxVelocity=0.50 maxTorque=2.00, wheel_right maxVelocity=3.14 maxTorque=0.10"


class TestFormatDataRow:
    """Test suite for format_data_row."""

    def test_format_data_row_base_case(self):
        """Base case with 4 extras: 15 tokens, third is pitch."""
        t = 1.234
        imu = ImuSample(1.234, 0.01, -0.1086, 0.02, (0.0, 0.3, 0.0))
        enc = EncoderSample(1.5, 2.5)
        extras = (0.0, 0.3, 0.1, 0.2)
        result = format_data_row(t, imu, enc, 0.25, -0.25, extras)
        tokens = result.split()
        assert len(tokens) == 15
        assert tokens[2] == "-0.1086"
        expected = "1.234 0.0100 -0.1086 0.0200 0.0000 0.0000 0.0000 1.5000 2.5000 0.2500 -0.2500 0.0000 0.3000 0.1000 0.2000"
        assert result == expected

    def test_format_data_row_five_extras(self):
        """With 5 extras: 16 tokens, ends with ' -0.5000'."""
        t = 1.234
        imu = ImuSample(1.234, 0.01, -0.1086, 0.02, (0.0, 0.3, 0.0))
        enc = EncoderSample(1.5, 2.5)
        extras = (0.0, 0.3, 0.1, 0.2, -0.5)
        result = format_data_row(t, imu, enc, 0.25, -0.25, extras)
        tokens = result.split()
        assert len(tokens) == 16
        assert result.endswith(" -0.5000")


class TestFormatSummaryLines:
    """Test suite for format_summary_lines."""

    def test_format_summary_lines_empty_fire_times(self):
        """Empty fire_times and 0 saturation: one line."""
        lines = format_summary_lines([], 20, 0, 0)
        assert lines == ["Saturation events: 0 of 0 control steps"]

    def test_format_summary_lines_one_fire(self):
        """One fire time: no frequency line (need >= 2), saturation line only."""
        lines = format_summary_lines([0.032], 20, 0, 1)
        assert lines == ["Saturation events: 0 of 1 control steps"]

    def test_format_summary_lines_multi_fire_with_saturation(self):
        """Multiple fires [0.032,0.048,0.064,0.080,0.112] with 2 saturation out of 5."""
        lines = format_summary_lines([0.032, 0.048, 0.064, 0.080, 0.112], 20, 2, 5)
        assert len(lines) == 2
        assert lines[0] == "Control rate: 50.00Hz (target 50.00Hz), periods 16.0-32.0ms (target 20ms), max deviation 12.00ms"
        assert lines[1] == "Saturation events: 2 of 5 control steps"

    def test_format_summary_lines_higher_freq(self):
        """Multiple fires [0.032,0.048,0.064] yield higher frequency."""
        lines = format_summary_lines([0.032, 0.048, 0.064], 20, 0, 3)
        assert len(lines) == 2
        assert lines[0] == "Control rate: 62.50Hz (target 50.00Hz), periods 16.0-16.0ms (target 20ms), max deviation 4.00ms"
        assert lines[1] == "Saturation events: 0 of 3 control steps"


class TestRegressionPinStageC:
    """Regression test for Stage C contract: jitter_budget integration."""

    def test_format_summary_lines_with_webots_pattern(self):
        """Use real 16,16,16,32 ms fire pattern starting at 0.032 for 41 fires."""
        from hal.jitter_budget import webots_fire_periods

        fire_periods = webots_fire_periods(41)  # seconds; first interval from t=0 is 0.032
        fire_times = []
        t = 0.0
        for period in fire_periods:
            t += period
            fire_times.append(t)
        assert fire_times[0] == pytest.approx(0.032)

        lines = format_summary_lines(fire_times, 20, 0, 41)
        assert len(lines) >= 1

        import re
        freq_match = re.match(r"^Control rate: ([0-9.]+)Hz", lines[0])
        assert freq_match is not None, f"Frequency regex failed on: {lines[0]}"
        freq_hz = float(freq_match.group(1))
        assert freq_hz >= 50.00


class TestStaticLeakGuard:
    """Static guard: forbidden Webots/controller keywords must not appear in loop_support.py."""

    def test_no_forbidden_imports_in_source(self):
        """loop_support.py must not contain getRollPitchYaw, getValues, setVelocity, getDevice, or 'import numpy'."""
        loop_support_path = Path(__file__).resolve().parent / "loop_support.py"
        source = loop_support_path.read_text()

        # Build forbidden strings dynamically to avoid literal presence in this test file
        forbidden = [
            "get" + "RollPitchYaw",
            "get" + "Values",
            "set" + "Velocity",
            "get" + "Device",
            "import numpy",
        ]

        for forbidden_str in forbidden:
            assert forbidden_str not in source, \
                f"loop_support.py contains forbidden substring: {forbidden_str!r}"
