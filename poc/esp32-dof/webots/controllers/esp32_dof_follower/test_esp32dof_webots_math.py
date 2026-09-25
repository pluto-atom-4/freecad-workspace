#!/usr/bin/env python3
"""
Unit tests for Webots-side orientation parsing and Euler-to-axis-angle conversion.

Pure Python / pytest, no external dependencies beyond math and pytest.

Usage:
    cd poc/esp32-dof/webots/controllers/esp32_dof_follower
    mamba run -n esp32-dof python3 -m pytest test_esp32dof_webots_math.py -v
"""

import sys
from pathlib import Path

import math
import pytest

# Add the script directory to path for imports
sys.path.insert(0, str(Path(__file__).resolve().parent))

from dof_webots_math import parse_orientation_msg, euler_to_axis_angle


class TestParserValid:
    """Test: parser accepts valid JSON and returns correct tuples."""

    def test_valid_floats(self):
        """parse_orientation_msg() parses valid floats."""
        data = b'{"roll": 0.1, "pitch": -0.05, "yaw": 0.0}'
        result = parse_orientation_msg(data)
        assert result is not None
        roll, pitch, yaw = result
        assert abs(roll - 0.1) < 1e-9
        assert abs(pitch - (-0.05)) < 1e-9
        assert abs(yaw - 0.0) < 1e-9

    def test_ints_accepted(self):
        """parse_orientation_msg() accepts ints and converts to float."""
        data = b'{"roll": 1, "pitch": -2, "yaw": 0}'
        result = parse_orientation_msg(data)
        assert result is not None
        roll, pitch, yaw = result
        assert roll == 1.0
        assert pitch == -2.0
        assert yaw == 0.0

    def test_extra_keys_ignored(self):
        """parse_orientation_msg() ignores extra keys."""
        data = b'{"roll": 0.1, "pitch": -0.05, "yaw": 0.0, "seq": 12, "extra": "value"}'
        result = parse_orientation_msg(data)
        assert result is not None
        roll, pitch, yaw = result
        assert abs(roll - 0.1) < 1e-9
        assert abs(pitch - (-0.05)) < 1e-9
        assert abs(yaw - 0.0) < 1e-9

    def test_seq_optional(self):
        """parse_orientation_msg() works without seq key."""
        data = b'{"roll": 0.2, "pitch": 0.3, "yaw": 0.4}'
        result = parse_orientation_msg(data)
        assert result is not None


class TestParserRejectionsInvalid:
    """Test: parser rejects invalid input."""

    def test_garbage_bytes(self):
        """parse_orientation_msg() rejects garbage bytes."""
        result = parse_orientation_msg(b"\xff\xfe\x00\x01")
        assert result is None

    def test_empty_bytes(self):
        """parse_orientation_msg() rejects empty bytes."""
        result = parse_orientation_msg(b"")
        assert result is None

    def test_missing_roll(self):
        """parse_orientation_msg() rejects missing roll key."""
        data = b'{"pitch": 0.0, "yaw": 0.0}'
        result = parse_orientation_msg(data)
        assert result is None

    def test_missing_pitch(self):
        """parse_orientation_msg() rejects missing pitch key."""
        data = b'{"roll": 0.0, "yaw": 0.0}'
        result = parse_orientation_msg(data)
        assert result is None

    def test_missing_yaw(self):
        """parse_orientation_msg() rejects missing yaw key."""
        data = b'{"roll": 0.0, "pitch": 0.0}'
        result = parse_orientation_msg(data)
        assert result is None

    def test_nan_literal(self):
        """parse_orientation_msg() rejects NaN literal."""
        data = b'{"roll": NaN, "pitch": 0.0, "yaw": 0.0}'
        result = parse_orientation_msg(data)
        assert result is None

    def test_infinity_literal(self):
        """parse_orientation_msg() rejects Infinity literal."""
        data = b'{"roll": Infinity, "pitch": 0.0, "yaw": 0.0}'
        result = parse_orientation_msg(data)
        assert result is None

    def test_neg_infinity_literal(self):
        """parse_orientation_msg() rejects -Infinity literal."""
        data = b'{"roll": -Infinity, "pitch": 0.0, "yaw": 0.0}'
        result = parse_orientation_msg(data)
        assert result is None

    def test_1e999_overflow(self):
        """parse_orientation_msg() rejects 1e999 (overflows to Infinity)."""
        data = b'{"roll": 1e999, "pitch": 0.0, "yaw": 0.0}'
        result = parse_orientation_msg(data)
        assert result is None

    def test_bool_true_rejected(self):
        """parse_orientation_msg() rejects bool true."""
        data = b'{"roll": true, "pitch": 0.0, "yaw": 0.0}'
        result = parse_orientation_msg(data)
        assert result is None

    def test_bool_false_rejected(self):
        """parse_orientation_msg() rejects bool false."""
        data = b'{"roll": false, "pitch": 0.0, "yaw": 0.0}'
        result = parse_orientation_msg(data)
        assert result is None

    def test_string_value_rejected(self):
        """parse_orientation_msg() rejects string "1.0"."""
        data = b'{"roll": "1.0", "pitch": 0.0, "yaw": 0.0}'
        result = parse_orientation_msg(data)
        assert result is None

    def test_null_value_rejected(self):
        """parse_orientation_msg() rejects null."""
        data = b'{"roll": null, "pitch": 0.0, "yaw": 0.0}'
        result = parse_orientation_msg(data)
        assert result is None

    def test_list_top_level_rejected(self):
        """parse_orientation_msg() rejects list at top level."""
        data = b'[0.1, -0.05, 0.0]'
        result = parse_orientation_msg(data)
        assert result is None

    def test_very_large_integer_overflow(self):
        """parse_orientation_msg() rejects 10**400 as digits."""
        # 10**400 overflows to Infinity
        large_num = b"1" + b"0" * 400
        data = b'{"roll": ' + large_num + b', "pitch": 0, "yaw": 0}'
        result = parse_orientation_msg(data)
        assert result is None


class TestEulerToAxisAngleBasic:
    """Test: euler_to_axis_angle basic cases."""

    def test_zero_rotation(self):
        """euler_to_axis_angle((0,0,0)) -> (0,0,1,0)."""
        x, y, z, angle = euler_to_axis_angle(0.0, 0.0, 0.0)
        assert abs(x - 0.0) < 1e-9
        assert abs(y - 0.0) < 1e-9
        assert abs(z - 1.0) < 1e-9
        assert abs(angle - 0.0) < 1e-9

    def test_roll_90deg(self):
        """euler_to_axis_angle((π/2, 0, 0)) -> (1, 0, 0, π/2)."""
        x, y, z, angle = euler_to_axis_angle(math.pi / 2, 0.0, 0.0)
        assert abs(x - 1.0) < 1e-6
        assert abs(y - 0.0) < 1e-6
        assert abs(z - 0.0) < 1e-6
        assert abs(angle - math.pi / 2) < 1e-6

    def test_yaw_90deg(self):
        """euler_to_axis_angle((0, 0, π/2)) -> (0, 0, 1, π/2)."""
        x, y, z, angle = euler_to_axis_angle(0.0, 0.0, math.pi / 2)
        assert abs(x - 0.0) < 1e-6
        assert abs(y - 0.0) < 1e-6
        assert abs(z - 1.0) < 1e-6
        assert abs(angle - math.pi / 2) < 1e-6

    def test_pitch_90deg(self):
        """euler_to_axis_angle((0, π/2, 0)) -> (0, 1, 0, π/2)."""
        x, y, z, angle = euler_to_axis_angle(0.0, math.pi / 2, 0.0)
        assert abs(x - 0.0) < 1e-6
        assert abs(y - 1.0) < 1e-6
        assert abs(z - 0.0) < 1e-6
        assert abs(angle - math.pi / 2) < 1e-6


class TestEulerToAxisAngleComposite:
    """Test: euler_to_axis_angle composite angles."""

    def test_composite_0_3_minus_0_2_1_1(self):
        """euler_to_axis_angle((0.3, -0.2, 1.1)) has expected axis and angle."""
        x, y, z, angle = euler_to_axis_angle(0.3, -0.2, 1.1)

        # Expected (from reference implementation): axis ≈ (0.32059, -0.01157, 0.94711), angle ≈ 1.1800
        assert abs(x - 0.32059) < 1e-3
        assert abs(y - (-0.01157)) < 1e-3
        assert abs(z - 0.94711) < 1e-3
        assert abs(angle - 1.1800) < 1e-3


class TestEulerToAxisAngleAxisLength:
    """Test: axis is always unit-length."""

    def test_axis_unit_length_zero_rotation(self):
        """Axis is unit-length (or zero when angle=0)."""
        x, y, z, angle = euler_to_axis_angle(0.0, 0.0, 0.0)
        length = math.sqrt(x * x + y * y + z * z)
        # For zero rotation, axis is (0,0,1), length 1
        assert abs(length - 1.0) < 1e-9

    def test_axis_unit_length_various(self):
        """Axis is unit-length for various angles."""
        test_cases = [
            (math.pi / 6, 0.0, 0.0),
            (0.0, math.pi / 4, 0.0),
            (0.0, 0.0, math.pi / 3),
            (0.3, -0.2, 1.1),
            (-0.5, 0.7, -0.3),
        ]
        for roll, pitch, yaw in test_cases:
            x, y, z, angle = euler_to_axis_angle(roll, pitch, yaw)
            length = math.sqrt(x * x + y * y + z * z)
            assert abs(length - 1.0) < 1e-9


class TestEulerToAxisAngleAngleBounds:
    """Test: angle is in [0, π]."""

    def test_angle_in_bounds_various(self):
        """Angle is always in [0, π] for various Euler angles."""
        test_cases = [
            (0.0, 0.0, 0.0),
            (math.pi / 2, 0.0, 0.0),
            (0.0, math.pi / 2, 0.0),
            (0.0, 0.0, math.pi / 2),
            (0.3, -0.2, 1.1),
            (-0.5, 0.7, -0.3),
            (-math.pi, 0.0, 0.0),
            (math.pi, 0.0, 0.0),
            (0.0, -math.pi / 2, 0.0),
            (0.0, math.pi / 2, 0.0),
        ]
        for roll, pitch, yaw in test_cases:
            x, y, z, angle = euler_to_axis_angle(roll, pitch, yaw)
            assert angle >= 0.0
            assert angle <= math.pi + 1e-12


class TestEulerToAxisAngleRollPi:
    """Test: roll=π special case."""

    def test_roll_pi(self):
        """roll=π -> angle≈π with x≈±1, y≈0, z≈0."""
        x, y, z, angle = euler_to_axis_angle(math.pi, 0.0, 0.0)
        assert abs(angle - math.pi) < 1e-6
        assert abs(abs(x) - 1.0) < 1e-9
        assert abs(y) < 1e-9
        assert abs(z) < 1e-9

    def test_roll_neg_pi(self):
        """roll=-π -> angle≈π with x≈±1, y≈0, z≈0."""
        x, y, z, angle = euler_to_axis_angle(-math.pi, 0.0, 0.0)
        assert abs(angle - math.pi) < 1e-6
        assert abs(abs(x) - 1.0) < 1e-9
        assert abs(y) < 1e-9
        assert abs(z) < 1e-9


class TestEulerToAxisAngleMatrixRoundTrip:
    """Test: matrix round-trip (Rodrigues formula)."""

    def _euler_to_matrix(self, roll, pitch, yaw):
        """Build rotation matrix from ZYX Euler angles."""
        # R = Rz(yaw) * Ry(pitch) * Rx(roll)
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        # Rx(roll)
        Rx = [
            [1.0, 0.0, 0.0],
            [0.0, cr, -sr],
            [0.0, sr, cr],
        ]

        # Ry(pitch)
        Ry = [
            [cp, 0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp],
        ]

        # Rz(yaw)
        Rz = [
            [cy, -sy, 0.0],
            [sy, cy, 0.0],
            [0.0, 0.0, 1.0],
        ]

        # Multiply: Rz * Ry * Rx (right-to-left)
        # First: Ry * Rx
        temp = [
            [
                Ry[i][0] * Rx[0][j]
                + Ry[i][1] * Rx[1][j]
                + Ry[i][2] * Rx[2][j]
                for j in range(3)
            ]
            for i in range(3)
        ]

        # Then: Rz * temp
        R = [
            [
                Rz[i][0] * temp[0][j]
                + Rz[i][1] * temp[1][j]
                + Rz[i][2] * temp[2][j]
                for j in range(3)
            ]
            for i in range(3)
        ]

        return R

    def _rodrigues_matrix(self, x, y, z, angle):
        """Build rotation matrix from axis-angle (Rodrigues formula)."""
        c = math.cos(angle)
        s = math.sin(angle)
        t = 1.0 - c

        # Skew-symmetric matrix of (x, y, z)
        K = [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ]

        # K^2
        K2 = [
            [
                K[i][0] * K[0][j] + K[i][1] * K[1][j] + K[i][2] * K[2][j]
                for j in range(3)
            ]
            for i in range(3)
        ]

        # R = I + s*K + t*K^2
        R = [
            [
                (1.0 if i == j else 0.0) + s * K[i][j] + t * K2[i][j]
                for j in range(3)
            ]
            for i in range(3)
        ]

        return R

    def test_round_trip_grid(self):
        """Matrix built from euler_to_axis_angle output matches Rodrigues formula."""
        test_cases = [
            (0.0, 0.0, 0.0),
            (math.pi / 6, 0.0, 0.0),
            (0.0, math.pi / 4, 0.0),
            (0.0, 0.0, math.pi / 3),
            (0.3, -0.2, 1.1),
            (-0.5, 0.7, -0.3),
            (math.pi / 2, math.pi / 2, math.pi / 2),
            (-math.pi, 0.0, 0.0),
            (math.pi, 0.0, 0.0),
            (0.0, -math.pi / 2, 0.0),
            (0.0, math.pi / 2, 0.0),
        ]

        for roll, pitch, yaw in test_cases:
            # Expected matrix from Euler angles
            R_euler = self._euler_to_matrix(roll, pitch, yaw)

            # Axis-angle from this function
            x, y, z, angle = euler_to_axis_angle(roll, pitch, yaw)

            # Matrix from Rodrigues formula
            R_axis_angle = self._rodrigues_matrix(x, y, z, angle)

            # Compare all entries (tolerance 1e-9 for numerical precision)
            for i in range(3):
                for j in range(3):
                    assert (
                        abs(R_euler[i][j] - R_axis_angle[i][j]) < 1e-9
                    ), f"Mismatch at ({roll}, {pitch}, {yaw})[{i},{j}]: {R_euler[i][j]} vs {R_axis_angle[i][j]}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
