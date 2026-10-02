#!/usr/bin/env python3
"""
Unit tests for imu_euler_math.py (issue #285, sub-issue of #258).

Pure Python plus numpy as an independent oracle. No Webots, no hardware, must
NOT import `controller` or `serial`.

Usage:
    cd poc/servo-wheel-dof
    mamba run -n servo-wheel-dof python3 -m pytest -q \
        webots/controllers/servo_wheel_dof/test_swd_imu_euler_math.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from imu_euler_math import euler_to_axis_angle  # noqa: E402

PI = math.pi
TOL = 1e-9


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rodrigues(x: float, y: float, z: float, angle: float) -> np.ndarray:
    """Rotation matrix from a unit axis (x, y, z) and an angle."""
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


class TestSingleAxis:
    def test_zero_returns_default(self):
        """No rotation returns the documented default axis and zero angle."""
        assert euler_to_axis_angle(0.0, 0.0, 0.0) == (0.0, 0.0, 1.0, 0.0)

    def test_yaw_90(self):
        """Yaw pi/2 is a rotation of pi/2 about Z."""
        got = euler_to_axis_angle(0.0, 0.0, PI / 2)
        assert got == pytest.approx((0.0, 0.0, 1.0, PI / 2), abs=TOL)

    def test_roll_90(self):
        """Roll pi/2 is a rotation of pi/2 about X (issue #291 relies on this)."""
        got = euler_to_axis_angle(PI / 2, 0.0, 0.0)
        assert got == pytest.approx((1.0, 0.0, 0.0, PI / 2), abs=TOL)

    def test_pitch_90(self):
        """Pitch pi/2 is a rotation of pi/2 about Y."""
        got = euler_to_axis_angle(0.0, PI / 2, 0.0)
        assert got == pytest.approx((0.0, 1.0, 0.0, PI / 2), abs=TOL)

    def test_negative_yaw_flips_axis(self):
        """Negative yaw keeps a positive angle and flips the axis."""
        got = euler_to_axis_angle(0.0, 0.0, -PI / 2)
        assert got == pytest.approx((0.0, 0.0, -1.0, PI / 2), abs=TOL)

    def test_roll_pi(self):
        """Roll pi is a half turn; the axis sign is ambiguous, so use |x|."""
        x, y, z, angle = euler_to_axis_angle(PI, 0.0, 0.0)
        assert abs(x) == pytest.approx(1.0, abs=TOL)
        assert y == pytest.approx(0.0, abs=TOL)
        assert z == pytest.approx(0.0, abs=TOL)
        assert angle == pytest.approx(PI, abs=TOL)

    def test_w_negative_takes_short_arc(self):
        """Roll 4.0 rad (> pi) is canonicalised to the short way round."""
        got = euler_to_axis_angle(4.0, 0.0, 0.0)
        assert got == pytest.approx((-1.0, 0.0, 0.0, 2 * PI - 4.0), abs=TOL)


class TestNearZero:
    def test_below_threshold_returns_default(self):
        """A vanishingly small angle returns the default axis, not NaN."""
        assert euler_to_axis_angle(1e-13, 0.0, 0.0) == (0.0, 0.0, 1.0, 0.0)

    def test_small_angle_keeps_axis_and_angle(self):
        """A small but resolvable angle keeps its real axis and angle."""
        x, y, z, angle = euler_to_axis_angle(1e-6, 0.0, 0.0)
        assert (x, y, z) == pytest.approx((1.0, 0.0, 0.0), abs=TOL)
        assert angle == pytest.approx(1e-6, abs=1e-12)


CASES = [
    (0.3, -0.2, 0.5),
    (-0.5, 0.7, -0.3),
    (PI, 0.0, 0.0),
    (0.0, -PI / 2, 0.0),
    (4.0, 0.1, -0.2),
]


class TestInvariants:
    @pytest.mark.parametrize("roll,pitch,yaw", CASES)
    def test_axis_unit_and_angle_bounds(self, roll, pitch, yaw):
        """The axis has unit length and the angle lies in [0, pi]."""
        x, y, z, angle = euler_to_axis_angle(roll, pitch, yaw)
        assert x * x + y * y + z * z == pytest.approx(1.0, abs=TOL)
        assert 0.0 <= angle <= PI + TOL


class TestAgainstNumpy:
    @pytest.mark.parametrize("roll,pitch,yaw", CASES)
    def test_matches_independent_zyx_matrix(self, roll, pitch, yaw):
        """Output equals Rz(yaw) @ Ry(pitch) @ Rx(roll), built independently."""
        r_euler = _rz(yaw) @ _ry(pitch) @ _rx(roll)
        x, y, z, angle = euler_to_axis_angle(roll, pitch, yaw)
        r_rod = _rodrigues(x, y, z, angle)
        assert np.allclose(r_euler, r_rod, atol=TOL)

    @pytest.mark.parametrize("roll,pitch,yaw", CASES)
    def test_angle_matches_matrix_trace(self, roll, pitch, yaw):
        """The angle equals arccos((trace - 1) / 2) of the same matrix."""
        r_euler = _rz(yaw) @ _ry(pitch) @ _rx(roll)
        expected = math.acos(max(-1.0, min(1.0, (np.trace(r_euler) - 1) / 2)))
        _x, _y, _z, angle = euler_to_axis_angle(roll, pitch, yaw)
        assert angle == pytest.approx(expected, abs=1e-6)