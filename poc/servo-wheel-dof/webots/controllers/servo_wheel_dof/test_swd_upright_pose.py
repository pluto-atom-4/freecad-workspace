#!/usr/bin/env python3
"""
Unit tests for upright_pose.py (issue #291, sub-issue of #258).

Pure Python plus numpy as an independent oracle: the rotation matrix is built as
Rz(yaw) @ Ry(pitch) @ Rx(pi/2) without touching the module under test. No Webots,
no hardware, must NOT `import controller` or `serial`.

Mutation check (done by hand): flipping the sign of either yaw term, using
c0 + R*a, or dropping `gap` makes the sweep tests fail.

Usage:
    cd poc/servo-wheel-dof
    mamba run -n servo-wheel-dof python3 -m pytest -q \
        webots/controllers/servo_wheel_dof/test_swd_upright_pose.py
"""

from __future__ import annotations

import itertools
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from imu_euler_math import euler_to_axis_angle  # noqa: E402
from upright_pose import upright_pose  # noqa: E402

PI = math.pi
TOL = 1e-9
RADIUS = 0.03
WHEEL_OFFSET = 0.026
EZ = np.array([0.0, 0.0, 1.0])
A = np.array([0.0, 0.0, WHEEL_OFFSET])  # wheel centre, Robot-local

PITCHES = [-1.2, 0.0, 0.7]
YAWS = [-2.5, 0.0, PI / 2, PI, 4.0, 7.0]  # includes values beyond pi
SWEEP = list(itertools.product(PITCHES, YAWS))


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


def _oracle_r(pitch: float, yaw: float) -> np.ndarray:
    """Independent R = Rz(yaw) @ Ry(pitch) @ Rx(pi/2)."""
    return _rz(yaw) @ _ry(pitch) @ _rx(PI / 2)


class TestZeroInput:
    def test_translation(self):
        """Zero pitch and yaw leave the Robot at (0, 0, radius)."""
        t, _ = upright_pose(0.0, 0.0)
        assert t == pytest.approx((0.0, 0.0, 0.03), abs=TOL)

    def test_rotation(self):
        """Zero pitch and yaw give the base rotation Rx(+90 deg)."""
        _, aa = upright_pose(0.0, 0.0)
        assert aa == pytest.approx((1.0, 0.0, 0.0, PI / 2), abs=TOL)


class TestKnownValues:
    def test_yaw_90(self):
        """Yaw pi/2 shifts the origin to (-wo, -wo)."""
        t, _ = upright_pose(0.0, PI / 2)
        assert t == pytest.approx((-WHEEL_OFFSET, -WHEEL_OFFSET, 0.03), abs=TOL)

    def test_yaw_180(self):
        """Yaw pi moves the origin to y = -2 * wo."""
        t, _ = upright_pose(0.0, PI)
        assert t == pytest.approx((0.0, -2 * WHEEL_OFFSET, 0.03), abs=TOL)

    def test_pitch_only_keeps_translation(self):
        """Pitch alone does not move the origin (centre is on the pitch axis)."""
        t, _ = upright_pose(0.7, 0.0)
        assert t == pytest.approx((0.0, 0.0, 0.03), abs=TOL)


class TestSweep:
    @pytest.mark.parametrize("pitch,yaw", SWEEP)
    def test_wheel_centre_fixed(self, pitch, yaw):
        """T + R @ a equals c0 = (0, -wo, radius) for every pitch and yaw."""
        t, _ = upright_pose(pitch, yaw)
        centre = np.array(t) + _oracle_r(pitch, yaw) @ A
        c0 = np.array([0.0, -WHEEL_OFFSET, RADIUS])
        assert np.allclose(centre, c0, atol=TOL)

    @pytest.mark.parametrize("pitch,yaw", SWEEP)
    def test_hinge_axis_horizontal(self, pitch, yaw):
        """The hinge axis R @ ez has no z component, so the disk stays upright."""
        assert (_oracle_r(pitch, yaw) @ EZ)[2] == pytest.approx(0.0, abs=TOL)

    @pytest.mark.parametrize("pitch,yaw", SWEEP)
    @pytest.mark.parametrize("gap", [0.0, 0.0005])
    def test_lowest_rim_point_at_gap(self, pitch, yaw, gap):
        """Lowest rim z = centre_z - radius * hypot(u_z, v_z) equals gap."""
        t, _ = upright_pose(pitch, yaw, gap=gap)
        r = _oracle_r(pitch, yaw)
        centre_z = (np.array(t) + r @ A)[2]
        lowest = centre_z - RADIUS * math.hypot(r[2, 0], r[2, 1])
        assert lowest == pytest.approx(gap, abs=TOL)

    @pytest.mark.parametrize("pitch,yaw", SWEEP)
    def test_rotation_matches_euler(self, pitch, yaw):
        """The rotation is exactly euler_to_axis_angle(pi/2, pitch, yaw)."""
        _, aa = upright_pose(pitch, yaw)
        assert aa == euler_to_axis_angle(PI / 2, pitch, yaw)

    @pytest.mark.parametrize("pitch,yaw", SWEEP)
    def test_rotation_is_the_matrix_used(self, pitch, yaw):
        """The returned axis-angle encodes the same R the centre test uses."""
        _, aa = upright_pose(pitch, yaw)
        assert np.allclose(_rodrigues(*aa), _oracle_r(pitch, yaw), atol=TOL)


class TestSignature:
    def test_returns_two_plain_tuples(self):
        """Return value is (3-tuple, 4-tuple) of floats, not arrays."""
        t, aa = upright_pose(0.1, 0.2)
        assert isinstance(t, tuple) and len(t) == 3
        assert isinstance(aa, tuple) and len(aa) == 4

    def test_custom_radius_offset_gap(self):
        """Keyword overrides change c0; yaw pi/2 gives T = (-wo, -wo, r + g)."""
        t, _ = upright_pose(0.0, PI / 2, radius=0.05, wheel_offset=0.04, gap=0.001)
        assert t == pytest.approx((-0.04, -0.04, 0.051), abs=TOL)

    def test_options_are_keyword_only(self):
        """radius cannot be passed positionally."""
        with pytest.raises(TypeError):
            upright_pose(0.0, 0.0, 0.03)  # type: ignore[misc]

    def test_roll_is_not_a_parameter(self):
        """Roll is ignored by design, so it is rejected as a keyword."""
        with pytest.raises(TypeError):
            upright_pose(0.0, 0.0, roll=0.1)  # type: ignore[call-arg]
