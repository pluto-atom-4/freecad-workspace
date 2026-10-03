#!/usr/bin/env python3
"""
Unit tests for alignment_metrics.py (issue #304, sub-issue of #303).

Pure Python, no numpy: orientations are built by hand as row-major lists and
the anchor is recomputed independently from the documented formula. No Webots,
no hardware, must NOT `import controller` or `serial`.

Reference numbers: cos(1 deg) = 0.999848 < 0.9999 (warns); cos(0.5 deg) =
0.999962 > 0.9999 (does not warn). Rx(+90 deg) row-major is
[1,0,0, 0,0,-1, 0,1,0] and its column 2 is (0, -1, 0).

Usage:
    cd poc/servo-wheel-dof
    mamba run -n servo-wheel-dof python3 -m pytest -q \
        webots/controllers/servo_wheel_dof/test_swd_alignment_metrics.py
"""

from __future__ import annotations

import dataclasses
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from alignment_metrics import (  # noqa: E402
    AlignmentMetrics,
    anchor_metrics,
    relative_position,
)
from upright_pose import WHEEL_OFFSET  # noqa: E402

PI = math.pi
OFFSET = 0.026
ROBOT_POS = (0.0, 0.0, 0.03)
IDENTITY = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


def _rx_ori(theta: float) -> list[float]:
    """Row-major Rx(theta); column 2 is (0, -sin(theta), cos(theta))."""
    c, s = math.cos(theta), math.sin(theta)
    return [1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c]


def _col2(ori: list[float]) -> tuple[float, float, float]:
    return (ori[2], ori[5], ori[8])


def _anchor(pos, ori, offset: float = OFFSET) -> tuple[float, float, float]:
    """Independent anchor: pos + column2 * offset."""
    z = _col2(ori)
    return (pos[0] + z[0] * offset, pos[1] + z[1] * offset, pos[2] + z[2] * offset)


ORI_X90 = _rx_ori(PI / 2)


def _healthy_args() -> dict:
    return {
        "robot_pos": list(ROBOT_POS),
        "robot_ori": list(ORI_X90),
        "wheel_pos": list(_anchor(ROBOT_POS, ORI_X90)),
        "wheel_ori": list(ORI_X90),
    }


def _shift(pos, axis: int, delta: float) -> list[float]:
    out = list(pos)
    out[axis] += delta
    return out


class TestAligned:
    def test_healthy_case(self):
        m = anchor_metrics(**_healthy_args())
        assert m.deviation_m == pytest.approx(0.0, abs=1e-12)
        assert m.axis_dot == pytest.approx(1.0, abs=1e-12)
        assert m.wheel_z == pytest.approx(0.03)
        assert m.warn is False

    def test_robot_yawed_still_aligned(self):
        yaw = 0.7
        c, s = math.cos(yaw), math.sin(yaw)
        ori = [c, 0.0, s, s, 0.0, -c, 0.0, 1.0, 0.0]  # Rz(yaw) @ Rx(90)
        assert _col2(ori) == (s, -c, 0.0)
        pos = (0.1, 0.2, 0.03)
        wheel = (pos[0] + s * OFFSET, pos[1] - c * OFFSET, pos[2])
        m = anchor_metrics(pos, ori, wheel, ori)
        assert m.deviation_m == pytest.approx(0.0, abs=1e-12)
        assert m.axis_dot == pytest.approx(1.0, abs=1e-12)
        assert m.wheel_z == pytest.approx(0.03)
        assert m.warn is False

    def test_default_offset_is_wheel_offset(self):
        assert WHEEL_OFFSET == OFFSET
        args = _healthy_args()
        explicit = anchor_metrics(**args, anchor_offset=WHEEL_OFFSET)
        assert anchor_metrics(**args) == explicit

    def test_result_types(self):
        m = anchor_metrics(**_healthy_args())
        assert isinstance(m, AlignmentMetrics)
        assert isinstance(m.deviation_m, float)
        assert isinstance(m.axis_dot, float)
        assert isinstance(m.wheel_z, float)
        assert isinstance(m.warn, bool)


class TestOffset:
    def test_two_mm_offset_warns(self):
        args = _healthy_args()
        args["wheel_pos"] = _shift(args["wheel_pos"], 2, 0.002)
        m = anchor_metrics(**args)
        assert m.deviation_m == pytest.approx(0.002, abs=1e-9)
        assert m.axis_dot == pytest.approx(1.0, abs=1e-12)
        assert m.warn is True

    def test_half_mm_offset_does_not_warn(self):
        args = _healthy_args()
        args["wheel_pos"] = _shift(args["wheel_pos"], 2, 0.0005)
        m = anchor_metrics(**args)
        assert m.deviation_m == pytest.approx(0.0005, abs=1e-9)
        assert m.warn is False

    def test_wheel_on_wrong_side_warns(self):
        args = _healthy_args()
        args["wheel_pos"] = [0.0, OFFSET, 0.03]  # anchor is at y = -0.026
        m = anchor_metrics(**args)
        assert m.deviation_m == pytest.approx(0.052, abs=1e-9)
        assert m.warn is True

    @pytest.mark.parametrize("axis", [0, 1, 2])
    def test_offset_along_each_world_axis(self, axis):
        args = _healthy_args()
        args["wheel_pos"] = _shift(args["wheel_pos"], axis, -0.002)
        m = anchor_metrics(**args)
        assert m.deviation_m == pytest.approx(0.002, abs=1e-9)
        assert m.warn is True


class TestTilt:
    def test_one_degree_tilt_warns(self):
        args = _healthy_args()
        args["wheel_ori"] = _rx_ori(PI / 2 + math.radians(1.0))
        m = anchor_metrics(**args)
        assert m.axis_dot == pytest.approx(math.cos(math.radians(1.0)), abs=1e-9)
        assert m.axis_dot < 0.9999
        assert m.deviation_m == pytest.approx(0.0, abs=1e-12)
        assert m.warn is True

    def test_half_degree_tilt_does_not_warn(self):
        args = _healthy_args()
        args["wheel_ori"] = _rx_ori(PI / 2 + math.radians(0.5))
        m = anchor_metrics(**args)
        assert m.axis_dot == pytest.approx(math.cos(math.radians(0.5)), abs=1e-9)
        assert m.axis_dot > 0.9999
        assert m.warn is False

    def test_flipped_axis_warns(self):
        args = _healthy_args()
        args["wheel_ori"] = _rx_ori(-PI / 2)  # column 2 is (0, +1, 0)
        m = anchor_metrics(**args)
        assert m.axis_dot == pytest.approx(-1.0, abs=1e-12)
        assert m.warn is True


class TestNonFinite:
    @pytest.mark.parametrize(
        "key", ["robot_pos", "robot_ori", "wheel_pos", "wheel_ori"]
    )
    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    @pytest.mark.parametrize("idx", [0, 2])
    def test_non_finite_input_warns_with_nan_fields(self, key, bad, idx):
        args = _healthy_args()
        args[key][idx] = bad
        m = anchor_metrics(**args)
        assert math.isnan(m.deviation_m)
        assert math.isnan(m.axis_dot)
        assert math.isnan(m.wheel_z)
        assert m.warn is True

    def test_non_finite_anchor_offset_warns(self):
        m = anchor_metrics(**_healthy_args(), anchor_offset=float("nan"))
        assert math.isnan(m.deviation_m)
        assert m.warn is True


class TestClamp:
    def test_dot_above_one_is_clamped(self):
        wheel_ori = list(IDENTITY)
        wheel_ori[8] = 1.0000001  # unnormalised: raw dot is 1.0000001
        m = anchor_metrics(
            (0.0, 0.0, 0.0), IDENTITY, (0.0, 0.0, OFFSET), wheel_ori
        )
        assert m.axis_dot == 1.0
        assert m.warn is False

    def test_dot_below_minus_one_is_clamped(self):
        wheel_ori = list(IDENTITY)
        wheel_ori[8] = -1.0000001
        m = anchor_metrics(
            (0.0, 0.0, 0.0), IDENTITY, (0.0, 0.0, OFFSET), wheel_ori
        )
        assert m.axis_dot == -1.0
        assert m.warn is True


class TestValidation:
    @pytest.mark.parametrize(
        "key, bad",
        [
            ("robot_pos", [0.0, 0.0]),
            ("wheel_pos", [0.0, 0.0, 0.0, 0.0]),
            ("robot_ori", IDENTITY[:8]),
            ("wheel_ori", IDENTITY + [0.0]),
        ],
    )
    def test_wrong_length_raises(self, key, bad):
        args = _healthy_args()
        args[key] = bad
        with pytest.raises(ValueError, match=key):
            anchor_metrics(**args)

    def test_options_are_keyword_only(self):
        a = _healthy_args()
        with pytest.raises(TypeError):
            anchor_metrics(
                a["robot_pos"], a["robot_ori"], a["wheel_pos"], a["wheel_ori"], 0.026
            )


class TestThresholds:
    def test_loose_dev_threshold_accepts_two_mm(self):
        args = _healthy_args()
        args["wheel_pos"] = _shift(args["wheel_pos"], 2, 0.002)
        assert anchor_metrics(**args, dev_warn_m=0.005).warn is False

    def test_tight_dev_threshold_rejects_half_mm(self):
        args = _healthy_args()
        args["wheel_pos"] = _shift(args["wheel_pos"], 2, 0.0005)
        assert anchor_metrics(**args, dev_warn_m=0.0001).warn is True

    def test_loose_dot_threshold_accepts_one_degree(self):
        args = _healthy_args()
        args["wheel_ori"] = _rx_ori(PI / 2 + math.radians(1.0))
        assert anchor_metrics(**args, dot_warn=0.99).warn is False

    def test_tight_dot_threshold_rejects_half_degree(self):
        args = _healthy_args()
        args["wheel_ori"] = _rx_ori(PI / 2 + math.radians(0.5))
        assert anchor_metrics(**args, dot_warn=0.99999).warn is True

    def test_custom_anchor_offset(self):
        pos, wheel = (0.0, 0.0, 0.0), (0.0, 0.0, 0.05)
        custom = anchor_metrics(pos, IDENTITY, wheel, IDENTITY, anchor_offset=0.05)
        default = anchor_metrics(pos, IDENTITY, wheel, IDENTITY)
        assert custom.deviation_m == pytest.approx(0.0, abs=1e-12)
        assert custom.warn is False
        assert default.deviation_m == pytest.approx(0.024, abs=1e-9)
        assert default.warn is True

    def test_deviation_exactly_at_threshold_does_not_warn(self):
        args = _healthy_args()
        args["wheel_pos"] = _shift(args["wheel_pos"], 2, 0.002)
        measured = anchor_metrics(**args).deviation_m
        assert anchor_metrics(**args, dev_warn_m=measured).warn is False

    def test_dot_exactly_at_threshold_does_not_warn(self):
        args = _healthy_args()
        args["wheel_ori"] = _rx_ori(PI / 2 + math.radians(1.0))
        measured = anchor_metrics(**args).axis_dot
        assert anchor_metrics(**args, dot_warn=measured).warn is False

    def test_negative_anchor_offset(self):
        pos, ori = ROBOT_POS, ORI_X90
        wheel = _anchor(pos, ori, -OFFSET)
        m = anchor_metrics(pos, ori, wheel, ori, anchor_offset=-OFFSET)
        assert m.deviation_m == pytest.approx(0.0, abs=1e-12)
        assert m.warn is False


class TestFrozen:
    def test_assignment_raises(self):
        m = anchor_metrics(**_healthy_args())
        with pytest.raises(dataclasses.FrozenInstanceError):
            m.warn = True  # type: ignore[misc]


# --- relative_position (issue #312) -----------------------------------------
# Hand-checked numbers. With Rx(+90) (row-major [1,0,0, 0,0,-1, 0,1,0]) the
# healthy wheel sits at world offset (0, -0.026, 0), so rel = (0, 0, 0.026).
# ORI_YAW90 = Rz(90) @ Rx(90) = [0,0,1, 1,0,0, 0,1,0] has columns
# col0 = (0,1,0), col1 = (0,0,1), col2 = (1,0,0). For world offset
# (0.001, 0.002, 0.003): rel = (0.002, 0.003, 0.001), a pure permutation that
# differs from the transpose result (0.003, 0.001, 0.002).
ORI_YAW90 = [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]


def _rel_args() -> dict:
    h = _healthy_args()
    return {
        "robot_pos": h["robot_pos"],
        "robot_ori": h["robot_ori"],
        "wheel_pos": h["wheel_pos"],
    }


def _col(ori: list[float], k: int) -> tuple[float, float, float]:
    """Column k of a row-major 3x3."""
    return (ori[k], ori[k + 3], ori[k + 6])


class TestRelativePosition:
    def test_healthy_is_zero_zero_offset(self):
        rel = relative_position(**_rel_args())
        assert rel[0] == pytest.approx(0.0, abs=1e-12)
        assert rel[1] == pytest.approx(0.0, abs=1e-12)
        assert rel[2] == pytest.approx(OFFSET, abs=1e-12)

    def test_result_is_tuple_of_three_floats(self):
        rel = relative_position(**_rel_args())
        assert isinstance(rel, tuple)
        assert len(rel) == 3
        assert all(isinstance(v, float) for v in rel)

    def test_world_x_push_gives_rel_x_only(self):
        args = _rel_args()
        args["wheel_pos"] = _shift(args["wheel_pos"], 0, 0.003)
        rel = relative_position(**args)
        assert rel[0] == pytest.approx(0.003, abs=1e-12)
        assert rel[1] == pytest.approx(0.0, abs=1e-12)
        assert rel[2] == pytest.approx(OFFSET, abs=1e-12)

    def test_world_y_push_with_rx90_changes_rel_z(self):
        # Robot Rx(+90): Robot z is world -Y, wheel y -0.026 -> -0.023,
        # so the wheel is 0.023 along Robot z.
        args = _rel_args()
        args["wheel_pos"] = _shift(args["wheel_pos"], 1, 0.003)
        rel = relative_position(**args)
        assert rel[0] == pytest.approx(0.0, abs=1e-12)
        assert rel[1] == pytest.approx(0.0, abs=1e-12)
        assert rel[2] == pytest.approx(0.023, abs=1e-12)

    def test_world_z_push_with_rx90_gives_rel_y(self):
        # Robot Rx(+90): Robot y is world +Z (column 1 = (0, 0, 1)).
        args = _rel_args()
        args["wheel_pos"] = _shift(args["wheel_pos"], 2, 0.003)
        rel = relative_position(**args)
        assert rel[0] == pytest.approx(0.0, abs=1e-12)
        assert rel[1] == pytest.approx(0.003, abs=1e-12)
        assert rel[2] == pytest.approx(OFFSET, abs=1e-12)

    def test_robot_yawed_is_invariant(self):
        yaw = 0.7
        c, s = math.cos(yaw), math.sin(yaw)
        ori = [c, 0.0, s, s, 0.0, -c, 0.0, 1.0, 0.0]  # Rz(yaw) @ Rx(90)
        pos = (0.1, 0.2, 0.03)
        wheel = (pos[0] + s * OFFSET, pos[1] - c * OFFSET, pos[2])
        rel = relative_position(pos, ori, wheel)
        assert rel[0] == pytest.approx(0.0, abs=1e-12)
        assert rel[1] == pytest.approx(0.0, abs=1e-12)
        assert rel[2] == pytest.approx(OFFSET, abs=1e-12)

    def test_distinct_axes_use_columns_not_rows(self):
        rel = relative_position(
            (0.0, 0.0, 0.0), ORI_YAW90, (0.001, 0.002, 0.003)
        )
        assert rel[0] == pytest.approx(0.002, abs=1e-12)
        assert rel[1] == pytest.approx(0.003, abs=1e-12)
        assert rel[2] == pytest.approx(0.001, abs=1e-12)

    def test_matches_independent_column_dot(self):
        offset = (0.004, -0.002, 0.001)
        pos = (0.1, 0.2, 0.3)
        wheel = (pos[0] + offset[0], pos[1] + offset[1], pos[2] + offset[2])
        rel = relative_position(pos, ORI_YAW90, wheel)
        for k in range(3):
            col = _col(ORI_YAW90, k)
            expected = sum(col[i] * offset[i] for i in range(3))
            assert rel[k] == pytest.approx(expected, abs=1e-12)

    @pytest.mark.parametrize(
        "push",
        [(0.003, 0.0, 0.0), (0.0, 0.002, -0.001), (0.001, -0.002, 0.004)],
    )
    def test_consistent_with_anchor_deviation(self, push):
        args = _healthy_args()
        args["wheel_pos"] = [a + b for a, b in zip(args["wheel_pos"], push)]
        rel = relative_position(
            args["robot_pos"], args["robot_ori"], args["wheel_pos"]
        )
        dev = anchor_metrics(**args).deviation_m
        assert math.dist(rel, (0.0, 0.0, OFFSET)) == pytest.approx(
            dev, abs=1e-12
        )

    def test_does_not_mutate_inputs(self):
        args = _rel_args()
        before = {k: list(v) for k, v in args.items()}
        relative_position(**args)
        assert args == before


class TestRelativePositionNonFinite:
    @pytest.mark.parametrize("key", ["robot_pos", "robot_ori", "wheel_pos"])
    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    @pytest.mark.parametrize("idx", [0, 2])
    def test_non_finite_input_gives_three_nan(self, key, bad, idx):
        args = _rel_args()
        args[key][idx] = bad
        rel = relative_position(**args)
        assert len(rel) == 3
        assert all(math.isnan(v) for v in rel)

    def test_overflow_gives_three_nan(self):
        args = _rel_args()
        args["robot_ori"] = [1e308] * 9
        args["wheel_pos"] = [1e308, 1e308, 1e308]
        rel = relative_position(**args)
        assert all(math.isnan(v) for v in rel)


class TestRelativePositionValidation:
    @pytest.mark.parametrize(
        "key, bad",
        [
            ("robot_pos", [0.0, 0.0]),
            ("wheel_pos", [0.0, 0.0, 0.0, 0.0]),
            ("robot_ori", IDENTITY[:8]),
        ],
    )
    def test_wrong_length_raises(self, key, bad):
        args = _rel_args()
        args[key] = bad
        with pytest.raises(ValueError, match=key):
            relative_position(**args)

    def test_robot_ori_too_long_raises(self):
        args = _rel_args()
        args["robot_ori"] = IDENTITY + [0.0]
        with pytest.raises(ValueError, match="robot_ori"):
            relative_position(**args)
