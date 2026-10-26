#!/usr/bin/env python3
"""
PROTO vs URDF joint consistency (#392, Part of #103 S2).

Compares the real protos/InvertedPendulumRobot.proto against
06_Exports/urdf/robot.urdf (source of truth). Data and unit tests only:
no Webots, no GUI, no hardware. The PROTO is only read, never written
(an autouse module guard checks its bytes are unchanged).

A missing PROTO FAILS (human decision), it is not skipped.

Usage:
    cd inverted-pendulum-project
    mamba run -n pendulum-tools python3 -m pytest 07_Simulation/webots/test_proto_urdf_consistency.py -q
"""

import sys
from dataclasses import replace
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from proto_urdf_compare import (
    IDENTITY_ROTATION,
    PROTO_PATH,
    URDF_PATH,
    Physics,
    ProtoJoint,
    ProtoRobot,
    axis_angle_to_matrix,
    compare_joints,
    mat_vec,
    max_abs_diff,
    normalize,
    parse_proto,
    rpy_to_matrix,
    urdf_joints,
)

TOL = 1e-6
EXPECTED_JOINTS = {
    "wheel_left_joint",
    "wheel_right_joint",
    "pendulum_pivot_joint",
    "pendulum_pivot_right_joint",
}
PENDULUM_JOINTS = ("pendulum_pivot_joint", "pendulum_pivot_right_joint")

# --- synthetic data: no file reads, no PROTO writes -------------------------

PENDULUM_ANCHOR = (-0.008001, -0.018276, 0.070750)
WHEEL_ANCHOR = (0.014001, -0.028526, 0.054010)
ZERO_INERTIA = (0.0,) * 6


def _joint(name, axis, anchor, end_name, end_rotation, rpy, urdf_axis):
    """A matching (ProtoJoint, urdf entry) pair built directly, no file I/O."""
    return ProtoJoint(
        name=name,
        axis=axis,
        anchor=anchor,
        end_name=end_name,
        end_translation=anchor,
        end_rotation=end_rotation,
        mass=0.1,
        com=(0.0, 0.0, 0.0),
        inertia=ZERO_INERTIA,
    ), {
        "parent": "Base_Link",
        "child": end_name,
        "xyz": anchor,
        "rpy": rpy,
        "axis": urdf_axis,
    }


def _synthetic():
    """Two joints that match each other exactly (wheel + pendulum, real-data style)."""
    wheel, wheel_u = _joint(
        "wheel_left_joint", (0.0, 1.0, 0.0), WHEEL_ANCHOR, "Wheel_Left",
        IDENTITY_ROTATION, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0),
    )
    pend, pend_u = _joint(
        "pendulum_pivot_joint", (0.0, 1.0, 0.0), PENDULUM_ANCHOR, "Pendulum_Link",
        (1.0, 0.0, 0.0, 1.570796), (1.570796, 0.0, 0.0), (0.0, 0.0, -1.0),
    )
    robot = ProtoRobot(
        root=Physics(mass=0.25, com=(0.0, 0.0, 0.0), inertia=ZERO_INERTIA),
        joints={wheel.name: wheel, pend.name: pend},
    )
    urdf = {wheel.name: wheel_u, pend.name: pend_u}
    return robot, urdf


# --- real data ---------------------------------------------------------------


@pytest.fixture(scope="module", autouse=True)
def proto_file_unchanged():
    """The PROTO is read-only for this module: bytes must match after all tests run."""
    before = PROTO_PATH.read_bytes() if PROTO_PATH.exists() else None
    yield
    after = PROTO_PATH.read_bytes() if PROTO_PATH.exists() else None
    assert after == before, f"PROTO was modified by the tests: {PROTO_PATH}"


@pytest.fixture(scope="module")
def proto():
    assert PROTO_PATH.exists(), f"PROTO missing: {PROTO_PATH} (tracked file must exist)"
    return parse_proto(PROTO_PATH.read_text())


@pytest.fixture(scope="module")
def urdf():
    assert URDF_PATH.exists(), f"URDF missing: {URDF_PATH}"
    return urdf_joints(URDF_PATH)


def _parent_frame_axis(urdf_entry: dict) -> tuple[float, ...]:
    return normalize(mat_vec(rpy_to_matrix(urdf_entry["rpy"]), urdf_entry["axis"]))


def test_joint_sets_non_vacuous(proto, urdf):
    # parse_proto and urdf_joints both raise on duplicate names, so set equality suffices.
    assert set(proto.joints) == set(urdf) == EXPECTED_JOINTS


def test_pendulum_world_axes_are_plus_y(proto, urdf):
    """
    Pendulum axes are +Y in the parent (Base_Link) frame. The check assumes the
    default Robot pose (identity), which is what makes parent frame equal world.
    """
    for name in PENDULUM_JOINTS:
        urdf_axis = _parent_frame_axis(urdf[name])
        proto_axis = normalize(proto.joints[name].axis)
        assert max_abs_diff(urdf_axis, (0.0, 1.0, 0.0)) < TOL, f"{name}: urdf parent-frame axis {urdf_axis}"
        assert max_abs_diff(proto_axis, (0.0, 1.0, 0.0)) < TOL, f"{name}: proto axis {proto_axis}"


def test_compare_joints_matches(proto, urdf):
    assert compare_joints(proto, urdf) == []


# --- helpers (no real files) -------------------------------------------------


def test_rpy_to_matrix_single_axis():
    rotated = mat_vec(rpy_to_matrix((1.570796, 0.0, 0.0)), (0.0, 0.0, -1.0))
    assert max_abs_diff(rotated, (0.0, 1.0, 0.0)) < 1e-6


def test_axis_angle_matches_rpy():
    from_aa = axis_angle_to_matrix((-1.0, 0.0, 0.0, 1.570796))
    assert max_abs_diff(from_aa, rpy_to_matrix((-1.570796, 0.0, 0.0))) < 1e-6


def test_axis_angle_normalises_non_unit_axis():
    assert max_abs_diff(
        axis_angle_to_matrix((2.0, 0.0, 0.0, 1.570796)),
        axis_angle_to_matrix((1.0, 0.0, 0.0, 1.570796)),
    ) < 1e-12


def test_axis_angle_identity_and_errors():
    identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    assert axis_angle_to_matrix((0.0, 0.0, 0.0, 0.0)) == identity
    assert axis_angle_to_matrix((1.0, 0.0, 0.0, 0.0)) == identity
    with pytest.raises(ValueError):
        axis_angle_to_matrix((0.0, 0.0, 0.0, 1.0))
    with pytest.raises(ValueError):
        normalize((0.0, 0.0, 0.0))


def test_compare_joints_multi_axis_rpy_clean():
    # The single rotation-order check: rpy (pi/2, 0, pi/2) is a 120 deg rotation
    # about (1,1,1) (x->y->z->x). An Rx@Ry@Rz order in compare_joints or in
    # rpy_to_matrix gives a different matrix and fails here.
    joint, entry = _joint(
        "wheel_left_joint", (0.0, 0.0, 1.0), WHEEL_ANCHOR, "Wheel_Left",
        (1.0, 1.0, 1.0, 2.0943951), (1.570796, 0.0, 1.570796), (0.0, 1.0, 0.0),
    )
    solo = ProtoRobot(
        root=Physics(mass=0.25, com=(0.0, 0.0, 0.0), inertia=ZERO_INERTIA),
        joints={joint.name: joint},
    )
    assert compare_joints(solo, {joint.name: entry}) == []


def test_urdf_joints_defaults(tmp_path):
    path = tmp_path / "mini.urdf"
    path.write_text(
        '<robot name="r"><joint name="j" type="fixed">'
        '<parent link="a"/><child link="b"/></joint></robot>'
    )
    entry = urdf_joints(path)["j"]
    assert entry == {
        "parent": "a",
        "child": "b",
        "xyz": (0.0, 0.0, 0.0),
        "rpy": (0.0, 0.0, 0.0),
        "axis": (1.0, 0.0, 0.0),
    }


# --- negative tests on synthetic data (each asserts a specific error) --------


def test_compare_joints_detects_axis_sign_flip():
    robot, urdf = _synthetic()
    pend = robot.joints["pendulum_pivot_joint"]
    flipped = replace(pend, axis=(0.0, -1.0, 0.0))
    bad = ProtoRobot(root=robot.root, joints={**robot.joints, pend.name: flipped})
    errors = compare_joints(bad, urdf)
    assert len(errors) == 1  # exactly one error, so the rest of the synthetic baseline is clean
    assert errors[0].startswith("pendulum_pivot_joint: axis ")


def test_compare_joints_detects_wrong_anchor():
    robot, urdf = _synthetic()
    wheel = robot.joints["wheel_left_joint"]
    moved = replace(wheel, anchor=(0.02, -0.028526, 0.054010))
    bad = ProtoRobot(root=robot.root, joints={**robot.joints, wheel.name: moved})
    errors = compare_joints(bad, urdf)
    assert any(e.startswith("wheel_left_joint: anchor ") for e in errors)


def test_compare_joints_detects_wrong_child_name():
    robot, urdf = _synthetic()
    pend = robot.joints["pendulum_pivot_joint"]
    renamed = replace(pend, end_name="Pendulum_Link_Right")
    bad = ProtoRobot(root=robot.root, joints={**robot.joints, pend.name: renamed})
    errors = compare_joints(bad, urdf)
    assert any(e.startswith("pendulum_pivot_joint: child ") for e in errors)


def test_compare_joints_reports_extra_and_missing_keys():
    robot, urdf = _synthetic()
    extra_urdf = {**urdf, "ghost_joint": {**urdf["wheel_left_joint"]}}
    errors = compare_joints(robot, extra_urdf)
    assert any(e.startswith("ghost_joint: ") and "urdf" in e for e in errors)

    missing_urdf = {k: v for k, v in urdf.items() if k != "wheel_left_joint"}
    errors = compare_joints(robot, missing_urdf)
    assert any(e.startswith("wheel_left_joint: ") and "PROTO" in e for e in errors)


def test_compare_joints_reports_rotation_mismatch():
    robot, urdf = _synthetic()
    pend = robot.joints["pendulum_pivot_joint"]
    # Rotation about X by -90 deg instead of +90 deg: matrix differs.
    turned = replace(pend, end_rotation=(1.0, 0.0, 0.0, -1.570796))
    bad = ProtoRobot(root=robot.root, joints={**robot.joints, pend.name: turned})
    errors = compare_joints(bad, urdf)
    assert any(e.startswith("pendulum_pivot_joint: ") and "rotation" in e for e in errors)
