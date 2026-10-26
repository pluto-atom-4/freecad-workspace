#!/usr/bin/env python3
"""
PROTO vs URDF consistency: joints (#392, Part of #103 S2) and links (#393, S3).

Compares the real protos/InvertedPendulumRobot.proto against
06_Exports/urdf/robot.urdf (source of truth). Data and unit tests only:
no Webots, no GUI, no hardware. The PROTO is only read, never written
(an autouse module guard checks its bytes are unchanged).

A missing PROTO FAILS (human decision), it is not skipped.

Usage:
    cd inverted-pendulum-project
    mamba run -n pendulum-tools python3 -m pytest 07_Simulation/webots/test_proto_urdf_consistency.py -q
"""

import json
import math
import os
import subprocess
import sys
from dataclasses import replace
from decimal import Decimal
from fractions import Fraction
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
    compare_links,
    mat_vec,
    max_abs_diff,
    normalize,
    parse_proto,
    proto_links,
    rpy_to_matrix,
    urdf_joints,
    urdf_links,
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


def _with_proto_field(robot, name, field, value):
    """Copy of robot with one field of one joint replaced (frozen dataclasses)."""
    moved = replace(robot.joints[name], **{field: value})
    return ProtoRobot(root=robot.root, joints={**robot.joints, name: moved})


def _with_urdf_field(urdf, name, field, value):
    return {**urdf, name: {**urdf[name], field: value}}


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


def test_all_urdf_parents_are_base_link(urdf):
    # compare_joints does not compare the parent: the PROTO has no parent name.
    # This test documents the assumption that every joint hangs off Base_Link.
    parents = {name: entry["parent"] for name, entry in urdf.items()}
    assert all(p == "Base_Link" for p in parents.values()), parents


def test_pendulum_parent_frame_axes_are_plus_y(proto, urdf):
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


@pytest.mark.parametrize("joint", sorted(EXPECTED_JOINTS))
def test_anchor_perturbation_reported_for_every_joint(proto, urdf, joint):
    # Perturb the parsed real PROTO in memory (frozen dataclasses, no file write).
    # Covers each joint, so suppressing errors for any one of them fails here.
    pj = proto.joints[joint]
    bad = _with_proto_field(proto, joint, "anchor", (pj.anchor[0] + 1e-3, pj.anchor[1], pj.anchor[2]))
    errors = compare_joints(bad, urdf)
    assert any(e.startswith(f"{joint}: ") and "anchor" in e for e in errors), errors


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


def test_max_abs_diff_propagates_nan_wherever_it_sits():
    # max() alone would return 0.0 here, hiding the NaN in the second slot.
    assert math.isnan(max_abs_diff((0.0, math.nan), (0.0, 0.0)))
    assert math.isnan(max_abs_diff((math.nan, 0.0), (0.0, 0.0)))
    assert max_abs_diff((0.0, 2.0), (0.0, 0.5)) == 1.5


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
    bad = _with_proto_field(robot, "pendulum_pivot_joint", "axis", (0.0, -1.0, 0.0))
    errors = compare_joints(bad, urdf)
    assert len(errors) == 1  # exactly one error, so the rest of the synthetic baseline is clean
    assert errors[0].startswith("pendulum_pivot_joint: axis ")


def test_compare_joints_detects_wrong_anchor():
    # Synthetic (no real file). Kept because it is the only synthetic anchor test.
    robot, urdf = _synthetic()
    bad = _with_proto_field(robot, "wheel_left_joint", "anchor", (0.02, -0.028526, 0.054010))
    errors = compare_joints(bad, urdf)
    assert any(e.startswith("wheel_left_joint: anchor ") for e in errors)


def test_compare_joints_detects_wrong_child_name():
    robot, urdf = _synthetic()
    bad = _with_proto_field(robot, "pendulum_pivot_joint", "end_name", "Pendulum_Link_Right")
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
    # Rotation about X by -90 deg instead of +90 deg: matrix differs.
    bad = _with_proto_field(robot, "pendulum_pivot_joint", "end_rotation", (1.0, 0.0, 0.0, -1.570796))
    errors = compare_joints(bad, urdf)
    assert any(e.startswith("pendulum_pivot_joint: ") and "rotation" in e for e in errors)


def _wheel_errors(proto_axis, urdf_axis, rpy=(0.0, 0.0, 0.0)):
    robot, urdf = _synthetic()
    robot = _with_proto_field(robot, "wheel_left_joint", "axis", proto_axis)
    urdf = _with_urdf_field(urdf, "wheel_left_joint", "axis", urdf_axis)
    urdf = _with_urdf_field(urdf, "wheel_left_joint", "rpy", rpy)
    return [e for e in compare_joints(robot, urdf) if e.startswith("wheel_left_joint: ")]


# --- axis scale: squares must not overflow or underflow ----------------------


@pytest.mark.parametrize(
    "proto_axis,urdf_axis",
    [((0.0, 0.0, 1e200), (1e200, 0.0, 0.0)), ((0.0, 0.0, 1.4e154), (1.4e154, 0.0, 0.0))],
    ids=["1e200", "1.4e154"],
)
def test_wrong_direction_huge_axes_reported(proto_axis, urdf_axis):
    errors = _wheel_errors(proto_axis, urdf_axis)
    assert any(e.startswith("wheel_left_joint: axis ") for e in errors), errors


def test_correct_huge_urdf_axis_matches():
    assert _wheel_errors((1.0, 0.0, 0.0), (1e308, 0.0, 0.0)) == []


def test_correct_tiny_axis_matches():
    assert _wheel_errors((1e-160, 0.0, 0.0), (1.0, 0.0, 0.0)) == []


def test_subnormal_axis_is_a_direction_not_zero():
    assert normalize((1e-320, 0.0, 0.0)) == (1.0, 0.0, 0.0)
    assert _wheel_errors((1e-320, 0.0, 0.0), (1.0, 0.0, 0.0)) == []


def test_truly_zero_axis_still_errors():
    with pytest.raises(ValueError):
        normalize((0.0, 0.0, 0.0))
    errors = _wheel_errors((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert any(e.startswith("wheel_left_joint: axis invalid") for e in errors), errors


def test_huge_axis_angle_axis_matches():
    huge = axis_angle_to_matrix((1e200, 0.0, 0.0, 1.570796))
    unit = axis_angle_to_matrix((1.0, 0.0, 0.0, 1.570796))
    assert max_abs_diff(huge, unit) < 1e-12


def test_huge_finite_urdf_axis_rotated_mismatch_reported():
    # Rotation-consistent: rpy (0,0,pi/4) and end_rotation (0,0,1,pi/4) agree, so
    # only the axis differs. The URDF axis direction (1,-1,0)/sqrt2 rotates to (1,0,0).
    robot, urdf = _synthetic()
    robot = _with_proto_field(robot, "wheel_left_joint", "end_rotation", (0.0, 0.0, 1.0, math.pi / 4))
    urdf = _with_urdf_field(urdf, "wheel_left_joint", "rpy", (0.0, 0.0, math.pi / 4))
    urdf = _with_urdf_field(urdf, "wheel_left_joint", "axis", (1.5e308, -1.5e308, 0.0))
    errors = compare_joints(robot, urdf)
    assert any(e.startswith("wheel_left_joint: axis ") for e in errors), errors


# --- non-finite values: one case per field x {nan, inf} ----------------------

# (target, field, error word): target "proto" changes the PROTO joint, "urdf" the entry.
NONFINITE_FIELDS = [
    ("proto", "anchor", "anchor"),
    ("proto", "end_translation", "end_translation"),
    ("proto", "end_rotation", "end_rotation"),
    ("proto", "axis", "axis"),
    ("urdf", "xyz", "urdf xyz"),
    ("urdf", "rpy", "urdf rpy"),
    ("urdf", "axis", "urdf axis"),
]


def _non_finite_errors(target, field, index, bad):
    name = "wheel_left_joint"
    robot, urdf = _synthetic()
    if target == "proto":
        vec = list(getattr(robot.joints[name], field))
        vec[index] = bad
        robot = _with_proto_field(robot, name, field, tuple(vec))
    else:
        vec = list(urdf[name][field])
        vec[index] = bad
        urdf = _with_urdf_field(urdf, name, field, tuple(vec))
    return compare_joints(robot, urdf)


@pytest.mark.parametrize("target,field,word", NONFINITE_FIELDS, ids=[f[2] for f in NONFINITE_FIELDS])
def test_nan_value_reported(target, field, word):
    errors = _non_finite_errors(target, field, 0, math.nan)
    assert any(e.startswith("wheel_left_joint: ") and word in e and "non-finite" in e for e in errors), errors


@pytest.mark.parametrize(
    "target,field,word,index,bad",
    [
        ("proto", "anchor", "anchor", 0, math.inf),
        ("proto", "end_rotation", "end_rotation", 3, math.inf),   # the angle element
        ("proto", "end_rotation", "end_rotation", 3, -math.inf),  # the angle element
        ("urdf", "axis", "urdf axis", 1, -math.inf),
    ],
    ids=["anchor-inf", "end_rotation-angle-inf", "end_rotation-angle-minus-inf", "urdf-axis-minus-inf"],
)
def test_infinite_value_reported(target, field, word, index, bad):
    errors = _non_finite_errors(target, field, index, bad)
    assert any(e.startswith("wheel_left_joint: ") and word in e and "non-finite" in e for e in errors), errors


# --- malformed per-joint shapes and types: error strings, never raises ------

MALFORMED_CASES = [
    ("urdf", "rpy", (0.0, 0.0), "urdf rpy"),
    ("urdf", "rpy", (0.0, "a", 0.0), "urdf rpy"),
    ("urdf", "xyz", (0.0, 0.0), "urdf xyz"),
    ("urdf", "xyz", None, "urdf xyz"),
    ("urdf", "axis", (0.0, 1.0), "urdf axis"),
    ("proto", "anchor", (0.0, 0.0), "anchor"),
    ("proto", "end_translation", (0.0, 0.0), "end_translation"),
    ("proto", "end_rotation", (1.0, 0.0, 0.0), "end_rotation"),
    ("proto", "axis", (0.0, 1.0), "axis"),
    ("urdf", "xyz", (10**400, 0.0, 0.0), "urdf xyz"),
]


@pytest.mark.parametrize(
    "target,field,value,word",
    MALFORMED_CASES,
    ids=[f"{i}-{c[0]}-{c[1]}" for i, c in enumerate(MALFORMED_CASES)],
)
def test_malformed_shape_reported_not_raised(target, field, value, word):
    name = "wheel_left_joint"
    robot, urdf = _synthetic()
    if target == "proto":
        robot = _with_proto_field(robot, name, field, value)
    else:
        urdf = _with_urdf_field(urdf, name, field, value)
    errors = compare_joints(robot, urdf)  # must not raise
    assert any(e.startswith(f"{name}: ") and word in e and "malformed" in e for e in errors), errors


@pytest.mark.parametrize("key", ["xyz", "rpy", "axis", "child"])
def test_missing_urdf_key_reported_not_raised(key):
    name = "wheel_left_joint"
    robot, urdf = _synthetic()
    entry = {k: v for k, v in urdf[name].items() if k != key}
    errors = compare_joints(robot, {**urdf, name: entry})  # must not raise
    assert f"{name}: urdf entry missing key {key!r}" in errors, errors


def test_non_mapping_urdf_entry_reported_not_raised():
    name = "wheel_left_joint"
    robot, urdf = _synthetic()
    errors = compare_joints(robot, {**urdf, name: "not a dict"})  # must not raise
    assert f"{name}: urdf entry is not a mapping" in errors, errors


def test_malformed_rpy_and_malformed_end_rotation_both_reported():
    # Each conversion reports its own failure; a bad rpy must not hide a bad end_rotation.
    name = "wheel_left_joint"
    robot, urdf = _synthetic()
    robot = _with_proto_field(robot, name, "end_rotation", (0.0, 0.0, 0.0, 0.5))
    urdf = _with_urdf_field(urdf, name, "rpy", (0.0, 0.0))
    errors = compare_joints(robot, urdf)
    assert any(e.startswith(f"{name}: rotation invalid") for e in errors), errors
    assert any(e.startswith(f"{name}: urdf rpy malformed") for e in errors), errors


# --- per-link mass, centre of mass and inertia (#393, S3) --------------------

LINK_NAMES = {"Base_Link", "Wheel_Left", "Wheel_Right", "Pendulum_Link", "Pendulum_Link_Right"}
WHEEL_INERTIA = (2e-6, 3e-6, 2e-6, 0.0, 0.0, 0.0)


@pytest.fixture(scope="module")
def urdf_link_data():
    assert URDF_PATH.exists(), f"URDF missing: {URDF_PATH}"
    return urdf_links(URDF_PATH)


def _with_link_field(robot, name, **fields):
    """Copy of robot with fields of one link (Base_Link or an endPoint Solid) replaced."""
    if name == "Base_Link":
        return ProtoRobot(root=replace(robot.root, **fields), joints=robot.joints)
    joint = next(j for j in robot.joints.values() if j.end_name == name)
    return ProtoRobot(root=robot.root, joints={**robot.joints, joint.name: replace(joint, **fields)})


def _link_pair():
    """Base_Link + Wheel_Left built directly (no file I/O); compare_links returns []."""
    wheel = ProtoJoint(
        name="wheel_left_joint", axis=(0.0, 1.0, 0.0), anchor=WHEEL_ANCHOR,
        end_name="Wheel_Left", end_translation=WHEEL_ANCHOR, end_rotation=IDENTITY_ROTATION,
        mass=0.03, com=(0.0, 0.0, 0.0), inertia=WHEEL_INERTIA,
    )
    robot = ProtoRobot(
        root=Physics(mass=0.25, com=(0.0, 0.0, 0.0), inertia=ZERO_INERTIA),
        joints={wheel.name: wheel},
    )
    urdf = {
        "Base_Link": {"mass": 0.25, "com": (0.0, 0.0, 0.0), "rpy": (0.0, 0.0, 0.0), "inertia": ZERO_INERTIA},
        "Wheel_Left": {"mass": 0.03, "com": (0.0, 0.0, 0.0), "rpy": (0.0, 0.0, 0.0), "inertia": WHEEL_INERTIA},
    }
    return robot, urdf


# real data

def test_link_sets_non_vacuous(proto, urdf_link_data):
    assert set(proto_links(proto)) == set(urdf_link_data) == LINK_NAMES


def test_compare_links_matches(proto, urdf_link_data):
    assert compare_links(proto, urdf_link_data) == []


@pytest.mark.parametrize("link", sorted(LINK_NAMES))
def test_mass_perturbation_reported_for_every_link(proto, urdf_link_data, link):
    mass = proto_links(proto)[link].mass
    bad = _with_link_field(proto, link, mass=mass + 1e-3)
    errors = compare_links(bad, urdf_link_data)
    assert any(e.startswith(f"{link}: ") and "mass" in e for e in errors), errors


def test_com_perturbation_reported(proto, urdf_link_data):
    com = proto_links(proto)["Pendulum_Link"].com
    bad = _with_link_field(proto, "Pendulum_Link", com=(com[0], com[1] + 1e-3, com[2]))
    errors = compare_links(bad, urdf_link_data)
    assert any(e.startswith("Pendulum_Link: ") and "com" in e for e in errors), errors


def test_pendulum_inertia_product_sign_flip_reported(proto, urdf_link_data):
    inertia = proto_links(proto)["Pendulum_Link"].inertia
    flipped = (*inertia[:3], -inertia[3], *inertia[4:])
    errors = compare_links(_with_link_field(proto, "Pendulum_Link", inertia=flipped), urdf_link_data)
    assert any(e.startswith("Pendulum_Link: inertia") for e in errors), errors


def test_inertia_perturbation_reported(proto, urdf_link_data):
    inertia = proto_links(proto)["Base_Link"].inertia
    bad = _with_link_field(proto, "Base_Link", inertia=(*inertia[:2], inertia[2] + 1e-7, *inertia[3:]))
    errors = compare_links(bad, urdf_link_data)
    assert any(e.startswith("Base_Link: ") and "inertia" in e for e in errors), errors


def test_pendulum_ixz_iyz_swap_reported(proto, urdf_link_data):
    inertia = proto_links(proto)["Pendulum_Link"].inertia
    swapped = (*inertia[:4], inertia[5], inertia[4])
    errors = compare_links(_with_link_field(proto, "Pendulum_Link", inertia=swapped), urdf_link_data)
    assert any(e.startswith("Pendulum_Link: inertia") for e in errors), errors


def test_key_set_extra_and_missing_links_reported(proto, urdf_link_data):
    extra = {**urdf_link_data, "ghost_link": dict(urdf_link_data["Wheel_Left"])}
    assert "ghost_link: link key only in urdf, missing from PROTO" in compare_links(proto, extra)
    missing = {k: v for k, v in urdf_link_data.items() if k != "Wheel_Left"}
    assert "Wheel_Left: link key only in PROTO, missing from urdf" in compare_links(proto, missing)


def test_duplicate_proto_end_name_reported_not_overwritten(proto, urdf_link_data):
    bad = _with_link_field(proto, "Pendulum_Link", end_name="Wheel_Left")
    errors = compare_links(bad, urdf_link_data)
    assert len(errors) == 1 and "duplicate link name 'Wheel_Left'" in errors[0], errors


def test_urdf_inertial_rpy_nonzero_reported():
    robot, urdf = _link_pair()
    errors = compare_links(robot, _with_urdf_field(urdf, "Wheel_Left", "rpy", (0.0, 0.0, 0.1)))
    assert len(errors) == 1 and errors[0].startswith("Wheel_Left: urdf inertial rpy"), errors


# synthetic

def test_wheel_ixx_iyy_swap_caught_by_inertia_tol():
    robot, urdf = _link_pair()
    swapped = _with_urdf_field(urdf, "Wheel_Left", "inertia", (3e-6, 2e-6, 2e-6, 0.0, 0.0, 0.0))
    errors = compare_links(robot, swapped)
    assert any(e.startswith("Wheel_Left: inertia") for e in errors), errors
    # Swap diff is 1e-6 (+2e-16 float): a loose tolerance such as 1.5e-6 lets it pass.
    assert compare_links(robot, swapped, inertia_tol=1.5e-6) == []


@pytest.mark.parametrize(
    "target,name,key,value,word",
    [
        ("urdf", "Wheel_Left", "mass", math.nan, "non-finite"),
        ("proto", "Wheel_Left", "com", (math.inf, 0.0, 0.0), "non-finite"),
        ("urdf", "Base_Link", "inertia", (0.0,) * 5, "malformed"),
        ("urdf", "Wheel_Left", "com", None, "malformed"),
    ],
    ids=["nan-mass", "inf-com", "5-inertia", "none-com"],
)
def test_malformed_link_value_reported_not_raised(target, name, key, value, word):
    robot, urdf = _link_pair()
    if target == "proto":
        robot = _with_link_field(robot, name, **{key: value})
    else:
        urdf = _with_urdf_field(urdf, name, key, value)
    errors = compare_links(robot, urdf)  # must not raise
    assert any(e.startswith(f"{name}: ") and word in e for e in errors), errors


@pytest.mark.parametrize(
    "entry,message",
    [
        ({k: v for k, v in _link_pair()[1]["Wheel_Left"].items() if k != "mass"}, "urdf entry missing key 'mass'"),
        (None, "urdf entry is not a mapping"),
    ],
    ids=["missing-mass", "non-mapping"],
)
def test_bad_link_entry_reported_not_raised(entry, message):
    robot, urdf = _link_pair()
    errors = compare_links(robot, {**urdf, "Wheel_Left": entry})  # must not raise
    assert f"Wheel_Left: {message}" in errors, errors


@pytest.mark.parametrize("case", ["urdf-none", "root-none", "joints-none"])
def test_wrong_type_containers_reported_not_raised(proto, urdf_link_data, case):
    args = {
        "urdf-none": (proto, None),
        "root-none": (replace(proto, root=None), urdf_link_data),
        "joints-none": (replace(proto, joints=None), urdf_link_data),
    }[case]
    assert compare_links(*args)  # must not raise, must report


@pytest.mark.parametrize(
    "kwargs", [{"tol": math.nan}, {"tol": -1}, {"tol": None}, {"inertia_tol": math.nan}],
    ids=["tol-nan", "tol-negative", "tol-none", "inertia_tol-nan"],
)
def test_invalid_tolerance_raises(proto, urdf_link_data, kwargs):
    with pytest.raises(ValueError):
        compare_links(proto, urdf_link_data, **kwargs)


@pytest.mark.parametrize(
    "tol", [math.nan, -1, None, "1e-6", math.inf, True, Decimal("1e-6"), complex(0, 1)],
    ids=["nan", "negative", "none", "str", "inf", "bool", "decimal", "complex"],
)
def test_compare_joints_invalid_tol_raises(proto, urdf, tol):
    with pytest.raises(ValueError):
        compare_joints(proto, urdf, tol=tol)


def test_fraction_tolerance_accepted(proto, urdf, urdf_link_data):
    # numbers.Real covers Fraction (and numpy scalars) without importing numpy here.
    assert compare_joints(proto, urdf, tol=Fraction(1, 10**6)) == []
    assert compare_links(proto, urdf_link_data, tol=Fraction(1, 10**6)) == []


def test_mixed_key_types_reported_not_raised(proto, urdf_link_data):
    # Non-str keys on both sides: PROTO end_name 5 and URDF key 5 match (compared, so mismatched),
    # None and "Q" are URDF-only, and the renamed Pendulum_Link is URDF-only.
    robot = _with_link_field(proto, "Pendulum_Link", end_name=5)
    urdf = {**urdf_link_data, 5: dict(urdf_link_data["Wheel_Left"]), None: {}, "Q": {}}
    errors = compare_links(robot, urdf)  # must not raise
    assert any(e.startswith("5: ") for e in errors), errors
    assert "Pendulum_Link: link key only in urdf, missing from PROTO" in errors, errors
    assert "None: link key only in urdf, missing from PROTO" in errors, errors
    assert "Q: link key only in urdf, missing from PROTO" in errors, errors


def test_urdf_links_mass_is_float(urdf_link_data):
    mass = urdf_link_data["Wheel_Left"]["mass"]
    assert isinstance(mass, float) and abs(mass - 0.03) < 1e-9, mass


# Probe for the tie case: PROTO end_name 5 (int) and "5" (str) plus URDF keys 5 and "5"
# land in the same intersection sort, so their error order is what the sort key decides.
_TIE_PROBE = """
import json, sys
from dataclasses import replace
sys.path.insert(0, sys.argv[1])
from proto_urdf_compare import PROTO_PATH, ProtoRobot, URDF_PATH, compare_joints, compare_links, parse_proto, urdf_joints, urdf_links
p = parse_proto(PROTO_PATH.read_text())
j = dict(p.joints)
j["wheel_left_joint"] = replace(j["wheel_left_joint"], end_name="5")
j["pendulum_pivot_joint"] = replace(j["pendulum_pivot_joint"], end_name=5)
u = urdf_links(URDF_PATH)
u[5] = dict(u["Wheel_Right"])
u["5"] = dict(u["Pendulum_Link"])
links = compare_links(ProtoRobot(root=p.root, joints=j), u)
# Joint side: PROTO joints keyed 5 and "5" against swapped URDF entries (distinct messages).
uj = urdf_joints(URDF_PATH)
pj_joints = {5: j["pendulum_pivot_joint"], "5": j["wheel_left_joint"]}
uj_swapped = {5: uj["wheel_left_joint"], "5": uj["pendulum_pivot_joint"]}
joints = compare_joints(ProtoRobot(root=p.root, joints=pj_joints), uj_swapped)
print(json.dumps({"links": links, "joints": joints}))
"""


def test_mixed_key_tie_order_independent_of_hash_seed():
    outputs = []
    for seed in ("0", "1", "2", "3"):
        env = {**os.environ, "PYTHONHASHSEED": seed}
        run = subprocess.run(
            [sys.executable, "-c", _TIE_PROBE, str(Path(__file__).resolve().parent)],
            env=env, capture_output=True, text=True, check=True,
        )
        outputs.append(run.stdout)
    assert len(set(outputs)) == 1, outputs
    result = json.loads(outputs[0])
    mass = [e for e in result["links"] if e.startswith("5: mass ")]
    # Total order: the str "5" (repr "'5'") sorts before the int 5 (repr "5").
    assert len(mass) == 2 and mass[0].startswith("5: mass 0.03 ") and mass[1].startswith("5: mass 0.175 "), mass
    anchors = [e for e in result["joints"] if e.startswith("5: anchor ")]
    assert len(anchors) == 2 and anchors[0].startswith("5: anchor (0.014001"), anchors


INERTIA_XML = '<inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/>'
MALFORMED_INERTIAL = {
    "com-two-numbers": '<mass value="1"/><origin xyz="1 2"/>' + INERTIA_XML,
    "missing-mass": '<origin xyz="0 0 0"/>' + INERTIA_XML,
    "missing-inertia": '<mass value="1"/><origin xyz="0 0 0"/>',
    "non-numeric-mass": '<mass value="abc"/><origin xyz="0 0 0"/>' + INERTIA_XML,
}


def _urdf_file(tmp_path, inertial):
    path = tmp_path / "mini.urdf"  # a temp file: the real URDF is never touched
    path.write_text(f'<robot name="r"><link name="L"><inertial>{inertial}</inertial></link></robot>')
    return path


def test_urdf_links_minimal_valid_file(tmp_path):
    entry = urdf_links(_urdf_file(tmp_path, '<mass value="2"/><origin xyz="1 2 3"/>' + INERTIA_XML))["L"]
    assert entry == {"mass": 2.0, "com": (1.0, 2.0, 3.0), "rpy": (0.0, 0.0, 0.0), "inertia": (1.0, 1.0, 1.0, 0.0, 0.0, 0.0)}


@pytest.mark.parametrize("inertial", list(MALFORMED_INERTIAL.values()), ids=list(MALFORMED_INERTIAL))
def test_malformed_urdf_inertial_raises(tmp_path, inertial):
    with pytest.raises(ValueError):
        urdf_links(_urdf_file(tmp_path, inertial))


def _inertial_robot(mass='<mass value="1"/>', origin='<origin xyz="0 0 0"/>', inertia=INERTIA_XML):
    return f'<robot name="r"><link name="L"><inertial>{mass}{origin}{inertia}</inertial></link></robot>'


@pytest.mark.parametrize(
    "text",
    [
        "",  # empty file
        '<robot name="r"><link name="L">',  # malformed XML (unclosed tags)
        _inertial_robot().replace("<robot ", "<foo ").replace("</robot>", "</foo>"),  # valid body, wrong root
        _inertial_robot(mass='<mass value="nan"/>'),
        _inertial_robot(mass='<mass value="inf"/>'),
        _inertial_robot(mass='<mass value="1e400"/>'),  # float("1e400") is inf
        _inertial_robot(mass='<mass value="-0.25"/>'),
        _inertial_robot(origin='<origin xyz="nan 0 0"/>'),
        _inertial_robot(origin='<origin xyz="0 inf 0"/>'),
        _inertial_robot(inertia=INERTIA_XML.replace('ixx="1"', 'ixx="nan"')),
    ],
    ids=["empty", "malformed-xml", "wrong-root", "mass-nan", "mass-inf", "mass-1e400",
         "mass-negative", "com-nan", "com-inf", "inertia-nan"],
)
def test_bad_urdf_file_raises_value_error(tmp_path, text):
    path = tmp_path / "bad.urdf"  # a temp file: the real URDF is never touched
    path.write_text(text)
    with pytest.raises(ValueError):
        urdf_links(path)


@pytest.mark.parametrize("value", ["0", "-0.0"])
def test_zero_mass_accepted_and_normalised(tmp_path, value):
    path = tmp_path / "zero.urdf"
    path.write_text(_inertial_robot(mass=f'<mass value="{value}"/>'))
    mass = urdf_links(path)["L"]["mass"]
    assert mass == 0.0 and math.copysign(1.0, mass) == 1.0, mass


def test_robot_without_links_is_empty(tmp_path):
    path = tmp_path / "empty_robot.urdf"
    path.write_text('<robot name="r"/>')
    assert urdf_links(path) == {}  # legal; compare_links then reports the missing links


def test_urdf_joints_bad_root_and_empty_file_raise(tmp_path):
    wrong = tmp_path / "wrong.urdf"
    wrong.write_text('<foo><joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint></foo>')
    empty = tmp_path / "empty.urdf"
    empty.write_text("")
    with pytest.raises(ValueError):
        urdf_joints(wrong)
    with pytest.raises(ValueError):
        urdf_joints(empty)


def test_missing_urdf_file_raises_oserror(tmp_path):
    with pytest.raises(OSError):
        urdf_links(tmp_path / "does_not_exist.urdf")
