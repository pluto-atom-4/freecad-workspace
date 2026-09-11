#!/usr/bin/env python3
"""
Tests for Stage 2's `08_configure_assembly_joints.py` (Issue #61).

Pure-Python (no FreeCAD import, no FreeCAD process, no live bridge) regression
test asserting `joint_config.json`'s 4 joint origins match Stage 1's link
placements from `07_body_wheels_metadata.json`. This gap was flagged during
Issue #63 investigation -- no existing test covered it -- and the match was
confirmed live via the FreeCAD MCP bridge already; this test pins that
live-verified result so it can't silently regress.

Usage:
    mamba run -n pendulum-tools python3 -m pytest -q test_08_configure_assembly_joints.py
    # or, with the env already active:
    python3 -m pytest -q test_08_configure_assembly_joints.py
"""

import json
from pathlib import Path

import pytest

SCRIPT_DIR = Path(__file__).resolve().parent
JOINT_CONFIG_JSON = SCRIPT_DIR / "joint_config.json"
METADATA_JSON = SCRIPT_DIR / "07_body_wheels_metadata.json"
ORIGIN_TOLERANCE_MM = 0.01

# joint name -> Stage 1 link name whose placement it must originate from
JOINT_TO_LINK = {
    "wheel_left_joint": "Wheel_Left",
    "wheel_right_joint": "Wheel_Right",
    "pendulum_pivot_joint": "Pendulum_Link",
    "pendulum_pivot_right_joint": "Pendulum_Link_Right",
}


def _load_joint_config():
    if not JOINT_CONFIG_JSON.is_file():
        pytest.skip(f"{JOINT_CONFIG_JSON.name} not found -- run 08_configure_assembly_joints.py first")
    with open(JOINT_CONFIG_JSON) as f:
        return json.load(f)


def _load_metadata():
    if not METADATA_JSON.is_file():
        pytest.skip(f"{METADATA_JSON.name} not found -- run 07_create_body_and_wheels.py first")
    with open(METADATA_JSON) as f:
        return json.load(f)


def _assert_origin_matches_link_placement(config, metadata, joint_name, link_name):
    origin = config["joints"][joint_name]["origin_mm"]
    position = metadata["links"][link_name]["placement"]["position"]
    for axis in ("x", "y", "z"):
        assert origin[axis] == pytest.approx(position[axis], abs=ORIGIN_TOLERANCE_MM), (
            f"{joint_name}.origin_mm.{axis}={origin[axis]} does not match "
            f"{link_name}.placement.position.{axis}={position[axis]} within "
            f"{ORIGIN_TOLERANCE_MM} mm"
        )


def test_joint_config_has_four_joints():
    config = _load_joint_config()
    assert set(config["joints"].keys()) == {
        "wheel_left_joint",
        "wheel_right_joint",
        "pendulum_pivot_joint",
        "pendulum_pivot_right_joint",
    }


def test_all_joints_are_revolute():
    config = _load_joint_config()
    for name, joint in config["joints"].items():
        assert joint["type"] == "Revolute", f"{name}.type={joint['type']!r}"


def test_all_joints_share_global_y_axis():
    config = _load_joint_config()
    for name, joint in config["joints"].items():
        assert joint["axis"] == "global Y (0,1,0)", f"{name}.axis={joint['axis']!r}"


def test_wheel_left_joint_origin_matches_link_placement():
    config = _load_joint_config()
    metadata = _load_metadata()
    _assert_origin_matches_link_placement(config, metadata, "wheel_left_joint", "Wheel_Left")


def test_wheel_right_joint_origin_matches_link_placement():
    config = _load_joint_config()
    metadata = _load_metadata()
    _assert_origin_matches_link_placement(config, metadata, "wheel_right_joint", "Wheel_Right")


def test_pendulum_pivot_joint_origin_matches_link_placement():
    config = _load_joint_config()
    metadata = _load_metadata()
    _assert_origin_matches_link_placement(config, metadata, "pendulum_pivot_joint", "Pendulum_Link")


def test_pendulum_pivot_right_joint_origin_matches_link_placement():
    config = _load_joint_config()
    metadata = _load_metadata()
    _assert_origin_matches_link_placement(
        config, metadata, "pendulum_pivot_right_joint", "Pendulum_Link_Right"
    )


def test_all_joints_reference_base_link_link_as_ground():
    config = _load_joint_config()
    for name, joint in config["joints"].items():
        assert joint["reference2"] == "Base_Link_Link", f"{name}.reference2={joint['reference2']!r}"


def test_joint_reference1_matches_moving_link():
    config = _load_joint_config()
    for joint_name, link_name in JOINT_TO_LINK.items():
        expected = f"{link_name}_Link"
        actual = config["joints"][joint_name]["reference1"]
        assert actual == expected, f"{joint_name}.reference1={actual!r} (expected {expected!r})"


def test_links_section_covers_all_five_link_sources():
    config = _load_joint_config()
    links = config["links"]
    assert set(links.keys()) == {
        "Base_Link_Link",
        "Wheel_Left_Link",
        "Wheel_Right_Link",
        "Pendulum_Link_Link",
        "Pendulum_Link_Right_Link",
    }
    for link_name, link in links.items():
        expected_linked_object = link_name[: -len("_Link")]
        assert link["linked_object"] == expected_linked_object, (
            f"{link_name}.linked_object={link['linked_object']!r} "
            f"(expected {expected_linked_object!r})"
        )


def test_all_validations_passed():
    config = _load_joint_config()
    for validation in config["validations"]:
        assert validation["passed"] is True, (
            f"{validation['check']} did not pass: {validation.get('details')}"
        )


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
