"""
Tests for joint geometry orientation (unit-test level, no Webots needed).

Validates that primitive cylinder geometries (visual and collision) are correctly oriented
to align with their parent joint's rotation axis.

Pure Python (no FreeCAD imports) — reads the exported URDF and validates the math.
Follows the pattern of test_mesh_integrity.py and test_10_export_urdf.py.
"""

import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

# Locate the URDF and meshes
SCRIPT_DIR = Path(__file__).resolve().parent
EXPORTS_DIR = SCRIPT_DIR.parent.parent / "06_Exports"
URDF_FILE = EXPORTS_DIR / "urdf" / "robot.urdf"


def load_urdf():
    """Load and parse the URDF XML file, skip test if missing."""
    if not URDF_FILE.exists():
        pytest.skip(f"URDF file not found: {URDF_FILE}")
    tree = ET.parse(URDF_FILE)
    return tree.getroot()


def _rpy_to_matrix(rpy_str: str) -> np.ndarray:
    """Convert URDF rpy (roll, pitch, yaw) string to rotation matrix.

    URDF convention: fixed-axis (extrinsic) X->Y->Z rotation.
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll).

    Deliberately NOT reusing 10_export_urdf.py's apply_rotation_to_vector
    (would make this test circular/tautological — a bug in that function
    would still "pass" trivially if we reused it to verify the URDF).
    """
    roll, pitch, yaw = (float(x) for x in rpy_str.split())
    return Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()


def _axis_alignment_dot(rpy_str: str, joint_axis: np.ndarray) -> float:
    """Compute absolute dot product of rotated local Z-axis with joint axis.

    In URDF, a cylinder's default local axis is Z [0, 0, 1].
    After applying the origin's rpy rotation, we check if the rotated Z-axis
    aligns with the joint's rotation axis.

    Returns abs(dot), so both +1 (aligned) and -1 (anti-aligned) pass.
    """
    local_z = np.array([0.0, 0.0, 1.0])
    rot_matrix = _rpy_to_matrix(rpy_str)
    world_z = rot_matrix @ local_z
    joint_axis_normalized = joint_axis / np.linalg.norm(joint_axis)
    return abs(float(np.dot(world_z, joint_axis_normalized)))


def test_rpy_to_matrix_axis_alignment_math_sanity():
    """Pure math meta-test: verify _rpy_to_matrix and _axis_alignment_dot logic.

    No URDF load needed. This guards the helper functions' correctness.

    Test: wheel cylinder rpy "-1.570796 0 0" (the actual fix from #109)
    should align local Z [0,0,1] with joint axis [0,1,0].
    Expected dot product ≈ 1.0, within tolerance 1e-4.
    """
    rpy = "-1.570796 0 0"
    joint_axis = np.array([0.0, 1.0, 0.0])
    dot = _axis_alignment_dot(rpy, joint_axis)
    assert dot > 1.0 - 1e-4, (
        f"Math sanity check failed: rpy='{rpy}' and joint_axis={joint_axis} "
        f"should align (dot ≈ 1.0), got {dot:.6f}"
    )


def test_wheel_visual_cylinder_axis_aligns_with_joint_axis():
    """Wheel visual cylinder geometry is correctly oriented with its joint axis.

    Tests both wheel_left_joint and wheel_right_joint.
    For each wheel, finds the joint's axis from the URDF, then verifies
    the child link's visual cylinder's rpy aligns the local Z-axis with that axis.
    """
    root = load_urdf()
    tolerance = 1e-4

    wheel_specs = [
        ('wheel_left_joint', 'Wheel_Left'),
        ('wheel_right_joint', 'Wheel_Right'),
    ]

    for joint_name, link_name in wheel_specs:
        # Find joint element
        joint_elem = root.find(f".//joint[@name='{joint_name}']")
        assert joint_elem is not None, f"Joint '{joint_name}' not found in URDF"

        # Extract joint axis
        axis_elem = joint_elem.find('axis')
        assert axis_elem is not None, f"Joint '{joint_name}' has no <axis> element"
        axis_xyz_str = axis_elem.get('xyz')
        assert axis_xyz_str, f"Joint '{joint_name}' axis missing 'xyz' attribute"
        joint_axis = np.array([float(x) for x in axis_xyz_str.split()])

        # Find link element
        link_elem = root.find(f".//link[@name='{link_name}']")
        assert link_elem is not None, f"Link '{link_name}' not found in URDF"

        # Find visual geometry (cylinder)
        visual_elem = link_elem.find('visual')
        assert visual_elem is not None, f"Link '{link_name}' has no <visual> element"

        geometry_elem = visual_elem.find('geometry')
        assert geometry_elem is not None, f"Visual in '{link_name}' has no <geometry>"

        cylinder_elem = geometry_elem.find('cylinder')
        assert cylinder_elem is not None, (
            f"Visual geometry in '{link_name}' is not a cylinder"
        )

        # Get origin rpy (default to '0 0 0' if not present)
        origin_elem = visual_elem.find('origin')
        if origin_elem is not None:
            rpy_str = origin_elem.get('rpy', '0 0 0')
        else:
            rpy_str = '0 0 0'

        # Check alignment
        dot = _axis_alignment_dot(rpy_str, joint_axis)
        assert dot > 1.0 - tolerance, (
            f"Joint '{joint_name}' / Link '{link_name}' visual cylinder: "
            f"rpy='{rpy_str}' does not align local Z [0,0,1] with joint axis {joint_axis}. "
            f"Dot product = {dot:.6f}, expected > {1.0 - tolerance:.6f}"
        )


def test_wheel_collision_cylinder_axis_aligns_with_joint_axis():
    """Wheel collision cylinder geometry is correctly oriented with its joint axis.

    Tests both wheel_left_joint and wheel_right_joint.
    For each wheel, finds the joint's axis from the URDF, then verifies
    the child link's collision cylinder's rpy aligns the local Z-axis with that axis.
    """
    root = load_urdf()
    tolerance = 1e-4

    wheel_specs = [
        ('wheel_left_joint', 'Wheel_Left'),
        ('wheel_right_joint', 'Wheel_Right'),
    ]

    for joint_name, link_name in wheel_specs:
        # Find joint element
        joint_elem = root.find(f".//joint[@name='{joint_name}']")
        assert joint_elem is not None, f"Joint '{joint_name}' not found in URDF"

        # Extract joint axis
        axis_elem = joint_elem.find('axis')
        assert axis_elem is not None, f"Joint '{joint_name}' has no <axis> element"
        axis_xyz_str = axis_elem.get('xyz')
        assert axis_xyz_str, f"Joint '{joint_name}' axis missing 'xyz' attribute"
        joint_axis = np.array([float(x) for x in axis_xyz_str.split()])

        # Find link element
        link_elem = root.find(f".//link[@name='{link_name}']")
        assert link_elem is not None, f"Link '{link_name}' not found in URDF"

        # Find collision geometry (cylinder)
        collision_elem = link_elem.find('collision')
        assert collision_elem is not None, f"Link '{link_name}' has no <collision> element"

        geometry_elem = collision_elem.find('geometry')
        assert geometry_elem is not None, f"Collision in '{link_name}' has no <geometry>"

        cylinder_elem = geometry_elem.find('cylinder')
        assert cylinder_elem is not None, (
            f"Collision geometry in '{link_name}' is not a cylinder"
        )

        # Get origin rpy (default to '0 0 0' if not present)
        origin_elem = collision_elem.find('origin')
        if origin_elem is not None:
            rpy_str = origin_elem.get('rpy', '0 0 0')
        else:
            rpy_str = '0 0 0'

        # Check alignment
        dot = _axis_alignment_dot(rpy_str, joint_axis)
        assert dot > 1.0 - tolerance, (
            f"Joint '{joint_name}' / Link '{link_name}' collision cylinder: "
            f"rpy='{rpy_str}' does not align local Z [0,0,1] with joint axis {joint_axis}. "
            f"Dot product = {dot:.6f}, expected > {1.0 - tolerance:.6f}"
        )
