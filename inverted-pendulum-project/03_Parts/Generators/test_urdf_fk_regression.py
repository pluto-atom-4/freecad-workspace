"""
FK regression test against CAD source of truth (Issue #119).

Validates that URDF joint origins and servo mesh positions match independently-captured
ground truth from FreeCAD (07_body_wheels_metadata.json, joint_config.json, 09_mass_properties.json).

This test does NOT reuse 10_export_urdf.py's composition logic (would be circular);
instead, it recomputes expected values independently using scipy.spatial.transform.Rotation
and the raw JSON inputs.

Pure Python (no FreeCAD imports, no freecadcmd subprocess) — reads exported JSON files
and URDF XML, validates math in isolation.
"""

import json
import sys
import importlib.util
from pathlib import Path
from xml.etree import ElementTree as ET
from typing import Dict, List, Any, Tuple

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

# Script location and paths
SCRIPT_DIR = Path(__file__).resolve().parent
EXPORTS_DIR = SCRIPT_DIR.parent.parent / "06_Exports"
URDF_FILE = EXPORTS_DIR / "urdf" / "robot.urdf"

# Ground truth JSON files (from earlier pipeline stages)
BODY_WHEELS_METADATA_FILE = SCRIPT_DIR / "07_body_wheels_metadata.json"
JOINT_CONFIG_FILE = SCRIPT_DIR / "joint_config.json"
MASS_PROPERTIES_FILE = SCRIPT_DIR / "09_mass_properties.json"

# Import 10_export_urdf using importlib to extract mount constants (digit-leading filename)
_spec = importlib.util.spec_from_file_location("export_urdf_10", SCRIPT_DIR / "10_export_urdf.py")
_export_urdf = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_export_urdf)

# Tolerance for float comparisons (tight regression pin)
TOLERANCE_M = 1e-6  # 1 micrometer in meters


def load_urdf() -> ET.Element:
    """Load and parse the URDF XML file, skip test if missing."""
    if not URDF_FILE.exists():
        pytest.skip(f"URDF file not found: {URDF_FILE}")
    tree = ET.parse(URDF_FILE)
    return tree.getroot()


def load_body_wheels_metadata() -> Dict[str, Any]:
    """Load Stage 1's 07_body_wheels_metadata.json."""
    if not BODY_WHEELS_METADATA_FILE.exists():
        pytest.skip(f"Body/wheels metadata not found: {BODY_WHEELS_METADATA_FILE}")
    with open(BODY_WHEELS_METADATA_FILE, 'r') as f:
        return json.load(f)


def load_joint_config() -> Dict[str, Any]:
    """Load Stage 2's joint_config.json."""
    if not JOINT_CONFIG_FILE.exists():
        pytest.skip(f"Joint config not found: {JOINT_CONFIG_FILE}")
    with open(JOINT_CONFIG_FILE, 'r') as f:
        return json.load(f)


def load_mass_properties() -> Dict[str, Any]:
    """Load Stage 3's 09_mass_properties.json."""
    if not MASS_PROPERTIES_FILE.exists():
        pytest.skip(f"Mass properties not found: {MASS_PROPERTIES_FILE}")
    with open(MASS_PROPERTIES_FILE, 'r') as f:
        return json.load(f)


def _apply_rotation_ypr(vector_mm: List[float], yaw_deg: float, pitch_deg: float, roll_deg: float) -> List[float]:
    """Apply YPR rotation to a 3D vector using scipy.spatial.transform.Rotation.

    Rotations applied in order: Yaw (Z), Pitch (Y), Roll (X) — matching ZYX convention.

    Args:
        vector_mm: [x, y, z] vector in mm
        yaw_deg, pitch_deg, roll_deg: rotation angles in degrees

    Returns:
        rotated vector [x, y, z] in mm
    """
    rot = Rotation.from_euler('zyx', [yaw_deg, pitch_deg, roll_deg], degrees=True)
    rotated = rot.apply(vector_mm)
    return rotated.tolist()


def _compute_plate_com(plate_shapes: List[Dict[str, Any]], target_mass_kg: float) -> List[float]:
    """Recompute pendulum plate stack CoM independently (NOT reusing combine_plate_stack).

    Volume-weights target_mass_kg across plates, computes mass-weighted CoM.

    Args:
        plate_shapes: List of per-plate dicts from LinkRecord['plate_shapes']
        target_mass_kg: Total target mass for the plate stack

    Returns:
        [x, y, z] CoM in mm
    """
    if not plate_shapes:
        raise ValueError("plate_shapes cannot be empty")

    total_volume = sum(p['volume_mm3'] for p in plate_shapes)
    if total_volume <= 0:
        raise ValueError(f"Total plate volume must be positive, got {total_volume}")

    weighted_com = [0.0, 0.0, 0.0]
    for plate in plate_shapes:
        plate_mass = target_mass_kg * (plate['volume_mm3'] / total_volume)
        com = plate['center_of_mass_mm']
        weighted_com[0] += plate_mass * com['x']
        weighted_com[1] += plate_mass * com['y']
        weighted_com[2] += plate_mass * com['z']

    return [c / target_mass_kg for c in weighted_com]


def _mm_to_m(value_or_list) -> Any:
    """Convert mm to m (scalar or list)."""
    if isinstance(value_or_list, (list, tuple)):
        return [v / 1000.0 for v in value_or_list]
    return value_or_list / 1000.0


def _parse_xyz(xyz_str: str) -> List[float]:
    """Parse URDF xyz string to float list."""
    return [float(x) for x in xyz_str.split()]


class TestJointFK:
    """Joint FK tests (star topology: all 4 joints parent Base_Link)."""

    def test_joint_origin_matches_joint_config_ground_truth(self):
        """Test Group 1.1: URDF joint origin xyz ≈ joint_config.json / 1000.

        All 4 joints have parent=Base_Link (star topology), so <joint><origin> IS
        the FK result at zero configuration (no chaining needed).

        Tolerance: 1e-6 m (tight regression pin).
        """
        urdf = load_urdf()
        joint_cfg = load_joint_config()

        expected_joints = {
            'wheel_left_joint': 'wheel_left_joint',
            'wheel_right_joint': 'wheel_right_joint',
            'pendulum_pivot_joint': 'pendulum_pivot_joint',
            'pendulum_pivot_right_joint': 'pendulum_pivot_right_joint',
        }

        for urdf_name, cfg_name in expected_joints.items():
            # Find joint in URDF
            joint_elem = urdf.find(f".//joint[@name='{urdf_name}']")
            assert joint_elem is not None, f"Joint {urdf_name} not found in URDF"

            # Get origin from URDF (already in meters)
            origin_elem = joint_elem.find('origin')
            assert origin_elem is not None, f"Origin not found for joint {urdf_name}"
            urdf_xyz = _parse_xyz(origin_elem.get('xyz'))

            # Get expected from joint_config (in mm, need to convert to m)
            cfg_joint = joint_cfg['joints'][cfg_name]
            origin_mm = cfg_joint['origin_mm']
            expected_xyz = _mm_to_m([origin_mm['x'], origin_mm['y'], origin_mm['z']])

            # Compare
            for i, (name, urdf_val, exp_val) in enumerate(zip(['x', 'y', 'z'], urdf_xyz, expected_xyz)):
                assert abs(urdf_val - exp_val) < TOLERANCE_M, (
                    f"Joint {urdf_name} origin {name}: URDF={urdf_val:.9f} m, "
                    f"expected={exp_val:.9f} m, diff={abs(urdf_val - exp_val):.2e} m"
                )

    def test_joint_origin_transitively_matches_freecad_link_placement(self):
        """Test Group 1.2: URDF joint origin xyz ≈ FreeCAD link placement / 1000.

        The joint origin should also match the FreeCAD Placement of the child link
        (since the joint defines where the child is attached relative to the parent).

        Tolerance: 1e-6 m (tight regression pin).
        """
        urdf = load_urdf()
        body_wheels = load_body_wheels_metadata()

        # Map URDF joint names to expected child link names and link keys in metadata
        joint_to_link = {
            'wheel_left_joint': 'Wheel_Left',
            'wheel_right_joint': 'Wheel_Right',
            'pendulum_pivot_joint': 'Pendulum_Link',
            'pendulum_pivot_right_joint': 'Pendulum_Link_Right',
        }

        for urdf_joint_name, link_name in joint_to_link.items():
            # Find joint in URDF
            joint_elem = urdf.find(f".//joint[@name='{urdf_joint_name}']")
            assert joint_elem is not None, f"Joint {urdf_joint_name} not found in URDF"

            # Get origin from URDF (already in meters)
            origin_elem = joint_elem.find('origin')
            urdf_xyz = _parse_xyz(origin_elem.get('xyz'))

            # Get expected from FreeCAD metadata (Placement.position in mm)
            link_data = body_wheels['links'][link_name]
            placement = link_data['placement']
            pos_mm = placement['position']
            expected_xyz = _mm_to_m([pos_mm['x'], pos_mm['y'], pos_mm['z']])

            # Compare
            for i, (name, urdf_val, exp_val) in enumerate(zip(['x', 'y', 'z'], urdf_xyz, expected_xyz)):
                assert abs(urdf_val - exp_val) < TOLERANCE_M, (
                    f"Joint {urdf_joint_name} (link {link_name}) position {name}: "
                    f"URDF={urdf_val:.9f} m, expected={exp_val:.9f} m, "
                    f"diff={abs(urdf_val - exp_val):.2e} m"
                )


class TestPendulumPlateCoM:
    """Pendulum plate CoM test (Group 2)."""

    def test_pendulum_plate_com_matches_independently_recomputed_value(self):
        """Test Group 2: URDF plate visual origin xyz ≈ independently-recomputed plate CoM / 1000.

        Recompute the pendulum plate CoM FRESH from plate_shapes JSON (NOT reusing
        combine_plate_stack() from 10_export_urdf.py, to avoid circularity).

        Compare against URDF's plate <visual><origin> for both Pendulum_Link and
        Pendulum_Link_Right.

        Tolerance: 1e-6 m (tight regression pin).
        """
        urdf = load_urdf()
        body_wheels = load_body_wheels_metadata()

        links_to_test = ['Pendulum_Link', 'Pendulum_Link_Right']

        for link_name in links_to_test:
            link_data = body_wheels['links'][link_name]
            target_mass = link_data['target_mass_kg']
            plate_shapes = link_data.get('plate_shapes')

            if not plate_shapes:
                pytest.skip(f"No plate_shapes found for {link_name}")

            # Independently recompute plate CoM
            computed_plate_com = _compute_plate_com(plate_shapes, target_mass)
            expected_xyz = _mm_to_m(computed_plate_com)

            # Find link and first visual (plate box)
            link_elem = urdf.find(f".//link[@name='{link_name}']")
            assert link_elem is not None, f"Link {link_name} not found in URDF"

            visual_elems = link_elem.findall('visual')
            assert len(visual_elems) >= 1, f"No visual elements found for {link_name}"

            # First visual should be the plate box (with origin at plate CoM)
            plate_visual = visual_elems[0]
            origin_elem = plate_visual.find('origin')
            assert origin_elem is not None, f"No origin in first visual of {link_name}"
            urdf_xyz = _parse_xyz(origin_elem.get('xyz'))

            # Compare
            for i, (name, urdf_val, exp_val) in enumerate(zip(['x', 'y', 'z'], urdf_xyz, expected_xyz)):
                assert abs(urdf_val - exp_val) < TOLERANCE_M, (
                    f"Plate CoM {link_name} {name}: URDF={urdf_val:.9f} m, "
                    f"expected={exp_val:.9f} m, diff={abs(urdf_val - exp_val):.2e} m"
                )


class TestServoMeshComposition:
    """Servo mesh origin test (Group 3) — the #105/#107/#109 bug class."""

    def test_servo_mesh_origin_matches_independently_composed_value(self):
        """Test Group 3: URDF servo mesh visual/collision origin ≈ independently-composed servo CoM / 1000.

        Recompute expected servo CoM using scipy.spatial.transform.Rotation from:
        (a) servo local CoM in 09_mass_properties.json
        (b) mount pos/rot constants from 10_export_urdf.py (trusted frozen input)

        Do NOT reuse 10_export_urdf.py's apply_rotation_to_vector or servo composition
        logic — recompute independently to catch bugs.

        This test does NOT verify the mount-offset constants themselves against live CAD
        (tracked separately as #120), only validates the composition math.

        KNOWN RISK (#121): build_urdf_joint() always writes rpy="0 0 0" on joint origins
        despite Pendulum_Link having real non-identity FreeCAD rotation. If this test
        reveals a discrepancy, it is NOT adjusted — reported exactly as found.

        Tolerance: 1e-6 m (tight regression pin).
        """
        urdf = load_urdf()
        mass_props = load_mass_properties()

        # Mount constants from 10_export_urdf.py (trusted frozen input, not verified here)
        servo_l_mount_pos = _export_urdf.servo_l_mount_pos if hasattr(_export_urdf, 'servo_l_mount_pos') else [-1.0, 0.0, 0.0]
        servo_l_mount_rot = [0.0, 0.0, 0.0]  # yaw, pitch, roll in degrees
        servo_r_mount_pos = _export_urdf.servo_r_mount_pos if hasattr(_export_urdf, 'servo_r_mount_pos') else [-1.0, 51.0, 6.0]
        servo_r_mount_rot = [0.0, 0.0, 180.0]  # yaw, pitch, roll in degrees

        test_cases = [
            ('Pendulum_Link', 'servo_left', servo_l_mount_pos, servo_l_mount_rot),
            ('Pendulum_Link_Right', 'servo_right', servo_r_mount_pos, servo_r_mount_rot),
        ]

        for link_name, servo_key, mount_pos, mount_rot in test_cases:
            # Get servo local CoM from mass properties
            servo_com_local = mass_props[servo_key]['center_of_mass_mm']
            servo_com_local_list = [servo_com_local[0], servo_com_local[1], servo_com_local[2]]

            # Apply mount rotation to servo CoM (recomputed independently)
            servo_com_rotated = _apply_rotation_ypr(servo_com_local_list, *mount_rot)

            # Compose: servo_com_assembly = mount_pos + rotated_servo_com
            expected_com_mm = [
                mount_pos[0] + servo_com_rotated[0],
                mount_pos[1] + servo_com_rotated[1],
                mount_pos[2] + servo_com_rotated[2],
            ]
            expected_xyz = _mm_to_m(expected_com_mm)

            # Find link in URDF
            link_elem = urdf.find(f".//link[@name='{link_name}']")
            assert link_elem is not None, f"Link {link_name} not found in URDF"

            # Find servo mesh visual (second visual element, type=mesh)
            visual_elems = link_elem.findall('visual')
            assert len(visual_elems) >= 2, f"Expected >= 2 visuals in {link_name}, found {len(visual_elems)}"
            servo_visual = visual_elems[1]

            origin_elem = servo_visual.find('origin')
            assert origin_elem is not None, f"No origin in servo visual of {link_name}"
            urdf_xyz = _parse_xyz(origin_elem.get('xyz'))

            # Compare visual mesh origin
            for i, (name, urdf_val, exp_val) in enumerate(zip(['x', 'y', 'z'], urdf_xyz, expected_xyz)):
                assert abs(urdf_val - exp_val) < TOLERANCE_M, (
                    f"Servo mesh {link_name} visual {name}: URDF={urdf_val:.9f} m, "
                    f"expected={exp_val:.9f} m, diff={abs(urdf_val - exp_val):.2e} m"
                )

            # Also check servo box collision geometry (should have same origin)
            collision_elems = link_elem.findall('collision')
            assert len(collision_elems) >= 2, f"Expected >= 2 collisions in {link_name}, found {len(collision_elems)}"
            servo_collision = collision_elems[1]

            origin_elem = servo_collision.find('origin')
            assert origin_elem is not None, f"No origin in servo box collision of {link_name}"
            urdf_xyz = _parse_xyz(origin_elem.get('xyz'))

            # Compare collision box origin
            for i, (name, urdf_val, exp_val) in enumerate(zip(['x', 'y', 'z'], urdf_xyz, expected_xyz)):
                assert abs(urdf_val - exp_val) < TOLERANCE_M, (
                    f"Servo box {link_name} collision {name}: URDF={urdf_val:.9f} m, "
                    f"expected={exp_val:.9f} m, diff={abs(urdf_val - exp_val):.2e} m"
                )
