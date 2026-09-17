#!/usr/bin/env python3
"""
Unit tests for axis_validation module (Issue #163).

Tests:
  - compose_axis(): Given joint_config entry + RPY, compute expected axis
  - validate_joint_axes(): Validate URDF joint axes (norm + direction)
  - Edge cases: empty URDF, singular URDFs, malformed data
"""

import sys
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Any, List

# Add script dir to path
try:
    SCRIPT_DIR = Path(__file__).resolve().parent
except NameError:
    SCRIPT_DIR = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from axis_validation import compose_axis, validate_joint_axes


def test_compose_axis_identity():
    """Test compose_axis with identity rotation (no rotation)."""
    axis_global = [0, 1, 0]
    rotation_ypr_deg = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}

    result = compose_axis(axis_global, rotation_ypr_deg)
    assert result is not None, "compose_axis should not return None for valid input"

    # With no rotation, axis_local should equal axis_global (normalized)
    expected = [0, 1, 0]
    for i in range(3):
        assert abs(result[i] - expected[i]) < 1e-6, \
            f"Expected {expected}, got {result}"
    print("✓ test_compose_axis_identity passed")


def test_compose_axis_90deg_z_rotation():
    """Test compose_axis with 90° rotation around Z (yaw)."""
    # Global Y axis: [0, 1, 0]
    # After 90° Z rotation (yaw), local should be close to X axis: [1, 0, 0]
    # (actually [-1, 0, 0] depending on convention)
    axis_global = [0, 1, 0]
    rotation_ypr_deg = {'yaw': 90.0, 'pitch': 0.0, 'roll': 0.0}

    result = compose_axis(axis_global, rotation_ypr_deg)
    assert result is not None, "compose_axis should not return None"

    # After 90° Z rotation, Y axis should map to -X or X (depending on matrix direction)
    # The magnitude of the X component should be close to 1
    assert abs(abs(result[0]) - 1.0) < 1e-2, \
        f"Expected X component near ±1, got {result}"
    assert abs(result[1]) < 1e-2, \
        f"Expected Y component near 0, got {result}"
    print("✓ test_compose_axis_90deg_z_rotation passed")


def test_compose_axis_norm_check():
    """Test that compose_axis returns unit vectors."""
    test_cases = [
        ({'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}, [0, 1, 0]),
        ({'yaw': 45.0, 'pitch': 0.0, 'roll': 0.0}, [0, 1, 0]),
        ({'yaw': 0.0, 'pitch': 45.0, 'roll': 0.0}, [1, 0, 0]),
        ({'yaw': 0.0, 'pitch': 0.0, 'roll': 45.0}, [0, 1, 0]),
    ]

    for rotation_ypr_deg, axis_global in test_cases:
        result = compose_axis(axis_global, rotation_ypr_deg)
        if result is not None:
            norm = math.sqrt(sum(x*x for x in result))
            assert abs(norm - 1.0) < 1e-6, \
                f"Expected unit vector, got norm={norm} for rotation={rotation_ypr_deg}"

    print("✓ test_compose_axis_norm_check passed")


def test_validate_joint_axes_empty_urdf():
    """Test validate_joint_axes with a URDF that has no joints."""
    urdf_xml = """<?xml version="1.0"?>
    <robot name="test">
        <link name="base_link">
            <inertial>
                <mass value="1.0"/>
            </inertial>
        </link>
    </robot>"""

    urdf_root = ET.fromstring(urdf_xml)
    result = validate_joint_axes(urdf_root, None)

    assert result['passed'] is True, "Empty URDF should pass vacuously"
    assert len(result['errors']) == 0, "No errors for empty URDF"
    print("✓ test_validate_joint_axes_empty_urdf passed")


def test_validate_joint_axes_correct_axes():
    """Test validate_joint_axes with correct unit vector axes."""
    urdf_xml = """<?xml version="1.0"?>
    <robot name="test">
        <link name="base_link"/>
        <link name="child_link"/>
        <joint name="test_joint" type="revolute">
            <parent link="base_link"/>
            <child link="child_link"/>
            <axis xyz="0 1 0"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <limit lower="0" upper="1.57" effort="10" velocity="1"/>
        </joint>
    </robot>"""

    urdf_root = ET.fromstring(urdf_xml)
    result = validate_joint_axes(urdf_root, None)

    assert result['passed'] is True, "Valid unit vector axes should pass"
    assert len(result['errors']) == 0, "No errors for valid axes"
    print("✓ test_validate_joint_axes_correct_axes passed")


def test_validate_joint_axes_bad_norm():
    """Test validate_joint_axes detects non-unit vector axes."""
    urdf_xml = """<?xml version="1.0"?>
    <robot name="test">
        <link name="base_link"/>
        <link name="child_link"/>
        <joint name="test_joint" type="revolute">
            <parent link="base_link"/>
            <child link="child_link"/>
            <axis xyz="0 2 0"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <limit lower="0" upper="1.57" effort="10" velocity="1"/>
        </joint>
    </robot>"""

    urdf_root = ET.fromstring(urdf_xml)
    result = validate_joint_axes(urdf_root, None)

    assert result['passed'] is False, "Non-unit vector should fail"
    assert len(result['errors']) > 0, "Should have errors"
    assert 'norm' in result['errors'][0].get('check_type', ''), \
        f"Error should be about norm, got: {result['errors'][0]}"
    print("✓ test_validate_joint_axes_bad_norm passed")


def test_validate_joint_axes_with_direction_check():
    """Test validate_joint_axes with joint_config direction validation."""
    # Create a simple joint_config
    joint_config = {
        'joints': {
            'test_joint': {
                'axis_global': [0, 1, 0],
                'rotation_ypr_deg': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}
            }
        }
    }

    urdf_xml = """<?xml version="1.0"?>
    <robot name="test">
        <link name="base_link"/>
        <link name="child_link"/>
        <joint name="test_joint" type="revolute">
            <parent link="base_link"/>
            <child link="child_link"/>
            <axis xyz="0 1 0"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <limit lower="0" upper="1.57" effort="10" velocity="1"/>
        </joint>
    </robot>"""

    urdf_root = ET.fromstring(urdf_xml)
    result = validate_joint_axes(urdf_root, joint_config)

    assert result['passed'] is True, "Matching axes should pass direction check"
    assert len(result['errors']) == 0, "No errors for matching direction"
    print("✓ test_validate_joint_axes_with_direction_check passed")


def test_validate_joint_axes_direction_mismatch():
    """Test validate_joint_axes detects direction mismatches."""
    # Create a joint_config that expects Y axis
    joint_config = {
        'joints': {
            'test_joint': {
                'axis_global': [0, 1, 0],
                'rotation_ypr_deg': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}
            }
        }
    }

    # But URDF has Z axis (wrong direction)
    urdf_xml = """<?xml version="1.0"?>
    <robot name="test">
        <link name="base_link"/>
        <link name="child_link"/>
        <joint name="test_joint" type="revolute">
            <parent link="base_link"/>
            <child link="child_link"/>
            <axis xyz="0 0 1"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <limit lower="0" upper="1.57" effort="10" velocity="1"/>
        </joint>
    </robot>"""

    urdf_root = ET.fromstring(urdf_xml)
    result = validate_joint_axes(urdf_root, joint_config)

    assert result['passed'] is False, "Mismatched axes should fail direction check"
    assert len(result['errors']) > 0, "Should have errors"
    error = result['errors'][0]
    assert error.get('check_type') == 'direction', \
        f"Error should be about direction, got: {error}"
    print("✓ test_validate_joint_axes_direction_mismatch passed")


def test_validate_joint_axes_multiple_joints():
    """Test validate_joint_axes with multiple joints."""
    urdf_xml = """<?xml version="1.0"?>
    <robot name="test">
        <link name="base_link"/>
        <link name="link1"/>
        <link name="link2"/>
        <joint name="joint1" type="revolute">
            <parent link="base_link"/>
            <child link="link1"/>
            <axis xyz="0 1 0"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <limit lower="0" upper="1.57" effort="10" velocity="1"/>
        </joint>
        <joint name="joint2" type="revolute">
            <parent link="link1"/>
            <child link="link2"/>
            <axis xyz="1 0 0"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <limit lower="0" upper="1.57" effort="10" velocity="1"/>
        </joint>
    </robot>"""

    urdf_root = ET.fromstring(urdf_xml)
    result = validate_joint_axes(urdf_root, None)

    assert result['passed'] is True, "All valid axes should pass"
    assert len(result['errors']) == 0, "No errors for valid axes"
    print("✓ test_validate_joint_axes_multiple_joints passed")


def main():
    """Run all tests."""
    tests = [
        test_compose_axis_identity,
        test_compose_axis_90deg_z_rotation,
        test_compose_axis_norm_check,
        test_validate_joint_axes_empty_urdf,
        test_validate_joint_axes_correct_axes,
        test_validate_joint_axes_bad_norm,
        test_validate_joint_axes_with_direction_check,
        test_validate_joint_axes_direction_mismatch,
        test_validate_joint_axes_multiple_joints,
    ]

    print("\n" + "=" * 70)
    print("Running axis_validation tests")
    print("=" * 70 + "\n")

    passed = 0
    failed = 0

    for test in tests:
        try:
            test()
            passed += 1
        except AssertionError as e:
            print(f"✗ {test.__name__} failed: {e}")
            failed += 1
        except Exception as e:
            print(f"✗ {test.__name__} error: {e}")
            import traceback
            traceback.print_exc()
            failed += 1

    print(f"\n" + "=" * 70)
    print(f"Results: {passed} passed, {failed} failed")
    print("=" * 70 + "\n")

    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
