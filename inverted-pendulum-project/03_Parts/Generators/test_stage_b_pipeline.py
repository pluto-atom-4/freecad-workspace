#!/usr/bin/env python3
"""
Stage B pipeline E2E test: URDF export → rewrite → structure/axis validate.

Skips FreeCAD-dependent Phases 7-9. Gated PROTO validate if artifacts present.

Validates the complete Stage B pipeline:
  - URDF export (Phase 4 output)
  - Phase 12 validation (mesh paths, unit consistency, link connectivity, axis validation)
  - URDF rewrite for Webots (mesh path transformation)
  - Structure validation (link/joint counts)
  - Axis validation (unit vectors, direction)
  - (Optional) PROTO validation (if artifacts present)

Test execution:
    cd inverted-pendulum-project
    mamba run -n pendulum-tools python3 -m pytest test_stage_b_pipeline.py -v

Hardcoded values (locked):
  - Expected: 5 links, 4 joints
  - Pendulum pivot axes: [0, 0, -1] (standard orientation)
"""

import sys
import os
import json
import tempfile
from pathlib import Path
from xml.etree import ElementTree as ET
from typing import Dict, List, Any, Optional
import importlib.util

import pytest

# Script location
SCRIPT_DIR = Path(__file__).resolve().parent
EXPORTS_DIR = SCRIPT_DIR.parent.parent / "06_Exports"
SIMULATION_DIR = SCRIPT_DIR.parent.parent / "07_Simulation"
URDF_FILE = EXPORTS_DIR / "urdf" / "robot.urdf"
JOINT_CONFIG_FILE = SCRIPT_DIR / "joint_config.json"

# Add script dir to path for imports
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

# Import modules
from urdf_mesh_path_resolver import resolve_mesh_path, rewrite_for_webots, rewrite_urdf_file
from urdf_structure_checker import check_urdf_structure
from axis_validation import validate_joint_axes
import importlib.util


# ============================================================================
# FIXTURES
# ============================================================================

@pytest.fixture(scope="module")
def urdf_root() -> ET.Element:
    """Load and parse the URDF XML file, skip test if missing."""
    if not URDF_FILE.exists():
        pytest.skip(f"URDF file not found: {URDF_FILE}")
    tree = ET.parse(URDF_FILE)
    return tree.getroot()


@pytest.fixture(scope="module")
def joint_config() -> Optional[Dict[str, Any]]:
    """Load joint_config.json if available."""
    if not JOINT_CONFIG_FILE.exists():
        return None
    try:
        with open(JOINT_CONFIG_FILE, 'r') as f:
            return json.load(f)
    except (json.JSONDecodeError, IOError):
        return None


def load_urdf() -> ET.Element:
    """Directly load URDF for inline use."""
    if not URDF_FILE.exists():
        pytest.skip(f"URDF file not found: {URDF_FILE}")
    tree = ET.parse(URDF_FILE)
    return tree.getroot()


def load_joint_config() -> Optional[Dict[str, Any]]:
    """Directly load joint_config for inline use."""
    if not JOINT_CONFIG_FILE.exists():
        return None
    try:
        with open(JOINT_CONFIG_FILE, 'r') as f:
            return json.load(f)
    except (json.JSONDecodeError, IOError):
        return None


# ============================================================================
# TEST FUNCTIONS
# ============================================================================

class TestStageBPipeline:
    """Stage B pipeline end-to-end tests."""

    def test_stage_b_urdf_exists(self):
        """Test: URDF file exists and is valid XML."""
        if not URDF_FILE.exists():
            pytest.skip(f"URDF file not found: {URDF_FILE}")

        # Load and parse URDF
        urdf_root = load_urdf()

        # Assert basic validity
        assert urdf_root.tag == 'robot', f"Expected <robot> root element, got <{urdf_root.tag}>"

    def test_stage_b_phase_12_validation(self):
        """Test: Phase 12 validation functions pass.

        Imports and calls validators from 12_validate_urdf_export.py directly.
        """
        if not URDF_FILE.exists():
            pytest.skip(f"URDF file not found: {URDF_FILE}")

        # Import Phase 12 validators
        spec = importlib.util.spec_from_file_location(
            "validate_urdf_export_12",
            SCRIPT_DIR / "12_validate_urdf_export.py"
        )
        phase12 = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(phase12)

        # Load URDF
        urdf_root = load_urdf()

        # Test 1: Mesh paths validation
        mesh_validation = phase12.validate_mesh_paths(urdf_root, URDF_FILE)
        assert isinstance(mesh_validation, list), "validate_mesh_paths should return list"
        assert len(mesh_validation) > 0, "Should have at least one mesh validation result"
        for result in mesh_validation:
            assert result.get('passed') is True, f"Mesh validation failed: {result}"

        # Test 2: Unit consistency validation
        unit_validation = phase12.validate_unit_consistency(urdf_root)
        assert isinstance(unit_validation, list), "validate_unit_consistency should return list"
        assert len(unit_validation) > 0, "Should have at least one unit validation result"
        for result in unit_validation:
            assert result.get('passed') is True, f"Unit validation failed: {result}"

        # Test 3: Link connectivity validation
        connectivity_validation = phase12.validate_link_connectivity(urdf_root)
        assert isinstance(connectivity_validation, list), "validate_link_connectivity should return list"
        assert len(connectivity_validation) > 0, "Should have at least one connectivity result"
        for result in connectivity_validation:
            assert result.get('passed') is True, f"Connectivity validation failed: {result}"

    def test_stage_b_mesh_inodes_before_rewrite(self):
        """Test: Capture mesh file inode numbers before rewrite.

        Stores inode snapshot in fixture for comparison after rewrite.
        Verifies mesh files exist.
        """
        if not URDF_FILE.exists():
            pytest.skip(f"URDF file not found: {URDF_FILE}")

        urdf_root = load_urdf()
        urdf_dir = URDF_FILE.parent
        mesh_inodes = {}

        # Find all mesh elements
        for mesh_elem in urdf_root.findall('.//mesh'):
            filename = mesh_elem.get('filename', '')
            if not filename:
                continue

            try:
                mesh_path = resolve_mesh_path(filename, urdf_dir)
                assert mesh_path.exists(), f"Mesh file does not exist: {mesh_path}"

                # Capture inode
                stat = os.stat(mesh_path)
                mesh_inodes[str(mesh_path)] = stat.st_ino
            except Exception as e:
                pytest.fail(f"Failed to stat mesh file {filename}: {e}")

        # Verify we found at least one mesh
        assert len(mesh_inodes) > 0, "Expected at least one mesh file in URDF"

        # Store for later comparison (pytest fixture state via class attribute)
        TestStageBPipeline._mesh_inodes_before = mesh_inodes

    def test_stage_b_urdf_rewrite_to_webots(self):
        """Test: Rewrite URDF for Webots (mesh paths).

        Creates temp directory, rewrites URDF with relative paths,
        verifies no package:// URIs remain.
        """
        if not URDF_FILE.exists():
            pytest.skip(f"URDF file not found: {URDF_FILE}")

        urdf_root = load_urdf()
        urdf_dir = URDF_FILE.parent

        # Create temp directory for rewritten URDF
        with tempfile.TemporaryDirectory() as tmpdir:
            tmpdir = Path(tmpdir)
            webots_gen_dir = tmpdir / "webots_generated"
            webots_gen_dir.mkdir(parents=True, exist_ok=True)

            output_urdf = webots_gen_dir / "robot_webots.urdf"

            # Rewrite URDF
            rewrite_urdf_file(URDF_FILE, output_urdf, urdf_dir, webots_gen_dir)

            # Verify output exists and is non-empty
            assert output_urdf.exists(), f"Rewritten URDF not created: {output_urdf}"
            assert output_urdf.stat().st_size > 0, f"Rewritten URDF is empty: {output_urdf}"

            # Parse rewritten URDF
            rewritten_tree = ET.parse(output_urdf)
            rewritten_root = rewritten_tree.getroot()

            # Verify no package:// URIs remain
            rewritten_content = output_urdf.read_text()
            assert 'package://' not in rewritten_content, \
                "Rewritten URDF should not contain package:// URIs"

            # Store rewritten URDF path for next test (via class attribute)
            TestStageBPipeline._rewritten_urdf_path = output_urdf
            TestStageBPipeline._rewritten_urdf_root = rewritten_root
            TestStageBPipeline._rewritten_urdf_dir = webots_gen_dir

    def test_stage_b_mesh_inodes_after_rewrite(self):
        """Test: Verify mesh inodes unchanged after rewrite.

        Compares inode snapshot from before rewrite.
        Ensures rewrite only touched URDF text, never mesh files.
        """
        if not URDF_FILE.exists():
            pytest.skip(f"URDF file not found: {URDF_FILE}")

        if not hasattr(TestStageBPipeline, '_mesh_inodes_before'):
            pytest.skip("test_stage_b_mesh_inodes_before_rewrite must run first")

        mesh_inodes_before = TestStageBPipeline._mesh_inodes_before
        urdf_dir = URDF_FILE.parent

        # Re-capture inodes
        mesh_inodes_after = {}
        for mesh_path_str in mesh_inodes_before.keys():
            mesh_path = Path(mesh_path_str)
            assert mesh_path.exists(), f"Mesh file disappeared: {mesh_path}"

            stat = os.stat(mesh_path)
            mesh_inodes_after[mesh_path_str] = stat.st_ino

        # Compare: all inodes should match (no file replacement)
        for mesh_path_str, inode_before in mesh_inodes_before.items():
            inode_after = mesh_inodes_after.get(mesh_path_str)
            assert inode_after is not None, f"Mesh file missing after rewrite: {mesh_path_str}"
            assert inode_before == inode_after, \
                f"Mesh file inode changed (file replaced?): {mesh_path_str} " \
                f"before={inode_before}, after={inode_after}"

    def test_stage_b_structure_validation(self):
        """Test: URDF structure validation (link/joint counts).

        Hardcoded expectations:
          - 5 links (Base_Link, Wheel_Left, Wheel_Right, Pendulum_Link, Pendulum_Link_Right)
          - 4 joints (wheel_left, wheel_right, pendulum_pivot, pendulum_pivot_right)
        """
        if not URDF_FILE.exists():
            pytest.skip(f"URDF file not found: {URDF_FILE}")

        # Expected hardcoded counts
        EXPECTED_LINKS = 5
        EXPECTED_JOINTS = 4

        # Use the structure checker function
        result = check_urdf_structure(URDF_FILE, EXPECTED_LINKS, EXPECTED_JOINTS)
        assert result is True, \
            f"URDF structure validation failed. Expected {EXPECTED_LINKS} links, " \
            f"{EXPECTED_JOINTS} joints"

    def test_stage_b_axis_validation(self):
        """Test: Joint axes validation (unit vectors, direction).

        Validates all joint axes:
          - Norm is approximately 1.0 (unit vector check)
          - Direction matches expected (if joint_config available)
          - Pendulum pivot axes are [0, 0, -1] (hardcoded check)
        """
        if not URDF_FILE.exists():
            pytest.skip(f"URDF file not found: {URDF_FILE}")

        urdf_root = load_urdf()
        joint_config = load_joint_config()

        # Call axis validation
        result = validate_joint_axes(urdf_root, joint_config, tolerance=1e-3)

        assert result['passed'] is True, \
            f"Axis validation failed: {result.get('errors', [])}"

        # Additional hardcoded check: pendulum pivot axes should be [0, 0, -1]
        for joint in urdf_root.findall('joint'):
            joint_name = joint.get('name', '')
            if 'pendulum_pivot' in joint_name:
                axis_elem = joint.find('axis')
                if axis_elem is not None:
                    xyz_str = axis_elem.get('xyz', '')
                    axis = [float(x) for x in xyz_str.split()]
                    expected = [0, 0, -1]
                    # Check each component with tolerance
                    for i, (a, e) in enumerate(zip(axis, expected)):
                        assert abs(a - e) < 1e-3, \
                            f"Joint {joint_name} axis component {i}: " \
                            f"expected {e}, got {a}"

    def test_stage_b_proto_validation(self):
        """Test: PROTO artifact validation (gated).

        Skips if PROTO artifacts not present (run generate_proto.sh first).
        If present, validates:
          - PROTO file exists and is readable
          - robot_webots.urdf exists
          - PROTO structure (node counts)
          - Link/joint counts match URDF
        """
        proto_file = SIMULATION_DIR / "webots" / "protos" / "InvertedPendulumRobot.proto"
        webots_urdf = SIMULATION_DIR / "webots" / ".generated" / "robot_webots.urdf"

        # Gated skip if artifacts missing
        if not proto_file.exists() or not webots_urdf.exists():
            pytest.skip(
                f"PROTO artifacts not found. "
                f"Run 'generate_proto.sh' first. "
                f"Missing: {proto_file if not proto_file.exists() else ''} "
                f"{webots_urdf if not webots_urdf.exists() else ''}"
            )

        # Validate PROTO file is readable
        assert proto_file.stat().st_size > 0, f"PROTO file is empty: {proto_file}"
        proto_content = proto_file.read_text()
        assert len(proto_content) > 0, "Could not read PROTO content"

        # Validate robot_webots.urdf exists and is readable
        assert webots_urdf.exists(), f"robot_webots.urdf not found: {webots_urdf}"
        webots_tree = ET.parse(webots_urdf)
        webots_root = webots_tree.getroot()

        # Validate link/joint counts in robot_webots.urdf match expected (same as source)
        EXPECTED_LINKS = 5
        EXPECTED_JOINTS = 4

        webots_links = webots_root.findall('.//link')
        webots_joints = webots_root.findall('.//joint')

        assert len(webots_links) == EXPECTED_LINKS, \
            f"robot_webots.urdf: expected {EXPECTED_LINKS} links, found {len(webots_links)}"
        assert len(webots_joints) == EXPECTED_JOINTS, \
            f"robot_webots.urdf: expected {EXPECTED_JOINTS} joints, found {len(webots_joints)}"

        # Validate all mesh references resolve relative to .generated dir
        webots_dir = webots_urdf.parent
        for mesh_elem in webots_root.findall('.//mesh'):
            mesh_filename = mesh_elem.get('filename', '')
            if mesh_filename:
                mesh_path = (webots_dir / mesh_filename).resolve()
                assert mesh_path.exists(), \
                    f"Mesh referenced in robot_webots.urdf does not exist: {mesh_path}"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
