"""
Tests for mesh integrity (unit-test level, no Webots needed).

Validates that all meshes referenced in the exported URDF:
- Exist on disk and are non-empty
- Have correct scale attributes (mm→m conversion, 0.001 for each axis)
- Are centered near the local origin (bbox center < 50mm tolerance)

Pure Python (no FreeCAD imports) — reads the exported URDF and mesh files.
Follows the pattern of test_10_export_urdf.py.
"""

import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest

# Add trimesh to import skip at module level (not per-test)
pytest.importorskip("trimesh")
import trimesh

# Locate the URDF and meshes
SCRIPT_DIR = Path(__file__).resolve().parent
EXPORTS_DIR = SCRIPT_DIR.parent.parent / "06_Exports"
URDF_FILE = EXPORTS_DIR / "urdf" / "robot.urdf"
URDF_MESHES_DIR = EXPORTS_DIR / "urdf" / "meshes"


def load_urdf():
    """Load and parse the URDF XML file, skip test if missing."""
    if not URDF_FILE.exists():
        pytest.skip(f"URDF file not found: {URDF_FILE}")
    tree = ET.parse(URDF_FILE)
    return tree.getroot()


def test_mesh_files_referenced_exist_on_disk():
    """Every mesh file referenced in URDF exists on disk and is non-empty.

    Discovers mesh filenames from <mesh filename=...> tags, resolves
    package://<robot_name>/meshes/X URIs to absolute paths under URDF_MESHES_DIR,
    and verifies each file exists and has non-zero size.
    """
    root = load_urdf()
    robot_name = root.get('name')
    assert robot_name, "URDF root element missing 'name' attribute"

    # Find all <mesh> tags with filename attribute
    mesh_elems = root.findall(".//mesh[@filename]")
    assert len(mesh_elems) > 0, "No mesh elements with filename attribute found in URDF"

    # Collect and resolve mesh filenames
    mesh_files = []
    for mesh_elem in mesh_elems:
        filename = mesh_elem.get('filename')
        assert filename, f"Mesh element has empty filename attribute"

        # Resolve package:// URI
        prefix = f"package://{robot_name}/meshes/"
        if filename.startswith(prefix):
            # Strip the prefix and resolve relative to URDF_MESHES_DIR
            mesh_path_relative = filename[len(prefix):]
            mesh_path = URDF_MESHES_DIR / mesh_path_relative
        else:
            # Already an absolute path or other format; use as-is
            mesh_path = Path(filename)

        mesh_files.append(mesh_path)

    # Verify all files exist and are non-empty
    for mesh_path in mesh_files:
        assert mesh_path.exists(), (
            f"Mesh file not found: {mesh_path}"
        )
        assert mesh_path.stat().st_size > 0, (
            f"Mesh file is empty: {mesh_path}"
        )


def test_mesh_scale_attribute_present_and_correct():
    """Every <mesh> tag has scale attribute with correct mm→m conversion [0.001, 0.001, 0.001].

    This is a per-tag test (not deduplicated by file), since scale is
    a per-reference attribute, not a per-file property.
    """
    root = load_urdf()

    # Find all <mesh> tags
    mesh_elems = root.findall(".//mesh[@filename]")
    assert len(mesh_elems) > 0, "No mesh elements found in URDF"

    expected_scale = [0.001, 0.001, 0.001]

    for i, mesh_elem in enumerate(mesh_elems):
        filename = mesh_elem.get('filename', '<unknown>')

        # Check scale attribute exists
        scale_attr = mesh_elem.get('scale')
        assert scale_attr is not None, (
            f"Mesh {i} ({filename}) missing 'scale' attribute"
        )

        # Parse scale values
        scale_parts = scale_attr.split()
        assert len(scale_parts) == 3, (
            f"Mesh {i} ({filename}) scale should have 3 values, got {len(scale_parts)}: {scale_attr}"
        )

        scale_values = []
        for j, part in enumerate(scale_parts):
            try:
                scale_values.append(float(part))
            except ValueError:
                pytest.fail(f"Mesh {i} ({filename}) scale[{j}] = '{part}' is not a valid float")

        # Verify each component matches expected [0.001, 0.001, 0.001]
        for j, (actual, expected) in enumerate(zip(scale_values, expected_scale)):
            assert abs(actual - expected) < 1e-9, (
                f"Mesh {i} ({filename}) scale[{j}] = {actual:.6f}, expected {expected:.6f}"
            )


def test_mesh_bbox_centered_near_local_origin():
    """Mesh files have bounding box center < 50mm from origin (flat tolerance, no relative check).

    Loads each unique mesh file and checks that the center of its bounding box
    (computed as (min + max) / 2) is within 50mm of the origin in Euclidean distance.

    This is a per-file test (deduplicated by absolute path), since bbox-center
    is a file property, not per-tag.
    """
    root = load_urdf()
    robot_name = root.get('name')
    assert robot_name, "URDF root element missing 'name' attribute"

    # Collect unique mesh file paths
    mesh_elems = root.findall(".//mesh[@filename]")
    assert len(mesh_elems) > 0, "No mesh elements found in URDF"

    unique_mesh_paths = set()
    for mesh_elem in mesh_elems:
        filename = mesh_elem.get('filename')
        prefix = f"package://{robot_name}/meshes/"
        if filename.startswith(prefix):
            mesh_path_relative = filename[len(prefix):]
            mesh_path = URDF_MESHES_DIR / mesh_path_relative
        else:
            mesh_path = Path(filename)

        # Resolve to absolute path and add to set
        unique_mesh_paths.add(mesh_path.resolve())

    assert len(unique_mesh_paths) > 0, "No unique mesh paths found"

    # Check bbox center for each unique file
    bbox_tolerance_mm = 50.0

    for mesh_path in sorted(unique_mesh_paths):
        assert mesh_path.exists(), f"Mesh file not found: {mesh_path}"

        # Load mesh with trimesh
        try:
            mesh = trimesh.load(str(mesh_path))
        except Exception as e:
            pytest.fail(f"Failed to load mesh {mesh_path}: {e}")

        # Get bounding box (bounds is an Nx3 array: [min_point, max_point])
        bounds = mesh.bounds
        if bounds is None or len(bounds) < 2:
            pytest.fail(f"Mesh {mesh_path} has invalid bounds: {bounds}")

        min_point = bounds[0]
        max_point = bounds[1]

        # Compute center
        center = (min_point + max_point) / 2.0

        # Compute Euclidean distance from origin
        distance = np.linalg.norm(center)

        assert distance < bbox_tolerance_mm, (
            f"Mesh {mesh_path}: bbox center {center} is {distance:.2f}mm from origin, "
            f"exceeds tolerance of {bbox_tolerance_mm}mm"
        )
