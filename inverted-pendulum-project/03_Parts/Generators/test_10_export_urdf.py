"""
Tests for Stage 4's URDF export (10_export_urdf.py).

Pure Python (no FreeCAD imports) — reads the generated URDF XML file and validates
its structure. Follows the pattern of test_08_/test_09_*.py.
"""

import json
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest


# Locate the URDF and metadata outputs
SCRIPT_DIR = Path(__file__).resolve().parent
EXPORTS_DIR = SCRIPT_DIR.parent.parent / "06_Exports"
URDF_FILE = EXPORTS_DIR / "urdf" / "robot.urdf"
URDF_MESHES_DIR = EXPORTS_DIR / "urdf" / "meshes"
URDF_VISUAL_MESH = URDF_MESHES_DIR / "feetech-STS3032-visual.stl"
METADATA_FILE = SCRIPT_DIR / "10_urdf_export_metadata.json"


def load_urdf():
    """Load and parse the URDF XML file, skip test if missing."""
    if not URDF_FILE.exists():
        pytest.skip(f"URDF file not found: {URDF_FILE}")
    tree = ET.parse(URDF_FILE)
    return tree.getroot()


def load_metadata():
    """Load the metadata JSON file, skip test if missing."""
    if not METADATA_FILE.exists():
        pytest.skip(f"Metadata file not found: {METADATA_FILE}")
    with open(METADATA_FILE, 'r') as f:
        return json.load(f)


def test_urdf_file_exists():
    """URDF file exists at expected location."""
    assert URDF_FILE.exists(), f"URDF file not found: {URDF_FILE}"


def test_urdf_well_formed_xml():
    """URDF is well-formed XML."""
    try:
        root = load_urdf()
        assert root.tag == 'robot', "Root element should be <robot>"
    except ET.ParseError as e:
        pytest.fail(f"URDF XML parsing failed: {e}")


def test_robot_name():
    """URDF robot name matches configuration."""
    root = load_urdf()
    robot_name = root.get('name')
    assert robot_name == 'inverted_pendulum_robot', (
        f"Robot name '{robot_name}' != 'inverted_pendulum_robot'"
    )


def test_all_links_present():
    """All expected links are defined."""
    root = load_urdf()
    links = {link.get('name') for link in root.findall('link')}
    expected_links = {'Base_Link', 'Wheel_Left', 'Wheel_Right', 'Pendulum_Link', 'Pendulum_Link_Right'}
    assert links == expected_links, f"Missing or extra links: {links} vs {expected_links}"


def test_all_joints_present():
    """All expected joints are defined."""
    root = load_urdf()
    joints = {joint.get('name') for joint in root.findall('joint')}
    expected_joints = {
        'wheel_left_joint',
        'wheel_right_joint',
        'pendulum_pivot_joint',
        'pendulum_pivot_right_joint'
    }
    assert joints == expected_joints, f"Missing or extra joints: {joints} vs {expected_joints}"


def test_all_joints_are_revolute():
    """All joints are type 'revolute'."""
    root = load_urdf()
    joints = root.findall('joint')
    for joint in joints:
        assert joint.get('type') == 'revolute', (
            f"Joint {joint.get('name')} has type {joint.get('type')}, expected 'revolute'"
        )


def test_joint_references_valid_links():
    """All joint parent/child references point to existing links."""
    root = load_urdf()
    links = {link.get('name') for link in root.findall('link')}
    joints = root.findall('joint')

    for joint in joints:
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        assert parent in links, (
            f"Joint {joint.get('name')} references non-existent parent link '{parent}'"
        )
        assert child in links, (
            f"Joint {joint.get('name')} references non-existent child link '{child}'"
        )


def test_base_link_no_incoming_joint():
    """Base_Link has no incoming joint (it's the root/fixed frame)."""
    root = load_urdf()
    joints = root.findall('joint')

    for joint in joints:
        child = joint.find('child').get('link')
        assert child != 'Base_Link', (
            f"Joint {joint.get('name')} has Base_Link as child (should be root only)"
        )


def test_joint_axes():
    """All joints have valid axes (should be [0, 1, 0] for Y-axis rotation)."""
    root = load_urdf()
    joints = root.findall('joint')

    for joint in joints:
        axis_elem = joint.find('axis')
        axis_str = axis_elem.get('xyz')
        axis = [float(x) for x in axis_str.split()]
        assert axis == [0.0, 1.0, 0.0], (
            f"Joint {joint.get('name')} has axis {axis}, expected [0, 1, 0]"
        )


def test_every_link_has_inertial():
    """Every link has exactly one <inertial> element."""
    root = load_urdf()
    links = root.findall('link')

    for link in links:
        inertial_count = len(link.findall('inertial'))
        assert inertial_count == 1, (
            f"Link {link.get('name')} has {inertial_count} inertial elements (expected 1)"
        )


def test_inertial_mass_positive():
    """All link masses are positive."""
    root = load_urdf()
    links = root.findall('link')

    for link in links:
        inertial = link.find('inertial')
        mass_elem = inertial.find('mass')
        mass_value = float(mass_elem.get('value'))
        assert mass_value > 0, (
            f"Link {link.get('name')} has non-positive mass: {mass_value}"
        )


def test_inertial_tensor_structure():
    """All links have valid inertia tensors (6 components: ixx, iyy, izz, ixy, ixz, iyz)."""
    root = load_urdf()
    links = root.findall('link')

    for link in links:
        inertial = link.find('inertial')
        inertia_elem = inertial.find('inertia')
        required_attrs = {'ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'}
        actual_attrs = set(inertia_elem.attrib.keys())
        assert actual_attrs == required_attrs, (
            f"Link {link.get('name')} inertia has unexpected attrs: {actual_attrs}"
        )

        # Diagonal elements should be positive
        for attr in ['ixx', 'iyy', 'izz']:
            value = float(inertia_elem.get(attr))
            assert value > 0, (
                f"Link {link.get('name')} {attr}={value} is not positive"
            )


def test_pendulum_links_have_combined_mass():
    """Pendulum_Link and Pendulum_Link_Right combined masses ~0.175 kg (plate 0.12 + servo 0.055)."""
    root = load_urdf()
    metadata = load_metadata()

    expected_combined = 0.175

    for link_name in ['Pendulum_Link', 'Pendulum_Link_Right']:
        link = root.find(f".//link[@name='{link_name}']")
        assert link is not None, f"Link {link_name} not found"

        inertial = link.find('inertial')
        mass_elem = inertial.find('mass')
        mass_value = float(mass_elem.get('value'))

        assert abs(mass_value - expected_combined) < 0.001, (
            f"Link {link_name} mass {mass_value:.3f} != expected {expected_combined:.3f}"
        )


def test_visual_mesh_file_exists():
    """Visual mesh file exists on disk."""
    assert URDF_VISUAL_MESH.exists(), (
        f"Visual mesh file not found: {URDF_VISUAL_MESH}"
    )


def test_visual_mesh_referenced_in_urdf():
    """URDF references the visual mesh file."""
    root = load_urdf()
    meshes = root.findall(".//mesh[@filename]")

    assert len(meshes) > 0, "No mesh elements found in URDF"

    mesh_files = [mesh.get('filename') for mesh in meshes]
    # Should reference the package-relative path
    assert any('feetech-STS3032-visual.stl' in f for f in mesh_files), (
        f"No reference to feetech-STS3032-visual.stl found in: {mesh_files}"
    )


def test_pendulum_links_visual_geometry_structure():
    """Pendulum_Link and Pendulum_Link_Right each have exactly 2 visual elements:
    plate box + servo mesh.
    """
    root = load_urdf()

    for link_name in ['Pendulum_Link', 'Pendulum_Link_Right']:
        link = root.find(f".//link[@name='{link_name}']")
        assert link is not None, f"Link {link_name} not found"

        visuals = link.findall('visual')
        assert len(visuals) == 2, (
            f"Link {link_name} has {len(visuals)} visual elements, expected 2 "
            "(plate box + servo mesh)"
        )

        # Collect geometry types from all visual elements
        geometry_types = []
        for i, visual in enumerate(visuals):
            geometry = visual.find('geometry')
            assert geometry is not None, f"Visual {i} in {link_name} has no geometry"

            box = geometry.find('box')
            mesh = geometry.find('mesh')
            cylinder = geometry.find('cylinder')

            if box is not None:
                geometry_types.append('box')
            elif mesh is not None:
                geometry_types.append('mesh')
            elif cylinder is not None:
                geometry_types.append('cylinder')
            else:
                raise AssertionError(f"Visual {i} in {link_name} has no recognized geometry")

        # Verify the expected geometry types (order: plate box, servo mesh)
        assert geometry_types == ['box', 'mesh'], (
            f"Link {link_name} visual geometries {geometry_types}, "
            f"expected ['box', 'mesh']"
        )


def test_servo_no_mesh_in_collision():
    """Servo collision geometry uses primitives (box/cylinder), NOT mesh.

    Pendulum_Link and Pendulum_Link_Right each must have exactly 3 collision
    elements: plate box + servo box + servo cylinder.
    """
    root = load_urdf()

    for link_name in ['Pendulum_Link', 'Pendulum_Link_Right']:
        link = root.find(f".//link[@name='{link_name}']")
        assert link is not None, f"Link {link_name} not found"

        collisions = link.findall('collision')
        assert len(collisions) == 3, (
            f"Link {link_name} has {len(collisions)} collision elements, expected 3 "
            "(plate box + servo box + servo cylinder)"
        )

        # Collect geometry types from all collision elements
        geometry_types = []
        for i, collision in enumerate(collisions):
            geometry = collision.find('geometry')
            assert geometry is not None, f"Collision {i} in {link_name} has no geometry"

            # Check for mesh (should NOT exist in collision)
            mesh = geometry.find('mesh')
            assert mesh is None, (
                f"Link {link_name} collision {i} has a mesh (should use primitives only)"
            )

            # Determine geometry type
            box = geometry.find('box')
            cylinder = geometry.find('cylinder')
            if box is not None:
                geometry_types.append('box')
            elif cylinder is not None:
                geometry_types.append('cylinder')
            else:
                raise AssertionError(f"Collision {i} in {link_name} has neither box nor cylinder")

        # Verify the expected geometry types (order: plate box, servo box, servo cylinder)
        assert geometry_types == ['box', 'box', 'cylinder'], (
            f"Link {link_name} collision geometries {geometry_types}, "
            f"expected ['box', 'box', 'cylinder']"
        )


def test_metadata_exists():
    """Metadata JSON file exists."""
    assert METADATA_FILE.exists(), f"Metadata file not found: {METADATA_FILE}"


def test_metadata_structure():
    """Metadata JSON has expected structure."""
    metadata = load_metadata()

    expected_keys = {'issue', 'phase', 'timestamp', 'robot_name', 'input_documents',
                     'output_files', 'collision_primitives', 'visual_mesh', 'links',
                     'ground_joint', 'validations'}
    actual_keys = set(metadata.keys())
    assert actual_keys == expected_keys, (
        f"Metadata keys mismatch: {actual_keys} vs {expected_keys}"
    )

    assert metadata['issue'] == 84, "Metadata issue should be 84"
    assert metadata['phase'] == 10, "Metadata phase should be 10"
    assert metadata['robot_name'] == 'inverted_pendulum_robot'


def test_collision_primitives_documented():
    """Collision primitives dimensions are documented in metadata."""
    metadata = load_metadata()

    primitives = metadata['collision_primitives']
    assert 'servo_box' in primitives
    assert 'servo_cylinder' in primitives

    box = primitives['servo_box']
    assert box['length_mm'] == 32.0
    assert box['width_mm'] == 12.0
    assert box['height_mm'] == 28.0

    cylinder = primitives['servo_cylinder']
    assert cylinder['radius_mm'] == 4.65
    assert cylinder['height_mm'] == 16.15


def test_servo_collision_primitives_in_urdf():
    """Servo collision box and cylinder have correct dimensions in URDF.

    Pendulum_Link and Pendulum_Link_Right each must have a servo box with
    32x12x28mm and a servo cylinder with r=4.65mm, h=16.15mm (after mm->m conversion).
    """
    root = load_urdf()

    # Expected dimensions (mm -> m conversion)
    servo_box_length_m = 32.0 / 1000.0
    servo_box_width_m = 12.0 / 1000.0
    servo_box_height_m = 28.0 / 1000.0
    servo_cyl_radius_m = 4.65 / 1000.0
    servo_cyl_height_m = 16.15 / 1000.0

    for link_name in ['Pendulum_Link', 'Pendulum_Link_Right']:
        link = root.find(f".//link[@name='{link_name}']")
        assert link is not None

        collisions = link.findall('collision')
        assert len(collisions) == 3, f"Expected 3 collisions in {link_name}, got {len(collisions)}"

        # Collision 0: plate box (check exists, exact dims not verified here)
        plate_collision = collisions[0]
        plate_geom = plate_collision.find('geometry/box')
        assert plate_geom is not None, f"{link_name} collision 0 should be a box (plate)"

        # Collision 1: servo box
        servo_box_collision = collisions[1]
        servo_box_geom = servo_box_collision.find('geometry/box')
        assert servo_box_geom is not None, f"{link_name} collision 1 should be a box (servo)"

        servo_box_size = servo_box_geom.get('size')
        assert servo_box_size is not None, f"{link_name} servo box missing 'size' attribute"

        box_dims = [float(x) for x in servo_box_size.split()]
        assert len(box_dims) == 3, f"Servo box size should have 3 dimensions, got {len(box_dims)}"

        # Check dimensions (with small tolerance for floating point)
        tolerance = 0.0001  # 0.1 micron tolerance in meters
        assert abs(box_dims[0] - servo_box_length_m) < tolerance, (
            f"{link_name} servo box length {box_dims[0]:.6f}m != {servo_box_length_m:.6f}m"
        )
        assert abs(box_dims[1] - servo_box_width_m) < tolerance, (
            f"{link_name} servo box width {box_dims[1]:.6f}m != {servo_box_width_m:.6f}m"
        )
        assert abs(box_dims[2] - servo_box_height_m) < tolerance, (
            f"{link_name} servo box height {box_dims[2]:.6f}m != {servo_box_height_m:.6f}m"
        )

        # Collision 2: servo cylinder
        servo_cyl_collision = collisions[2]
        servo_cyl_geom = servo_cyl_collision.find('geometry/cylinder')
        assert servo_cyl_geom is not None, f"{link_name} collision 2 should be a cylinder (servo shaft)"

        cyl_radius = servo_cyl_geom.get('radius')
        cyl_length = servo_cyl_geom.get('length')
        assert cyl_radius is not None and cyl_length is not None, (
            f"{link_name} servo cylinder missing radius/length"
        )

        cyl_radius_val = float(cyl_radius)
        cyl_length_val = float(cyl_length)

        assert abs(cyl_radius_val - servo_cyl_radius_m) < tolerance, (
            f"{link_name} servo cylinder radius {cyl_radius_val:.6f}m != {servo_cyl_radius_m:.6f}m"
        )
        assert abs(cyl_length_val - servo_cyl_height_m) < tolerance, (
            f"{link_name} servo cylinder length {cyl_length_val:.6f}m != {servo_cyl_height_m:.6f}m"
        )


def test_visual_mesh_metadata():
    """Visual mesh metadata is present and correct."""
    metadata = load_metadata()

    visual_mesh = metadata['visual_mesh']
    assert 'source_file' in visual_mesh
    assert 'facet_count' in visual_mesh
    assert 'tolerance_mm' in visual_mesh

    assert visual_mesh['facet_count'] == 37556, (
        f"Expected 37556 facets, got {visual_mesh['facet_count']}"
    )
    assert visual_mesh['tolerance_mm'] == 1.0


def test_ground_joint_logged():
    """Ground joint from Stage 2 is logged (not emitted in URDF)."""
    metadata = load_metadata()

    ground_joint = metadata['ground_joint']
    assert 'name' in ground_joint
    assert 'type' in ground_joint
    assert ground_joint['type'] == 'ObjectToGround'
    assert 'object_grounded' in ground_joint


def test_link_inertia_metadata():
    """Link inertias are documented in metadata."""
    metadata = load_metadata()

    for link_name in ['Base_Link', 'Wheel_Left', 'Wheel_Right', 'Pendulum_Link', 'Pendulum_Link_Right']:
        assert link_name in metadata['links'], f"Link {link_name} not in metadata"

        link_data = metadata['links'][link_name]
        assert 'inertia_kg_mm2' in link_data or 'combined_inertia_kg_mm2' in link_data


def test_pendulum_servo_combination_documented():
    """Pendulum_Link/Right servo+plate combination is documented."""
    metadata = load_metadata()

    for link_name in ['Pendulum_Link', 'Pendulum_Link_Right']:
        link_data = metadata['links'][link_name]

        assert 'plate_mass_kg' in link_data, f"{link_name} missing plate_mass_kg"
        assert 'servo_mass_kg' in link_data, f"{link_name} missing servo_mass_kg"
        assert 'combined_mass_kg' in link_data, f"{link_name} missing combined_mass_kg"
        assert 'combined_com_mm' in link_data, f"{link_name} missing combined_com_mm"
        assert 'combined_inertia_kg_mm2' in link_data, f"{link_name} missing combined_inertia_kg_mm2"

        combined = link_data['combined_mass_kg']
        plate = link_data['plate_mass_kg']
        servo = link_data['servo_mass_kg']

        assert abs(combined - (plate + servo)) < 0.0001, (
            f"{link_name} combined mass {combined} != plate {plate} + servo {servo}"
        )


def test_validations_passed():
    """All validations in metadata passed."""
    metadata = load_metadata()

    validations = metadata['validations']
    for validation in validations:
        assert validation['passed'], (
            f"Validation '{validation['check']}' failed: {validation.get('details', '')}"
        )
