"""
Tests for Stage 4's URDF export (10_export_urdf.py).

Pure Python (no FreeCAD imports) — reads the generated URDF XML file and validates
its structure. Follows the pattern of test_08_/test_09_*.py.
"""

import json
import sys
import importlib.util
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest

# Add script dir to path to import combine_plate_stack
SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

# Import 10_export_urdf using importlib (module name starts with digit)
_spec = importlib.util.spec_from_file_location("export_urdf_10", SCRIPT_DIR / "10_export_urdf.py")
_export_urdf = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_export_urdf)


# Locate the URDF and metadata outputs
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
    """Servo collision box and cylinder have correct dimensions and origins in URDF.

    Pendulum_Link and Pendulum_Link_Right each must have a servo box with
    32x12x28mm and a servo cylinder with r=4.65mm, h=16.15mm (after mm->m conversion).
    The right servo's collision elements should have rpy reflecting the 180° roll.
    """
    root = load_urdf()
    metadata = load_metadata()

    # Expected dimensions (mm -> m conversion)
    servo_box_length_m = 32.0 / 1000.0
    servo_box_width_m = 12.0 / 1000.0
    servo_box_height_m = 28.0 / 1000.0
    servo_cyl_radius_m = 4.65 / 1000.0
    servo_cyl_height_m = 16.15 / 1000.0

    # Expected origins from metadata (mm -> m)
    left_servo_box_origin = metadata['collision_primitives']['left_servo_box_origin_mm']
    left_servo_box_origin_m = [x / 1000.0 for x in left_servo_box_origin]
    left_servo_cyl_origin = metadata['collision_primitives']['left_servo_cylinder_origin_mm']
    left_servo_cyl_origin_m = [x / 1000.0 for x in left_servo_cyl_origin]
    right_servo_box_origin = metadata['collision_primitives']['right_servo_box_origin_mm']
    right_servo_box_origin_m = [x / 1000.0 for x in right_servo_box_origin]
    right_servo_cyl_origin = metadata['collision_primitives']['right_servo_cylinder_origin_mm']
    right_servo_cyl_origin_m = [x / 1000.0 for x in right_servo_cyl_origin]

    # Expected rpy values
    left_servo_rpy = [0.0, 0.0, 0.0]
    right_servo_rpy = metadata['collision_primitives']['right_servo_box_rpy_rad']

    tolerance = 0.0001  # 0.1 micron tolerance in meters
    rpy_tolerance = 0.0001  # radians

    for link_name, expected_box_origin_m, expected_cyl_origin_m, expected_rpy in [
        ('Pendulum_Link', left_servo_box_origin_m, left_servo_cyl_origin_m, left_servo_rpy),
        ('Pendulum_Link_Right', right_servo_box_origin_m, right_servo_cyl_origin_m, right_servo_rpy),
    ]:
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
        servo_box_origin = servo_box_collision.find('origin')
        assert servo_box_origin is not None, f"{link_name} servo box should have origin element"

        # Check servo box xyz origin
        servo_box_xyz_str = servo_box_origin.get('xyz')
        assert servo_box_xyz_str is not None, f"{link_name} servo box origin missing xyz"
        servo_box_xyz = [float(x) for x in servo_box_xyz_str.split()]
        for i in range(3):
            assert abs(servo_box_xyz[i] - expected_box_origin_m[i]) < tolerance, (
                f"{link_name} servo box origin[{i}] {servo_box_xyz[i]:.6f}m != {expected_box_origin_m[i]:.6f}m"
            )

        # Check servo box rpy
        servo_box_rpy_str = servo_box_origin.get('rpy')
        assert servo_box_rpy_str is not None, f"{link_name} servo box origin missing rpy"
        servo_box_rpy = [float(x) for x in servo_box_rpy_str.split()]
        for i in range(3):
            assert abs(servo_box_rpy[i] - expected_rpy[i]) < rpy_tolerance, (
                f"{link_name} servo box rpy[{i}] {servo_box_rpy[i]:.6f} != {expected_rpy[i]:.6f}"
            )

        servo_box_geom = servo_box_collision.find('geometry/box')
        assert servo_box_geom is not None, f"{link_name} collision 1 should be a box (servo)"

        servo_box_size = servo_box_geom.get('size')
        assert servo_box_size is not None, f"{link_name} servo box missing 'size' attribute"

        box_dims = [float(x) for x in servo_box_size.split()]
        assert len(box_dims) == 3, f"Servo box size should have 3 dimensions, got {len(box_dims)}"

        # Check dimensions (with small tolerance for floating point)
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
        servo_cyl_origin = servo_cyl_collision.find('origin')
        assert servo_cyl_origin is not None, f"{link_name} servo cylinder should have origin element"

        # Check servo cylinder xyz origin
        servo_cyl_xyz_str = servo_cyl_origin.get('xyz')
        assert servo_cyl_xyz_str is not None, f"{link_name} servo cylinder origin missing xyz"
        servo_cyl_xyz = [float(x) for x in servo_cyl_xyz_str.split()]
        for i in range(3):
            assert abs(servo_cyl_xyz[i] - expected_cyl_origin_m[i]) < tolerance, (
                f"{link_name} servo cylinder origin[{i}] {servo_cyl_xyz[i]:.6f}m != {expected_cyl_origin_m[i]:.6f}m"
            )

        # Check servo cylinder rpy
        servo_cyl_rpy_str = servo_cyl_origin.get('rpy')
        assert servo_cyl_rpy_str is not None, f"{link_name} servo cylinder origin missing rpy"
        servo_cyl_rpy = [float(x) for x in servo_cyl_rpy_str.split()]
        for i in range(3):
            assert abs(servo_cyl_rpy[i] - expected_rpy[i]) < rpy_tolerance, (
                f"{link_name} servo cylinder rpy[{i}] {servo_cyl_rpy[i]:.6f} != {expected_rpy[i]:.6f}"
            )

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


# ============================================================================
# Unit tests for combine_plate_stack() (Issue #96)
# ============================================================================
# These tests directly exercise combine_plate_stack() with hand-computable
# expected values, catching regressions in density scaling and inertia math.


def test_combine_plate_stack_single_plate():
    """Single plate at origin CoM simplifies to identity case.

    When only one plate exists with CoM at (0, 0, 0), the combined result
    should equal the plate's mass (= target mass), CoM at origin, and inertia
    scaled by density = (target_mass / volume).

    Verified manually:
      - mass = target_mass (by definition, single plate)
      - combined_com = [0, 0, 0] (plate's CoM)
      - inertia = unit_density_tensor * (target_mass / volume)
    """
    combine_plate_stack = _export_urdf.combine_plate_stack

    # Single synthetic plate: 100 mm³ volume, CoM at origin, unit-density inertia
    target_mass = 0.05  # kg
    plate_volume = 100.0  # mm³
    unit_inertia = {
        'ixx': 500.0,
        'iyy': 600.0,
        'izz': 700.0,
        'ixy': 0.0,
        'ixz': 0.0,
        'iyz': 0.0,
    }

    plate_shapes = [
        {
            'volume_mm3': plate_volume,
            'center_of_mass_mm': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'inertia_unit_density_kg_mm2': unit_inertia,
        }
    ]

    mass, com, inertia = combine_plate_stack(plate_shapes, target_mass)

    # Mass should equal target (single plate distributes all target mass to itself)
    assert abs(mass - target_mass) < 1e-9, (
        f"Single-plate mass {mass} != target {target_mass}"
    )

    # CoM should be at origin (plate CoM is at origin)
    assert abs(com[0]) < 1e-9 and abs(com[1]) < 1e-9 and abs(com[2]) < 1e-9, (
        f"CoM {com} should be near [0, 0, 0]"
    )

    # Inertia should be unit_inertia * (mass / volume)
    # At origin, parallel-axis shift is zero (d=0)
    expected_density = target_mass / plate_volume
    for key in unit_inertia:
        expected_val = unit_inertia[key] * expected_density
        assert abs(inertia[key] - expected_val) < 1e-6, (
            f"Inertia {key}: {inertia[key]} != {expected_val}"
        )


def test_combine_plate_stack_two_plates_volume_weighting():
    """Two plates with 2:1 volume ratio, distinct CoMs, verify mass split and combined CoM.

    Plate A: 200 mm³, CoM at (10, 0, 0)
    Plate B: 100 mm³, CoM at (-10, 0, 0)
    Total volume: 300 mm³, target mass: 0.3 kg

    Expected:
      - Plate A mass = 0.3 * (200/300) = 0.2 kg
      - Plate B mass = 0.3 * (100/300) = 0.1 kg
      - Combined CoM = (0.2*10 + 0.1*(-10)) / 0.3 = (2 - 1) / 0.3 = 1/0.3 ≈ 3.333 mm
      - Combined mass = 0.3 kg (sum preservation)
    """
    combine_plate_stack = _export_urdf.combine_plate_stack

    target_mass = 0.3  # kg

    # Plate A: 200 mm³, CoM at (10, 0, 0)
    # Simple uniform inertia for testing
    plate_a_volume = 200.0
    plate_a_com = {'x': 10.0, 'y': 0.0, 'z': 0.0}
    plate_a_inertia = {
        'ixx': 100.0,
        'iyy': 100.0,
        'izz': 100.0,
        'ixy': 0.0,
        'ixz': 0.0,
        'iyz': 0.0,
    }

    # Plate B: 100 mm³, CoM at (-10, 0, 0)
    plate_b_volume = 100.0
    plate_b_com = {'x': -10.0, 'y': 0.0, 'z': 0.0}
    plate_b_inertia = {
        'ixx': 50.0,
        'iyy': 50.0,
        'izz': 50.0,
        'ixy': 0.0,
        'ixz': 0.0,
        'iyz': 0.0,
    }

    plate_shapes = [
        {
            'volume_mm3': plate_a_volume,
            'center_of_mass_mm': plate_a_com,
            'inertia_unit_density_kg_mm2': plate_a_inertia,
        },
        {
            'volume_mm3': plate_b_volume,
            'center_of_mass_mm': plate_b_com,
            'inertia_unit_density_kg_mm2': plate_b_inertia,
        }
    ]

    mass, com, inertia = combine_plate_stack(plate_shapes, target_mass)

    # Verify combined mass
    assert abs(mass - target_mass) < 1e-9, (
        f"Combined mass {mass} != target {target_mass}"
    )

    # Verify combined CoM (mass-weighted average)
    # Expected: x = (0.2*10 + 0.1*(-10)) / 0.3 = 10/3 ≈ 3.333 mm
    expected_com_x = (0.2 * 10.0 + 0.1 * (-10.0)) / 0.3
    assert abs(com[0] - expected_com_x) < 1e-6, (
        f"CoM X {com[0]} != expected {expected_com_x}"
    )
    assert abs(com[1]) < 1e-9, f"CoM Y {com[1]} should be ~0"
    assert abs(com[2]) < 1e-9, f"CoM Z {com[2]} should be ~0"


def test_combine_plate_stack_matches_real_data():
    """Load real plate_shapes from 07_body_wheels_metadata.json and verify sanity.

    Verifies the function handles real data without error and produces
    a combined mass equal to the target_mass_kg.
    """
    combine_plate_stack = _export_urdf.combine_plate_stack

    # Load real fixture data
    metadata_path = SCRIPT_DIR / "07_body_wheels_metadata.json"
    if not metadata_path.exists():
        pytest.skip(f"Real data fixture not found: {metadata_path}")

    with open(metadata_path, 'r') as f:
        body_wheels_metadata = json.load(f)

    # Extract Pendulum_Link data
    pend_link_data = body_wheels_metadata['links']['Pendulum_Link']
    plate_shapes = pend_link_data.get('plate_shapes')

    if not plate_shapes:
        pytest.skip("Pendulum_Link plate_shapes not found in fixture")

    target_mass_kg = pend_link_data['target_mass_kg']

    # Call the function with real data
    mass, com, inertia = combine_plate_stack(plate_shapes, target_mass_kg)

    # Verify combined mass equals target (within floating-point tolerance)
    assert abs(mass - target_mass_kg) < 1e-9, (
        f"Real-data combined mass {mass} != target {target_mass_kg}"
    )

    # Verify all inertia components exist and are finite
    for key in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']:
        assert key in inertia, f"Missing inertia component: {key}"
        assert isinstance(inertia[key], (int, float)), f"{key} not a number"
        assert not (inertia[key] != inertia[key]), f"{key} is NaN"  # NaN check

    # Verify diagonal components are positive
    for key in ['ixx', 'iyy', 'izz']:
        assert inertia[key] > 0, (
            f"Diagonal inertia {key}={inertia[key]} should be positive"
        )
