"""
Tests for Stage 3's servo mass properties computation (09_compute_mass_properties.py).

Pure Python (no FreeCAD imports) — reads the JSON metadata output file and validates
contents. Follows the pattern of test_08_configure_assembly_joints.py.
"""

import json
import math
from pathlib import Path

import pytest
import yaml


# Locate the JSON output and robot_parameters.yaml
SCRIPT_DIR = Path(__file__).resolve().parent
JSON_OUTPUT = SCRIPT_DIR / "09_mass_properties.json"
DESIGN_INPUTS_DIR = SCRIPT_DIR.parent.parent / "02_Design_Inputs"
ROBOT_PARAMS_YAML = DESIGN_INPUTS_DIR / "robot_parameters.yaml"


def load_json_output():
    """Load the JSON output file, skip test if missing."""
    if not JSON_OUTPUT.exists():
        pytest.skip(f"JSON output file not found: {JSON_OUTPUT}")
    with open(JSON_OUTPUT, 'r') as f:
        return json.load(f)


def load_robot_parameters():
    """Load robot_parameters.yaml."""
    if not ROBOT_PARAMS_YAML.exists():
        pytest.skip(f"robot_parameters.yaml not found: {ROBOT_PARAMS_YAML}")
    with open(ROBOT_PARAMS_YAML, 'r') as f:
        return yaml.safe_load(f)


def test_both_servos_present():
    """Both servo_left and servo_right entries exist."""
    data = load_json_output()
    assert 'servo_left' in data, "Missing servo_left in JSON output"
    assert 'servo_right' in data, "Missing servo_right in JSON output"


def test_servo_volumes_nonzero():
    """Both servo visual mesh volumes are nonzero."""
    data = load_json_output()
    assert data['servo_left']['volume_mm3'] > 0, "servo_left volume is not positive"
    assert data['servo_right']['volume_mm3'] > 0, "servo_right volume is not positive"


def test_servo_left_right_volumes_close():
    """Servo left and right volumes are within a few percent of each other.

    Same physical part, two instances — volumes should be nearly identical.
    Allow 5% tolerance for any minor geometric/measurement noise.
    """
    data = load_json_output()
    left_vol = data['servo_left']['volume_mm3']
    right_vol = data['servo_right']['volume_mm3']

    diff_percent = abs(left_vol - right_vol) / left_vol * 100
    tolerance = 5.0

    assert diff_percent <= tolerance, (
        f"Left/right servo volumes differ by {diff_percent:.1f}% "
        f"(threshold: {tolerance}%). "
        f"Left: {left_vol:.2f} mm³, Right: {right_vol:.2f} mm³"
    )


def test_servo_masses_match_datasheet_target():
    """Both servo masses match the target from robot_parameters.yaml."""
    data = load_json_output()
    params = load_robot_parameters()

    target_mass = params['servo']['target_mass_kg']

    assert data['servo_left']['mass_kg'] == target_mass, (
        f"servo_left mass {data['servo_left']['mass_kg']} != "
        f"target {target_mass}"
    )
    assert data['servo_right']['mass_kg'] == target_mass, (
        f"servo_right mass {data['servo_right']['mass_kg']} != "
        f"target {target_mass}"
    )


def test_provisional_geometric_sanity_check():
    """PROVISIONAL: servo volumes check (tightened tolerance now that we source from clean collision-proxy).

    Per Issue #85's plan, validated against Issue #76's collision-proxy mesh:

    The test computes an estimated reference volume from nominal envelope dimensions:
      - Box: 32 x 12 x 28.5 mm (nominal servo external dimensions)
      - Minus a cylindrical bore: radius 4.65 mm, height 16.15 mm (output shaft area)
      - Expected reference ≈ (32*12*28.5) - π*4.65²*16.15 ≈ 9847 mm³

    PREVIOUS BUG (now fixed): The earlier script sourced volume from feetech_STS3032_visual_1_0mm,
    which is SELF-INTERSECTING and NON-SOLID (hasSelfIntersections()==True, isSolid()==False,
    Issue #76). Its volume reading was unreliable (~1037 mm³ vs. the correct clean collision-proxy
    mesh's ~11308 mm³). This led to a VERY LOOSE ±90% tolerance.

    CURRENT BEHAVIOR: The script now sources all volume (and mass properties) from the clean
    collision-proxy mesh (feetech_STS3032_collision_proxy), which is solid and has no self-
    intersections. The collision-proxy volume should be close to the reference envelope estimate
    (within ±30%). Once Issue #8 measures real servo mass/volume empirically, this tolerance
    can be tightened further or replaced with measured data.

    TODO (Issue #8): After hardware measurement spike, update this tolerance and the
    nominal dimensions to match reality, or replace with measured envelope dimensions.
    """
    data = load_json_output()

    # Reference box dimensions (mm) — nominal envelope, accounting for material only
    box_l, box_w, box_h = 32.0, 12.0, 28.5
    box_volume = box_l * box_w * box_h

    # Subtract cylindrical bore (output shaft opening)
    cyl_radius = 4.65
    cyl_height = 16.15
    cyl_volume = math.pi * (cyl_radius ** 2) * cyl_height

    reference_volume = box_volume - cyl_volume

    # ±30% tolerance: sourced from clean collision-proxy mesh (Issue #76).
    # This should be much tighter now that we're not using the self-intersecting visual mesh.
    tolerance_percent = 30.0
    tolerance_abs = reference_volume * tolerance_percent / 100.0

    left_vol = data['servo_left']['volume_mm3']
    right_vol = data['servo_right']['volume_mm3']

    left_diff = abs(left_vol - reference_volume)
    right_diff = abs(right_vol - reference_volume)

    assert left_vol > 0, "servo_left volume is not positive"
    assert right_vol > 0, "servo_right volume is not positive"
    assert left_diff <= tolerance_abs, (
        f"servo_left volume {left_vol:.2f} mm³ differs from reference "
        f"{reference_volume:.2f} mm³ by {left_diff:.2f} mm³ ({left_diff/reference_volume*100:.1f}%), "
        f"exceeding ±{tolerance_percent}% tolerance. "
        f"This is sourced from the collision-proxy mesh (clean geometry, Issue #76). "
        f"If this fails, check issue #8 hardware measurement spike data."
    )

    assert right_diff <= tolerance_abs, (
        f"servo_right volume {right_vol:.2f} mm³ differs from reference "
        f"{reference_volume:.2f} mm³ by {right_diff:.2f} mm³ ({right_diff/reference_volume*100:.1f}%), "
        f"exceeding ±{tolerance_percent}% tolerance. "
        f"This is sourced from the collision-proxy mesh (clean geometry, Issue #76). "
        f"If this fails, check issue #8 hardware measurement spike data."
    )


def test_inertia_tensor_structure():
    """Inertia tensor has expected structure (ixx, iyy, izz, ixy, ixz, iyz)."""
    data = load_json_output()

    for servo_name in ['servo_left', 'servo_right']:
        inertia = data[servo_name]['inertia_kg_mm2']
        required_keys = {'ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'}
        assert set(inertia.keys()) == required_keys, (
            f"{servo_name} inertia tensor has unexpected keys: {set(inertia.keys())}"
        )

        # All diagonal elements should be positive (moment of inertia > 0)
        assert inertia['ixx'] > 0, f"{servo_name} ixx is not positive"
        assert inertia['iyy'] > 0, f"{servo_name} iyy is not positive"
        assert inertia['izz'] > 0, f"{servo_name} izz is not positive"


def test_center_of_mass_structure():
    """Center of mass has expected [x, y, z] structure."""
    data = load_json_output()

    for servo_name in ['servo_left', 'servo_right']:
        com = data[servo_name]['center_of_mass_mm']
        assert isinstance(com, list) and len(com) == 3, (
            f"{servo_name} center_of_mass_mm has unexpected structure: {com}"
        )
        for i, coord in enumerate(com):
            assert isinstance(coord, (int, float)), (
                f"{servo_name} center_of_mass_mm[{i}] is not numeric: {coord}"
            )


def test_visual_mesh_volumes_nonzero():
    """Visual mesh volumes (informational, self-intersecting) are logged."""
    data = load_json_output()

    assert data['servo_left']['visual_mesh_volume_mm3'] > 0, (
        "servo_left visual_mesh_volume_mm3 is not positive"
    )
    assert data['servo_right']['visual_mesh_volume_mm3'] > 0, (
        "servo_right visual_mesh_volume_mm3 is not positive"
    )


def test_issue_85_metadata():
    """Output JSON has Issue #85 metadata (issue field, timestamp)."""
    data = load_json_output()

    assert data.get('issue') == 85, "JSON output missing 'issue: 85' metadata"
    assert 'timestamp' in data, "JSON output missing timestamp"
    assert 'input_document' in data, "JSON output missing input_document"


def test_collision_proxy_bbox_present():
    """Both servo entries have collision_proxy_bbox_mm field (Stage 3 retrofit)."""
    data = load_json_output()

    for servo_name in ['servo_left', 'servo_right']:
        assert 'collision_proxy_bbox_mm' in data[servo_name], (
            f"{servo_name} missing 'collision_proxy_bbox_mm' field"
        )

        bbox = data[servo_name]['collision_proxy_bbox_mm']
        required_keys = {'x_length', 'y_length', 'z_length', 'x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max'}
        assert set(bbox.keys()) == required_keys, (
            f"{servo_name} collision_proxy_bbox_mm has unexpected keys: {set(bbox.keys())}"
        )

        # All values should be numeric and non-negative (or x/y/z_min/max can be negative)
        for key in ['x_length', 'y_length', 'z_length']:
            assert bbox[key] > 0, (
                f"{servo_name} collision_proxy_bbox_mm[{key}] should be positive, got {bbox[key]}"
            )


def test_collision_proxy_bbox_sanity():
    """Collision proxy BoundBox dimensions match servo envelope expectations."""
    data = load_json_output()

    # From servo_link_config.json's specifications:
    # body_length=32.0, body_width=12.0, body_height=28.0
    # Expected envelope: approximately 32x12x32mm (with some tolerance)
    # BoundBox from live read should be close to these nominal dims.

    expected_x = 32.0  # body_length
    expected_y = 12.0  # body_width
    expected_z_nominal = 28.0  # body_height
    tolerance_percent = 10.0  # ±10% tolerance

    for servo_name in ['servo_left', 'servo_right']:
        bbox = data[servo_name]['collision_proxy_bbox_mm']

        # X (length) should match
        x_diff_percent = abs(bbox['x_length'] - expected_x) / expected_x * 100
        assert x_diff_percent <= tolerance_percent, (
            f"{servo_name} X length {bbox['x_length']:.2f}mm differs from {expected_x}mm "
            f"by {x_diff_percent:.1f}% (tolerance: {tolerance_percent}%)"
        )

        # Y (width) should match exactly
        y_diff_percent = abs(bbox['y_length'] - expected_y) / expected_y * 100
        assert y_diff_percent <= tolerance_percent, (
            f"{servo_name} Y length {bbox['y_length']:.2f}mm differs from {expected_y}mm "
            f"by {y_diff_percent:.1f}% (tolerance: {tolerance_percent}%)"
        )

        # Z (height) should be close to body_height or slightly larger (envelope)
        # Actual collision proxy may include the shaft, so allow up to 35mm
        assert bbox['z_length'] > 25.0 and bbox['z_length'] <= 35.0, (
            f"{servo_name} Z length {bbox['z_length']:.2f}mm is outside expected range [25, 35]mm"
        )
