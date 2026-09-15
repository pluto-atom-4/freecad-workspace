"""
FCStd <-> URDF consistency test (Issue #149).

Validates that a live FreeCAD document's object placements match the independently-
recomputed global frames from exported URDF + joint_config.json composition.

This test catches composition bugs (e.g., #146 Amendment 3's PlateStack gap, #148's servo
recentering gap) that can slip past pure-Python regression tests by reading the ACTUAL,
live FCStd file's global placements and cross-checking against URDF composition.

Layer 2: freecadcmd-driven integration test (mirrors test_07_body_wheels_geometry.py's
pattern). Opens robot_body_wheels.FCStd headlessly, computes ground-truth global points
for 7 key objects, and asserts they match independent URDF composition.

Reuses helper functions from test_urdf_fk_regression.py (load_urdf, _apply_rotation_ypr,
_parse_xyz, _mm_to_m) to avoid code duplication.

Tolerance: 0.1 mm (1e-4 m) — deliberately looser than test_urdf_fk_regression.py's 1e-6,
since this test crosses a live-FreeCAD-read boundary.

Usage:
    FREECAD_BIN=~/.local/bin/freecadcmd1.1 \\
    mamba run -n pendulum-tools python3 -m pytest -q test_urdf_fcstd_consistency.py -v
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest

# Import helpers from sibling test module
from test_urdf_fk_regression import (
    load_urdf,
    load_joint_config,
    _apply_rotation_ypr,
    _parse_xyz,
    _mm_to_m,
)

SCRIPT_DIR = Path(__file__).resolve().parent
EXPORTS_DIR = SCRIPT_DIR.parent.parent / "06_Exports"
URDF_FILE = EXPORTS_DIR / "urdf" / "robot.urdf"
JOINT_CONFIG_FILE = SCRIPT_DIR / "joint_config.json"

FCSTD_FILE = SCRIPT_DIR / "robot_body_wheels.FCStd"
METADATA_JSON = SCRIPT_DIR / "07_body_wheels_metadata.json"

# Tolerance for global position comparisons (0.1 mm)
TOLERANCE_M = 1e-4


def _resolve_freecad_bin():
    """Same resolution order documented in mamba-envs.yaml / CLAUDE.md:
    FREECAD_BIN env var, else `freecadcmd` on PATH."""
    env_bin = os.environ.get("FREECAD_BIN")
    if env_bin:
        return env_bin if shutil.which(env_bin) or Path(env_bin).is_file() else None
    return shutil.which("freecadcmd")


FREECAD_BIN = _resolve_freecad_bin()

skip_reason_bin = (
    "No FreeCAD binary available (set FREECAD_BIN or put freecadcmd on PATH)"
)
skip_reason_fcstd = f"{FCSTD_FILE.name} not found"
skip_reason_urdf = f"{URDF_FILE.name} not found"


def _generate_fcstd_ground_truth():
    """Generate inline script that computes ground-truth points from live FCStd.

    This script opens robot_body_wheels.FCStd and extracts global placements for:
    1. Wheel_Left: getGlobalPlacement().Base
    2. Wheel_Right: getGlobalPlacement().Base
    3. Base_Link: Shape.BoundBox.Center
    4. STS3032_Mount (left): getGlobalPlacement().multVec(mesh BoundBox center)
    5. STS3032_Mount_Right: getGlobalPlacement().multVec(mesh BoundBox center)
    6. PlateStack (Bottom_Plate center via local bbox): getGlobalPlacement().multVec(local center)
    7. PlateStack_Right: same as #6 for right side

    Returns JSON dict with results dict keyed by object name.
    """
    return f'''
import json
import sys
sys.path.insert(0, "{SCRIPT_DIR}")

import FreeCAD as App
from pathlib import Path

# Open the FCStd file
doc = App.openDocument(r"{FCSTD_FILE}")
if not doc:
    raise RuntimeError("Failed to open {{}}".format(r"{FCSTD_FILE}"))

results = {{}}

# 1. Wheel_Left
obj = doc.getObject("Wheel_Left")
if obj:
    global_pt = obj.getGlobalPlacement().Base
    results["Wheel_Left"] = {{"x": global_pt.x, "y": global_pt.y, "z": global_pt.z}}

# 2. Wheel_Right
obj = doc.getObject("Wheel_Right")
if obj:
    global_pt = obj.getGlobalPlacement().Base
    results["Wheel_Right"] = {{"x": global_pt.x, "y": global_pt.y, "z": global_pt.z}}

# 3. Base_Link: Shape.BoundBox.Center
obj = doc.getObject("Base_Link")
if obj:
    bbox_center = obj.Shape.BoundBox.Center
    results["Base_Link"] = {{"x": bbox_center.x, "y": bbox_center.y, "z": bbox_center.z}}

# 4. STS3032_Mount (left): mount placement + visual mesh center
obj_mount = doc.getObject("STS3032_Mount")
obj_mesh = doc.getObject("feetech_STS3032_visual_1_0mm")
if obj_mount and obj_mesh:
    # Get mesh BoundBox center (live from FCStd, not cached JSON)
    mesh_bbox = obj_mesh.Mesh.BoundBox
    mesh_center = (
        (mesh_bbox.XMin + mesh_bbox.XMax) / 2.0,
        (mesh_bbox.YMin + mesh_bbox.YMax) / 2.0,
        (mesh_bbox.ZMin + mesh_bbox.ZMax) / 2.0,
    )
    # Apply mount placement: global = mount_placement.multVec(local_center)
    global_pt = obj_mount.getGlobalPlacement().multVec(
        App.Vector(*mesh_center)
    )
    results["STS3032_Mount"] = {{"x": global_pt.x, "y": global_pt.y, "z": global_pt.z}}

# 5. STS3032_Mount_Right: same pattern
obj_mount = doc.getObject("STS3032_Mount_Right")
obj_mesh = doc.getObject("feetech_STS3032_visual_1_0mm_Right")
if obj_mount and obj_mesh:
    mesh_bbox = obj_mesh.Mesh.BoundBox
    mesh_center = (
        (mesh_bbox.XMin + mesh_bbox.XMax) / 2.0,
        (mesh_bbox.YMin + mesh_bbox.YMax) / 2.0,
        (mesh_bbox.ZMin + mesh_bbox.ZMax) / 2.0,
    )
    global_pt = obj_mount.getGlobalPlacement().multVec(
        App.Vector(*mesh_center)
    )
    results["STS3032_Mount_Right"] = {{"x": global_pt.x, "y": global_pt.y, "z": global_pt.z}}

# 6. PlateStack (Bottom_Plate local center composed globally)
obj_plate = doc.getObject("Bottom_Plate")
if obj_plate:
    # Get local bbox center (unrotated frame, transform=False)
    import Part
    raw_shape = Part.getShape(obj_plate, "", needSubElement=False, transform=False)
    local_bbox = raw_shape.BoundBox
    local_center = App.Vector(
        (local_bbox.XMin + local_bbox.XMax) / 2.0,
        (local_bbox.YMin + local_bbox.YMax) / 2.0,
        (local_bbox.ZMin + local_bbox.ZMax) / 2.0,
    )
    # Apply plate's global placement to local center
    global_pt = obj_plate.getGlobalPlacement().multVec(local_center)
    results["PlateStack"] = {{"x": global_pt.x, "y": global_pt.y, "z": global_pt.z}}

# 7. PlateStack_Right: same pattern
obj_plate = doc.getObject("Bottom_Plate_Right")
if obj_plate:
    import Part
    raw_shape = Part.getShape(obj_plate, "", needSubElement=False, transform=False)
    local_bbox = raw_shape.BoundBox
    local_center = App.Vector(
        (local_bbox.XMin + local_bbox.XMax) / 2.0,
        (local_bbox.YMin + local_bbox.YMax) / 2.0,
        (local_bbox.ZMin + local_bbox.ZMax) / 2.0,
    )
    global_pt = obj_plate.getGlobalPlacement().multVec(local_center)
    results["PlateStack_Right"] = {{"x": global_pt.x, "y": global_pt.y, "z": global_pt.z}}

# Write results to stdout as JSON (the parent process captures it)
print(json.dumps(results))
'''


@pytest.fixture(scope="module")
def fcstd_ground_truth():
    """
    Spawn ONE freecadcmd subprocess per test session to read robot_body_wheels.FCStd
    and extract ground-truth global positions for all 7 objects.

    Yields dict with keys like "Wheel_Left", "Base_Link", etc., each containing
    {{x, y, z}} in mm (as read from FreeCAD).

    Tears down the subprocess after all tests in module complete.
    """
    script_code = _generate_fcstd_ground_truth()

    # Run freecadcmd subprocess with inline script
    cmd = [FREECAD_BIN, "-c"]
    result = subprocess.run(
        cmd,
        input=script_code,
        cwd=str(SCRIPT_DIR),
        capture_output=True,
        text=True,
        timeout=180,
    )

    if result.returncode != 0:
        pytest.skip(
            f"freecadcmd failed to read FCStd: return code {result.returncode}\n"
            f"stderr: {result.stderr}"
        )

    # Parse JSON output from stdout (last line should be the JSON dict)
    # If there are warnings/debug output, skip them and find the JSON
    lines = result.stdout.strip().split('\n')
    json_line = None
    for line in reversed(lines):  # Start from end to find last valid JSON
        if line.strip().startswith('{'):
            json_line = line
            break

    if not json_line:
        pytest.skip(
            f"Could not parse JSON from freecadcmd output:\n{result.stdout}\n"
            f"stderr: {result.stderr}"
        )

    try:
        data = json.loads(json_line)
    except json.JSONDecodeError as e:
        pytest.skip(f"Invalid JSON from freecadcmd: {e}\nOutput: {json_line}")

    yield data
    # No cleanup needed for subprocess (already completed)


# ============================================================================
# Tests: 7 separate test functions, one per object
# ============================================================================


@pytest.mark.skipif(FREECAD_BIN is None, reason=skip_reason_bin)
@pytest.mark.skipif(not FCSTD_FILE.is_file(), reason=skip_reason_fcstd)
@pytest.mark.skipif(not URDF_FILE.is_file(), reason=skip_reason_urdf)
def test_wheel_left_global_position(fcstd_ground_truth):
    """Test Wheel_Left: global placement from FCStd vs URDF joint origin."""
    urdf = load_urdf()
    joint_cfg = load_joint_config()

    # Ground truth from live FCStd
    gt_pt = fcstd_ground_truth.get("Wheel_Left")
    if not gt_pt:
        pytest.skip("Wheel_Left not found in FCStd")
    gt_xyz = [gt_pt["x"], gt_pt["y"], gt_pt["z"]]

    # URDF composition: joint origin (direct, no further composition)
    joint_elem = urdf.find(".//joint[@name='wheel_left_joint']")
    assert joint_elem is not None, "wheel_left_joint not found in URDF"
    origin_elem = joint_elem.find('origin')
    assert origin_elem is not None, "origin not found in wheel_left_joint"
    urdf_xyz = _parse_xyz(origin_elem.get('xyz'))  # Already in meters

    # Compare
    for axis, gt_val, urdf_val in zip(['x', 'y', 'z'], gt_xyz, urdf_xyz):
        gt_m = gt_val / 1000.0  # Convert mm to m
        diff = abs(gt_m - urdf_val)
        assert diff < TOLERANCE_M, (
            f"Wheel_Left {axis}: ground truth={gt_m:.6f} m, "
            f"URDF={urdf_val:.6f} m, diff={diff:.2e} m (tolerance={TOLERANCE_M:.2e} m)"
        )


@pytest.mark.skipif(FREECAD_BIN is None, reason=skip_reason_bin)
@pytest.mark.skipif(not FCSTD_FILE.is_file(), reason=skip_reason_fcstd)
@pytest.mark.skipif(not URDF_FILE.is_file(), reason=skip_reason_urdf)
def test_wheel_right_global_position(fcstd_ground_truth):
    """Test Wheel_Right: global placement from FCStd vs URDF joint origin."""
    urdf = load_urdf()

    # Ground truth from live FCStd
    gt_pt = fcstd_ground_truth.get("Wheel_Right")
    if not gt_pt:
        pytest.skip("Wheel_Right not found in FCStd")
    gt_xyz = [gt_pt["x"], gt_pt["y"], gt_pt["z"]]

    # URDF composition: joint origin (direct)
    joint_elem = urdf.find(".//joint[@name='wheel_right_joint']")
    assert joint_elem is not None, "wheel_right_joint not found in URDF"
    origin_elem = joint_elem.find('origin')
    assert origin_elem is not None, "origin not found in wheel_right_joint"
    urdf_xyz = _parse_xyz(origin_elem.get('xyz'))

    # Compare
    for axis, gt_val, urdf_val in zip(['x', 'y', 'z'], gt_xyz, urdf_xyz):
        gt_m = gt_val / 1000.0
        diff = abs(gt_m - urdf_val)
        assert diff < TOLERANCE_M, (
            f"Wheel_Right {axis}: ground truth={gt_m:.6f} m, "
            f"URDF={urdf_val:.6f} m, diff={diff:.2e} m"
        )


@pytest.mark.skipif(FREECAD_BIN is None, reason=skip_reason_bin)
@pytest.mark.skipif(not FCSTD_FILE.is_file(), reason=skip_reason_fcstd)
@pytest.mark.skipif(not URDF_FILE.is_file(), reason=skip_reason_urdf)
def test_base_link_global_position(fcstd_ground_truth):
    """Test Base_Link: BoundBox center from FCStd vs URDF root link visual origin."""
    urdf = load_urdf()

    # Ground truth from live FCStd
    gt_pt = fcstd_ground_truth.get("Base_Link")
    if not gt_pt:
        pytest.skip("Base_Link not found in FCStd")
    gt_xyz = [gt_pt["x"], gt_pt["y"], gt_pt["z"]]

    # URDF: Base_Link is root (no joint, visual origin is global)
    link_elem = urdf.find(".//link[@name='Base_Link']")
    assert link_elem is not None, "Base_Link not found in URDF"
    visual_elem = link_elem.find('visual')
    assert visual_elem is not None, "visual not found in Base_Link"
    origin_elem = visual_elem.find('origin')
    assert origin_elem is not None, "origin not found in Base_Link visual"
    urdf_xyz = _parse_xyz(origin_elem.get('xyz'))

    # Compare
    for axis, gt_val, urdf_val in zip(['x', 'y', 'z'], gt_xyz, urdf_xyz):
        gt_m = gt_val / 1000.0
        diff = abs(gt_m - urdf_val)
        assert diff < TOLERANCE_M, (
            f"Base_Link {axis}: ground truth={gt_m:.6f} m, "
            f"URDF={urdf_val:.6f} m, diff={diff:.2e} m"
        )


@pytest.mark.skipif(FREECAD_BIN is None, reason=skip_reason_bin)
@pytest.mark.skipif(not FCSTD_FILE.is_file(), reason=skip_reason_fcstd)
@pytest.mark.skipif(not URDF_FILE.is_file(), reason=skip_reason_urdf)
def test_sts3032_mount_left_global_position(fcstd_ground_truth):
    """Test STS3032_Mount (left): servo mesh center vs URDF Pendulum_Link composition.

    Composition chain:
    1. STS3032_Mount's global placement + mesh BoundBox center (ground truth from FCStd)
    2. URDF: Pendulum_Link's servo visual (search by mesh filename) origin
       + pendulum_pivot_joint's origin+rpy (compose to global)
    """
    urdf = load_urdf()
    joint_cfg = load_joint_config()
    body_wheels = json.loads(METADATA_JSON.read_text())

    # Ground truth from live FCStd
    gt_pt = fcstd_ground_truth.get("STS3032_Mount")
    if not gt_pt:
        pytest.skip("STS3032_Mount not found in FCStd")
    gt_xyz = [gt_pt["x"], gt_pt["y"], gt_pt["z"]]

    # URDF composition:
    # Step 1: Find servo mesh visual in Pendulum_Link
    link_elem = urdf.find(".//link[@name='Pendulum_Link']")
    assert link_elem is not None, "Pendulum_Link not found in URDF"

    servo_visual = None
    for visual in link_elem.findall('visual'):
        geometry = visual.find('geometry')
        if geometry is not None:
            mesh = geometry.find('mesh')
            if mesh is not None and 'feetech-STS3032-visual.stl' in mesh.get('filename', ''):
                servo_visual = visual
                break
    assert servo_visual is not None, "Servo mesh visual not found in Pendulum_Link"

    servo_origin_elem = servo_visual.find('origin')
    assert servo_origin_elem is not None, "origin not found in servo visual"
    servo_xyz_m = _parse_xyz(servo_origin_elem.get('xyz'))

    # Step 2: Compose with pendulum_pivot_joint origin + rpy
    joint_elem = urdf.find(".//joint[@name='pendulum_pivot_joint']")
    assert joint_elem is not None, "pendulum_pivot_joint not found in URDF"

    joint_origin_elem = joint_elem.find('origin')
    assert joint_origin_elem is not None, "origin not found in pendulum_pivot_joint"
    joint_xyz_m = _parse_xyz(joint_origin_elem.get('xyz'))
    joint_rpy = _parse_xyz(joint_origin_elem.get('rpy', '0 0 0'))

    # Convert rpy (radians) to ypr (degrees) for _apply_rotation_ypr
    import math
    yaw_deg = math.degrees(joint_rpy[2])
    pitch_deg = math.degrees(joint_rpy[1])
    roll_deg = math.degrees(joint_rpy[0])

    # Apply joint rotation to servo position
    # servo_xyz_m is in meters (from URDF), convert to mm for rotation
    servo_xyz_mm = [v * 1000 for v in servo_xyz_m]
    servo_rotated = _apply_rotation_ypr(servo_xyz_mm, yaw_deg, pitch_deg, roll_deg)

    # Final position = joint_xyz + servo_rotated
    joint_xyz_mm = [v * 1000 for v in joint_xyz_m]
    final_xyz_mm = [
        joint_xyz_mm[0] + servo_rotated[0],
        joint_xyz_mm[1] + servo_rotated[1],
        joint_xyz_mm[2] + servo_rotated[2],
    ]
    final_xyz_m = [v / 1000 for v in final_xyz_mm]

    # Compare
    for axis, gt_val, urdf_val in zip(['x', 'y', 'z'], gt_xyz, final_xyz_m):
        gt_m = gt_val / 1000.0
        diff = abs(gt_m - urdf_val)
        assert diff < TOLERANCE_M, (
            f"STS3032_Mount {axis}: ground truth={gt_m:.6f} m, "
            f"URDF={urdf_val:.6f} m, diff={diff:.2e} m, "
            f"composition: servo_xyz={servo_xyz_m}, joint_xyz={joint_xyz_m}, joint_rpy={joint_rpy}"
        )


@pytest.mark.skipif(FREECAD_BIN is None, reason=skip_reason_bin)
@pytest.mark.skipif(not FCSTD_FILE.is_file(), reason=skip_reason_fcstd)
@pytest.mark.skipif(not URDF_FILE.is_file(), reason=skip_reason_urdf)
def test_sts3032_mount_right_global_position(fcstd_ground_truth):
    """Test STS3032_Mount_Right: servo mesh center vs URDF Pendulum_Link_Right composition."""
    urdf = load_urdf()
    body_wheels = json.loads(METADATA_JSON.read_text())

    # Ground truth from live FCStd
    gt_pt = fcstd_ground_truth.get("STS3032_Mount_Right")
    if not gt_pt:
        pytest.skip("STS3032_Mount_Right not found in FCStd")
    gt_xyz = [gt_pt["x"], gt_pt["y"], gt_pt["z"]]

    # URDF composition: same pattern as left but for Pendulum_Link_Right
    link_elem = urdf.find(".//link[@name='Pendulum_Link_Right']")
    assert link_elem is not None, "Pendulum_Link_Right not found in URDF"

    servo_visual = None
    for visual in link_elem.findall('visual'):
        geometry = visual.find('geometry')
        if geometry is not None:
            mesh = geometry.find('mesh')
            if mesh is not None and 'feetech-STS3032-visual.stl' in mesh.get('filename', ''):
                servo_visual = visual
                break
    assert servo_visual is not None, "Servo mesh visual not found in Pendulum_Link_Right"

    servo_origin_elem = servo_visual.find('origin')
    assert servo_origin_elem is not None, "origin not found in servo visual"
    servo_xyz_m = _parse_xyz(servo_origin_elem.get('xyz'))

    joint_elem = urdf.find(".//joint[@name='pendulum_pivot_right_joint']")
    assert joint_elem is not None, "pendulum_pivot_right_joint not found in URDF"

    joint_origin_elem = joint_elem.find('origin')
    assert joint_origin_elem is not None, "origin not found in pendulum_pivot_right_joint"
    joint_xyz_m = _parse_xyz(joint_origin_elem.get('xyz'))
    joint_rpy = _parse_xyz(joint_origin_elem.get('rpy', '0 0 0'))

    import math
    yaw_deg = math.degrees(joint_rpy[2])
    pitch_deg = math.degrees(joint_rpy[1])
    roll_deg = math.degrees(joint_rpy[0])

    servo_xyz_mm = [v * 1000 for v in servo_xyz_m]
    servo_rotated = _apply_rotation_ypr(servo_xyz_mm, yaw_deg, pitch_deg, roll_deg)

    joint_xyz_mm = [v * 1000 for v in joint_xyz_m]
    final_xyz_mm = [
        joint_xyz_mm[0] + servo_rotated[0],
        joint_xyz_mm[1] + servo_rotated[1],
        joint_xyz_mm[2] + servo_rotated[2],
    ]
    final_xyz_m = [v / 1000 for v in final_xyz_mm]

    # Compare
    for axis, gt_val, urdf_val in zip(['x', 'y', 'z'], gt_xyz, final_xyz_m):
        gt_m = gt_val / 1000.0
        diff = abs(gt_m - urdf_val)
        assert diff < TOLERANCE_M, (
            f"STS3032_Mount_Right {axis}: ground truth={gt_m:.6f} m, "
            f"URDF={urdf_val:.6f} m, diff={diff:.2e} m"
        )


@pytest.mark.skipif(FREECAD_BIN is None, reason=skip_reason_bin)
@pytest.mark.skipif(not FCSTD_FILE.is_file(), reason=skip_reason_fcstd)
@pytest.mark.skipif(not URDF_FILE.is_file(), reason=skip_reason_urdf)
def test_platestack_global_position(fcstd_ground_truth):
    """Test PlateStack (Bottom_Plate): local center placed globally vs URDF composition.

    Composition:
    1. Ground truth: Bottom_Plate's local BoundBox center, globally placed
    2. URDF: Pendulum_Link's first plate visual (index from metadata list)
       + pendulum_pivot_joint composition
    """
    urdf = load_urdf()
    body_wheels = json.loads(METADATA_JSON.read_text())

    # Ground truth from live FCStd
    gt_pt = fcstd_ground_truth.get("PlateStack")
    if not gt_pt:
        pytest.skip("PlateStack (Bottom_Plate) not found in FCStd")
    gt_xyz = [gt_pt["x"], gt_pt["y"], gt_pt["z"]]

    # URDF composition:
    # Step 1: Find plate index in metadata's Pendulum_Link.plate_shapes
    link_data = body_wheels['links']['Pendulum_Link']
    plate_shapes = link_data.get('plate_shapes', [])
    bottom_plate_idx = None
    for idx, plate in enumerate(plate_shapes):
        if plate['name'] == 'Bottom_Plate':
            bottom_plate_idx = idx
            break

    assert bottom_plate_idx is not None, "Bottom_Plate not found in Pendulum_Link's plate_shapes"

    # Step 2: Get URDF Pendulum_Link's first visual (bottom plate, index matches metadata order)
    link_elem = urdf.find(".//link[@name='Pendulum_Link']")
    assert link_elem is not None, "Pendulum_Link not found in URDF"

    visual_elems = link_elem.findall('visual')
    assert len(visual_elems) > bottom_plate_idx, (
        f"Expected at least {bottom_plate_idx + 1} visuals in Pendulum_Link, found {len(visual_elems)}"
    )

    plate_visual = visual_elems[bottom_plate_idx]
    plate_origin_elem = plate_visual.find('origin')
    assert plate_origin_elem is not None, "origin not found in plate visual"
    plate_xyz_m = _parse_xyz(plate_origin_elem.get('xyz'))

    # Step 3: Compose with pendulum_pivot_joint
    joint_elem = urdf.find(".//joint[@name='pendulum_pivot_joint']")
    assert joint_elem is not None, "pendulum_pivot_joint not found"

    joint_origin_elem = joint_elem.find('origin')
    assert joint_origin_elem is not None, "origin not found in pendulum_pivot_joint"
    joint_xyz_m = _parse_xyz(joint_origin_elem.get('xyz'))
    joint_rpy = _parse_xyz(joint_origin_elem.get('rpy', '0 0 0'))

    import math
    yaw_deg = math.degrees(joint_rpy[2])
    pitch_deg = math.degrees(joint_rpy[1])
    roll_deg = math.degrees(joint_rpy[0])

    plate_xyz_mm = [v * 1000 for v in plate_xyz_m]
    plate_rotated = _apply_rotation_ypr(plate_xyz_mm, yaw_deg, pitch_deg, roll_deg)

    joint_xyz_mm = [v * 1000 for v in joint_xyz_m]
    final_xyz_mm = [
        joint_xyz_mm[0] + plate_rotated[0],
        joint_xyz_mm[1] + plate_rotated[1],
        joint_xyz_mm[2] + plate_rotated[2],
    ]
    final_xyz_m = [v / 1000 for v in final_xyz_mm]

    # Compare
    for axis, gt_val, urdf_val in zip(['x', 'y', 'z'], gt_xyz, final_xyz_m):
        gt_m = gt_val / 1000.0
        diff = abs(gt_m - urdf_val)
        assert diff < TOLERANCE_M, (
            f"PlateStack {axis}: ground truth={gt_m:.6f} m, "
            f"URDF={urdf_val:.6f} m, diff={diff:.2e} m, "
            f"plate_idx={bottom_plate_idx}, composition: plate_xyz={plate_xyz_m}, joint={joint_xyz_m}"
        )


@pytest.mark.skipif(FREECAD_BIN is None, reason=skip_reason_bin)
@pytest.mark.skipif(not FCSTD_FILE.is_file(), reason=skip_reason_fcstd)
@pytest.mark.skipif(not URDF_FILE.is_file(), reason=skip_reason_urdf)
def test_platestack_right_global_position(fcstd_ground_truth):
    """Test PlateStack_Right: same as PlateStack but for right side."""
    urdf = load_urdf()
    body_wheels = json.loads(METADATA_JSON.read_text())

    # Ground truth from live FCStd
    gt_pt = fcstd_ground_truth.get("PlateStack_Right")
    if not gt_pt:
        pytest.skip("PlateStack_Right (Bottom_Plate_Right) not found in FCStd")
    gt_xyz = [gt_pt["x"], gt_pt["y"], gt_pt["z"]]

    # Find plate index for Bottom_Plate_Right
    link_data = body_wheels['links']['Pendulum_Link_Right']
    plate_shapes = link_data.get('plate_shapes', [])
    bottom_plate_idx = None
    for idx, plate in enumerate(plate_shapes):
        if plate['name'] == 'Bottom_Plate_Right':
            bottom_plate_idx = idx
            break

    assert bottom_plate_idx is not None, "Bottom_Plate_Right not found in Pendulum_Link_Right's plate_shapes"

    # Get URDF Pendulum_Link_Right
    link_elem = urdf.find(".//link[@name='Pendulum_Link_Right']")
    assert link_elem is not None, "Pendulum_Link_Right not found in URDF"

    visual_elems = link_elem.findall('visual')
    assert len(visual_elems) > bottom_plate_idx, (
        f"Expected at least {bottom_plate_idx + 1} visuals in Pendulum_Link_Right, "
        f"found {len(visual_elems)}"
    )

    plate_visual = visual_elems[bottom_plate_idx]
    plate_origin_elem = plate_visual.find('origin')
    assert plate_origin_elem is not None, "origin not found in plate visual"
    plate_xyz_m = _parse_xyz(plate_origin_elem.get('xyz'))

    joint_elem = urdf.find(".//joint[@name='pendulum_pivot_right_joint']")
    assert joint_elem is not None, "pendulum_pivot_right_joint not found"

    joint_origin_elem = joint_elem.find('origin')
    assert joint_origin_elem is not None, "origin not found in joint"
    joint_xyz_m = _parse_xyz(joint_origin_elem.get('xyz'))
    joint_rpy = _parse_xyz(joint_origin_elem.get('rpy', '0 0 0'))

    import math
    yaw_deg = math.degrees(joint_rpy[2])
    pitch_deg = math.degrees(joint_rpy[1])
    roll_deg = math.degrees(joint_rpy[0])

    plate_xyz_mm = [v * 1000 for v in plate_xyz_m]
    plate_rotated = _apply_rotation_ypr(plate_xyz_mm, yaw_deg, pitch_deg, roll_deg)

    joint_xyz_mm = [v * 1000 for v in joint_xyz_m]
    final_xyz_mm = [
        joint_xyz_mm[0] + plate_rotated[0],
        joint_xyz_mm[1] + plate_rotated[1],
        joint_xyz_mm[2] + plate_rotated[2],
    ]
    final_xyz_m = [v / 1000 for v in final_xyz_mm]

    # Compare
    for axis, gt_val, urdf_val in zip(['x', 'y', 'z'], gt_xyz, final_xyz_m):
        gt_m = gt_val / 1000.0
        diff = abs(gt_m - urdf_val)
        assert diff < TOLERANCE_M, (
            f"PlateStack_Right {axis}: ground truth={gt_m:.6f} m, "
            f"URDF={urdf_val:.6f} m, diff={diff:.2e} m"
        )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
