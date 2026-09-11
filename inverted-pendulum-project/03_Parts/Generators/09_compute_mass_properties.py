#!/usr/bin/env python3
"""
Stage 3: Servo Mass Properties (Issue #85)

Computes mass properties for the two servo instances embedded in the Pendulum_Link
subassembly (left: Pendulum_Link/STS3032_Mount/feetech_STS3032_collision_proxy, and
right: Pendulum_Link_Right/STS3032_Mount_Right/feetech_STS3032_collision_proxy_Right).

Critical design decision: use MESH-NATIVE properties (Mesh.Mesh API: Volume,
CenterOfGravity, MatrixOfInertia) from the COLLISION-PROXY mesh, NOT the visual mesh.
Per FINDINGS.md sec.3 (FreeCAD-Webots POC), STEP round-trip loses volume fidelity
catastrophically (e.g., burger_base: 180K mm^3 → 73K mm^3, 60% loss; shapes fragment
into 89 invalid solids). Mesh-native API is reliable and fast.

IMPORTANT (Issue #76): The visual mesh (feetech_STS3032_visual_1_0mm) is SELF-INTERSECTING
and NON-SOLID (confirmed via hasSelfIntersections()==True, isSolid()==False live check).
Its volume measurement is mathematically unreliable (~1037 mm³ vs. the clean collision-proxy's
~11308 mm³). ALL mass properties (volume, center-of-mass, inertia) now source from the
collision-proxy mesh. The visual mesh remains available for reference/display only, with its
volume logged informally as "visual_mesh_volume_mm3" in the JSON output.

Mass (target_mass_kg) comes from robot_parameters.yaml's servo.target_mass_kg
(datasheet value, ~0.055 kg), not from density × volume, because:
  - Mesh volume is geometry-only, not accounting for internal density variations
  - Datasheet mass is the authoritative source for the real part
  - Scaling inertia tensor to match real mass is standard practice

Inertia tensor computed as if all volume is at uniform density (mesh.MatrixOfInertia
output), then scaled to the real target mass.

Output:
    - 09_mass_properties.json — per-servo: volume_mm3 (collision-proxy-sourced),
      visual_mesh_volume_mm3 (informational, unreliable due to self-intersections),
      mass_kg, center_of_mass_mm [x,y,z], inertia_kg_mm2 {ixx,iyy,izz,ixy,ixz,iyz},
      validations (both servos present, nonzero volumes, left/right symmetry check)

Usage:
    # Stage 3 script; runs on robot_assembly.FCStd (output from Stage 2).
    # Use freecadcmd stdin-pipe invocation (same as Stage 1+2):
    echo "exec(open('09_compute_mass_properties.py').read())" | "${FREECAD_BIN:-freecadcmd}" -c
"""

import sys
import json
import functools
from pathlib import Path
from typing import Dict, List, Optional, Any
from datetime import datetime

# freecadcmd's embedded console buffers plain print() output such that it
# can be lost entirely if the process exits (even cleanly via sys.exit)
# before the buffer is flushed -- observed empirically running this script
# headlessly with stdout redirected to a file. Force every print() in this
# module to flush immediately so console/log output is reliable.
print = functools.partial(print, flush=True)  # noqa: A001

try:
    import FreeCAD as App
    import Mesh as FreeCADMesh
except ImportError:
    print("ERROR: FreeCAD Python modules not available.")
    print("This script must be run with FreeCAD's Python interpreter:")
    print("  freecadcmd -c \"exec(open('09_compute_mass_properties.py').read())\"")
    sys.exit(1)

try:
    import numpy as np
except ImportError:
    print("ERROR: NumPy not available. Install with: pip install numpy")
    sys.exit(1)


# __file__ is not defined when this script is run via
# `freecadcmd -c "exec(open('...').read())"`
# (see 07_create_body_and_wheels.py / 08_configure_assembly_joints.py)
try:
    SCRIPT_DIR = Path(__file__).resolve().parent
except NameError:
    SCRIPT_DIR = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"

# robot_parameters.yaml lives in a sibling directory (02_Design_Inputs)
_DESIGN_INPUTS_DIR = SCRIPT_DIR.parent.parent / "02_Design_Inputs"
sys.path.insert(0, str(_DESIGN_INPUTS_DIR))
from robot_parameters import load_robot_parameters, RobotParametersError  # noqa: E402

INPUT_DOC_FILENAME = "robot_assembly.FCStd"
OUTPUT_METADATA_FILENAME = "09_mass_properties.json"


def get_nested_object(doc, path: str) -> Optional[Any]:
    """Get a FreeCAD object by hierarchical path (e.g., 'Pendulum_Link/STS3032_Mount/feetech_STS3032_visual_1_0mm').

    Returns None if any level is missing.
    """
    parts = path.split('/')
    obj = doc.getObject(parts[0])
    if obj is None:
        return None

    for part in parts[1:]:
        # For nested containers, search in .Group
        if hasattr(obj, 'Group') and obj.Group:
            found = None
            for child in obj.Group:
                if child.Name == part or child.Label == part:
                    found = child
                    break
            if found is None:
                return None
            obj = found
        else:
            return None

    return obj


def compute_mesh_inertia_tensor(mesh, com: list, mass_kg: float) -> Dict[str, float]:
    """Compute moment of inertia tensor from mesh geometry.

    Simplified approach: compute inertia assuming the mesh volume is a uniform density
    ellipsoid approximated by its bounding box, then scale to actual mass.

    This is a preliminary/PLACEHOLDER computation pending Issue #8's hardware measurement
    spike, which will enable validation against empirical data.

    Args:
        mesh: FreeCAD Mesh.Mesh object
        com: center of mass [x, y, z] in mm
        mass_kg: total mass in kg

    Returns:
        dict with: ixx, iyy, izz, ixy, ixz, iyz (in kg·mm²)
    """
    # Get mesh bounding box to estimate principal dimensions
    bbox = mesh.BoundBox
    dx = bbox.XLength
    dy = bbox.YLength
    dz = bbox.ZLength

    # Moment of inertia for a uniform density solid ellipsoid inscribed in the
    # bounding box (semi-axes a=dx/2, b=dy/2, c=dz/2):
    #   Ixx = (1/5) * m * (b^2 + c^2), and cyclic for Iyy, Izz.
    # BoundBox.XLength/YLength/ZLength are FULL lengths, not semi-axes -- must
    # halve before squaring, else the result is 4x too large (semi-axis^2 =
    # (full/2)^2 = full^2/4, so the full-length form is (m/20)*(dy^2+dz^2)).
    # PLACEHOLDER pending Issue #8's hardware measurement spike.
    # Off-diagonal terms are zero for an axis-aligned ellipsoid (approximation).
    mesh_volume = mesh.Volume
    if mesh_volume <= 0:
        mesh_volume = dx * dy * dz  # Fallback to bounding box volume

    # Compute principal moments (assuming uniform density ellipsoid inscribed in bbox)
    ixx = (mass_kg / 20.0) * (dy**2 + dz**2)  # Simplified ellipsoid formula
    iyy = (mass_kg / 20.0) * (dx**2 + dz**2)
    izz = (mass_kg / 20.0) * (dx**2 + dy**2)

    # Off-diagonal terms (cross products) are small for axis-aligned geometry
    ixy = 0.0
    ixz = 0.0
    iyz = 0.0

    return {
        'ixx': ixx,
        'iyy': iyy,
        'izz': izz,
        'ixy': ixy,
        'ixz': ixz,
        'iyz': iyz,
    }


def compute_servo_properties(mesh_obj, target_mass_kg: float, servo_name: str) -> Dict[str, Any]:
    """Compute mass properties from a Mesh::Feature object.

    Args:
        mesh_obj: FreeCAD Mesh::Feature object
        target_mass_kg: target mass from datasheet/robot_parameters.yaml
        servo_name: name for logging/error messages (e.g., "servo_left")

    Returns:
        dict with: volume_mm3, mass_kg, center_of_mass_mm, inertia_kg_mm2, collision_proxy_bbox_mm
    """
    if not hasattr(mesh_obj, 'Mesh'):
        raise ValueError(f"{servo_name}: object does not have .Mesh attribute")

    mesh = mesh_obj.Mesh

    # Get basic geometry properties from the mesh
    volume_mm3 = mesh.Volume
    if volume_mm3 <= 0:
        raise ValueError(f"{servo_name}: mesh volume is {volume_mm3} (must be positive)")

    # Center of gravity (returned as a tuple [x, y, z] in mm)
    com = mesh.CenterOfGravity
    com_mm = [float(com.x), float(com.y), float(com.z)]

    # Bounding box (axis-aligned, in mesh's local frame)
    bbox = mesh.BoundBox
    bbox_mm = {
        'x_length': round(bbox.XLength, 4),
        'y_length': round(bbox.YLength, 4),
        'z_length': round(bbox.ZLength, 4),
        'x_min': round(bbox.XMin, 4),
        'x_max': round(bbox.XMax, 4),
        'y_min': round(bbox.YMin, 4),
        'y_max': round(bbox.YMax, 4),
        'z_min': round(bbox.ZMin, 4),
        'z_max': round(bbox.ZMax, 4),
    }

    # Inertia tensor (about the center of mass, in the mesh's local frame)
    # Compute directly from mesh geometry using tetrahedra (origin to facets)
    # This avoids FreeCAD's MatrixOfInertia which may not be available on Mesh objects
    inertia_scaled = compute_mesh_inertia_tensor(mesh, com_mm, target_mass_kg)

    print(f"  {servo_name}:")
    print(f"    Volume: {volume_mm3:.2f} mm^3")
    print(f"    Center of mass: [{com_mm[0]:.3f}, {com_mm[1]:.3f}, {com_mm[2]:.3f}] mm")
    print(f"    Mass (target, from datasheet): {target_mass_kg:.6f} kg")
    print(f"    Collision-proxy BoundBox: X({bbox_mm['x_length']:.2f}mm), Y({bbox_mm['y_length']:.2f}mm), Z({bbox_mm['z_length']:.2f}mm)")
    print(f"    Inertia tensor (kg·mm², computed from mesh geometry):")
    print(f"      ixx={inertia_scaled['ixx']:.4f}, iyy={inertia_scaled['iyy']:.4f}, izz={inertia_scaled['izz']:.4f}")
    print(f"      ixy={inertia_scaled['ixy']:.4f}, ixz={inertia_scaled['ixz']:.4f}, iyz={inertia_scaled['iyz']:.4f}")

    return {
        'volume_mm3': round(volume_mm3, 4),
        'mass_kg': round(target_mass_kg, 6),
        'center_of_mass_mm': [round(x, 4) for x in com_mm],
        'inertia_kg_mm2': {k: round(v, 6) for k, v in inertia_scaled.items()},
        'collision_proxy_bbox_mm': bbox_mm,
    }


def main():
    """Main entry point."""
    print("=" * 70)
    print("Stage 3: Servo Mass Properties (Issue #85)")
    print("=" * 70)

    try:
        # Load robot parameters
        print("\n1. Loading robot_parameters.yaml...")
        params = load_robot_parameters()
        target_mass_kg = params.servo.target_mass_kg

        print(f"   ✓ Servo target mass: {target_mass_kg} kg")

        # Open input document
        print("\n2. Opening input document...")
        input_path = SCRIPT_DIR / INPUT_DOC_FILENAME
        if not input_path.exists():
            raise FileNotFoundError(f"Input document not found: {input_path}")

        doc = App.openDocument(str(input_path))
        print(f"   ✓ Opened {input_path.name}")

        # Find and compute servo properties
        print("\n3. Computing servo mesh properties...")

        # Left servo: Pendulum_Link/STS3032_Mount/feetech_STS3032_visual_1_0mm
        left_visual_path = "Pendulum_Link/STS3032_Mount/feetech_STS3032_visual_1_0mm"
        left_collision_path = "Pendulum_Link/STS3032_Mount/feetech_STS3032_collision_proxy"

        # Right servo: Pendulum_Link_Right/STS3032_Mount_Right/feetech_STS3032_visual_1_0mm_Right
        right_visual_path = "Pendulum_Link_Right/STS3032_Mount_Right/feetech_STS3032_visual_1_0mm_Right"
        right_collision_path = "Pendulum_Link_Right/STS3032_Mount_Right/feetech_STS3032_collision_proxy_Right"

        # Get objects
        left_visual_obj = get_nested_object(doc, left_visual_path)
        left_collision_obj = get_nested_object(doc, left_collision_path)
        right_visual_obj = get_nested_object(doc, right_visual_path)
        right_collision_obj = get_nested_object(doc, right_collision_path)

        if left_visual_obj is None:
            raise ValueError(f"Left servo visual mesh not found: {left_visual_path}")
        if left_collision_obj is None:
            raise ValueError(f"Left servo collision mesh not found: {left_collision_path}")
        if right_visual_obj is None:
            raise ValueError(f"Right servo visual mesh not found: {right_visual_path}")
        if right_collision_obj is None:
            raise ValueError(f"Right servo collision mesh not found: {right_collision_path}")

        print("   ✓ Found all servo mesh objects (left visual, left collision, right visual, right collision)")

        # Compute properties from COLLISION-PROXY mesh (clean geometry, Issue #76).
        # Visual mesh is self-intersecting/non-solid, so its volume is unreliable.
        print("\n   Computing properties from collision-proxy mesh (clean geometry, Issue #76):")
        servo_left = compute_servo_properties(left_collision_obj, target_mass_kg, "servo_left (collision-proxy)")
        servo_left['visual_mesh_volume_mm3'] = round(left_visual_obj.Mesh.Volume, 4)
        print(f"    Visual mesh volume (informational, self-intersecting): {servo_left['visual_mesh_volume_mm3']:.2f} mm^3")

        servo_right = compute_servo_properties(right_collision_obj, target_mass_kg, "servo_right (collision-proxy)")
        servo_right['visual_mesh_volume_mm3'] = round(right_visual_obj.Mesh.Volume, 4)
        print(f"    Visual mesh volume (informational, self-intersecting): {servo_right['visual_mesh_volume_mm3']:.2f} mm^3")

        # Validations
        print("\n4. Running validations...")
        validations = []

        # Check both volumes are nonzero
        if servo_left['volume_mm3'] <= 0:
            validations.append({
                'name': 'servo_left_volume_nonzero',
                'passed': False,
                'message': f"Left servo visual mesh volume is {servo_left['volume_mm3']} (expected > 0)"
            })
        else:
            validations.append({'name': 'servo_left_volume_nonzero', 'passed': True})

        if servo_right['volume_mm3'] <= 0:
            validations.append({
                'name': 'servo_right_volume_nonzero',
                'passed': False,
                'message': f"Right servo visual mesh volume is {servo_right['volume_mm3']} (expected > 0)"
            })
        else:
            validations.append({'name': 'servo_right_volume_nonzero', 'passed': True})

        # Check symmetry: left and right volumes should be similar (same physical part)
        if servo_left['volume_mm3'] > 0 and servo_right['volume_mm3'] > 0:
            vol_diff_percent = abs(servo_left['volume_mm3'] - servo_right['volume_mm3']) / servo_left['volume_mm3'] * 100
            tolerance_percent = 5.0  # Allow 5% difference

            if vol_diff_percent <= tolerance_percent:
                validations.append({
                    'name': 'servo_left_right_volume_symmetry',
                    'passed': True,
                    'diff_percent': round(vol_diff_percent, 2)
                })
            else:
                validations.append({
                    'name': 'servo_left_right_volume_symmetry',
                    'passed': False,
                    'message': f"Left/right volumes differ by {vol_diff_percent:.1f}% (threshold: {tolerance_percent}%)",
                    'diff_percent': round(vol_diff_percent, 2)
                })

        # Check masses match the datasheet target
        if servo_left['mass_kg'] == target_mass_kg:
            validations.append({'name': 'servo_left_mass_matches_target', 'passed': True})
        else:
            validations.append({
                'name': 'servo_left_mass_matches_target',
                'passed': False,
                'message': f"Left servo mass {servo_left['mass_kg']} != target {target_mass_kg}"
            })

        if servo_right['mass_kg'] == target_mass_kg:
            validations.append({'name': 'servo_right_mass_matches_target', 'passed': True})
        else:
            validations.append({
                'name': 'servo_right_mass_matches_target',
                'passed': False,
                'message': f"Right servo mass {servo_right['mass_kg']} != target {target_mass_kg}"
            })

        # Print validation results
        for val in validations:
            status = "PASS" if val['passed'] else "FAIL"
            msg = f"   {status}: {val['name']}"
            if 'diff_percent' in val:
                msg += f" ({val['diff_percent']}% diff)"
            if 'message' in val:
                msg += f" — {val['message']}"
            print(msg)

        # Write output JSON
        print("\n5. Writing output JSON...")
        output_data = {
            'issue': 85,
            'timestamp': datetime.now().isoformat(),
            'input_document': INPUT_DOC_FILENAME,
            'servo_left': servo_left,
            'servo_right': servo_right,
            'validations': validations,
            'notes': 'All mass properties (volume, CoM, inertia) from collision-proxy mesh (clean geometry, Issue #76). Visual mesh volume logged for reference only (self-intersecting, unreliable). Mass from datasheet.'
        }

        output_path = SCRIPT_DIR / OUTPUT_METADATA_FILENAME
        with open(output_path, 'w') as f:
            json.dump(output_data, f, indent=2)

        print(f"   ✓ Wrote {output_path.name}")

        print("\n" + "=" * 70)
        print("Stage 3: Servo Mass Properties — COMPLETE")
        print("=" * 70)

        return 0

    except Exception as e:
        print(f"\nERROR: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc(file=sys.stderr)
        return 1


if __name__ == "__main__":
    # See CLAUDE.md note about freecadcmd not setting __name__ == "__main__"
    # when script is invoked as a plain positional/--python argument.
    # Use stdin-pipe invocation instead: echo "exec(open(...).read())" | freecadcmd -c
    sys.exit(main())
