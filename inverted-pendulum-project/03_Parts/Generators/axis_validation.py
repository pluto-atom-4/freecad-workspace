#!/usr/bin/env python3
"""
Shared axis validation module for URDF export.

Provides:
  - compose_axis(): Compute expected axis in local frame from global axis + RPY rotation
  - validate_joint_axes(): Validate all joint axes in URDF (norm + direction)

Used by:
  - 10_export_urdf.py: Post-write validation (in-situ)
  - 12_validate_urdf_export.py: Phase 5 comprehensive validation

This module is pure Python (no FreeCAD imports) and depends only on:
  - 10_export_urdf.ypr_deg_to_rotation_matrix() for rotation math
  - Standard library: xml.etree.ElementTree, math, json
"""

import sys
import math
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Any

# Script directory resolution
try:
    SCRIPT_DIR = Path(__file__).resolve().parent
except NameError:
    SCRIPT_DIR = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"

# Add script dir to path for importing sibling modules
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))


def compose_axis(axis_global: List[float], rotation_ypr_deg: Dict[str, float]) -> Optional[List[float]]:
    """Compose global axis with inverse rotation to express in local/child frame.

    The rotation matrix transforms from child→parent, so its inverse transforms
    parent→child. Apply R^-1 (= R^T for orthonormal matrices) to the global axis.

    Uses ypr_deg_to_rotation_matrix() from 10_export_urdf.py (imported to avoid
    duplication and ensure consistency across the codebase).

    Args:
        axis_global: Global axis as [x, y, z]
        rotation_ypr_deg: Dict with keys 'yaw', 'pitch', 'roll' (in degrees)

    Returns:
        Composed and normalized axis, or None if norm too small
    """
    # Import here to avoid circular dependency and maintain module independence
    from importlib import import_module
    exporter = import_module('10_export_urdf')

    yaw_deg = rotation_ypr_deg.get('yaw', 0.0)
    pitch_deg = rotation_ypr_deg.get('pitch', 0.0)
    roll_deg = rotation_ypr_deg.get('roll', 0.0)

    R = exporter.ypr_deg_to_rotation_matrix(yaw_deg, pitch_deg, roll_deg)

    # Convert numpy array to list if needed (10_export_urdf.py returns numpy array)
    if hasattr(R, 'tolist'):
        R = R.tolist()

    # R_inv = R^T for orthonormal matrix
    R_inv = [[R[j][i] for j in range(3)] for i in range(3)]

    # Apply R_inv to axis_global
    axis_local = [0, 0, 0]
    for i in range(3):
        for j in range(3):
            axis_local[i] += R_inv[i][j] * axis_global[j]

    # Normalize
    axis_norm = math.sqrt(sum(x*x for x in axis_local))
    if axis_norm > 1e-9:
        return [x / axis_norm for x in axis_local]
    else:
        return None


def validate_joint_axes(
    urdf_root: ET.Element,
    joint_config_dict: Dict[str, Any],
    tolerance: float = 1e-3
) -> Dict[str, Any]:
    """Validate that joint axes are unit vectors and have correct direction.

    Checks:
      1. Axis norm is approximately 1.0 (unit vector check)
      2. Axis direction matches the expected composed value computed from the
         global axis and RPY rotation (if joint_config provided).

    Args:
        urdf_root: Parsed URDF XML root element
        joint_config_dict: Joint configuration dict with 'joints' key mapping
                          {joint_name: {axis_global, rotation_ypr_deg, ...}}
        tolerance: Validation tolerance for norm/direction (default 1e-3)

    Returns:
        Structured result dict with keys:
            - passed: bool (True if all checks passed)
            - checks: list of check results
            - errors: list of error details
    """
    errors = []
    checks = []

    has_direction_check = joint_config_dict is not None and 'joints' in joint_config_dict

    for joint in urdf_root.findall('joint'):
        joint_name = joint.get('name', 'unknown')
        axis_elem = joint.find('axis')

        if axis_elem is None:
            continue

        xyz_str = axis_elem.get('xyz', '')
        try:
            axis = [float(x) for x in xyz_str.split()]
            if len(axis) != 3:
                errors.append({
                    'joint': joint_name,
                    'axis': axis,
                    'message': f'Axis has {len(axis)} components, expected 3'
                })
                continue

            # Check 1: Unit vector norm
            norm = math.sqrt(sum(x*x for x in axis))
            if abs(norm - 1.0) > tolerance:
                errors.append({
                    'joint': joint_name,
                    'axis': axis,
                    'norm': norm,
                    'check_type': 'norm',
                    'message': f'Axis {axis} has norm {norm:.6f}, expected ~1.0'
                })

            # Check 2: Axis direction (if joint_config available)
            if has_direction_check:
                joint_data = joint_config_dict['joints'].get(joint_name)
                if joint_data:
                    # Extract axis_global from joint_config
                    # Can be either a separate 'axis_global' field or embedded in 'axis' string
                    axis_global = joint_data.get('axis_global')
                    if axis_global is None:
                        # Fall back to parsing from 'axis' string if available
                        # Format: "global Y (0,1,0)" or similar
                        axis_str = joint_data.get('axis', '')
                        try:
                            axis_part = axis_str[axis_str.find('(') + 1:axis_str.find(')')]
                            axis_global = [float(x.strip()) for x in axis_part.split(',')]
                        except (ValueError, IndexError):
                            axis_global = [0, 1, 0]  # Default fallback

                    rotation_ypr_deg = joint_data.get('rotation_ypr_deg', {
                        'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0
                    })

                    expected_axis = compose_axis(axis_global, rotation_ypr_deg)
                    if expected_axis is not None:
                        # Compare with computed direction
                        direction_error = math.sqrt(sum((a - e)**2 for a, e in zip(axis, expected_axis)))
                        if direction_error > tolerance:
                            errors.append({
                                'joint': joint_name,
                                'axis': axis,
                                'expected_axis': expected_axis,
                                'check_type': 'direction',
                                'error': direction_error,
                                'axis_global': axis_global,
                                'rotation_ypr_deg': rotation_ypr_deg,
                                'message': f'Axis direction mismatch: {axis} vs expected {expected_axis} (error={direction_error:.6f})'
                            })

        except (ValueError, IndexError) as e:
            errors.append({
                'joint': joint_name,
                'xyz_str': xyz_str,
                'message': f'Could not parse axis: {e}'
            })

    # Build result
    if errors:
        direction_check_info = " (includes direction check)" if has_direction_check else ""
        return {
            'passed': False,
            'checks': [
                {
                    'check': 'joint_axes',
                    'passed': False,
                    'details': f'Found {len(errors)} axis validation issue(s){direction_check_info}',
                    'errors': errors,
                }
            ],
            'errors': errors,
        }
    else:
        direction_check_info = " (includes direction check)" if has_direction_check else ""
        return {
            'passed': True,
            'checks': [
                {
                    'check': 'joint_axes',
                    'passed': True,
                    'details': f'All joint axes are unit vectors{direction_check_info}',
                }
            ],
            'errors': [],
        }


if __name__ == '__main__':
    # Simple test: validate a URDF file if provided as argument
    if len(sys.argv) > 1:
        urdf_path = Path(sys.argv[1])
        if urdf_path.exists():
            tree = ET.parse(urdf_path)
            urdf_root = tree.getroot()

            # Try to load joint_config from script dir
            joint_config_file = SCRIPT_DIR / "joint_config.json"
            joint_config = None
            if joint_config_file.exists():
                try:
                    with open(joint_config_file, 'r') as f:
                        joint_config = json.load(f)
                except (json.JSONDecodeError, IOError):
                    pass

            result = validate_joint_axes(urdf_root, joint_config)
            print(f"Axis validation: {'PASSED' if result['passed'] else 'FAILED'}")
            if result['errors']:
                print(f"Errors: {len(result['errors'])}")
                for err in result['errors']:
                    print(f"  - {err.get('message', err)}")
            sys.exit(0 if result['passed'] else 1)
        else:
            print(f"ERROR: URDF file not found: {urdf_path}")
            sys.exit(1)
