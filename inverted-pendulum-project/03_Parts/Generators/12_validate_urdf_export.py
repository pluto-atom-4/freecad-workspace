#!/usr/bin/env python3
"""
Stage 5: Validate URDF Export (Issue #156)

Validates the exported URDF (robot.urdf from Stage 4) for correctness:
  - Mesh paths exist on disk
  - Unit consistency (no accidental mm→m conversions missed)
  - Link connectivity (single root, all links reachable, no cycles)
  - Joint axes are unit vectors

This script is pure Python (no FreeCAD imports) and reads only:
  - 06_Exports/urdf/robot.urdf (Stage 4 output)
  - Mesh files referenced in <mesh> elements (must exist relative to URDF dir)

Output:
  - 03_Parts/Generators/12_urdf_export_validation_report.json — detailed per-check
    results, pass/fail status, and error details.

Exit code:
  - 0: all checks passed
  - 1: one or more checks failed

Usage:
    python3 12_validate_urdf_export.py
"""

import sys
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Any, Tuple
from datetime import datetime, timezone
import math


# Script directory resolution (same pattern as Stage 3+4 scripts)
try:
    SCRIPT_DIR = Path(__file__).resolve().parent
except NameError:
    SCRIPT_DIR = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"

# Relative paths to inputs
_EXPORTS_DIR = SCRIPT_DIR.parent.parent / "06_Exports"

# File paths
URDF_FILE = _EXPORTS_DIR / "urdf" / "robot.urdf"
OUTPUT_REPORT_FILE = SCRIPT_DIR / "12_urdf_export_validation_report.json"


def validate_mesh_paths(urdf_root: ET.Element, urdf_path: Path) -> List[Dict[str, Any]]:
    """Validate that all mesh files referenced in the URDF exist on disk.

    Args:
        urdf_root: Parsed URDF XML root element
        urdf_path: Path to the URDF file (used as base for relative paths)

    Returns:
        List of validation result dicts with keys: {check, passed, details, missing_files}
    """
    missing_files = []
    checked_files = set()

    # Find all <mesh> elements
    for mesh_elem in urdf_root.findall('.//mesh'):
        filename = mesh_elem.get('filename', '')
        if not filename or filename in checked_files:
            continue
        checked_files.add(filename)

        # Handle package:// URIs: resolve relative to URDF directory
        if filename.startswith('package://'):
            # package://robot_name/path/to/mesh.stl
            # In our export, meshes are stored at urdf/meshes/...
            # Extract only the filename and subdirectory part (last components)
            # Heuristic: assume package:// URIs point to files that are actually
            # in the urdf/ directory tree (meshes/, etc.), not in a versioned subdirectory.
            # Common pattern: package://robot_name/meshes/file.stl
            # → check urdf_parent / meshes/file.stl
            parts = filename.split('/')
            if len(parts) >= 3:
                # Find the "meshes" part if present, otherwise use last two components
                try:
                    meshes_idx = parts.index('meshes')
                    # Reconstruct from meshes onward
                    relative_path = '/'.join(parts[meshes_idx:])
                    mesh_path = urdf_path.parent / relative_path
                except ValueError:
                    # No "meshes" directory, use generic fallback
                    # Use parts[-2:] if available (subdirectory + filename)
                    if len(parts) >= 4:
                        relative_path = '/'.join(parts[-2:])
                    else:
                        relative_path = parts[-1]
                    mesh_path = urdf_path.parent / relative_path
            else:
                mesh_path = urdf_path.parent / filename
        else:
            # Relative path: resolve relative to URDF directory
            mesh_path = urdf_path.parent / filename

        if not mesh_path.exists():
            missing_files.append(str(mesh_path))

    if missing_files:
        return [{
            'check': 'mesh_paths_exist',
            'passed': False,
            'details': f'Found {len(missing_files)} missing mesh file(s)',
            'missing_files': missing_files,
        }]
    else:
        return [{
            'check': 'mesh_paths_exist',
            'passed': True,
            'details': f'All {len(checked_files)} referenced mesh file(s) exist',
        }]


def validate_unit_consistency(urdf_root: ET.Element) -> List[Dict[str, Any]]:
    """Validate unit consistency by checking for suspiciously large values.

    HEURISTIC: Flag any raw numeric value in <origin xyz>, box <size>, or
    cylinder <radius>/<length> that exceeds 10 (unitless in URDF, should be
    metres for this robot). Values > 10m are physically implausible for a
    tabletop robot and likely indicate a missed mm→m conversion (Issues #105,
    #124, #125, #148).

    Args:
        urdf_root: Parsed URDF XML root element

    Returns:
        List of validation result dicts with keys: {check, passed, details, errors}
    """
    SUSPICION_THRESHOLD = 10.0  # Metres
    errors = []

    # Check <origin xyz> values
    for origin in urdf_root.findall('.//origin'):
        xyz_str = origin.get('xyz', '0 0 0')
        try:
            coords = [float(x) for x in xyz_str.split()]
            for coord in coords:
                if abs(coord) > SUSPICION_THRESHOLD:
                    parent = origin.find('..')
                    parent_name = parent.get('name', 'unknown') if parent is not None else 'unknown'
                    errors.append({
                        'element': 'origin',
                        'coordinate': coord,
                        'parent': parent_name,
                        'message': f'Coordinate {coord} exceeds {SUSPICION_THRESHOLD}m threshold'
                    })
        except (ValueError, IndexError):
            pass

    # Check box <size> values
    for box in urdf_root.findall('.//box'):
        size_str = box.get('size', '0 0 0')
        try:
            dims = [float(x) for x in size_str.split()]
            for i, dim in enumerate(dims):
                if dim > SUSPICION_THRESHOLD:
                    errors.append({
                        'element': 'box',
                        'dimension': ['x', 'y', 'z'][i],
                        'value': dim,
                        'message': f'Box dimension {dim}m exceeds {SUSPICION_THRESHOLD}m threshold'
                    })
        except (ValueError, IndexError):
            pass

    # Check cylinder <radius> and <length>
    for cyl in urdf_root.findall('.//cylinder'):
        radius = cyl.get('radius')
        length = cyl.get('length')
        try:
            if radius:
                r = float(radius)
                if r > SUSPICION_THRESHOLD:
                    errors.append({
                        'element': 'cylinder',
                        'dimension': 'radius',
                        'value': r,
                        'message': f'Cylinder radius {r}m exceeds {SUSPICION_THRESHOLD}m threshold'
                    })
            if length:
                l = float(length)
                if l > SUSPICION_THRESHOLD:
                    errors.append({
                        'element': 'cylinder',
                        'dimension': 'length',
                        'value': l,
                        'message': f'Cylinder length {l}m exceeds {SUSPICION_THRESHOLD}m threshold'
                    })
        except ValueError:
            pass

    if errors:
        return [{
            'check': 'unit_consistency',
            'passed': False,
            'details': f'Found {len(errors)} value(s) exceeding {SUSPICION_THRESHOLD}m threshold',
            'errors': errors,
        }]
    else:
        return [{
            'check': 'unit_consistency',
            'passed': True,
            'details': f'All geometric values within expected range (< {SUSPICION_THRESHOLD}m)',
        }]


def validate_link_connectivity(urdf_root: ET.Element) -> List[Dict[str, Any]]:
    """Validate URDF link connectivity: single root, all reachable, no cycles.

    Builds a parent→child graph from all <joint> elements and confirms:
      1. Exactly one link with no incoming joint (the root)
      2. All other links have exactly one parent
      3. No cycles in the graph
      4. All links are reachable from the root

    Args:
        urdf_root: Parsed URDF XML root element

    Returns:
        List of validation result dicts with keys: {check, passed, details, errors}
    """
    errors = []

    # Collect all links
    all_links = set()
    for link in urdf_root.findall('link'):
        link_name = link.get('name')
        if link_name:
            all_links.add(link_name)

    # Build parent→child and child→parent mappings from joints
    children = {}  # link_name → parent_name
    parents = {}   # link_name → [child_names]

    for joint in urdf_root.findall('joint'):
        parent_elem = joint.find('parent')
        child_elem = joint.find('child')

        if parent_elem is None or child_elem is None:
            continue

        parent_name = parent_elem.get('link')
        child_name = child_elem.get('link')

        if not parent_name or not child_name:
            continue

        # Record parent of child
        if child_name in children:
            errors.append({
                'type': 'multiple_parents',
                'child': child_name,
                'message': f'Link {child_name} has multiple parent joints'
            })
        children[child_name] = parent_name

        # Record children of parent
        if parent_name not in parents:
            parents[parent_name] = []
        parents[parent_name].append(child_name)

    # Find root links (no incoming joint)
    roots = [link for link in all_links if link not in children]

    if len(roots) != 1:
        errors.append({
            'type': 'root_count',
            'count': len(roots),
            'roots': roots,
            'message': f'Expected exactly 1 root link, found {len(roots)}'
        })
    else:
        root_link = roots[0]

        # BFS from root to find reachable links
        visited = set([root_link])
        queue = [root_link]

        while queue:
            link = queue.pop(0)
            for child in parents.get(link, []):
                if child in visited:
                    # Cycle detected
                    errors.append({
                        'type': 'cycle',
                        'link': child,
                        'message': f'Cycle detected: {child} already visited'
                    })
                else:
                    visited.add(child)
                    queue.append(child)

        # Check if all links are reachable
        unreachable = all_links - visited
        if unreachable:
            errors.append({
                'type': 'unreachable_links',
                'links': list(unreachable),
                'message': f'{len(unreachable)} link(s) unreachable from root {root_link}'
            })

    if errors:
        return [{
            'check': 'link_connectivity',
            'passed': False,
            'details': f'Found {len(errors)} connectivity issue(s)',
            'errors': errors,
        }]
    else:
        return [{
            'check': 'link_connectivity',
            'passed': True,
            'details': f'Graph is valid: single root, all {len(all_links)} link(s) reachable, no cycles',
        }]


def validate_joint_axes(urdf_root: ET.Element) -> List[Dict[str, Any]]:
    """Validate that joint axes are unit vectors (norm ≈ 1.0).

    Args:
        urdf_root: Parsed URDF XML root element

    Returns:
        List of validation result dicts with keys: {check, passed, details, errors}
    """
    TOLERANCE = 1e-3
    errors = []

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

            norm = math.sqrt(sum(x*x for x in axis))
            if abs(norm - 1.0) > TOLERANCE:
                errors.append({
                    'joint': joint_name,
                    'axis': axis,
                    'norm': norm,
                    'message': f'Axis {axis} has norm {norm:.6f}, expected ~1.0'
                })
        except (ValueError, IndexError) as e:
            errors.append({
                'joint': joint_name,
                'xyz_str': xyz_str,
                'message': f'Could not parse axis: {e}'
            })

    if errors:
        return [{
            'check': 'joint_axes',
            'passed': False,
            'details': f'Found {len(errors)} axis validation issue(s)',
            'errors': errors,
        }]
    else:
        return [{
            'check': 'joint_axes',
            'passed': True,
            'details': 'All joint axes are unit vectors',
        }]


def main():
    """Run all validators and write report."""
    if not URDF_FILE.exists():
        print(f"ERROR: URDF file not found: {URDF_FILE}")
        sys.exit(1)

    try:
        tree = ET.parse(URDF_FILE)
        urdf_root = tree.getroot()
    except ET.ParseError as e:
        print(f"ERROR: Failed to parse URDF: {e}")
        sys.exit(1)

    # Run all validators
    all_results = []
    all_results.extend(validate_mesh_paths(urdf_root, URDF_FILE))
    all_results.extend(validate_unit_consistency(urdf_root))
    all_results.extend(validate_link_connectivity(urdf_root))
    all_results.extend(validate_joint_axes(urdf_root))

    # Aggregate results
    passed_count = sum(1 for r in all_results if r['passed'])
    failed_count = len(all_results) - passed_count
    all_passed = failed_count == 0

    report = {
        'timestamp': datetime.now(timezone.utc).isoformat(),
        'urdf_file': str(URDF_FILE),
        'checks': all_results,
        'summary': {
            'total_checks': len(all_results),
            'passed_checks': passed_count,
            'failed_checks': failed_count,
            'overall_status': 'PASSED' if all_passed else 'FAILED',
        }
    }

    # Write report
    OUTPUT_REPORT_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(OUTPUT_REPORT_FILE, 'w') as f:
        json.dump(report, f, indent=2)

    # Print summary
    print(f"\nURDF Export Validation Report")
    print(f"{'='*60}")
    print(f"URDF: {URDF_FILE}")
    print(f"Timestamp: {report['timestamp']}")
    print(f"\nChecks: {passed_count}/{len(all_results)} passed")
    for result in all_results:
        status = "✓ PASS" if result['passed'] else "✗ FAIL"
        print(f"  {status}: {result['check']}")
        print(f"           {result['details']}")

    print(f"\nOverall: {report['summary']['overall_status']}")
    print(f"Report written to: {OUTPUT_REPORT_FILE}")

    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())
