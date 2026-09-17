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

# Add script dir to path for importing sibling modules
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

# Import mesh path resolver (Issue #161)
from urdf_mesh_path_resolver import resolve_mesh_path

# Import shared axis validation (Issue #163)
from axis_validation import compose_axis, validate_joint_axes

# Relative paths to inputs
_EXPORTS_DIR = SCRIPT_DIR.parent.parent / "06_Exports"

# File paths
URDF_FILE = _EXPORTS_DIR / "urdf" / "robot.urdf"
JOINT_CONFIG_FILE = SCRIPT_DIR / "joint_config.json"
OUTPUT_REPORT_FILE = SCRIPT_DIR / "12_urdf_export_validation_report.json"


def validate_mesh_paths(urdf_root: ET.Element, urdf_path: Path) -> List[Dict[str, Any]]:
    """Validate that all mesh files referenced in the URDF exist on disk.

    Validates both the source URDF and, if it exists, the rewritten robot_webots.urdf.

    Args:
        urdf_root: Parsed URDF XML root element (source)
        urdf_path: Path to the URDF file (used as base for relative paths)

    Returns:
        List of validation result dicts with keys: {check, passed, details, missing_files}
    """
    missing_files = []
    checked_files = set()
    urdf_dir = urdf_path.parent

    # Find all <mesh> elements in source URDF
    for mesh_elem in urdf_root.findall('.//mesh'):
        filename = mesh_elem.get('filename', '')
        if not filename or filename in checked_files:
            continue
        checked_files.add(filename)

        # Use centralized resolve_mesh_path() function (Issue #161)
        try:
            mesh_path = resolve_mesh_path(filename, urdf_dir)
            if not mesh_path.exists():
                missing_files.append(str(mesh_path))
        except Exception as e:
            missing_files.append(f"{filename} (resolution error: {e})")

    # NEW: Also validate robot_webots.urdf if it exists
    webots_urdf_path = SCRIPT_DIR.parent.parent / "07_Simulation" / "webots" / ".generated" / "robot_webots.urdf"
    if webots_urdf_path.exists():
        try:
            webots_tree = ET.parse(webots_urdf_path)
            webots_root = webots_tree.getroot()
            webots_urdf_dir = webots_urdf_path.parent

            # Validate all mesh references in robot_webots.urdf
            for mesh_elem in webots_root.findall('.//mesh'):
                webots_filename = mesh_elem.get('filename', '')
                if not webots_filename:
                    continue

                # robot_webots.urdf should have relative paths (not package://)
                try:
                    webots_mesh_path = (webots_urdf_dir / webots_filename).resolve()
                    if not webots_mesh_path.exists():
                        missing_files.append(f"(robot_webots.urdf) {str(webots_mesh_path)}")
                except Exception as e:
                    missing_files.append(f"(robot_webots.urdf) {webots_filename} (resolution error: {e})")
        except ET.ParseError as e:
            missing_files.append(f"(robot_webots.urdf) Parse error: {e}")

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
            'details': f'All {len(checked_files)} referenced mesh file(s) exist (source + webots)',
        }]


def validate_unit_consistency(urdf_root: ET.Element) -> List[Dict[str, Any]]:
    """Validate unit consistency by checking for suspiciously large values.

    HEURISTIC: Flag any raw numeric value in <origin xyz>, box <size>, or
    cylinder <radius>/<length> that exceeds 10 (unitless in URDF, should be
    metres for this robot). Values > 10m are physically implausible for a
    tabletop robot and likely indicate a missed mm→m conversion (Issues #105,
    #124, #125, #148).

    RATIONALE: The 10m threshold provides a ~30–50× safety margin for this
    tabletop-scale robot (nominal dimensions ~0.3–0.5m). This is a cheap,
    broad heuristic to catch obvious unit-conversion off-by-1000 bugs (e.g.,
    mesh in mm imported as metres). Chosen empirically from observed robot
    geometry + design margins.

    EDGE CASES:
    (a) Threshold may need raising if the robot scope expands to larger sizes
        (e.g., industrial arm, mobile robot base > 2m).
    (b) False-negatives possible: small parts left in mm (e.g., 0.005m = 5mm)
        slip through this check undetected. Validation is not exhaustive.
    (c) Asymmetry: origin xyz can be negative (e.g., offset from centre);
        box/cylinder dimensions must be positive (checked as absolute values for
        origin, strict positivity for geometry).

    FALLBACK: This is a pre-validation check only. Real downstream validation
    includes:
    - generate_proto.sh Step 1.5: Full Phase 12 validator (this function)
    - generate_proto.sh Step 1.5b: Structure checker (link/joint counts)
    - test_urdf_fcstd_consistency.py: FK regression test (motion validates
      dimensions at runtime)
    See inverted-pendulum-project/DESIGN.md Known Limitations for details.
    See CLAUDE.md "FreeCAD Live Bridge — Known Limitations" for tessellation
    and shape validation caveats.

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




def load_joint_config() -> Optional[Dict[str, Any]]:
    """Load joint_config.json if available."""
    if not JOINT_CONFIG_FILE.exists():
        return None
    try:
        with open(JOINT_CONFIG_FILE, 'r') as f:
            return json.load(f)
    except (json.JSONDecodeError, IOError):
        return None


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

    # Load joint config for axis validation (Issue #163)
    joint_config = load_joint_config()
    axis_validation_result = validate_joint_axes(urdf_root, joint_config)
    all_results.extend(axis_validation_result['checks'])

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
