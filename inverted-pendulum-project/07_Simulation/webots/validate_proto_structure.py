#!/usr/bin/env python3
"""
Stage B2: Validate PROTO Structure (Phase 13)

Validates the generated PROTO file (InvertedPendulumRobot.proto from urdf2webots.importer)
for structural correctness against the reference URDF:
  - Joint names match URDF
  - Position sensors auto-named correctly
  - Physics (mass, inertia, centerOfMass) propagated
  - Mesh URLs resolve to actual files
  - VRML syntax is valid

This script is pure Python (no FreeCAD imports) and reads:
  - 07_Simulation/webots/.generated/robot_webots.urdf (reference)
  - 07_Simulation/webots/protos/InvertedPendulumRobot.proto (generated)
  - Mesh files referenced in URDF (must exist on disk)

Output:
  - 13_proto_structure_validation_report.json (machine-readable results)
  - B2_PROTO_STRUCTURE_CHECKLIST.md (human-readable checklist)

Exit code:
  - 0: all checks passed
  - 1: one or more checks failed

Usage:
    python3 validate_proto_structure.py
"""

import sys
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Any, Tuple
from datetime import datetime, timezone
import re

from vrml_lexer import find_matching_brace

# Script directory resolution
try:
    SCRIPT_DIR = Path(__file__).resolve().parent
except NameError:
    SCRIPT_DIR = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "07_Simulation" / "webots"

# File paths
URDF_FILE = SCRIPT_DIR / ".generated" / "robot_webots.urdf"
PROTO_FILE = SCRIPT_DIR / "protos" / "InvertedPendulumRobot.proto"
OUTPUT_REPORT_FILE = SCRIPT_DIR / "13_proto_structure_validation_report.json"
OUTPUT_CHECKLIST_FILE = SCRIPT_DIR / "B2_PROTO_STRUCTURE_CHECKLIST.md"


def validate_joint_names_match_urdf(urdf_root: ET.Element, proto_content: str) -> Dict[str, Any]:
    """
    Validate that all joint names from URDF are present as RotationalMotor names in PROTO.

    Args:
        urdf_root: Parsed URDF XML root element
        proto_content: PROTO file content as string

    Returns:
        Dict with keys: {check, passed, details, joint_names, motor_names, missing_motors}
    """
    # Extract joint names from URDF
    joint_names = set()
    for joint_elem in urdf_root.findall('.//joint'):
        joint_name = joint_elem.get('name', '')
        if joint_name:
            joint_names.add(joint_name)

    # Extract RotationalMotor names from PROTO
    motor_pattern = r'RotationalMotor\s*{\s*name\s+"([^"]+)"'
    motor_matches = re.findall(motor_pattern, proto_content)
    motor_names = set(motor_matches)

    # Check that all URDF joints are represented as motors in PROTO
    missing_motors = joint_names - motor_names
    passed = len(missing_motors) == 0

    return {
        "check": "joint_names_match_urdf",
        "passed": passed,
        "details": f"Checked {len(joint_names)} URDF joints against {len(motor_names)} PROTO motors",
        "joint_names": sorted(list(joint_names)),
        "motor_names": sorted(list(motor_names)),
        "missing_motors": sorted(list(missing_motors)),
    }


def validate_position_sensors_auto_named(urdf_root: ET.Element, proto_content: str) -> Dict[str, Any]:
    """
    Validate that each joint has a PositionSensor with auto-generated name "<joint_name>_sensor".

    Args:
        urdf_root: Parsed URDF XML root element
        proto_content: PROTO file content as string

    Returns:
        Dict with keys: {check, passed, details, expected_sensors, found_sensors, missing_sensors}
    """
    # Extract joint names from URDF
    joint_names = set()
    for joint_elem in urdf_root.findall('.//joint'):
        joint_name = joint_elem.get('name', '')
        if joint_name:
            joint_names.add(joint_name)

    # Generate expected sensor names
    expected_sensors = {f"{joint}_sensor" for joint in joint_names}

    # Extract PositionSensor names from PROTO
    sensor_pattern = r'PositionSensor\s*{\s*name\s+"([^"]+)"'
    sensor_matches = re.findall(sensor_pattern, proto_content)
    found_sensors = set(sensor_matches)

    # Check that all expected sensors are present
    missing_sensors = expected_sensors - found_sensors
    passed = len(missing_sensors) == 0

    return {
        "check": "position_sensors_auto_named",
        "passed": passed,
        "details": f"Checked {len(expected_sensors)} expected sensors, found {len(found_sensors)}",
        "expected_sensors": sorted(list(expected_sensors)),
        "found_sensors": sorted(list(found_sensors)),
        "missing_sensors": sorted(list(missing_sensors)),
    }


def extract_solid_names_from_proto(proto_content: str) -> set:
    """
    Extract Solid node names from PROTO file.

    Finds all `endPoint Solid { ... name "..." ... }` blocks and extracts the name.
    Uses vrml_lexer.find_matching_brace() for proper brace matching.
    """
    solid_names = set()

    # Find all occurrences of "endPoint Solid {"
    endpoint_pattern = r'endPoint\s+Solid\s*{'
    for match in re.finditer(endpoint_pattern, proto_content):
        # From the opening brace, find the matching closing brace
        open_brace_pos = proto_content.rfind('{', match.start(), match.end() + 1)
        if open_brace_pos == -1:
            continue

        try:
            close_brace_pos = find_matching_brace(proto_content, open_brace_pos)
        except ValueError:
            # Skip blocks with malformed braces
            continue

        # Extract the block content (between the braces)
        block_content = proto_content[open_brace_pos:close_brace_pos + 1]

        # Within this block, find the name attribute
        name_pattern = r'name\s+"([^"]+)"'
        name_match = re.search(name_pattern, block_content)
        if name_match:
            solid_names.add(name_match.group(1))

    return solid_names


def validate_physics_propagated(urdf_root: ET.Element, proto_content: str) -> Dict[str, Any]:
    """
    Validate that each Solid link has physics with mass, inertiaMatrix, and centerOfMass.
    Also validates that PROTO has correct number of endPoint Solid nodes.

    Uses vrml_lexer for robust Physics block extraction.

    Args:
        urdf_root: Parsed URDF XML root element
        proto_content: PROTO file content as string

    Returns:
        Dict with keys: {check, passed, details, link_masses, physics_with_mass, physics_with_inertia, physics_with_com, solid_names}
    """
    # Extract link names and masses from URDF
    link_masses = {}
    for link_elem in urdf_root.findall('.//link'):
        link_name = link_elem.get('name', '')
        if link_name:
            inertial = link_elem.find('inertial')
            if inertial is not None:
                mass_elem = inertial.find('mass')
                if mass_elem is not None:
                    mass_value = mass_elem.get('value', '')
                    link_masses[link_name] = float(mass_value) if mass_value else None

    # Extract Solid node names from PROTO to validate structural correctness
    solid_names = extract_solid_names_from_proto(proto_content)

    # Find all Physics blocks using lexer-based block extraction
    physics_with_mass = 0
    physics_with_inertia = 0
    physics_with_com = 0

    physics_pattern = r'physics\s+Physics\s*{'
    for match in re.finditer(physics_pattern, proto_content):
        # Find opening brace
        open_brace_pos = proto_content.rfind('{', match.start(), match.end() + 1)
        if open_brace_pos == -1:
            continue

        try:
            close_brace_pos = find_matching_brace(proto_content, open_brace_pos)
        except ValueError:
            # Skip malformed blocks
            continue

        # Extract block content
        block_content = proto_content[open_brace_pos:close_brace_pos + 1]

        # Check for required attributes
        if re.search(r'mass\s+\d+\.?\d*', block_content):
            physics_with_mass += 1
        if re.search(r'inertiaMatrix\s*\[', block_content):
            physics_with_inertia += 1
        if re.search(r'centerOfMass\s*\[', block_content):
            physics_with_com += 1

    # All three must be present in equal numbers (one complete Physics block per link)
    expected_physics_blocks = len([v for v in link_masses.values() if v is not None])
    # Solid nodes correspond to joints (Base_Link doesn't have endPoint Solid)
    expected_solids = expected_physics_blocks - 1  # Exclude Base_Link

    passed = (physics_with_mass == expected_physics_blocks and
              physics_with_inertia == expected_physics_blocks and
              physics_with_com == expected_physics_blocks and
              len(solid_names) == expected_solids)

    return {
        "check": "physics_propagated",
        "passed": passed,
        "details": f"Found {physics_with_mass} mass, {physics_with_inertia} inertia, {physics_with_com} centerOfMass, {len(solid_names)} solids (expected {expected_physics_blocks} physics blocks, {expected_solids} solids)",
        "link_masses": link_masses,
        "physics_with_mass": physics_with_mass,
        "physics_with_inertia": physics_with_inertia,
        "physics_with_com": physics_with_com,
        "expected_blocks": expected_physics_blocks,
        "solid_names": sorted(list(solid_names)),
        "expected_solids": expected_solids,
    }


def validate_mesh_urls_resolve(urdf_root: ET.Element, proto_content: str) -> Dict[str, Any]:
    """
    Validate that all mesh URLs referenced in the URDF can be resolved to files.

    Args:
        urdf_root: Parsed URDF XML root element
        proto_content: PROTO file content as string (unused, but kept for consistency)

    Returns:
        Dict with keys: {check, passed, details, mesh_files, resolved_files, missing_files}
    """
    # Extract mesh filenames from URDF
    mesh_files = set()
    for mesh_elem in urdf_root.findall('.//mesh'):
        filename = mesh_elem.get('filename', '')
        if filename:
            # Handle relative paths (these are relative to the URDF file's directory)
            mesh_files.add(filename)

    # Resolve paths relative to URDF directory
    urdf_dir = URDF_FILE.parent
    resolved_files = {}
    missing_files = []

    for mesh_file in mesh_files:
        # Convert relative path from URDF to actual file path
        # URDF might reference: ../../../06_Exports/urdf/meshes/feetech-STS3032-visual.stl
        mesh_path = (urdf_dir / mesh_file).resolve()

        if mesh_path.exists():
            resolved_files[mesh_file] = str(mesh_path)
        else:
            missing_files.append(mesh_file)

    passed = len(missing_files) == 0

    return {
        "check": "mesh_urls_resolve",
        "passed": passed,
        "details": f"Checked {len(mesh_files)} mesh files, {len(resolved_files)} resolved, {len(missing_files)} missing",
        "mesh_files": sorted(list(mesh_files)),
        "resolved_files": resolved_files,
        "missing_files": missing_files,
    }


def validate_vrml_syntax(proto_content: str) -> Dict[str, Any]:
    """
    Validate VRML syntax: braces balanced, non-empty file, proper string/comment handling.

    Uses vrml_lexer.find_matching_brace() to validate proper brace matching across
    the entire file, catching unterminated strings/comments and nested structure issues.

    Args:
        proto_content: PROTO file content as string

    Returns:
        Dict with keys: {check, passed, details}
    """
    # Check file is non-empty
    if not proto_content.strip():
        return {
            "check": "vrml_syntax",
            "passed": False,
            "details": "PROTO file is empty",
        }

    # Count braces (quick check)
    open_braces = proto_content.count('{')
    close_braces = proto_content.count('}')

    # Check for VRML magic string
    has_vrml_header = "#VRML_SIM" in proto_content or "VRML" in proto_content

    # Detailed check: try to find matching braces for each top-level opening brace
    # This catches unterminated strings, comments, and nested structure issues
    lexer_error = None
    for match in re.finditer(r'\{', proto_content):
        pos = match.start()
        # Skip if this is not a top-level brace (naive heuristic: not inside another block)
        # For a full check, we'd track context, but for PROTO files, checking first few works
        try:
            find_matching_brace(proto_content, pos)
        except ValueError as e:
            lexer_error = str(e)
            break

    passed = (
        (open_braces == close_braces) and
        has_vrml_header and
        len(proto_content) > 0 and
        lexer_error is None
    )

    details = f"Braces: {open_braces} open, {close_braces} close; VRML header present: {has_vrml_header}"
    if lexer_error:
        details += f"; Lexer error: {lexer_error}"

    return {
        "check": "vrml_syntax",
        "passed": passed,
        "details": details,
        "open_braces": open_braces,
        "close_braces": close_braces,
        "has_vrml_header": has_vrml_header,
        "lexer_error": lexer_error,
    }


def load_urdf(urdf_path: Path) -> Optional[ET.Element]:
    """Load and parse URDF file."""
    try:
        tree = ET.parse(urdf_path)
        return tree.getroot()
    except Exception as e:
        print(f"ERROR: Failed to parse URDF at {urdf_path}: {e}")
        return None


def load_proto(proto_path: Path) -> Optional[str]:
    """Load PROTO file content."""
    try:
        with open(proto_path, 'r', encoding='utf-8') as f:
            return f.read()
    except Exception as e:
        print(f"ERROR: Failed to read PROTO at {proto_path}: {e}")
        return None


def generate_json_report(checks: List[Dict[str, Any]], timestamp: str) -> Dict[str, Any]:
    """Generate machine-readable JSON report."""
    return {
        "timestamp": timestamp,
        "phase": 13,
        "title": "PROTO Structure Validation (Phase 13 — Stage B2)",
        "urdf_file": str(URDF_FILE),
        "proto_file": str(PROTO_FILE),
        "checks": checks,
        "summary": {
            "total_checks": len(checks),
            "passed_checks": sum(1 for c in checks if c.get("passed", False)),
            "failed_checks": sum(1 for c in checks if not c.get("passed", True)),
        },
    }


def generate_markdown_checklist(urdf_root: ET.Element, checks: List[Dict[str, Any]]) -> str:
    """Generate human-readable Markdown checklist."""

    # Extract structure from URDF
    links = []
    for link_elem in urdf_root.findall('.//link'):
        link_name = link_elem.get('name', '')
        if link_name:
            links.append(link_name)

    joints = []
    for joint_elem in urdf_root.findall('.//joint'):
        joint_name = joint_elem.get('name', '')
        if joint_name:
            joints.append(joint_name)

    # Build checklist markdown
    lines = [
        "# B2 PROTO Structure Checklist",
        "",
        "**Phase 13 Validation Report** — Structural alignment between URDF and PROTO",
        "",
        "## Validation Results Summary",
        "",
    ]

    # Add check summaries
    for check in checks:
        status = "✅ PASS" if check.get("passed", False) else "❌ FAIL"
        check_name = check.get("check", "unknown")
        details = check.get("details", "")
        lines.append(f"### {status} {check_name}")
        lines.append(f"**Details:** {details}")
        lines.append("")

    # Add link validation table
    lines.append("## Links (Expected: 5)")
    lines.append("")
    lines.append("| # | Link Name | Status | Notes |")
    lines.append("|---|-----------|--------|-------|")
    for i, link in enumerate(links, 1):
        lines.append(f"| {i} | `{link}` | ✓ | Present in URDF |")
    lines.append("")

    # Add joint validation table
    lines.append("## Joints (Expected: 4)")
    lines.append("")
    lines.append("| # | Joint Name | RotationalMotor | PositionSensor | Status |")
    lines.append("|---|------------|-----------------|----------------|--------|")
    for i, joint in enumerate(joints, 1):
        motor_name = joint  # Motor is named same as joint
        sensor_name = f"{joint}_sensor"
        lines.append(f"| {i} | `{joint}` | `{motor_name}` | `{sensor_name}` | ✓ |")
    lines.append("")

    # Add footer
    lines.append("## Next Steps")
    lines.append("")
    lines.append("1. Review validation results above")
    lines.append("2. If all checks pass, proceed to **Stage B3** (IMU node + live Webots visual check)")
    lines.append("3. If any check fails, inspect the PROTO file and URDF reference, then regenerate")
    lines.append("")

    return "\n".join(lines)


def main() -> int:
    """Main validation pipeline."""
    print("=== Phase 13: PROTO Structure Validation ===\n")

    # Check input files exist
    if not URDF_FILE.exists():
        print(f"ERROR: URDF file not found at {URDF_FILE}")
        return 1

    if not PROTO_FILE.exists():
        print(f"ERROR: PROTO file not found at {PROTO_FILE}")
        return 1

    # Load files
    urdf_root = load_urdf(URDF_FILE)
    if urdf_root is None:
        return 1

    proto_content = load_proto(PROTO_FILE)
    if proto_content is None:
        return 1

    print(f"✓ Loaded URDF from {URDF_FILE}")
    print(f"✓ Loaded PROTO from {PROTO_FILE}")
    print("")

    # Run validation checks
    print("Running validation checks...")
    checks = [
        validate_joint_names_match_urdf(urdf_root, proto_content),
        validate_position_sensors_auto_named(urdf_root, proto_content),
        validate_physics_propagated(urdf_root, proto_content),
        validate_mesh_urls_resolve(urdf_root, proto_content),
        validate_vrml_syntax(proto_content),
    ]

    # Generate timestamp
    timestamp = datetime.now(timezone.utc).isoformat()

    # Generate reports
    json_report = generate_json_report(checks, timestamp)
    markdown_checklist = generate_markdown_checklist(urdf_root, checks)

    # Write JSON report
    with open(OUTPUT_REPORT_FILE, 'w', encoding='utf-8') as f:
        json.dump(json_report, f, indent=2)
    print(f"✓ JSON report written to {OUTPUT_REPORT_FILE}")

    # Write Markdown checklist
    with open(OUTPUT_CHECKLIST_FILE, 'w', encoding='utf-8') as f:
        f.write(markdown_checklist)
    print(f"✓ Markdown checklist written to {OUTPUT_CHECKLIST_FILE}")
    print("")

    # Print summary
    summary = json_report["summary"]
    print(f"Summary: {summary['passed_checks']}/{summary['total_checks']} checks passed")
    print("")

    # Print detailed results
    for check in checks:
        status = "✅ PASS" if check.get("passed", False) else "❌ FAIL"
        print(f"{status} {check['check']}: {check['details']}")

    print("")

    # Fail-hard if any check failed
    if summary["failed_checks"] > 0:
        print(f"ERROR: {summary['failed_checks']} validation check(s) failed")
        return 1

    print("=== All validation checks passed ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
