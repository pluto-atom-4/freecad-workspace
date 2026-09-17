#!/usr/bin/env python3
"""
URDF Structure Checker — Count and validate link/joint structure.

Validates that a URDF file has the expected number of links and joints.
Exits with code 0 if counts match expected, 1 if mismatch or file missing.

Usage:
    python3 urdf_structure_checker.py --urdf <path> --links <N> --joints <M>
"""

import sys
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


def check_urdf_structure(urdf_path: Path, expected_links: int, expected_joints: int) -> bool:
    """Check URDF link and joint counts.

    Args:
        urdf_path: Path to URDF file
        expected_links: Expected number of links
        expected_joints: Expected number of joints

    Returns:
        True if counts match expected, False otherwise
    """
    if not urdf_path.exists():
        print(f"ERROR: URDF file not found: {urdf_path}", file=sys.stderr)
        return False

    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
    except ET.ParseError as e:
        print(f"ERROR: Failed to parse URDF: {e}", file=sys.stderr)
        return False

    # Count links and joints
    links = root.findall('.//link')
    joints = root.findall('.//joint')

    actual_links = len(links)
    actual_joints = len(joints)

    # Check counts
    passed = True
    if actual_links != expected_links:
        print(f"ERROR: Expected {expected_links} links, found {actual_links}", file=sys.stderr)
        passed = False

    if actual_joints != expected_joints:
        print(f"ERROR: Expected {expected_joints} joints, found {actual_joints}", file=sys.stderr)
        passed = False

    if passed:
        print(f"✓ URDF structure valid: {actual_links} links, {actual_joints} joints")

    return passed


def main():
    """Parse args and run check."""
    parser = argparse.ArgumentParser(
        description="Validate URDF link and joint counts",
        prog="urdf_structure_checker.py"
    )
    parser.add_argument("--urdf", type=Path, required=True, help="Path to URDF file")
    parser.add_argument("--links", type=int, required=True, help="Expected number of links")
    parser.add_argument("--joints", type=int, required=True, help="Expected number of joints")

    args = parser.parse_args()

    if check_urdf_structure(args.urdf, args.links, args.joints):
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()
