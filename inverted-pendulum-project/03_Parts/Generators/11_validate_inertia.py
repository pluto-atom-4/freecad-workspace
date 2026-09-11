#!/usr/bin/env python3
"""
Stage 5: Validate URDF Inertia (Issue #91)

Compares calculated inertia properties from the URDF export (Stage 4 output,
robot.urdf) against real prototype measurements when available. Provides a
validation report and exit status for CI/CD pipelines.

This script is pure Python (no FreeCAD imports) and reads only:
  - 06_Exports/urdf/robot.urdf (Stage 4 output) — parse <link>/<inertial> blocks
  - 02_Design_Inputs/prototype_measurements.json (optional, future hardware data)
    See 02_Design_Inputs/prototype_measurements.schema.json for expected shape.
    See 02_Design_Inputs/prototype_measurements.example.json for example data.

Output:
  - 03_Parts/Generators/11_inertia_validation_report.json — detailed per-link
    comparisons, overall pass/fail status, and tolerance info.

Exit code:
  - 0: validation passed OR no prototype data available (graceful absence)
  - 1: validation failed (measured values exceed tolerance vs calculated)

Usage:
    python3 11_validate_inertia.py
"""

import sys
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Any, Tuple
from datetime import datetime, timezone


# Script directory resolution (same pattern as Stage 3+4 scripts)
try:
    SCRIPT_DIR = Path(__file__).resolve().parent
except NameError:
    SCRIPT_DIR = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"

# Relative paths to inputs
_DESIGN_INPUTS_DIR = SCRIPT_DIR.parent.parent / "02_Design_Inputs"
_EXPORTS_DIR = SCRIPT_DIR.parent.parent / "06_Exports"

# File paths
URDF_FILE = _EXPORTS_DIR / "urdf" / "robot.urdf"
PROTOTYPE_MEASUREMENTS_FILE = _DESIGN_INPUTS_DIR / "prototype_measurements.json"
OUTPUT_REPORT_FILE = SCRIPT_DIR / "11_inertia_validation_report.json"


def load_calculated_inertia_from_urdf(urdf_path: Path) -> Dict[str, Dict[str, Any]]:
    """Parse robot.urdf and extract inertia properties for all links.

    Converts SI units (meters, kg, kg·m²) to project native units (mm, kg, kg·mm²):
      - mass_kg: unchanged (SI)
      - com_mm: xyz in meters → multiply by 1000 to get mm
      - inertia_kg_mm2: kg·m² → multiply by 1e6 to get kg·mm²

    Args:
        urdf_path: Path to robot.urdf file

    Returns:
        dict mapping link_name -> {mass_kg, com_mm: [x,y,z], inertia_kg_mm2: {ixx,...}}

    Raises:
        FileNotFoundError: if URDF file doesn't exist
        ET.ParseError: if URDF is malformed XML
    """
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    tree = ET.parse(urdf_path)
    root = tree.getroot()

    result = {}

    for link_elem in root.findall('link'):
        link_name = link_elem.get('name')
        inertial_elem = link_elem.find('inertial')

        if inertial_elem is None:
            continue

        # Extract mass. .get(..., default) guards against a <mass> element
        # present without a value attribute -- a malformed URDF should read as
        # 0/identity here, not crash float(None)/None.split() (issue #92 review).
        mass_elem = inertial_elem.find('mass')
        mass_kg = float(mass_elem.get('value', '0')) if mass_elem is not None else None

        # Extract center of mass (origin xyz in meters)
        origin_elem = inertial_elem.find('origin')
        com_str = origin_elem.get('xyz', '0 0 0') if origin_elem is not None else "0 0 0"
        com_m = [float(x) for x in com_str.split()]
        com_mm = [x * 1000.0 for x in com_m]  # Convert meters to mm

        # Extract inertia (kg·m²)
        inertia_elem = inertial_elem.find('inertia')
        inertia_si = {}
        if inertia_elem is not None:
            for key in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']:
                val = inertia_elem.get(key)
                if val is not None:
                    inertia_si[key] = float(val)

        # Convert to kg·mm² (kg·m² * 1e6)
        inertia_mm2 = {k: v * 1e6 for k, v in inertia_si.items()}

        result[link_name] = {
            'mass_kg': mass_kg,
            'com_mm': com_mm,
            'inertia_kg_mm2': inertia_mm2,
        }

    return result


def load_prototype_measurements(path: Path) -> Optional[Dict[str, Any]]:
    """Load prototype measurements from JSON file if it exists.

    Returns None if file doesn't exist (graceful absence).

    Args:
        path: Path to prototype_measurements.json

    Returns:
        Parsed JSON dict, or None if file missing
    """
    if not path.exists():
        return None

    try:
        with open(path, 'r') as f:
            data = json.load(f)
        return data
    except (json.JSONDecodeError, IOError) as e:
        print(f"ERROR: Failed to load {path}: {e}")
        return None


def compare_link(
    link_name: str,
    calculated: Dict[str, Any],
    measured: Optional[Dict[str, Any]],
    tolerance_percent: float = 30.0,
) -> Dict[str, Any]:
    """Compare calculated vs measured inertia for a single link.

    Skips null/missing fields in measured data (no fail if unmeasured).
    Compares mass_kg and diagonal inertia terms (ixx, iyy, izz).
    Off-diagonal terms are skipped as they're typically noisy/near-zero.

    Args:
        link_name: Name of the link
        calculated: {mass_kg, com_mm, inertia_kg_mm2} from URDF
        measured: {mass_kg, com_mm, inertia_kg_mm2} from prototype_measurements.json,
                  or None if not measured
        tolerance_percent: Percent tolerance (default 30%)

    Returns:
        dict with:
          - link_name
          - passed: bool (true if all measured fields within tolerance)
          - comparisons: list of {field, calculated, measured, percent_diff, passed}
          - summary: human-readable string
    """
    comparisons = []
    all_passed = True

    if measured is None:
        return {
            'link_name': link_name,
            'passed': True,  # No measured data, so "pass" (not a failure)
            'comparisons': [],
            'summary': 'No measured data',
        }

    # Compare mass_kg
    meas_mass = measured.get('mass_kg')
    if meas_mass is not None and calculated.get('mass_kg') is not None:
        calc_mass = calculated['mass_kg']
        if calc_mass > 0:
            pct_diff = abs(calc_mass - meas_mass) / calc_mass * 100.0
            passed = pct_diff <= tolerance_percent
            comparisons.append({
                'field': 'mass_kg',
                'calculated': round(calc_mass, 6),
                'measured': round(meas_mass, 6),
                'percent_diff': round(pct_diff, 2),
                'passed': passed,
            })
            if not passed:
                all_passed = False

    # Compare diagonal inertia terms (ixx, iyy, izz)
    calc_inertia = calculated.get('inertia_kg_mm2', {})
    meas_inertia = measured.get('inertia_kg_mm2')

    if meas_inertia is not None:
        for term in ['ixx', 'iyy', 'izz']:
            meas_val = meas_inertia.get(term)
            if meas_val is not None and term in calc_inertia:
                calc_val = calc_inertia[term]
                if calc_val > 0:
                    pct_diff = abs(calc_val - meas_val) / calc_val * 100.0
                    passed = pct_diff <= tolerance_percent
                    comparisons.append({
                        'field': term,
                        'calculated': round(calc_val, 6),
                        'measured': round(meas_val, 6),
                        'percent_diff': round(pct_diff, 2),
                        'passed': passed,
                    })
                    if not passed:
                        all_passed = False

    # Off-diagonal terms are skipped (ixy, ixz, iyz)

    summary = 'PASS' if all_passed else 'FAIL'
    if not comparisons:
        summary = 'No comparable fields'

    return {
        'link_name': link_name,
        'passed': all_passed,
        'comparisons': comparisons,
        'summary': summary,
    }


def build_report(
    calculated: Dict[str, Dict[str, Any]],
    measured: Optional[Dict[str, Any]],
    tolerance_percent: float = 30.0,
) -> Dict[str, Any]:
    """Build the validation report.

    Args:
        calculated: {link_name: {...}} from URDF
        measured: {links: {link_name: {...}}} from prototype_measurements.json, or None
        tolerance_percent: Percent tolerance for comparison

    Returns:
        dict with report structure: issue, timestamp, calculated_source,
        prototype_file_present, comparisons, overall_status, tolerance_percent
    """
    prototype_present = measured is not None
    measured_links = measured.get('links', {}) if prototype_present else {}

    comparisons = []
    all_links_passed = True

    for link_name in sorted(calculated.keys()):
        calc_data = calculated[link_name]
        meas_data = measured_links.get(link_name) if prototype_present else None

        result = compare_link(link_name, calc_data, meas_data, tolerance_percent)
        comparisons.append(result)

        if result.get('comparisons'):  # Only check if there are actual comparisons
            if not result['passed']:
                all_links_passed = False

    # Overall status
    if not prototype_present:
        overall_status = 'no_prototype_data'
    elif all_links_passed:
        overall_status = 'pass'
    else:
        overall_status = 'fail'

    return {
        'issue': 91,
        'timestamp': datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z'),
        'calculated_source': 'robot.urdf (Stage 4 export)',
        'prototype_file_present': prototype_present,
        'tolerance_percent': tolerance_percent,
        'comparisons': comparisons,
        'overall_status': overall_status,
    }


def main():
    """Main entry point."""
    print("=" * 70)
    print("Stage 5: Validate URDF Inertia (Issue #91)")
    print("=" * 70)

    try:
        # Load calculated inertia from URDF
        print("\n1. Loading calculated inertia from URDF...")
        if not URDF_FILE.exists():
            print(f"   ERROR: URDF file not found: {URDF_FILE}")
            sys.exit(1)

        calculated = load_calculated_inertia_from_urdf(URDF_FILE)
        print(f"   ✓ Loaded inertia for {len(calculated)} links")
        for link_name in sorted(calculated.keys()):
            data = calculated[link_name]
            print(f"     - {link_name}: mass={data['mass_kg']:.6f} kg")

        # Load prototype measurements (optional)
        print("\n2. Checking for prototype measurements...")
        measured = load_prototype_measurements(PROTOTYPE_MEASUREMENTS_FILE)
        if measured is None:
            print(f"   ℹ  No prototype data at {PROTOTYPE_MEASUREMENTS_FILE}")
            print("      Measurements are optional; validation will proceed without them.")
        else:
            print(f"   ✓ Loaded prototype measurements")
            meas_links = measured.get('links', {})
            meas_count = sum(1 for link_data in meas_links.values()
                            if link_data and any(v is not None for v in link_data.values()))
            print(f"     ({meas_count} links with some measurements)")

        # Build report
        print("\n3. Building validation report...")
        tolerance = 30.0
        report = build_report(calculated, measured, tolerance_percent=tolerance)

        # Print summary
        print(f"\n4. Results (tolerance: {tolerance}%):")
        print(f"   Overall status: {report['overall_status'].upper()}")

        if measured is not None:
            for comp in report['comparisons']:
                link = comp['link_name']
                status = 'PASS' if comp['passed'] else 'FAIL'
                field_count = len(comp['comparisons'])
                print(f"   {link:25s} {status:4s} ({field_count} field(s) compared)")
                for field_comp in comp['comparisons']:
                    pct = field_comp['percent_diff']
                    print(f"      {field_comp['field']:4s}: {pct:.1f}% diff")
        else:
            print("   No prototype data to compare.")

        # Write report file
        print(f"\n5. Writing report to {OUTPUT_REPORT_FILE.name}...")
        with open(OUTPUT_REPORT_FILE, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"   ✓ Report written")

        # Exit status
        exit_code = 0 if report['overall_status'] in ('pass', 'no_prototype_data') else 1
        print(f"\n{'=' * 70}")
        if exit_code == 0:
            print("✓ Validation succeeded (no failure criteria met)")
        else:
            print("✗ Validation FAILED (measured values exceed tolerance)")
        print(f"{'=' * 70}\n")

        sys.exit(exit_code)

    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
