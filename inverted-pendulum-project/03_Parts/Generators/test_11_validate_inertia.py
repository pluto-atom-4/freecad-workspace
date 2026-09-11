"""
Tests for Stage 5's inertia validation (11_validate_inertia.py).

Pure Python (no FreeCAD imports) — tests URDF parsing and comparison logic.
Follows the pattern of test_08_/test_09_/test_10_*.py.
"""

import json
from pathlib import Path
from datetime import datetime

import pytest

# Import the validation functions
# (import path assumes this test file is in 03_Parts/Generators/)
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
import importlib.util
spec = importlib.util.spec_from_file_location(
    "validate_inertia",
    Path(__file__).resolve().parent / "11_validate_inertia.py"
)
validate_inertia = importlib.util.module_from_spec(spec)
spec.loader.exec_module(validate_inertia)


# Locate test files
SCRIPT_DIR = Path(__file__).resolve().parent
EXPORTS_DIR = SCRIPT_DIR.parent.parent / "06_Exports"
URDF_FILE = EXPORTS_DIR / "urdf" / "robot.urdf"
DESIGN_INPUTS_DIR = SCRIPT_DIR.parent.parent / "02_Design_Inputs"
PROTOTYPE_MEASUREMENTS_EXAMPLE = DESIGN_INPUTS_DIR / "prototype_measurements.example.json"
REPORT_FILE = SCRIPT_DIR / "11_inertia_validation_report.json"


# ============================================================================
# Tests for load_calculated_inertia_from_urdf()
# ============================================================================

def test_load_calculated_inertia_urdf_file_exists():
    """URDF file exists at expected location."""
    assert URDF_FILE.exists(), f"URDF file not found: {URDF_FILE}"


def test_load_calculated_inertia_from_urdf_parses_all_links():
    """All 5 links are parsed from URDF."""
    if not URDF_FILE.exists():
        pytest.skip(f"URDF file not found: {URDF_FILE}")

    calculated = validate_inertia.load_calculated_inertia_from_urdf(URDF_FILE)

    expected_links = {
        'Base_Link',
        'Wheel_Left',
        'Wheel_Right',
        'Pendulum_Link',
        'Pendulum_Link_Right',
    }
    assert set(calculated.keys()) == expected_links, (
        f"Links mismatch: got {set(calculated.keys())}, expected {expected_links}"
    )


def test_load_calculated_inertia_units_conversion():
    """Units are correctly converted from SI to mm-based."""
    if not URDF_FILE.exists():
        pytest.skip(f"URDF file not found: {URDF_FILE}")

    calculated = validate_inertia.load_calculated_inertia_from_urdf(URDF_FILE)

    # Check that we have reasonable mm-scale CoM values (not meter-scale)
    # Base_Link should have CoM near (0,0,0)
    base_link = calculated['Base_Link']
    assert 'com_mm' in base_link
    assert len(base_link['com_mm']) == 3

    # Check that inertia values are in kg·mm² (much larger than kg·m²)
    assert 'inertia_kg_mm2' in base_link
    inertia = base_link['inertia_kg_mm2']
    ixx = inertia['ixx']
    # Base_Link inertia should be on order of 80 kg·mm² (from 0.00008 kg·m² * 1e6)
    assert ixx > 0, f"ixx should be positive, got {ixx}"
    # Should be > 10 (mm-scale) not < 0.1 (meter-scale)
    assert ixx > 10, f"ixx={ixx} looks meter-scaled, not mm-scaled"


def test_load_calculated_inertia_mass_positive():
    """All link masses are positive."""
    if not URDF_FILE.exists():
        pytest.skip(f"URDF file not found: {URDF_FILE}")

    calculated = validate_inertia.load_calculated_inertia_from_urdf(URDF_FILE)

    for link_name, data in calculated.items():
        mass = data['mass_kg']
        assert mass is not None and mass > 0, (
            f"Link {link_name} has invalid mass: {mass}"
        )


# ============================================================================
# Tests for load_prototype_measurements()
# ============================================================================

def test_load_prototype_measurements_missing_file_returns_none():
    """Missing prototype measurements file returns None (graceful absence)."""
    nonexistent = DESIGN_INPUTS_DIR / "nonexistent_prototype_measurements.json"
    result = validate_inertia.load_prototype_measurements(nonexistent)
    assert result is None, "Missing file should return None, not raise"


def test_load_prototype_measurements_example_file():
    """Example prototype measurements file loads correctly."""
    if not PROTOTYPE_MEASUREMENTS_EXAMPLE.exists():
        pytest.skip(f"Example file not found: {PROTOTYPE_MEASUREMENTS_EXAMPLE}")

    result = validate_inertia.load_prototype_measurements(PROTOTYPE_MEASUREMENTS_EXAMPLE)
    assert result is not None, "Example file should load"
    assert 'links' in result, "Loaded data should have 'links' key"
    assert isinstance(result['links'], dict), "links should be a dict"


# ============================================================================
# Tests for compare_link()
# ============================================================================

def test_compare_link_no_measured_data():
    """Link with no measured data returns passed=True (no failure)."""
    calculated = {
        'mass_kg': 0.25,
        'com_mm': [0, 0, 0],
        'inertia_kg_mm2': {'ixx': 80, 'iyy': 20, 'izz': 100, 'ixy': 0, 'ixz': 0, 'iyz': 0},
    }
    measured = None

    result = validate_inertia.compare_link('test_link', calculated, measured)

    assert result['passed'] is True, "No measured data should pass (not fail)"
    assert result['comparisons'] == [], "Should have no comparisons"


def test_compare_link_within_tolerance():
    """Link with measurements within tolerance passes."""
    calculated = {
        'mass_kg': 1.0,
        'com_mm': [0, 0, 0],
        'inertia_kg_mm2': {'ixx': 100, 'iyy': 100, 'izz': 100, 'ixy': 0, 'ixz': 0, 'iyz': 0},
    }
    measured = {
        'mass_kg': 0.98,  # 2% diff
        'com_mm': [0, 0, 0],
        'inertia_kg_mm2': {'ixx': 99, 'iyy': 101, 'izz': 99, 'ixy': 0, 'ixz': 0, 'iyz': 0},  # ~1% diff
    }

    result = validate_inertia.compare_link('test_link', calculated, measured, tolerance_percent=30.0)

    assert result['passed'] is True, "Values within 30% tolerance should pass"
    assert len(result['comparisons']) > 0, "Should have comparisons"
    for comp in result['comparisons']:
        assert comp['percent_diff'] < 30.0, f"{comp['field']} diff should be < 30%"


def test_compare_link_exceeds_tolerance():
    """Link with measurements exceeding tolerance fails."""
    calculated = {
        'mass_kg': 1.0,
        'com_mm': [0, 0, 0],
        'inertia_kg_mm2': {'ixx': 100, 'iyy': 100, 'izz': 100, 'ixy': 0, 'ixz': 0, 'iyz': 0},
    }
    measured = {
        'mass_kg': 1.5,  # 50% diff
        'com_mm': [0, 0, 0],
        'inertia_kg_mm2': {'ixx': 100, 'iyy': 100, 'izz': 100, 'ixy': 0, 'ixz': 0, 'iyz': 0},
    }

    result = validate_inertia.compare_link('test_link', calculated, measured, tolerance_percent=30.0)

    assert result['passed'] is False, "Values exceeding 30% tolerance should fail"
    mass_comp = next((c for c in result['comparisons'] if c['field'] == 'mass_kg'), None)
    assert mass_comp is not None
    assert mass_comp['passed'] is False, "Mass comparison should fail at 50% diff"


def test_compare_link_skips_null_fields():
    """Null fields in measured data are skipped (not counted as fail)."""
    calculated = {
        'mass_kg': 1.0,
        'com_mm': [0, 0, 0],
        'inertia_kg_mm2': {'ixx': 100, 'iyy': 100, 'izz': 100, 'ixy': 0, 'ixz': 0, 'iyz': 0},
    }
    measured = {
        'mass_kg': None,  # Unmeasured
        'com_mm': None,
        'inertia_kg_mm2': None,
    }

    result = validate_inertia.compare_link('test_link', calculated, measured, tolerance_percent=30.0)

    assert result['passed'] is True, "All-null measured data should pass (no comparisons)"
    assert result['comparisons'] == [], "No comparisons for null fields"


def test_compare_link_skips_off_diagonal_inertia():
    """Off-diagonal inertia terms (ixy, ixz, iyz) are not compared."""
    calculated = {
        'mass_kg': 1.0,
        'com_mm': [0, 0, 0],
        'inertia_kg_mm2': {'ixx': 100, 'iyy': 100, 'izz': 100, 'ixy': 50, 'ixz': 50, 'iyz': 50},
    }
    measured = {
        'mass_kg': 1.0,
        'com_mm': [0, 0, 0],
        'inertia_kg_mm2': {'ixx': 100, 'iyy': 100, 'izz': 100, 'ixy': 1, 'ixz': 1, 'iyz': 1},  # Very different
    }

    result = validate_inertia.compare_link('test_link', calculated, measured, tolerance_percent=10.0)

    # Should pass because off-diagonal terms are not compared
    assert result['passed'] is True

    # Verify that only diagonal terms were compared
    comp_fields = {c['field'] for c in result['comparisons']}
    assert 'ixx' in comp_fields
    assert 'iyy' in comp_fields
    assert 'izz' in comp_fields
    assert 'ixy' not in comp_fields, "Off-diagonal ixy should not be compared"
    assert 'ixz' not in comp_fields, "Off-diagonal ixz should not be compared"
    assert 'iyz' not in comp_fields, "Off-diagonal iyz should not be compared"


# ============================================================================
# Tests for build_report()
# ============================================================================

def test_build_report_no_prototype_data():
    """Report with no prototype data has overall_status='no_prototype_data'."""
    calculated = {
        'Base_Link': {
            'mass_kg': 0.25,
            'com_mm': [0, 0, 0],
            'inertia_kg_mm2': {'ixx': 80, 'iyy': 20, 'izz': 100, 'ixy': 0, 'ixz': 0, 'iyz': 0},
        },
    }
    measured = None

    report = validate_inertia.build_report(calculated, measured)

    assert report['overall_status'] == 'no_prototype_data'
    assert report['prototype_file_present'] is False
    assert report['issue'] == 91


def test_build_report_all_passing():
    """Report with all links passing has overall_status='pass'."""
    calculated = {
        'Base_Link': {
            'mass_kg': 1.0,
            'com_mm': [0, 0, 0],
            'inertia_kg_mm2': {'ixx': 100, 'iyy': 100, 'izz': 100, 'ixy': 0, 'ixz': 0, 'iyz': 0},
        },
    }
    measured = {
        'links': {
            'Base_Link': {
                'mass_kg': 0.99,
                'com_mm': [0, 0, 0],
                'inertia_kg_mm2': {'ixx': 100, 'iyy': 100, 'izz': 100, 'ixy': 0, 'ixz': 0, 'iyz': 0},
            },
        },
    }

    report = validate_inertia.build_report(calculated, measured)

    assert report['overall_status'] == 'pass'
    assert report['prototype_file_present'] is True


def test_build_report_one_link_failing():
    """Report with any link failing has overall_status='fail'."""
    calculated = {
        'Base_Link': {
            'mass_kg': 1.0,
            'com_mm': [0, 0, 0],
            'inertia_kg_mm2': {'ixx': 100, 'iyy': 100, 'izz': 100, 'ixy': 0, 'ixz': 0, 'iyz': 0},
        },
    }
    measured = {
        'links': {
            'Base_Link': {
                'mass_kg': 2.0,  # 100% diff, exceeds 30% tolerance
                'com_mm': [0, 0, 0],
                'inertia_kg_mm2': {'ixx': 100, 'iyy': 100, 'izz': 100, 'ixy': 0, 'ixz': 0, 'iyz': 0},
            },
        },
    }

    report = validate_inertia.build_report(calculated, measured, tolerance_percent=30.0)

    assert report['overall_status'] == 'fail'


def test_build_report_structure():
    """Report has expected top-level fields."""
    calculated = {
        'Base_Link': {
            'mass_kg': 0.25,
            'com_mm': [0, 0, 0],
            'inertia_kg_mm2': {'ixx': 80, 'iyy': 20, 'izz': 100, 'ixy': 0, 'ixz': 0, 'iyz': 0},
        },
    }

    report = validate_inertia.build_report(calculated, None)

    assert 'issue' in report
    assert 'timestamp' in report
    assert 'calculated_source' in report
    assert 'prototype_file_present' in report
    assert 'tolerance_percent' in report
    assert 'comparisons' in report
    assert 'overall_status' in report
    assert report['issue'] == 91
    assert isinstance(report['comparisons'], list)
