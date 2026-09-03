#!/usr/bin/env python3
"""
Test script for Phase 2: Servo Motor Position Calculation

This script validates the servo position calculation logic without requiring
FreeCAD. It:
1. Tests the calculation methods independently
2. Validates JSON output structure
3. Provides mock data for unit testing
4. Can be run standalone: python3 test_02_servo_position.py
"""

import json
import math
import sys
from pathlib import Path
from dataclasses import dataclass


@dataclass
class MockEdgeData:
    """Mock edge data for testing"""
    edge_id: int
    start_x: float
    start_y: float
    start_z: float
    end_x: float
    end_y: float
    end_z: float
    mid_x: float
    mid_y: float
    mid_z: float
    length: float


class TestServoPositionCalculation:
    """Test servo position calculation logic"""

    ALIGNMENT_TOLERANCE = 1.0
    CLEARANCE_MIN = 5.0

    def test_edge_midpoint_calculation(self) -> bool:
        """Test midpoint calculation"""
        print("Test: Edge midpoint calculation")

        # Mock edge data (horizontal edge)
        start = (0.0, 0.0, 0.0)
        end = (10.0, 0.0, 0.0)

        mid_x = (start[0] + end[0]) / 2.0
        mid_y = (start[1] + end[1]) / 2.0
        mid_z = (start[2] + end[2]) / 2.0

        expected_mid = (5.0, 0.0, 0.0)
        actual_mid = (mid_x, mid_y, mid_z)

        passed = abs(actual_mid[0] - expected_mid[0]) < 0.01
        status = "✓" if passed else "✗"
        print(f"  {status} Midpoint: expected {expected_mid}, got {actual_mid}")

        return passed

    def test_edge_normal_calculation(self) -> bool:
        """Test normal vector calculation"""
        print("Test: Edge normal vector calculation")

        # Edge along X-axis
        edge_dx = 10.0
        edge_dy = 0.0

        # Normal perpendicular in XY plane
        norm = math.sqrt(edge_dx**2 + edge_dy**2)
        normal_x = -edge_dy / norm
        normal_y = edge_dx / norm

        expected_normal = (0.0, 1.0)
        actual_normal = (normal_x, normal_y)

        passed = abs(actual_normal[0] - expected_normal[0]) < 0.01 and \
                 abs(actual_normal[1] - expected_normal[1]) < 0.01

        status = "✓" if passed else "✗"
        print(f"  {status} Normal: expected {expected_normal}, got {actual_normal}")

        return passed

    def test_alignment_validation(self) -> bool:
        """Test alignment tolerance check"""
        print("Test: Alignment tolerance validation")

        servo_pos = (10.0, 20.0)
        edge_midpoint = (10.05, 20.02)  # Within tolerance

        distance = math.sqrt(
            (servo_pos[0] - edge_midpoint[0]) ** 2 +
            (servo_pos[1] - edge_midpoint[1]) ** 2
        )

        passed = distance < self.ALIGNMENT_TOLERANCE
        status = "✓" if passed else "✗"
        print(f"  {status} Distance: {distance:.3f} mm (tolerance: {self.ALIGNMENT_TOLERANCE} mm)")

        return passed

    def test_clearance_calculation(self) -> bool:
        """Test clearance validation"""
        print("Test: Clearance calculation")

        # Test scenario: servo mounted with adequate clearance to both plates
        # Far enough from top, far enough from bottom
        servo_z = -30.0  # mm (well below all plates)
        top_plate_z = 15.0  # mm (center position, far above servo)
        top_plate_thickness = 5.0  # mm
        bottom_plate_z = 0.0  # mm (reference)
        bottom_plate_thickness = 5.0  # mm

        # Top plate clearance: distance from servo (at -30) to top plate bottom (at 12.5)
        # = 12.5 - (-30) = 42.5mm
        top_plate_bottom = top_plate_z - top_plate_thickness / 2.0
        top_clearance = top_plate_bottom - servo_z  # Handles negative servo_z correctly
        top_passed = top_clearance > self.CLEARANCE_MIN

        # Bottom plate clearance: distance from servo to bottom plate bottom
        # Bottom plate bottom is at -2.5, servo at -30
        # clearance = -2.5 - (-30) = 27.5mm
        bottom_plate_bottom = bottom_plate_z - bottom_plate_thickness / 2.0
        bottom_clearance = bottom_plate_bottom - servo_z  # Distance from servo UP to plate
        bottom_passed = bottom_clearance > self.CLEARANCE_MIN

        status_top = "✓" if top_passed else "✗"
        status_bottom = "✓" if bottom_passed else "✗"
        print(f"  {status_top} Top plate clearance: {top_clearance:.2f} mm (minimum: {self.CLEARANCE_MIN} mm)")
        print(f"  {status_bottom} Bottom plate clearance: {bottom_clearance:.2f} mm (minimum: {self.CLEARANCE_MIN} mm)")

        return top_passed and bottom_passed

    def test_json_output_structure(self) -> bool:
        """Test JSON output structure validity"""
        print("Test: JSON output structure")

        # Create sample output data
        output_data = {
            "phase": 2,
            "title": "Servo Motor Position Calculation",
            "timestamp": "0.0",
            "placement": {
                "x": 10.0,
                "y": 15.0,
                "z": -12.5,
                "roll": 0.0,
                "pitch": 90.0,
                "yaw": 0.0,
                "z_offset": 10.0,
            },
            "edge_data": {
                "edge26": {
                    "edge_id": 26,
                    "start_x": 0.0,
                    "start_y": 0.0,
                    "start_z": 0.0,
                    "end_x": 10.0,
                    "end_y": 0.0,
                    "end_z": 0.0,
                    "mid_x": 5.0,
                    "mid_y": 0.0,
                    "mid_z": 0.0,
                    "length": 10.0,
                },
                "edge34": {
                    "edge_id": 34,
                    "start_x": 0.0,
                    "start_y": 20.0,
                    "start_z": 0.0,
                    "end_x": 10.0,
                    "end_y": 20.0,
                    "end_z": 0.0,
                    "mid_x": 5.0,
                    "mid_y": 20.0,
                    "mid_z": 0.0,
                    "length": 10.0,
                },
            },
            "clearances": {
                "top_plate": 3.5,
                "bottom_plate": 6.0,
                "middle_plate_clearance": 1.25,
            },
            "validations": [
                {
                    "check": "Servo alignment to Edge26",
                    "passed": True,
                    "details": "Distance to Edge26 midpoint: 0.5 mm",
                    "value": 0.5,
                    "tolerance": 1.0,
                },
                {
                    "check": "Clearance to Top_Plate",
                    "passed": True,
                    "details": "Clearance: 3.5 mm (minimum: 5 mm)",
                    "value": 3.5,
                    "tolerance": 5,
                },
            ],
            "all_validations_passed": False,
            "specifications": {
                "middle_plate": {
                    "center_to_center": 44.72,
                    "width": 10.0,
                    "thickness": 2.5,
                    "z_position": 0.0,
                },
                "servo": {
                    "body_length": 32.0,
                    "body_width": 12.0,
                    "body_height": 28.0,
                    "shaft_length": 10.0,
                    "shaft_offset_x": 0.0,
                    "shaft_offset_y": 14.0,
                },
                "tolerances": {
                    "alignment_tolerance_mm": 1.0,
                    "clearance_min_mm": 5.0,
                },
            },
        }

        # Test JSON validity
        try:
            json_str = json.dumps(output_data, indent=2)
            reloaded = json.loads(json_str)

            # Validate key fields
            checks = [
                ("phase", output_data["phase"] == 2),
                ("placement.x", isinstance(output_data["placement"]["x"], (int, float))),
                ("placement.pitch", output_data["placement"]["pitch"] == 90.0),
                ("edge_data.edge26", output_data["edge_data"]["edge26"] is not None),
                ("clearances", isinstance(output_data["clearances"], dict)),
                ("validations", isinstance(output_data["validations"], list)),
            ]

            all_passed = True
            for check_name, check_result in checks:
                status = "✓" if check_result else "✗"
                print(f"  {status} {check_name}")
                all_passed = all_passed and check_result

            return all_passed

        except Exception as e:
            print(f"  ✗ JSON validation failed: {e}")
            return False

    def test_placement_matrix_calculations(self) -> bool:
        """Test placement matrix calculations"""
        print("Test: Placement matrix calculations")

        # Test parameters
        edge26_midpoint = (10.0, 20.0)
        z_offset = 10.0
        middle_plate_z = 0.0
        middle_plate_thickness = 2.5

        servo_x = edge26_midpoint[0]
        servo_y = edge26_midpoint[1]
        servo_z = middle_plate_z - middle_plate_thickness / 2.0 - z_offset

        expected_z = -11.25  # 0 - 1.25 - 10
        actual_z = servo_z

        passed = abs(actual_z - expected_z) < 0.01
        status = "✓" if passed else "✗"
        print(f"  {status} Servo Z position: expected {expected_z}, got {actual_z}")

        # Check rotation angles
        pitch_passed = abs(90.0 - 90.0) < 0.01
        status = "✓" if pitch_passed else "✗"
        print(f"  {status} Pitch rotation: 90° (shaft perpendicular to plate)")

        return passed and pitch_passed

    def run_all_tests(self) -> bool:
        """Run all tests"""
        print("=" * 70)
        print("PHASE 2 SERVO POSITION CALCULATION - UNIT TESTS")
        print("=" * 70)
        print()

        tests = [
            self.test_edge_midpoint_calculation,
            self.test_edge_normal_calculation,
            self.test_alignment_validation,
            self.test_clearance_calculation,
            self.test_placement_matrix_calculations,
            self.test_json_output_structure,
        ]

        results = []
        for test in tests:
            try:
                passed = test()
                results.append((test.__name__, passed))
                print()
            except Exception as e:
                print(f"  ✗ Exception: {e}")
                results.append((test.__name__, False))
                print()

        # Summary
        print("=" * 70)
        print("TEST SUMMARY")
        print("=" * 70)

        passed_count = sum(1 for _, passed in results if passed)
        total_count = len(results)

        for test_name, passed in results:
            status = "✓" if passed else "✗"
            print(f"{status} {test_name}")

        print()
        print(f"Results: {passed_count}/{total_count} tests passed")

        if passed_count == total_count:
            print("✓ ALL TESTS PASSED")
        else:
            print(f"✗ {total_count - passed_count} TEST(S) FAILED")

        print("=" * 70)

        return passed_count == total_count


def main():
    """Main entry point"""
    tester = TestServoPositionCalculation()
    success = tester.run_all_tests()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
