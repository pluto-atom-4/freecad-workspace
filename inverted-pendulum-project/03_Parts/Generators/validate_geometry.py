#!/usr/bin/env python3
"""
Standalone geometry validator - tests logic without FreeCAD dependency

Validates plate dimensions, hole positions, and geometry parameters.
"""

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class PlateSpec:
    """Plate specification"""
    name: str
    overall_length: float
    center_to_center: float
    width: float = 10.0
    thickness: float = 5.0
    hole_diameter: float = 5.0
    end_radius: float = 5.0


class GeometryValidator:
    """Validate plate geometry without FreeCAD"""

    PLATES = [
        PlateSpec("Top", 60.0, 50.0),
        PlateSpec("Middle", 54.72, 44.72),
        PlateSpec("Bottom", 52.43, 42.43),
    ]

    @staticmethod
    def validate_dimensions(plate: PlateSpec) -> dict:
        """Validate plate dimensions"""
        results = {
            "plate": plate.name,
            "checks": []
        }

        # Check 1: Overall length vs center-to-center
        expected_overall = plate.center_to_center + 2 * plate.end_radius
        actual_overall = plate.overall_length
        match = abs(expected_overall - actual_overall) < 0.1

        results["checks"].append({
            "name": "Overall Length Match",
            "expected": expected_overall,
            "actual": actual_overall,
            "pass": match
        })

        # Check 2: Hole radius < half width
        hole_radius = plate.hole_diameter / 2.0
        half_width = plate.width / 2.0
        hole_fits = hole_radius < half_width

        results["checks"].append({
            "name": "Hole Fits in Width",
            "hole_radius": hole_radius,
            "half_width": half_width,
            "pass": hole_fits
        })

        # Check 3: Holes don't overlap
        hole_offset = plate.center_to_center / 2.0
        hole_spacing = hole_offset * 2
        min_spacing = plate.hole_diameter * 1.5  # Safety margin

        holes_ok = hole_spacing > min_spacing

        results["checks"].append({
            "name": "Holes Don't Overlap",
            "hole_spacing": hole_spacing,
            "min_spacing": min_spacing,
            "pass": holes_ok
        })

        # Check 4: Positive dimensions
        all_positive = all([
            plate.overall_length > 0,
            plate.center_to_center > 0,
            plate.width > 0,
            plate.thickness > 0,
            plate.hole_diameter > 0,
            plate.end_radius > 0,
        ])

        results["checks"].append({
            "name": "All Dimensions Positive",
            "pass": all_positive
        })

        results["all_pass"] = all(c["pass"] for c in results["checks"])
        return results

    @staticmethod
    def validate_assembly_spacing() -> dict:
        """Validate assembly stacking"""
        spacing_z = [20.0, 0.0, -20.0]
        spacing_diff = 20.0

        results = {
            "assembly_spacing": spacing_z,
            "spacing_interval": spacing_diff,
            "checks": []
        }

        # Check 1: Monotonic decrease
        decreasing = all(spacing_z[i] > spacing_z[i+1] for i in range(len(spacing_z)-1))
        results["checks"].append({
            "name": "Plates Monotonically Decreasing",
            "pass": decreasing
        })

        # Check 2: Equal spacing
        diffs = [spacing_z[i] - spacing_z[i+1] for i in range(len(spacing_z)-1)]
        equal_spacing = all(abs(d - spacing_diff) < 0.01 for d in diffs)

        results["checks"].append({
            "name": "Equal Spacing Between Plates",
            "diffs": diffs,
            "pass": equal_spacing
        })

        results["all_pass"] = all(c["pass"] for c in results["checks"])
        return results

    @staticmethod
    def calculate_plate_volume(plate: PlateSpec) -> float:
        """Estimate plate volume (rough calculation)"""
        # Main rectangular body
        rect_volume = plate.center_to_center * plate.width * plate.thickness

        # Two semi-circular ends (approximate as half cylinders)
        cylinder_volume = math.pi * (plate.end_radius ** 2) * plate.thickness

        # Two holes (cylinders to subtract)
        hole_radius = plate.hole_diameter / 2.0
        hole_volume = 2 * math.pi * (hole_radius ** 2) * plate.thickness

        total = rect_volume + cylinder_volume - hole_volume
        return total

    @classmethod
    def run_validation(cls) -> bool:
        """Run complete validation suite"""
        print("=" * 70)
        print("PLATE GEOMETRY VALIDATION")
        print("=" * 70)
        print()

        all_pass = True

        # Validate each plate
        print("PLATE SPECIFICATIONS")
        print("-" * 70)
        for plate in cls.PLATES:
            results = cls.validate_dimensions(plate)

            print(f"\n{results['plate']} Plate:")
            print(f"  Overall Length: {plate.overall_length} mm")
            print(f"  Center-to-Center: {plate.center_to_center} mm")
            print(f"  Width: {plate.width} mm | Thickness: {plate.thickness} mm")
            print(f"  Hole Diameter: {plate.hole_diameter} mm (M5)")
            print()

            for check in results["checks"]:
                status = "✓" if check["pass"] else "✗"
                print(f"  {status} {check['name']}")
                if "expected" in check:
                    print(f"     Expected: {check['expected']:.2f}, Actual: {check['actual']:.2f}")
                all_pass = all_pass and check["pass"]

            # Volume calculation
            volume = cls.calculate_plate_volume(plate)
            print(f"\n  Estimated Volume: {volume:.1f} mm³ ({volume/1000:.2f} cm³)")

        # Assembly validation
        print()
        print("ASSEMBLY CONFIGURATION")
        print("-" * 70)

        assembly_results = cls.validate_assembly_spacing()
        print(f"\nStacking (Z-axis):")
        print(f"  Top:    Z = {assembly_results['assembly_spacing'][0]:6.1f} mm")
        print(f"  Middle: Z = {assembly_results['assembly_spacing'][1]:6.1f} mm (servo coupled)")
        print(f"  Bottom: Z = {assembly_results['assembly_spacing'][2]:6.1f} mm")
        print(f"\n  Spacing: {assembly_results['spacing_interval']} mm between plates")

        for check in assembly_results["checks"]:
            status = "✓" if check["pass"] else "✗"
            print(f"  {status} {check['name']}")
            all_pass = all_pass and check["pass"]

        # Summary
        print()
        print("=" * 70)
        if all_pass:
            print("✓ ALL VALIDATION CHECKS PASSED")
            print("=" * 70)
            return True
        else:
            print("✗ SOME VALIDATION CHECKS FAILED")
            print("=" * 70)
            return False


def main():
    """Main entry point"""
    validator = GeometryValidator()
    success = validator.run_validation()

    return 0 if success else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
