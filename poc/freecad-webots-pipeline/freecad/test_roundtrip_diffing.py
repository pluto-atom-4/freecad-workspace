#!/usr/bin/env python3
"""
Test script for the STEP round-trip diffing logic used by
02_roundtrip_check.py.

This validates the pure-Python comparison logic (bbox-diff tolerance
check, facet-count ratio, percent-diff) WITHOUT requiring FreeCAD — mirrors
the standalone-runnable test style used elsewhere in this repo (see
inverted-pendulum-project/03_Parts/Generators/test_02_servo_position.py).

Run standalone: python3 test_roundtrip_diffing.py
"""

import sys


BBOX_TOLERANCE_MM = 0.5
VOLUME_TOLERANCE_PCT = 2.0


def pct_diff(a: float, b: float) -> float:
    """Same helper as 02_roundtrip_check.py — duplicated here (not imported)
    so this test has no dependency on the FreeCAD-only module."""
    if a == 0:
        return 0.0 if b == 0 else float("inf")
    return abs(a - b) / abs(a) * 100.0


def bbox_within_tolerance(source_bbox: dict, reimported_bbox: dict, tolerance_mm: float) -> bool:
    diffs = {
        k: abs(reimported_bbox[k] - source_bbox[k])
        for k in ("width", "height", "depth")
    }
    return all(v <= tolerance_mm for v in diffs.values())


class TestRoundtripDiffing:
    """Test the round-trip comparison logic used by 02_roundtrip_check.py"""

    def test_pct_diff_identical(self) -> bool:
        print("Test: pct_diff of identical values is 0")
        result = pct_diff(100.0, 100.0)
        passed = result == 0.0
        print(f"  {'OK' if passed else 'FAIL'}: pct_diff(100, 100) = {result}")
        return passed

    def test_pct_diff_basic(self) -> bool:
        print("Test: pct_diff basic case")
        result = pct_diff(100.0, 102.0)
        expected = 2.0
        passed = abs(result - expected) < 1e-9
        print(f"  {'OK' if passed else 'FAIL'}: pct_diff(100, 102) = {result}, expected {expected}")
        return passed

    def test_pct_diff_zero_source_zero_target(self) -> bool:
        print("Test: pct_diff(0, 0) does not divide by zero")
        result = pct_diff(0.0, 0.0)
        passed = result == 0.0
        print(f"  {'OK' if passed else 'FAIL'}: pct_diff(0, 0) = {result}")
        return passed

    def test_pct_diff_zero_source_nonzero_target(self) -> bool:
        print("Test: pct_diff(0, nonzero) is +inf, not a crash")
        result = pct_diff(0.0, 5.0)
        passed = result == float("inf")
        print(f"  {'OK' if passed else 'FAIL'}: pct_diff(0, 5) = {result}")
        return passed

    def test_bbox_within_tolerance_identical(self) -> bool:
        print("Test: identical bboxes are within tolerance")
        bbox = {"width": 140.0, "height": 140.0, "depth": 143.0}
        passed = bbox_within_tolerance(bbox, bbox, BBOX_TOLERANCE_MM)
        print(f"  {'OK' if passed else 'FAIL'}: identical bbox within {BBOX_TOLERANCE_MM}mm")
        return passed

    def test_bbox_within_tolerance_small_drift(self) -> bool:
        print("Test: small tessellation drift stays within tolerance")
        source = {"width": 140.0, "height": 140.0, "depth": 143.0}
        reimported = {"width": 140.2, "height": 139.9, "depth": 143.1}
        passed = bbox_within_tolerance(source, reimported, BBOX_TOLERANCE_MM)
        print(f"  {'OK' if passed else 'FAIL'}: 0.1-0.2mm drift within {BBOX_TOLERANCE_MM}mm tolerance")
        return passed

    def test_bbox_exceeds_tolerance(self) -> bool:
        print("Test: large drift correctly flagged as exceeding tolerance")
        source = {"width": 140.0, "height": 140.0, "depth": 143.0}
        reimported = {"width": 145.0, "height": 140.0, "depth": 143.0}  # 5mm drift on width
        passed = not bbox_within_tolerance(source, reimported, BBOX_TOLERANCE_MM)
        print(f"  {'OK' if passed else 'FAIL'}: 5mm drift correctly exceeds {BBOX_TOLERANCE_MM}mm tolerance")
        return passed

    def test_facet_count_ratio(self) -> bool:
        print("Test: facet count ratio calculation (BREP tessellation vs source mesh)")
        source_facets = 96524
        reimported_facets = 88000  # plausible: BREP tessellation != source mesh facetization
        ratio = round(reimported_facets / source_facets, 3)
        # Sanity: ratio should be a positive float reasonably close to 1.0,
        # not wildly off (which would indicate a broken conversion).
        passed = 0.1 < ratio < 10.0
        print(f"  {'OK' if passed else 'FAIL'}: ratio = {ratio} (source={source_facets}, reimported={reimported_facets})")
        return passed

    def run_all_tests(self) -> bool:
        tests = [
            self.test_pct_diff_identical,
            self.test_pct_diff_basic,
            self.test_pct_diff_zero_source_zero_target,
            self.test_pct_diff_zero_source_nonzero_target,
            self.test_bbox_within_tolerance_identical,
            self.test_bbox_within_tolerance_small_drift,
            self.test_bbox_exceeds_tolerance,
            self.test_facet_count_ratio,
        ]

        results = []
        for test in tests:
            try:
                passed = test()
                results.append((test.__name__, passed))
            except Exception as e:
                print(f"  FAIL: exception {e}")
                results.append((test.__name__, False))
            print()

        print("=" * 70)
        print("TEST SUMMARY")
        print("=" * 70)

        passed_count = sum(1 for _, passed in results if passed)
        total_count = len(results)

        for test_name, passed in results:
            status = "OK" if passed else "FAIL"
            print(f"{status} {test_name}")

        print()
        print(f"Results: {passed_count}/{total_count} tests passed")
        print("=" * 70)

        return passed_count == total_count


def main():
    tester = TestRoundtripDiffing()
    success = tester.run_all_tests()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
