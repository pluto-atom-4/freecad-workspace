#!/usr/bin/env python3
"""
Test script for turtlebot3_poc.urdf structural validity.

Validates the hand-authored POC URDF's joint/link structure using only the
Python stdlib (xml.etree) — no FreeCAD, no ROS/urdf_parser_py required.
Mirrors the standalone-runnable test style used elsewhere in this repo (see
inverted-pendulum-project/03_Parts/Generators/test_02_servo_position.py).

Run standalone: python3 test_urdf_structure.py
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

URDF_PATH = Path(__file__).resolve().parent / "turtlebot3_poc.urdf"

EXPECTED_LINKS = {
    "base_link",
    "wheel_left_link", "wheel_right_link",
    "caster_back_link",
}

EXPECTED_JOINTS = {
    "wheel_left_joint": "continuous",
    "wheel_right_joint": "continuous",
    "caster_back_joint": "fixed",
}


class TestUrdfStructure:
    """Test turtlebot3_poc.urdf structural validity"""

    def __init__(self):
        self.root = None

    def _load(self) -> bool:
        if not URDF_PATH.exists():
            print(f"  FAIL: URDF not found at {URDF_PATH}")
            return False
        try:
            self.root = ET.parse(URDF_PATH).getroot()
            return True
        except ET.ParseError as e:
            print(f"  FAIL: XML parse error: {e}")
            return False

    def test_urdf_parses(self) -> bool:
        print("Test: URDF is well-formed XML")
        passed = self._load()
        print(f"  {'OK' if passed else 'FAIL'}: parsed {URDF_PATH.name}")
        return passed

    def test_robot_root_tag(self) -> bool:
        print("Test: root element is <robot name=...>")
        if self.root is None:
            print("  FAIL: no root (parse failed earlier)")
            return False
        passed = self.root.tag == "robot" and self.root.get("name") == "turtlebot3_poc"
        print(f"  {'OK' if passed else 'FAIL'}: tag={self.root.tag}, name={self.root.get('name')}")
        return passed

    def test_expected_links_present(self) -> bool:
        print("Test: all expected links are present")
        if self.root is None:
            print("  FAIL: no root")
            return False
        links = {el.get("name") for el in self.root.findall("link")}
        missing = EXPECTED_LINKS - links
        passed = not missing
        print(f"  {'OK' if passed else 'FAIL'}: found {sorted(links)}, missing {sorted(missing)}")
        return passed

    def test_expected_joints_present_with_correct_type(self) -> bool:
        print("Test: all expected joints present with correct type")
        if self.root is None:
            print("  FAIL: no root")
            return False
        joints = {el.get("name"): el.get("type") for el in self.root.findall("joint")}
        all_ok = True
        for name, expected_type in EXPECTED_JOINTS.items():
            actual_type = joints.get(name)
            ok = actual_type == expected_type
            all_ok = all_ok and ok
            print(f"  {'OK' if ok else 'FAIL'}: {name} type={actual_type} (expected {expected_type})")
        return all_ok

    def test_wheel_joints_axis_and_rpy(self) -> bool:
        """The rpy="-1.57 0 0" + axis="0 0 1" combination is the specific
        gotcha called out in FINDINGS.md — verify it's actually present,
        not silently dropped/changed."""
        print("Test: wheel joints have axis=0 0 1 and origin rpy starting with -1.57")
        if self.root is None:
            print("  FAIL: no root")
            return False
        all_ok = True
        for joint_name in ("wheel_left_joint", "wheel_right_joint"):
            joint_el = next((j for j in self.root.findall("joint") if j.get("name") == joint_name), None)
            if joint_el is None:
                print(f"  FAIL: {joint_name} not found")
                all_ok = False
                continue
            axis_el = joint_el.find("axis")
            origin_el = joint_el.find("origin")
            axis_ok = axis_el is not None and axis_el.get("xyz") == "0 0 1"
            rpy = origin_el.get("rpy") if origin_el is not None else None
            rpy_ok = rpy is not None and rpy.strip().startswith("-1.57")
            ok = axis_ok and rpy_ok
            all_ok = all_ok and ok
            print(f"  {'OK' if ok else 'FAIL'}: {joint_name} axis={axis_el.get('xyz') if axis_el is not None else None}, rpy={rpy}")
        return all_ok

    def test_caster_link_has_no_visual_mesh(self) -> bool:
        print("Test: caster_back_link has collision but NO visual mesh (matches real ROBOTIS URDF)")
        if self.root is None:
            print("  FAIL: no root")
            return False
        caster = next((l for l in self.root.findall("link") if l.get("name") == "caster_back_link"), None)
        if caster is None:
            print("  FAIL: caster_back_link not found")
            return False
        has_visual = caster.find("visual") is not None
        has_collision = caster.find("collision") is not None
        passed = (not has_visual) and has_collision
        print(f"  {'OK' if passed else 'FAIL'}: has_visual={has_visual} (expected False), has_collision={has_collision} (expected True)")
        return passed

    def test_mesh_links_reference_roundtrip_files(self) -> bool:
        print("Test: base_link/wheel visuals reference *_roundtrip.stl (Stage 1 re-exports, decision #4)")
        if self.root is None:
            print("  FAIL: no root")
            return False
        all_ok = True
        for link_name in ("base_link", "wheel_left_link", "wheel_right_link"):
            link_el = next((l for l in self.root.findall("link") if l.get("name") == link_name), None)
            mesh_el = link_el.find("visual/geometry/mesh") if link_el is not None else None
            filename = mesh_el.get("filename") if mesh_el is not None else None
            ok = filename is not None and filename.endswith("_roundtrip.stl")
            all_ok = all_ok and ok
            print(f"  {'OK' if ok else 'FAIL'}: {link_name} mesh filename={filename}")
        return all_ok

    def test_mesh_scale_is_mm_to_m(self) -> bool:
        print("Test: mesh scale is 0.001 0.001 0.001 (mm source -> m URDF convention)")
        if self.root is None:
            print("  FAIL: no root")
            return False
        all_ok = True
        for link_name in ("base_link", "wheel_left_link", "wheel_right_link"):
            link_el = next((l for l in self.root.findall("link") if l.get("name") == link_name), None)
            mesh_el = link_el.find("visual/geometry/mesh") if link_el is not None else None
            scale = mesh_el.get("scale") if mesh_el is not None else None
            ok = scale == "0.001 0.001 0.001"
            all_ok = all_ok and ok
            print(f"  {'OK' if ok else 'FAIL'}: {link_name} scale={scale}")
        return all_ok

    def run_all_tests(self) -> bool:
        tests = [
            self.test_urdf_parses,
            self.test_robot_root_tag,
            self.test_expected_links_present,
            self.test_expected_joints_present_with_correct_type,
            self.test_wheel_joints_axis_and_rpy,
            self.test_caster_link_has_no_visual_mesh,
            self.test_mesh_links_reference_roundtrip_files,
            self.test_mesh_scale_is_mm_to_m,
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
    tester = TestUrdfStructure()
    success = tester.run_all_tests()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
