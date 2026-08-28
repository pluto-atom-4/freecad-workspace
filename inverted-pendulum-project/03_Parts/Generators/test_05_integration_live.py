#!/usr/bin/env python3
"""
Phase 5: Integration Tests (Live FreeCAD Required)

End-to-end integration tests for servo motor assembly.
Requires FreeCAD with Python access.

Usage:
  freecad --python test_05_integration_live.py
  or
  python3 test_05_integration_live.py (if FreeCAD Python available)

This script tests:
1. Full workflow: Phase 1 → Phase 4
2. Assembly loading and visualization
3. Servo visibility and correct positioning
4. Export to STEP/STL with validation
5. Performance benchmarks
"""

import sys
import time
import json
from pathlib import Path
from typing import Optional, Dict, Any
from datetime import datetime

try:
    import FreeCAD as App
    import Part
    import Mesh
    FREECAD_AVAILABLE = True
except ImportError:
    FREECAD_AVAILABLE = False
    print("INFO: FreeCAD not available - skipping live tests")


class LiveIntegrationTests:
    """End-to-end integration tests requiring FreeCAD"""

    def __init__(self):
        """Initialize test suite"""
        self.script_dir = Path(__file__).parent
        self.results = []

    def run_all_tests(self) -> bool:
        """Run all live integration tests"""
        if not FREECAD_AVAILABLE:
            print("✗ FreeCAD not available")
            return False

        print("=" * 70)
        print("PHASE 5: LIVE INTEGRATION TESTS (FreeCAD)")
        print("=" * 70)
        print()

        # Test 1: Assembly loading
        print("Test 1: Load assembly with servo link")
        print("-" * 70)
        doc = self.test_load_assembly()
        if not doc:
            print("✗ Assembly loading failed - cannot continue")
            return False
        print()

        # Test 2: Servo visibility
        print("Test 2: Verify servo motor visibility")
        print("-" * 70)
        self.test_servo_visibility(doc)
        print()

        # Test 3: Servo positioning
        print("Test 3: Verify servo positioning")
        print("-" * 70)
        self.test_servo_positioning(doc)
        print()

        # Test 4: Export STEP
        print("Test 4: Export to STEP format")
        print("-" * 70)
        self.test_export_step(doc)
        print()

        # Test 5: Export STL
        print("Test 5: Export to STL format")
        print("-" * 70)
        self.test_export_stl(doc)
        print()

        # Test 6: Performance benchmarks
        print("Test 6: Performance benchmarks")
        print("-" * 70)
        self.test_performance(doc)
        print()

        # Cleanup
        if doc:
            App.closeDocument(doc.Name)

        # Summary
        self._print_summary()

        return True

    def test_load_assembly(self) -> Optional[Any]:
        """Test: Load assembly with servo link"""
        try:
            doc_path = self.script_dir / "plates_assembled.FCStd"

            if not doc_path.exists():
                print(f"✗ Assembly file not found: {doc_path}")
                return None

            doc = App.openDocument(str(doc_path))
            print(f"✓ Assembly loaded: {doc_path.name}")

            # Verify objects
            objects = [obj.Name for obj in doc.Objects]
            print(f"  Objects: {', '.join(objects[:5])}")

            return doc

        except Exception as e:
            print(f"✗ Error loading assembly: {e}")
            return None

    def test_servo_visibility(self, doc: Any) -> None:
        """Test: Verify servo motor visibility"""
        try:
            servo_obj = None
            for obj in doc.Objects:
                if "Servo" in obj.Name or "servo" in obj.Name.lower():
                    servo_obj = obj
                    break

            if not servo_obj:
                print("✗ Servo object not found in assembly")
                return

            # Check if servo has geometry
            if hasattr(servo_obj, 'Shape'):
                shape = servo_obj.Shape
                num_faces = len(shape.Faces)
                print(f"✓ Servo motor visible with {num_faces} faces")
            elif hasattr(servo_obj, 'OutList'):
                num_children = len(servo_obj.OutList)
                print(f"✓ Servo body has {num_children} child object(s)")
            else:
                print("⚠ Servo object type unknown")

        except Exception as e:
            print(f"✗ Error checking servo visibility: {e}")

    def test_servo_positioning(self, doc: Any) -> None:
        """Test: Verify servo positioning"""
        try:
            servo_obj = None
            for obj in doc.Objects:
                if "Servo" in obj.Name:
                    servo_obj = obj
                    break

            if not servo_obj:
                print("✗ Servo object not found")
                return

            # Get placement
            placement = servo_obj.Placement
            position = placement.Base
            rotation = placement.Rotation

            print(f"✓ Servo position: X={position.x:.1f}, Y={position.y:.1f}, Z={position.z:.1f} mm")

            # Get rotation angles
            ypr = rotation.getYawPitchRoll()
            print(f"  Rotation: Yaw={ypr[0]:.1f}°, Pitch={ypr[1]:.1f}°, Roll={ypr[2]:.1f}°")

            # Validate pitch (should be ~90°)
            pitch = ypr[1]
            if 85 < pitch < 95:
                print(f"  ✓ Pitch rotation correct (~90°)")
            else:
                print(f"  ⚠ Pitch rotation unexpected (expected ~90°, got {pitch:.1f}°)")

        except Exception as e:
            print(f"✗ Error checking servo positioning: {e}")

    def test_export_step(self, doc: Any) -> None:
        """Test: Export to STEP format"""
        try:
            doc.recompute()

            output_path = self.script_dir / "test_export_merged.step"

            # Time the export
            start = time.time()

            # Extract all shapes
            shapes = []
            for obj in doc.Objects:
                if hasattr(obj, 'Shape'):
                    try:
                        shapes.append(obj.Shape)
                    except:
                        pass

            if not shapes:
                print("⚠ No shapes found to export")
                return

            # Create compound and export
            compound = Part.makeCompound(shapes)
            compound.exportStep(str(output_path))

            elapsed = time.time() - start

            # Check result
            if output_path.exists():
                size_mb = output_path.stat().st_size / (1024 * 1024)
                print(f"✓ STEP export: {size_mb:.2f} MB in {elapsed:.2f}s")

                # Cleanup
                output_path.unlink()
            else:
                print("✗ STEP export failed (file not created)")

        except Exception as e:
            print(f"✗ Error exporting STEP: {e}")

    def test_export_stl(self, doc: Any) -> None:
        """Test: Export to STL format"""
        try:
            doc.recompute()

            output_path = self.script_dir / "test_export_merged.stl"

            # Time the export
            start = time.time()

            # Extract all shapes
            shapes = []
            for obj in doc.Objects:
                if hasattr(obj, 'Shape'):
                    try:
                        shapes.append(obj.Shape)
                    except:
                        pass

            if not shapes:
                print("⚠ No shapes found to export")
                return

            # Create compound and mesh
            compound = Part.makeCompound(shapes)
            mesh = Mesh.Mesh(compound)

            # Export mesh
            mesh.write(str(output_path))

            elapsed = time.time() - start

            # Check result
            if output_path.exists():
                size_mb = output_path.stat().st_size / (1024 * 1024)
                triangles = len(mesh.Facets)
                print(f"✓ STL export: {size_mb:.2f} MB, {triangles} triangles in {elapsed:.2f}s")

                # Cleanup
                output_path.unlink()
            else:
                print("✗ STL export failed (file not created)")

        except Exception as e:
            print(f"✗ Error exporting STL: {e}")

    def test_performance(self, doc: Any) -> None:
        """Test: Performance benchmarks"""
        try:
            # Measure assembly load time
            print("Performance Benchmarks:")

            # Calculate assembly geometry
            total_shapes = sum(1 for obj in doc.Objects if hasattr(obj, 'Shape'))
            total_faces = sum(len(obj.Shape.Faces) if hasattr(obj, 'Shape') else 0
                            for obj in doc.Objects)

            print(f"  ✓ Assembly contains {total_shapes} shape(s) with {total_faces} faces total")

            # Time recomputation
            start = time.time()
            doc.recompute()
            recompute_time = time.time() - start

            print(f"  ✓ Recompute time: {recompute_time:.2f}s")

            if recompute_time < 2.0:
                print(f"  ✓ Performance target met (<2s)")
            else:
                print(f"  ⚠ Performance warning: recompute time {recompute_time:.2f}s (target <2s)")

        except Exception as e:
            print(f"✗ Error measuring performance: {e}")

    def _print_summary(self) -> None:
        """Print test summary"""
        print("=" * 70)
        print("✓ Live integration tests completed")
        print("=" * 70)


def main():
    """Main entry point"""
    if not FREECAD_AVAILABLE:
        print("ERROR: FreeCAD Python not available")
        print("Run with: freecad --python test_05_integration_live.py")
        return 1

    suite = LiveIntegrationTests()
    success = suite.run_all_tests()

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
