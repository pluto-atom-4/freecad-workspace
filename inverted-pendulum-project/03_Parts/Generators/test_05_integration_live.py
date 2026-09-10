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

        # Test 7: extract_assembly_shape() shape filter regression (issue #55)
        print("Test 7: extract_assembly_shape() rejects unexpected shapes (issue #55)")
        print("-" * 70)
        self.test_export_shape_filter_regression()
        print()

        # Test 8: Middle_Plate z_position regression (issue #56)
        print("Test 8: Middle_Plate z_position matches live document (issue #56)")
        print("-" * 70)
        self.test_middle_plate_z_position_regression()
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

            # Create mesh from compound by meshing individual solids
            try:
                # Try direct mesh first (works for Solid/Shell/Face)
                mesh = Mesh.Mesh(compound)
            except TypeError as e:
                if "Part.Compound" in str(e):
                    # Compound detected - mesh individual components
                    print("  Compound detected, meshing individual components...")
                    mesh = Mesh.Mesh()  # Empty mesh to accumulate

                    solids = list(compound.Solids)
                    print(f"    Found {len(solids)} solid(s)")

                    for i, solid in enumerate(solids):
                        try:
                            component_mesh = Mesh.Mesh(solid)
                            mesh.addMesh(component_mesh)
                            print(f"    ✓ Meshed component {i+1}/{len(solids)} ({len(component_mesh.Facets)} triangles)")
                        except Exception as component_e:
                            print(f"    ⚠ Could not mesh component {i+1}: {component_e}")
                            continue

                    if mesh.CountFacets == 0:
                        raise RuntimeError("No components could be meshed")
                else:
                    # Different error - re-raise
                    raise

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

    def test_export_shape_filter_regression(self) -> None:
        """Test: extract_assembly_shape() rejects unexpected shape objects.

        Regression test for issue #55: leftover prototype/scratch clutter
        objects (e.g. Top_Plate001, mirroring what was actually found in
        plates_servo_assembled.FCStd) sharing the document with the expected
        plates must make extract_assembly_shape() fail loudly instead of
        silently folding them into the merged compound.

        Deliberately independent of plates_servo_assembled.FCStd's current
        clutter state -- uses synthetic in-memory documents only, so this
        test stays meaningful regardless of whether that file's clutter is
        ever cleaned up (out of scope for this fix).
        """
        try:
            # Dynamically import AssemblyExporter, using the same
            # importlib.util pattern already established in
            # test_06_phase6_tooling.py.
            from importlib.util import spec_from_file_location, module_from_spec

            spec = spec_from_file_location(
                "export_assembly_merged",
                str(self.script_dir / "04_export_assembly_merged.py")
            )
            export_module = module_from_spec(spec)
            spec.loader.exec_module(export_module)

            AssemblyExporter = export_module.AssemblyExporter

            # Case 1 ("good"): exactly the expected three plates.
            doc_good = App.newDocument("Issue55RegressionGood")
            try:
                for name in ("Top_Plate", "Middle_Plate", "Bottom_Plate"):
                    obj = doc_good.addObject("Part::Feature", name)
                    obj.Shape = Part.makeBox(10, 10, 10)
                doc_good.recompute()

                exporter_good = AssemblyExporter()
                exporter_good.doc = doc_good
                result_good = exporter_good.extract_assembly_shape()

                passed_good = result_good is True
                self.results.append({
                    "name": "extract_assembly_shape accepts exactly 3 plates",
                    "passed": passed_good,
                })
                status = "✓" if passed_good else "✗"
                print(f"  {status} extract_assembly_shape accepts exactly 3 "
                      f"plates: returned {result_good}")
            finally:
                App.closeDocument(doc_good.Name)

            # Case 2 ("bad", mirrors the real issue): expected plates plus
            # an unexpected leftover clutter object.
            doc_bad = App.newDocument("Issue55RegressionBad")
            try:
                for name in ("Top_Plate", "Middle_Plate", "Bottom_Plate",
                             "Top_Plate001"):
                    obj = doc_bad.addObject("Part::Feature", name)
                    obj.Shape = Part.makeBox(10, 10, 10)
                doc_bad.recompute()

                exporter_bad = AssemblyExporter()
                exporter_bad.doc = doc_bad
                result_bad = exporter_bad.extract_assembly_shape()

                passed_bad = result_bad is False
                self.results.append({
                    "name": "extract_assembly_shape rejects unexpected Top_Plate001",
                    "passed": passed_bad,
                })
                status = "✓" if passed_bad else "✗"
                print(f"  {status} extract_assembly_shape rejects unexpected "
                      f"Top_Plate001: returned {result_bad}")
            finally:
                App.closeDocument(doc_bad.Name)

        except Exception as e:
            print(f"✗ Error running issue #55 shape filter regression test: {e}")
            self.results.append({
                "name": "extract_assembly_shape issue #55 regression",
                "passed": False,
            })

    def test_middle_plate_z_position_regression(self) -> None:
        """Test: MIDDLE_PLATE_SPECS["z_position"] matches the live document.

        Regression test for issue #56: 02_position_servo.py and
        03_link_servo_to_assembly.py each hardcode Middle_Plate's Z position
        as MIDDLE_PLATE_SPECS["z_position"] rather than reading it from the
        document. That constant previously drifted to a stale 4.0 after
        Middle_Plate was manually moved to Z=6.0 in
        plates_servo_assembled.FCStd, silently corrupting the reported
        "servo below Middle_Plate" clearance in servo_placement.json /
        servo_link_config.json without failing any pass/fail check.

        This opens the real assembly document, reads Middle_Plate's live Z,
        and asserts both scripts' hardcoded constants still match it —
        catching this exact class of drift the next time it recurs, without
        requiring the scripts themselves to read the value live (explicitly
        out of scope per issue #56).
        """
        doc = None
        try:
            from importlib.util import spec_from_file_location, module_from_spec

            def load_module(filename: str, module_name: str):
                spec = spec_from_file_location(
                    module_name, str(self.script_dir / filename)
                )
                module = module_from_spec(spec)
                spec.loader.exec_module(module)
                return module

            position_module = load_module(
                "02_position_servo.py", "position_servo_regression"
            )
            link_module = load_module(
                "03_link_servo_to_assembly.py", "link_servo_regression"
            )

            doc_path = self.script_dir / "plates_servo_assembled.FCStd"
            if not doc_path.exists():
                print(f"✗ Assembly file not found: {doc_path}")
                self.results.append({
                    "name": "Middle_Plate z_position matches live document (issue #56)",
                    "passed": False,
                })
                return

            doc = App.openDocument(str(doc_path))

            middle_plate = None
            for obj in doc.Objects:
                if obj.Name == "Middle_Plate":
                    middle_plate = obj
                    break

            if middle_plate is None:
                print("✗ Middle_Plate not found in document")
                self.results.append({
                    "name": "Middle_Plate z_position matches live document (issue #56)",
                    "passed": False,
                })
                return

            live_z = middle_plate.Placement.Base.z

            calc_z = position_module.ServoPositionCalculator.MIDDLE_PLATE_SPECS["z_position"]
            link_z = link_module.ServoLinkManager.MIDDLE_PLATE_SPECS["z_position"]

            calc_passed = abs(live_z - calc_z) < 1e-6
            link_passed = abs(live_z - link_z) < 1e-6

            self.results.append({
                "name": "02_position_servo.py MIDDLE_PLATE_SPECS.z_position matches live Middle_Plate.Placement.Position.z (issue #56)",
                "passed": calc_passed,
            })
            self.results.append({
                "name": "03_link_servo_to_assembly.py MIDDLE_PLATE_SPECS.z_position matches live Middle_Plate.Placement.Position.z (issue #56)",
                "passed": link_passed,
            })

            status = "✓" if calc_passed else "✗"
            print(f"  {status} 02_position_servo.py z_position={calc_z} vs live={live_z}")
            status = "✓" if link_passed else "✗"
            print(f"  {status} 03_link_servo_to_assembly.py z_position={link_z} vs live={live_z}")

        except Exception as e:
            print(f"✗ Error running issue #56 Middle_Plate z_position regression test: {e}")
            self.results.append({
                "name": "Middle_Plate z_position issue #56 regression",
                "passed": False,
            })
        finally:
            if doc is not None:
                try:
                    App.closeDocument(doc.Name)
                except Exception:
                    pass

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
