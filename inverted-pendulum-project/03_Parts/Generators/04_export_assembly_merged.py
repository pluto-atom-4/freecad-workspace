#!/usr/bin/env python3
"""
Phase 4: Export/Merge Servo Motor Assembly

Merges and exports the complete assembly (plates + servo) to multiple formats.

Process:
1. Load plates_assembled.FCStd (includes external servo link)
2. Load external servo.step file as temporary Part
3. Merge all geometry into single compound (plates + servo)
4. Export to STEP format (ISO 10303-21 AP203)
5. Export to STL format (binary mesh)
6. Export to 3MF format (optional, modern 3D format)
7. Validate merged geometry (solid, no gaps)
8. Create export_metadata.json with statistics

Output:
- plates_assembled_with_servo.step (STEP assembly, ~2.0-2.5 MB)
- plates_assembled_with_servo.stl (STL mesh, ~1.8-2.0 MB)
- plates_assembled_with_servo.3mf (3MF format, optional)
- export_metadata.json (metadata, file sizes, geometry stats, validation results)

Validation:
- Merged geometry is solid (no gaps/intersections)
- File sizes match expectations
- Servo placement pipeline integrity: the Placement actually applied to the
  exported servo geometry matches the frozen EXPECTED_SERVO_POSITION
  reference (±0.01mm tolerance). This is NOT an independent physical-fit
  check -- it confirms the export pipeline didn't silently drop or corrupt
  the placement sourced from servo_placement.json; the underlying physical
  fit against Middle_Plate hole B was verified separately (see #16).
- Mesh quality (vertex/face/triangle counts)
"""

import sys
import json
import time
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any, List, Tuple
from datetime import datetime

try:
    import FreeCAD as App
    import Part
    import Mesh
    import MeshPart
    from FreeCAD import Vector, Placement, Rotation
except ImportError:
    print("ERROR: FreeCAD Python modules not available.")
    print("This script must be run with FreeCAD's Python interpreter:")
    print("  freecad --python script.py")
    print("Or in FreeCAD Python console: exec(open('script.py').read())")
    sys.exit(1)


# __file__ is not defined when this script is run the way this project's
# tooling actually invokes it against freecadcmd -- e.g.
# `freecadcmd -c "exec(open('04_export_assembly_merged.py').read())"`
# (see run_export.sh / README.md) -- so resolve the script directory once
# here, with the same NameError fallback `run()` already used, and reuse it
# everywhere instead of each method separately calling Path(__file__).parent.
try:
    SCRIPT_DIR = Path(__file__).parent
except NameError:
    SCRIPT_DIR = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"


@dataclass
class GeometryStats:
    """Statistics about exported geometry"""
    vertices: int = 0
    edges: int = 0
    faces: int = 0
    triangles: int = 0
    volume: float = 0.0
    surface_area: float = 0.0
    bounding_box: Dict[str, float] = None

    def __post_init__(self):
        if self.bounding_box is None:
            self.bounding_box = {}

    def to_dict(self) -> dict:
        """Convert to dictionary"""
        return {
            "vertices": self.vertices,
            "edges": self.edges,
            "faces": self.faces,
            "triangles": self.triangles,
            "volume_mm3": round(self.volume, 2),
            "surface_area_mm2": round(self.surface_area, 2),
            "bounding_box": self.bounding_box,
        }


@dataclass
class ValidationResult:
    """Validation check result"""
    check_name: str
    passed: bool
    details: str
    value: Optional[float] = None
    tolerance: Optional[float] = None

    def to_dict(self) -> dict:
        """Convert to dictionary"""
        result = {
            "check": self.check_name,
            "passed": self.passed,
            "details": self.details,
        }
        if self.value is not None:
            result["value"] = round(self.value, 3)
        if self.tolerance is not None:
            result["tolerance"] = round(self.tolerance, 3)
        return result


class AssemblyExporter:
    """Export merged servo motor assembly to multiple formats"""

    # Target file sizes (approximate, in MB)
    EXPECTED_STEP_SIZE = 2.25  # 1.8-2.5 MB
    EXPECTED_STL_SIZE = 1.26   # 1.0-1.5 MB (reduced-servo pipeline)

    # STL tessellation deflection -- project's established 1.0mm
    # visual-mesh convention, explicit rather than Mesh.Mesh()'s
    # undocumented default.
    STL_LINEAR_DEFLECTION = 1.0   # mm
    STL_ANGULAR_DEFLECTION = 0.5  # radians

    # Servo placement tolerance (from Phase 3)
    SERVO_POSITION_TOLERANCE = 0.01  # mm

    # Expected servo placement, as a fallback if servo_placement.json can't
    # be read. Verified live-document placement (2026-09-02, via live
    # FreeCAD MCP inspection, ~0.03-0.04mm fit to Middle_Plate hole B).
    # load_servo_placement() overrides this instance attribute from
    # servo_placement.json's "placement" block when available -- see
    # 03_link_servo_to_assembly.py's load_placement_json() for the same
    # convention.
    EXPECTED_SERVO_POSITION = {
        "x": -150.1217,
        "y": -141.9987,
        "z": -3.1,
    }

    def __init__(self):
        """Initialize exporter"""
        self.doc = None
        self.doc_path = None
        self.assembly_shape = None
        self.servo_shape = None
        self.merged_shape = None
        self.validations = []
        self.geometry_stats = GeometryStats()
        self.export_stats = {}
        self.servo_step_path = None
        # Overridden per-instance by load_servo_placement() when
        # servo_placement.json is available; otherwise stays the class
        # fallback above.
        self.EXPECTED_SERVO_POSITION = dict(self.EXPECTED_SERVO_POSITION)

    def resolve_servo_step_path(self) -> bool:
        """Resolve servo STEP file path.

        Prefers the reduced-size servo STEP (~2.92 MB) so the merged export
        stays within the STEP size budget (1.5-3.0 MB, see
        test_export_file_sizes). The full-size servo STEP (~36 MB) is only
        used as a loudly-warned fallback if the reduced file is missing --
        picking it silently would blow the size budget.
        """
        try:
            mechanical_dir = SCRIPT_DIR.parent / "Mechanical"

            reduced_path = mechanical_dir / "feetech-STS3032-reduced.step"
            full_path = mechanical_dir / "feetech-STS3032.step"

            if reduced_path.exists():
                self.servo_step_path = reduced_path
                print(f"✓ Found reduced servo STEP file: {reduced_path}")
                return True
            elif full_path.exists():
                self.servo_step_path = full_path
                print(f"⚠ WARNING: Reduced servo STEP file not found at:")
                print(f"    {reduced_path}")
                print(f"  Falling back to FULL-SIZE servo STEP file: {full_path}")
                print(f"  This is ~36 MB and will likely blow the STEP export size budget!")
                return True
            else:
                print(f"⚠ WARNING: No servo STEP file found at:")
                print(f"    {reduced_path}")
                print(f"    {full_path}")
                print(f"  Export will include assembly without separate servo geometry")
                return False

        except Exception as e:
            print(f"ERROR resolving servo STEP path: {e}")
            return False

    def load_document(self, doc_path: str) -> bool:
        """Load FreeCAD assembly document"""
        try:
            path = Path(doc_path)
            if not path.exists():
                print(f"ERROR: Document not found: {doc_path}")
                return False

            self.doc = App.openDocument(str(path))
            self.doc_path = path
            print(f"✓ Loaded: {path.name}")
            return True

        except Exception as e:
            print(f"ERROR loading document: {e}")
            return False

    def extract_assembly_shape(self) -> bool:
        """Extract merged shape from all objects in assembly"""
        try:
            if not self.doc:
                return False

            shapes_to_merge = []

            # Collect all shape objects from document
            for obj in self.doc.Objects:
                try:
                    # Skip non-visual objects
                    if not hasattr(obj, "Shape"):
                        continue

                    shape = obj.Shape
                    if shape and shape.Vertexes:  # Check if shape has geometry
                        shapes_to_merge.append((obj.Name, shape))
                        print(f"  ✓ Extracted {obj.Name} ({len(shape.Faces)} faces)")

                except Exception as obj_e:
                    # Skip objects that can't be converted
                    continue

            if not shapes_to_merge:
                print(f"ERROR: No valid shapes found in assembly")
                return False

            print(f"\n✓ Found {len(shapes_to_merge)} shape(s) to merge")

            # Create compound from all shapes
            compound_shapes = [shape for _, shape in shapes_to_merge]
            self.assembly_shape = Part.makeCompound(compound_shapes)

            # Calculate stats
            self._calculate_geometry_stats(self.assembly_shape)

            print(f"  Combined: {len(self.assembly_shape.Faces)} faces, "
                  f"{len(self.assembly_shape.Edges)} edges, "
                  f"{len(self.assembly_shape.Vertexes)} vertices")

            return True

        except Exception as e:
            print(f"ERROR extracting assembly shape: {e}")
            return False

    def load_servo_placement(self) -> Placement:
        """Load the servo placement from Phase 2's servo_placement.json
        ("placement" block: x/y/z in mm, roll/pitch/yaw in degrees).

        The live document uses an identity rotation for the servo (roll,
        pitch and yaw are all 0.0 in servo_placement.json), matching the
        convention in 03_link_servo_to_assembly.py's apply_placement().
        Falls back to the verified live-document values (2026-09-02, via
        live FreeCAD MCP inspection, ~0.03-0.04mm fit to Middle_Plate hole B,
        also mirrored in EXPECTED_SERVO_POSITION above) if the JSON is
        missing or malformed.

        Also updates self.EXPECTED_SERVO_POSITION so servo position
        validation checks against the same source used for the transform.
        """
        fallback = dict(self.EXPECTED_SERVO_POSITION)
        json_path = SCRIPT_DIR / "servo_placement.json"

        x, y, z = fallback["x"], fallback["y"], fallback["z"]
        try:
            if json_path.exists():
                with open(json_path, 'r') as f:
                    data = json.load(f)
                placement = data.get("placement", {})
                x = placement.get("x", x)
                y = placement.get("y", y)
                z = placement.get("z", z)
                print(f"✓ Loaded servo placement from {json_path.name}: "
                      f"X={x:.4f}, Y={y:.4f}, Z={z:.4f} mm")
            else:
                print(f"⚠ WARNING: {json_path.name} not found, using verified "
                      f"fallback placement: X={x:.4f}, Y={y:.4f}, Z={z:.4f} mm")
        except Exception as e:
            print(f"⚠ WARNING: Could not read {json_path.name} ({e}), using "
                  f"verified fallback placement: X={x:.4f}, Y={y:.4f}, Z={z:.4f} mm")

        self.EXPECTED_SERVO_POSITION = {"x": x, "y": y, "z": z}

        return Placement(Vector(x, y, z), Rotation(Vector(0, 0, 1), 0))

    def load_servo_geometry(self) -> bool:
        """Load servo geometry from external STEP file"""
        try:
            if not self.servo_step_path:
                print(f"⚠ Servo STEP file not available, skipping separate load")
                return True  # Not fatal, assembly only

            if not self.servo_step_path.exists():
                print(f"⚠ Servo STEP file not found: {self.servo_step_path}")
                return True  # Not fatal

            # Import servo shape from STEP file
            self.servo_shape = Part.Shape()
            self.servo_shape.read(str(self.servo_step_path))

            # The servo STEP file is in its own native coordinate frame,
            # wildly different from the plate assembly's frame (~150-200mm
            # offset if left untransformed). Apply the verified mount
            # placement so the servo lands in the plate assembly's frame.
            servo_placement = self.load_servo_placement()
            self.servo_shape.Placement = servo_placement

            print(f"✓ Loaded servo geometry:")
            print(f"  Vertices: {len(self.servo_shape.Vertexes)}")
            print(f"  Edges: {len(self.servo_shape.Edges)}")
            print(f"  Faces: {len(self.servo_shape.Faces)}")
            print(f"✓ Applied servo placement: "
                  f"X={servo_placement.Base.x:.4f}, "
                  f"Y={servo_placement.Base.y:.4f}, "
                  f"Z={servo_placement.Base.z:.4f} mm")

            return True

        except Exception as e:
            print(f"⚠ WARNING: Could not load servo geometry: {e}")
            print(f"  Export will include assembly without separate servo shape")
            return True  # Not fatal

    def merge_assembly_and_servo(self) -> bool:
        """Merge assembly and servo into single compound"""
        try:
            if not self.assembly_shape:
                print(f"ERROR: Assembly shape not extracted")
                return False

            shapes = [self.assembly_shape]

            # Add servo shape if available
            if self.servo_shape:
                shapes.append(self.servo_shape)
                print(f"✓ Merging assembly with servo geometry")
            else:
                print(f"ℹ Using assembly shape only (servo shape not available)")

            # Create merged compound
            self.merged_shape = Part.makeCompound(shapes)

            # Recalculate geometry stats from the final merged shape (plates
            # + servo), not just the plates-only assembly_shape computed in
            # extract_assembly_shape() -- export_metadata.json's
            # geometry_stats.bounding_box needs to reflect what's actually
            # exported so a missing/wrong servo placement is visible there.
            self._calculate_geometry_stats(self.merged_shape)

            print(f"✓ Merged shape created:")
            print(f"  Total faces: {len(self.merged_shape.Faces)}")
            print(f"  Total edges: {len(self.merged_shape.Edges)}")
            print(f"  Total vertices: {len(self.merged_shape.Vertexes)}")

            return True

        except Exception as e:
            print(f"ERROR merging shapes: {e}")
            return False

    def export_step(self, output_path: Optional[str] = None) -> bool:
        """Export merged geometry to STEP format"""
        try:
            if not self.merged_shape:
                print(f"ERROR: No merged shape available")
                return False

            if output_path is None:
                output_dir = SCRIPT_DIR
                output_path = output_dir / "plates_assembled_with_servo.step"

            output_path = Path(output_path)

            # Export to STEP (ISO 10303-21, AP203)
            start_time = time.time()
            self.merged_shape.exportStep(str(output_path))
            elapsed = time.time() - start_time

            # Verify export
            if not output_path.exists():
                print(f"ERROR: STEP export failed (file not created)")
                return False

            file_size = output_path.stat().st_size
            size_mb = file_size / (1024 * 1024)
            size_kb = file_size / 1024

            print(f"✓ Exported STEP file: {output_path}")
            print(f"  File size: {size_mb:.2f} MB ({size_kb:.1f} KB)")
            print(f"  Export time: {elapsed:.2f}s")

            self.export_stats["step"] = {
                "path": str(output_path),
                "size_bytes": file_size,
                "size_mb": round(size_mb, 2),
                "export_time_s": round(elapsed, 2),
            }

            # Validate file size
            size_ok = 1.5 < size_mb < 3.0  # Reasonable range
            self.validations.append(ValidationResult(
                check_name="STEP file size",
                passed=size_ok,
                details=f"File size: {size_mb:.2f} MB (expected: ~{self.EXPECTED_STEP_SIZE} MB)",
                value=size_mb,
                tolerance=self.EXPECTED_STEP_SIZE
            ))

            return True

        except Exception as e:
            print(f"ERROR exporting STEP: {e}")
            return False

    def export_stl(self, output_path: Optional[str] = None) -> bool:
        """Export merged geometry to STL format"""
        try:
            if not self.merged_shape:
                print(f"ERROR: No merged shape available")
                return False

            if output_path is None:
                output_dir = SCRIPT_DIR
                output_path = output_dir / "plates_assembled_with_servo.stl"

            output_path = Path(output_path)

            # Create mesh from compound by meshing individual solids, with
            # explicit tessellation deflection (project's established 1.0mm
            # visual-mesh convention) instead of Mesh.Mesh()'s undocumented
            # default deflection.
            start_time = time.time()
            solids = list(self.merged_shape.Solids)

            if solids:
                print(f"  Meshing {len(solids)} solid(s) "
                      f"(LinearDeflection={self.STL_LINEAR_DEFLECTION}mm, "
                      f"AngularDeflection={self.STL_ANGULAR_DEFLECTION}rad)...")
                mesh = Mesh.Mesh()  # Empty mesh to accumulate

                for i, solid in enumerate(solids):
                    try:
                        component_mesh = MeshPart.meshFromShape(
                            Shape=solid,
                            LinearDeflection=self.STL_LINEAR_DEFLECTION,
                            AngularDeflection=self.STL_ANGULAR_DEFLECTION,
                        )
                        mesh.addMesh(component_mesh)
                        print(f"    ✓ Meshed component {i+1}/{len(solids)} ({len(component_mesh.Facets)} triangles)")
                    except Exception as component_e:
                        print(f"    ⚠ Could not mesh component {i+1}: {component_e}")
                        continue

                if mesh.CountFacets == 0:
                    raise RuntimeError("No components could be meshed")
            else:
                # No solids (e.g. shell/face-only shape) - mesh directly
                mesh = MeshPart.meshFromShape(
                    Shape=self.merged_shape,
                    LinearDeflection=self.STL_LINEAR_DEFLECTION,
                    AngularDeflection=self.STL_ANGULAR_DEFLECTION,
                )
            mesh_creation_time = time.time() - start_time

            # Export mesh to STL (binary format)
            mesh.write(str(output_path))
            export_time = time.time() - mesh_creation_time - start_time

            # Verify export
            if not output_path.exists():
                print(f"ERROR: STL export failed (file not created)")
                return False

            file_size = output_path.stat().st_size
            size_mb = file_size / (1024 * 1024)
            size_kb = file_size / 1024

            print(f"✓ Exported STL file: {output_path}")
            print(f"  File size: {size_mb:.2f} MB ({size_kb:.1f} KB)")
            print(f"  Mesh triangles: {len(mesh.Facets)}")
            print(f"  Mesh creation time: {mesh_creation_time:.2f}s")
            print(f"  Total export time: {export_time:.2f}s")

            self.export_stats["stl"] = {
                "path": str(output_path),
                "size_bytes": file_size,
                "size_mb": round(size_mb, 2),
                "triangles": len(mesh.Facets),
                "export_time_s": round(export_time, 2),
            }

            # Validate file size
            size_ok = 1.0 < size_mb < 1.5  # Reasonable range (reduced-servo pipeline)
            self.validations.append(ValidationResult(
                check_name="STL file size",
                passed=size_ok,
                details=f"File size: {size_mb:.2f} MB (expected: ~{self.EXPECTED_STL_SIZE} MB)",
                value=size_mb,
                tolerance=self.EXPECTED_STL_SIZE
            ))

            return True

        except Exception as e:
            print(f"ERROR exporting STL: {e}")
            return False

    def export_3mf(self, output_path: Optional[str] = None) -> bool:
        """Export merged geometry to 3MF format (optional)"""
        try:
            if not self.merged_shape:
                print(f"ERROR: No merged shape available")
                return False

            if output_path is None:
                output_dir = SCRIPT_DIR
                output_path = output_dir / "plates_assembled_with_servo.3mf"

            output_path = Path(output_path)

            # Try to export to 3MF if supported
            try:
                # Create mesh and export as 3MF
                mesh = Mesh.Mesh(self.merged_shape)

                # FreeCAD 3MF export (if available)
                # Note: 3MF support depends on FreeCAD version
                import importlib
                try:
                    import THREE_MF
                    start_time = time.time()
                    # Export using 3MF module if available
                    mesh.write(str(output_path))
                    elapsed = time.time() - start_time

                    if output_path.exists():
                        file_size = output_path.stat().st_size
                        size_mb = file_size / (1024 * 1024)

                        print(f"✓ Exported 3MF file: {output_path}")
                        print(f"  File size: {size_mb:.2f} MB")
                        print(f"  Export time: {elapsed:.2f}s")

                        self.export_stats["3mf"] = {
                            "path": str(output_path),
                            "size_bytes": file_size,
                            "size_mb": round(size_mb, 2),
                            "export_time_s": round(elapsed, 2),
                        }
                        return True
                except ImportError:
                    print(f"⚠ 3MF module not available, skipping 3MF export")
                    return True

            except Exception as inner_e:
                print(f"⚠ 3MF export not supported: {inner_e}")
                print(f"  (This is optional, proceeding without 3MF)")
                return True

        except Exception as e:
            print(f"⚠ WARNING: 3MF export failed: {e}")
            return True  # Not fatal

    def validate_merged_geometry(self) -> bool:
        """Validate that merged geometry is solid and valid"""
        try:
            print(f"✓ Geometry Validation:")

            if not self.merged_shape:
                return False

            # Check if shape is closed (solid)
            is_closed = self.merged_shape.isClosed()
            self.validations.append(ValidationResult(
                check_name="Geometry is closed (solid)",
                passed=is_closed,
                details=f"Shape closure: {is_closed}"
            ))
            status = "✓" if is_closed else "⚠"
            print(f"  {status} Geometry is closed: {is_closed}")

            # Check for valid topology
            has_faces = len(self.merged_shape.Faces) > 0
            self.validations.append(ValidationResult(
                check_name="Has valid faces",
                passed=has_faces,
                details=f"Face count: {len(self.merged_shape.Faces)}"
            ))
            status = "✓" if has_faces else "✗"
            print(f"  {status} Has valid faces: {len(self.merged_shape.Faces)}")

            # Check volume (non-zero for valid solid)
            try:
                volume = self.merged_shape.Volume
                has_volume = volume > 0
                self.validations.append(ValidationResult(
                    check_name="Has non-zero volume",
                    passed=has_volume,
                    details=f"Volume: {volume:.2f} mm³",
                    value=volume
                ))
                status = "✓" if has_volume else "✗"
                print(f"  {status} Volume: {volume:.2f} mm³")
            except:
                print(f"  ⚠ Could not calculate volume")

            return True

        except Exception as e:
            print(f"ERROR validating merged geometry: {e}")
            return False

    def validate_servo_position(self) -> bool:
        """Validate the export pipeline applied Phase 2/3's canonical servo
        placement to the exported servo geometry.

        Scope note: this is a pipeline-integrity check, not an independent
        physical-fit measurement. It confirms the Placement actually assigned
        to self.servo_shape (sourced from servo_placement.json, see
        load_servo_placement()) still matches the reference value frozen in
        the AssemblyExporter.EXPECTED_SERVO_POSITION class constant -- the
        ~0.03-0.04mm live-FreeCAD-MCP-verified fit against Middle_Plate hole B
        recorded there and in servo_placement.json's own "placement_source"
        field. That physical verification is not redone here: this STEP
        file's local BoundBox origin is an arbitrary leftover of the source
        assembly it was extracted from, not a documented feature (see #16).
        """
        try:
            print(f"✓ Servo Position Validation:")

            if not self.servo_shape:
                print(f"  ℹ Servo shape not available, skipping position check")
                return True

            # Compare against the class-level constant, NOT self.EXPECTED_
            # SERVO_POSITION -- load_servo_placement() has already overwritten
            # the instance attribute with this same run's servo_placement.json
            # values, so comparing against it would always trivially match.
            reference = type(self).EXPECTED_SERVO_POSITION
            applied = self.servo_shape.Placement.Base

            errors = {axis: abs(getattr(applied, axis) - reference[axis])
                      for axis in ("x", "y", "z")}
            max_error = max(errors.values())
            position_ok = max_error <= self.SERVO_POSITION_TOLERANCE

            self.validations.append(ValidationResult(
                check_name="Servo placement matches verified reference",
                passed=position_ok,
                details=(
                    f"Applied (X={applied.x:.3f}, Y={applied.y:.3f}, "
                    f"Z={applied.z:.3f}) mm vs frozen reference "
                    f"(X={reference['x']:.3f}, Y={reference['y']:.3f}, "
                    f"Z={reference['z']:.3f}) mm; max axis error: "
                    f"{max_error:.3f} mm (tolerance: "
                    f"{self.SERVO_POSITION_TOLERANCE} mm)"
                ),
                value=max_error,
                tolerance=self.SERVO_POSITION_TOLERANCE
            ))
            status = "✓" if position_ok else "⚠"
            print(f"  {status} Servo placement max-axis error: {max_error:.3f} mm "
                  f"(tolerance: {self.SERVO_POSITION_TOLERANCE} mm)")

            return True

        except Exception as e:
            print(f"ERROR validating servo position: {e}")
            return False

    def _calculate_geometry_stats(self, shape) -> None:
        """Calculate geometry statistics from shape"""
        try:
            self.geometry_stats.vertices = len(shape.Vertexes)
            self.geometry_stats.edges = len(shape.Edges)
            self.geometry_stats.faces = len(shape.Faces)

            # Calculate volume and surface area
            try:
                self.geometry_stats.volume = shape.Volume
            except:
                self.geometry_stats.volume = 0.0

            try:
                self.geometry_stats.surface_area = shape.Area
            except:
                self.geometry_stats.surface_area = 0.0

            # Get bounding box
            bbox = shape.BoundBox
            self.geometry_stats.bounding_box = {
                "x_min": round(bbox.XMin, 2),
                "x_max": round(bbox.XMax, 2),
                "y_min": round(bbox.YMin, 2),
                "y_max": round(bbox.YMax, 2),
                "z_min": round(bbox.ZMin, 2),
                "z_max": round(bbox.ZMax, 2),
                "x_length": round(bbox.XLength, 2),
                "y_length": round(bbox.YLength, 2),
                "z_length": round(bbox.ZLength, 2),
            }

        except Exception as e:
            print(f"⚠ WARNING: Could not calculate geometry stats: {e}")

    def save_metadata(self, output_path: Optional[str] = None) -> bool:
        """Save export metadata to JSON"""
        try:
            if output_path is None:
                output_dir = SCRIPT_DIR
                output_path = output_dir / "export_metadata.json"

            output_path = Path(output_path)

            # Prepare metadata
            metadata = {
                "phase": 4,
                "title": "Export/Merge Assembly Metadata",
                "timestamp": datetime.now().isoformat(),
                "source_assembly": self.doc_path.name if self.doc_path else None,
                "servo_step_file": str(self.servo_step_path) if self.servo_step_path else None,
                "servo_geometry_included": self.servo_shape is not None,
                "exports": self.export_stats,
                "geometry_stats": self.geometry_stats.to_dict(),
                "validations": [v.to_dict() for v in self.validations],
                "all_validations_passed": all(v.passed for v in self.validations),
                "export_notes": {
                    "step_format": "ISO 10303-21 (AP203)",
                    "stl_format": "Binary mesh format",
                    "3mf_format": "Modern 3D model format (optional)",
                    "step_expected_size_mb": self.EXPECTED_STEP_SIZE,
                    "stl_expected_size_mb": self.EXPECTED_STL_SIZE,
                    "servo_tolerance_mm": self.SERVO_POSITION_TOLERANCE,
                },
                "usage_examples": {
                    "step_export": "plates_assembled_with_servo.step (full assembly, CAD compatible)",
                    "stl_export": "plates_assembled_with_servo.stl (mesh format, 3D printing)",
                    "3mf_export": "plates_assembled_with_servo.3mf (modern 3D format, if available)",
                    "re_export_programmatically": (
                        "from 04_export_assembly_merged import AssemblyExporter\n"
                        "exporter = AssemblyExporter()\n"
                        "exporter.run()\n"
                    ),
                },
            }

            with open(output_path, 'w') as f:
                json.dump(metadata, f, indent=2)

            print(f"✓ Saved export metadata: {output_path}")
            return True

        except Exception as e:
            print(f"ERROR saving metadata: {e}")
            return False

    def run(self) -> bool:
        """Execute full export process"""
        print("=" * 70)
        print("PHASE 4: EXPORT/MERGE SERVO MOTOR ASSEMBLY")
        print("=" * 70)
        print()

        script_dir = SCRIPT_DIR

        doc_path = script_dir / "plates_servo_assembled.FCStd"

        # Resolve servo STEP path
        print("Step 1: Resolve servo STEP file path")
        print("-" * 70)
        self.resolve_servo_step_path()
        print()

        # Load assembly document
        print("Step 2: Load FreeCAD assembly document")
        print("-" * 70)
        if not self.load_document(str(doc_path)):
            return False
        print()

        # Extract assembly shape
        print("Step 3: Extract assembly geometry")
        print("-" * 70)
        if not self.extract_assembly_shape():
            return False
        print()

        # Load servo geometry
        print("Step 4: Load servo geometry from STEP file")
        print("-" * 70)
        if not self.load_servo_geometry():
            print("WARNING: Could not load servo geometry")
        print()

        # Merge geometries
        print("Step 5: Merge assembly and servo")
        print("-" * 70)
        if not self.merge_assembly_and_servo():
            return False
        print()

        # Export to STEP
        print("Step 6: Export to STEP format")
        print("-" * 70)
        if not self.export_step(script_dir / "plates_assembled_with_servo.step"):
            print("ERROR: STEP export failed")
            return False
        print()

        # Export to STL
        print("Step 7: Export to STL format")
        print("-" * 70)
        if not self.export_stl(script_dir / "plates_assembled_with_servo.stl"):
            print("ERROR: STL export failed")
            return False
        print()

        # Export to 3MF (optional)
        print("Step 8: Export to 3MF format (optional)")
        print("-" * 70)
        if not self.export_3mf(script_dir / "plates_assembled_with_servo.3mf"):
            print("WARNING: 3MF export not available")
        print()

        # Validate merged geometry
        print("Step 9: Validate merged geometry")
        print("-" * 70)
        if not self.validate_merged_geometry():
            print("WARNING: Geometry validation had issues")
        print()

        # Validate servo position
        print("Step 10: Validate servo position")
        print("-" * 70)
        if not self.validate_servo_position():
            print("WARNING: Servo position validation failed")
        print()

        # Save metadata
        print("Step 11: Save export metadata")
        print("-" * 70)
        if not self.save_metadata(script_dir / "export_metadata.json"):
            print("WARNING: Could not save metadata")
        print()

        print("=" * 70)
        print("✓ Phase 4 Complete")
        print("=" * 70)

        return True


def main():
    """Main entry point"""
    exporter = AssemblyExporter()
    success = exporter.run()

    # Close document
    if exporter.doc:
        try:
            App.closeDocument(exporter.doc.Name)
        except:
            pass

    return 0 if success else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
