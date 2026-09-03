#!/usr/bin/env python3
"""
Phase 3: Servo Motor Mesh Assembly

Integrates servo motor mesh geometry into the plate assembly, matching the
live, human-approved document structure.

NOTE (2026-09-02, issue #3 reconciliation): This script previously built a
`Part::Body` named "Servo_Motor" (STEP import, falling back to a box
approximation) with a Yaw-Pitch-Roll rotation derived from
`servo_placement.json`, targeting `plates_assembled.FCStd`. None of that
matches the live document: the servo is modeled as two `Mesh::Feature`
objects (visual + collision-proxy STL) grouped under an `App::Part` named
"STS3032_Mount", both placed identically with an IDENTITY rotation.

Process:
1. Load plates_servo_assembled.FCStd and verify plate objects
2. Load servo_placement.json from Phase 2 output
3. Create App::Part container named "STS3032_Mount"
4. Import the visual and collision-proxy STL meshes as its children
5. Apply the same placement (position + identity rotation) to both meshes
6. Validate mesh presence and placement
7. Save updated assembly
8. Export servo_link_config.json for reference

Output:
- Updated plates_servo_assembled.FCStd (with STS3032_Mount Part)
- servo_link_config.json: Link configuration and placement metadata
- Console log: Placement and validation details
"""

import sys
import json
import math
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any

try:
    import FreeCAD as App
    import Part
    import Mesh
    from FreeCAD import Vector, Placement, Rotation
except ImportError:
    print("ERROR: FreeCAD Python modules not available.")
    print("This script must be run with FreeCAD's Python interpreter:")
    print("  freecad --python script.py")
    print("Or in FreeCAD Python console: exec(open('script.py').read())")
    sys.exit(1)


@dataclass
class PlacementData:
    """Servo placement matrix from Phase 2"""
    x: float          # mm
    y: float          # mm
    z: float          # mm
    roll: float       # degrees
    pitch: float      # degrees
    yaw: float        # degrees
    z_offset: float   # kept for schema compatibility; see 02_position_servo.py docstring

    def to_dict(self) -> dict:
        """Convert to dictionary"""
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'roll': self.roll,
            'pitch': self.pitch,
            'yaw': self.yaw,
            'z_offset': self.z_offset,
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
            result["value"] = self.value
        if self.tolerance is not None:
            result["tolerance"] = self.tolerance
        return result


class ServoLinkManager:
    """Assemble servo motor meshes into the plate assembly as STS3032_Mount"""

    # Servo specifications
    SERVO_SPECS = {
        "model": "Feetech STS3032",
        "body_length": 32.0,        # mm
        "body_width": 12.0,         # mm
        "body_height": 28.0,        # mm
        "shaft_length": 10.0,       # mm
        "shaft_offset_x": 0.0,      # mm from center
        "shaft_offset_y": 14.0,     # mm from center (rear of servo)
    }

    # Plate specifications (live document values, verified 2026-09-02)
    MIDDLE_PLATE_SPECS = {
        "thickness": 2.5,           # mm
        "z_position": 4.0,          # mm (Middle_Plate.Placement.Position.z)
    }
    TOP_PLATE_Z = 6.0                # mm (Top_Plate.Placement.Position.z)
    BOTTOM_PLATE_Z = 3.0             # mm (Bottom_Plate.Placement.Position.z)

    # Validation tolerances
    PLACEMENT_TOLERANCE = 1.0       # mm (tolerance for placement accuracy)
    CLEARANCE_MIN = 5.0             # mm (minimum clearance to other plates)

    # STS3032_Mount mesh children: (object name, filename under 03_Parts/Mechanical)
    SERVO_MESHES = [
        ("feetech_STS3032_visual_1_0mm", "feetech-STS3032-visual-1.0mm.stl"),
        ("feetech_STS3032_collision_proxy", "feetech-STS3032-collision-proxy.stl"),
    ]

    def __init__(self):
        """Initialize link manager"""
        self.doc = None
        self.plates = {}
        self.servo_part = None
        self.mesh_objects = []
        self.placement_data = None
        self.validations = []
        self.mechanical_dir = None

    def resolve_mechanical_dir(self) -> bool:
        """Resolve the 03_Parts/Mechanical directory holding the servo STL meshes"""
        try:
            try:
                script_dir = Path(__file__).parent
            except NameError:
                script_dir = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"
            self.mechanical_dir = script_dir.parent / "Mechanical"

            if not self.mechanical_dir.exists():
                print(f"⚠ WARNING: Mechanical directory not found: {self.mechanical_dir}")
                return False

            print(f"✓ Mechanical directory: {self.mechanical_dir}")
            return True

        except Exception as e:
            print(f"ERROR resolving Mechanical directory: {e}")
            return False

    def load_document(self, doc_path: str) -> bool:
        """Load FreeCAD assembly document"""
        try:
            path = Path(doc_path)
            if not path.exists():
                print(f"ERROR: Document not found: {doc_path}")
                return False

            self.doc = App.openDocument(str(path))
            print(f"✓ Loaded: {path.name}")
            return True

        except Exception as e:
            print(f"ERROR loading document: {e}")
            return False

    def verify_plates(self) -> bool:
        """Verify that all three plates exist in the document"""
        try:
            required_plates = ["Top_Plate", "Middle_Plate", "Bottom_Plate"]
            found_plates = {}

            for obj in self.doc.Objects:
                if obj.Name in required_plates:
                    found_plates[obj.Name] = obj
                    print(f"  ✓ Found {obj.Name}")

            if len(found_plates) < 3:
                missing = set(required_plates) - set(found_plates.keys())
                print(f"ERROR: Missing plates: {missing}")
                return False

            self.plates = found_plates
            return True

        except Exception as e:
            print(f"ERROR verifying plates: {e}")
            return False

    def load_placement_json(self, json_path: Optional[str] = None) -> bool:
        """Load servo placement data from Phase 2 output"""
        try:
            if json_path is None:
                json_path = Path(__file__).parent / "servo_placement.json"
            else:
                json_path = Path(json_path)

            if not json_path.exists():
                print(f"⚠ WARNING: Placement JSON not found: {json_path}")
                print("  Using verified default placement values")
                # Verified live-document placement (2026-09-02) as fallback
                self.placement_data = PlacementData(
                    x=-150.1217,
                    y=-141.9987,
                    z=-3.1,
                    roll=0.0,
                    pitch=0.0,
                    yaw=0.0,
                    z_offset=0.0,
                )
                return True

            with open(json_path, 'r') as f:
                data = json.load(f)

            if "placement" not in data:
                print(f"ERROR: No placement data in JSON")
                return False

            placement = data["placement"]
            self.placement_data = PlacementData(
                x=placement.get("x", -150.1217),
                y=placement.get("y", -141.9987),
                z=placement.get("z", -3.1),
                roll=placement.get("roll", 0.0),
                pitch=placement.get("pitch", 0.0),
                yaw=placement.get("yaw", 0.0),
                z_offset=placement.get("z_offset", 0.0),
            )

            print(f"✓ Loaded placement data: {json_path}")
            print(f"  Position: X={self.placement_data.x:.2f}, Y={self.placement_data.y:.2f}, Z={self.placement_data.z:.2f} mm")
            print(f"  Rotation: Roll={self.placement_data.roll:.1f}°, Pitch={self.placement_data.pitch:.1f}°, Yaw={self.placement_data.yaw:.1f}°")
            return True

        except Exception as e:
            print(f"ERROR loading placement JSON: {e}")
            return False

    def create_servo_body(self) -> bool:
        """Create (or reuse) App::Part "STS3032_Mount" and import the visual +
        collision-proxy STL meshes as its children, matching the live document
        structure. Idempotent: if STS3032_Mount (and its mesh children) already
        exist — as they do in the live, human-approved document — reuse them
        instead of creating duplicates / auto-renamed objects on re-run."""
        try:
            if not self.mechanical_dir:
                print("ERROR: Mechanical directory not resolved")
                return False

            existing_part = self.doc.getObject("STS3032_Mount")
            if existing_part is not None:
                self.servo_part = existing_part
                print(f"✓ Reusing existing App::Part: STS3032_Mount")
            else:
                self.servo_part = self.doc.addObject("App::Part", "STS3032_Mount")
                print(f"✓ Created App::Part: STS3032_Mount")

            for obj_name, filename in self.SERVO_MESHES:
                existing_mesh = self.doc.getObject(obj_name)
                if existing_mesh is not None:
                    if existing_mesh not in self.servo_part.Group:
                        self.servo_part.addObject(existing_mesh)
                    self.mesh_objects.append(existing_mesh)
                    print(f"  ✓ Reusing existing mesh: {existing_mesh.Name}")
                    continue

                stl_path = self.mechanical_dir / filename
                if not stl_path.exists():
                    print(f"  ⚠ WARNING: Servo STL not found: {stl_path}")
                    continue

                mesh_obj = self.doc.addObject("Mesh::Feature", obj_name)
                mesh_obj.Mesh = Mesh.Mesh(str(stl_path))
                self.servo_part.addObject(mesh_obj)
                self.mesh_objects.append(mesh_obj)
                print(f"  ✓ Imported {filename} as {mesh_obj.Name}")

            if not self.mesh_objects:
                print("ERROR: No servo meshes were imported")
                return False

            return True

        except Exception as e:
            print(f"ERROR creating servo body: {e}")
            return False

    def apply_placement(self) -> bool:
        """Apply the verified placement (position + identity rotation) directly
        to both mesh children — mirroring how the collision proxy was synced
        to the visual mesh in the live document this session, rather than
        deriving each child's placement from separate roll/pitch/yaw math."""
        try:
            if not self.mesh_objects or not self.placement_data:
                print("ERROR: Servo meshes or placement data missing")
                return False

            position = Vector(
                self.placement_data.x,
                self.placement_data.y,
                self.placement_data.z
            )

            # Live document uses an identity rotation for the servo.
            rotation = Rotation(Vector(0, 0, 1), 0)
            placement = Placement(position, rotation)

            for mesh_obj in self.mesh_objects:
                mesh_obj.Placement = placement

            print(f"✓ Applied servo placement to {len(self.mesh_objects)} mesh(es):")
            print(f"  Position: X={position.x:.4f}, Y={position.y:.4f}, Z={position.z:.4f} mm")
            print(f"  Rotation: identity")

            return True

        except Exception as e:
            print(f"ERROR applying placement: {e}")
            return False

    def validate_link_resolution(self) -> bool:
        """Validate that servo meshes were imported correctly"""
        try:
            print(f"✓ Link Resolution Validation:")

            if not self.servo_part:
                self.validations.append(ValidationResult(
                    check_name="STS3032_Mount exists",
                    passed=False,
                    details="STS3032_Mount Part not created"
                ))
                return False

            has_meshes = len(self.mesh_objects) > 0
            self.validations.append(ValidationResult(
                check_name="STS3032_Mount has servo meshes",
                passed=has_meshes,
                details=f"STS3032_Mount contains {len(self.mesh_objects)} mesh object(s)"
            ))
            status = "✓" if has_meshes else "✗"
            print(f"  {status} STS3032_Mount contains {len(self.mesh_objects)} mesh(es)")

            expected_count = len(self.SERVO_MESHES)
            complete = len(self.mesh_objects) == expected_count
            self.validations.append(ValidationResult(
                check_name="Both servo meshes present",
                passed=complete,
                details=f"{len(self.mesh_objects)}/{expected_count} expected meshes imported"
            ))
            status = "✓" if complete else "⚠"
            print(f"  {status} {len(self.mesh_objects)}/{expected_count} expected meshes imported")

            return has_meshes

        except Exception as e:
            print(f"ERROR validating link resolution: {e}")
            return False

    def validate_placement(self) -> bool:
        """Validate servo placement accuracy on both mesh children"""
        try:
            print(f"✓ Placement Validation:")

            if not self.mesh_objects or not self.placement_data:
                return False

            expected = Vector(
                self.placement_data.x,
                self.placement_data.y,
                self.placement_data.z
            )

            all_passed = True
            for mesh_obj in self.mesh_objects:
                actual = mesh_obj.Placement.Base
                distance = expected.distanceToPoint(actual)

                passed = distance < self.PLACEMENT_TOLERANCE
                all_passed = all_passed and passed
                self.validations.append(ValidationResult(
                    check_name=f"Placement accuracy ({mesh_obj.Name})",
                    passed=passed,
                    details=f"Position error: {distance:.3f} mm",
                    value=distance,
                    tolerance=self.PLACEMENT_TOLERANCE
                ))
                status = "✓" if passed else "✗"
                print(f"  {status} {mesh_obj.Name} position error: {distance:.3f} mm (tolerance: {self.PLACEMENT_TOLERANCE} mm)")

            # Confirm both meshes share the exact same placement (synced, as in the live document)
            if len(self.mesh_objects) >= 2:
                base_placement = self.mesh_objects[0].Placement
                synced = all(m.Placement == base_placement for m in self.mesh_objects[1:])
                self.validations.append(ValidationResult(
                    check_name="Visual/collision-proxy placements synced",
                    passed=synced,
                    details="All servo meshes share an identical placement" if synced else "Servo mesh placements diverge"
                ))
                status = "✓" if synced else "✗"
                print(f"  {status} Visual/collision-proxy placements synced: {synced}")
                all_passed = all_passed and synced

            return all_passed

        except Exception as e:
            print(f"ERROR validating placement: {e}")
            return False

    def validate_clearances(self) -> bool:
        """Validate clearances between servo and plates.

        NOTE (issue #22): Top_Plate and Bottom_Plate clearances are measured
        with a real 3D shape-distance check (Part.Shape.distToShape) against a
        conservative axis-aligned bounding box built from the servo mesh's
        world-space BoundBox — a safe/conservative proxy for the servo
        envelope, since it is at least as large as the actual mesh in every
        direction. This replaces a previous Z-axis-only formula that assumed
        a uniform parallel plate stack; it did not account for Bottom_Plate's
        independent -36.9° rotation and reported a fictitious ~4.85mm
        (failing the 5.0mm minimum) where the real clearance is ~17.1mm.
        Middle_Plate is a mounting-contact fit, not a clearance gap, and
        stays a Z-only check (legitimately ~0mm) — see "Servo below
        Middle_Plate" below.
        """
        try:
            print(f"✓ Clearance Validation (real shape-distance for Top/Bottom_Plate):")

            if not self.placement_data:
                return False

            if not self.mesh_objects:
                print("ERROR: No servo mesh available to build clearance bounding box")
                return False

            # Conservative servo bounding solid from the mesh's world-space
            # BoundBox. Visual and collision-proxy meshes share placement, so
            # either mesh's BoundBox describes the same servo envelope.
            bbm = self.mesh_objects[0].Mesh.BoundBox
            servo_box = Part.makeBox(
                bbm.XLength, bbm.YLength, bbm.ZLength,
                Vector(bbm.XMin, bbm.YMin, bbm.ZMin)
            )

            # Check clearance to Top_Plate (real 3D shape distance)
            top_clearance = self.plates["Top_Plate"].Shape.distToShape(servo_box)[0]

            top_passed = top_clearance > self.CLEARANCE_MIN
            self.validations.append(ValidationResult(
                check_name="Clearance to Top_Plate",
                passed=top_passed,
                details=f"Clearance: {top_clearance:.2f} mm (minimum: {self.CLEARANCE_MIN} mm)",
                value=top_clearance,
                tolerance=self.CLEARANCE_MIN
            ))
            status = "✓" if top_passed else "✗"
            print(f"  {status} Clearance to Top_Plate: {top_clearance:.2f} mm")

            # Check clearance to Bottom_Plate (real 3D shape distance)
            bottom_clearance = self.plates["Bottom_Plate"].Shape.distToShape(servo_box)[0]

            bottom_passed = bottom_clearance > self.CLEARANCE_MIN
            self.validations.append(ValidationResult(
                check_name="Clearance to Bottom_Plate",
                passed=bottom_passed,
                details=f"Clearance: {bottom_clearance:.2f} mm (minimum: {self.CLEARANCE_MIN} mm)",
                value=bottom_clearance,
                tolerance=self.CLEARANCE_MIN
            ))
            status = "✓" if bottom_passed else "✗"
            print(f"  {status} Clearance to Bottom_Plate: {bottom_clearance:.2f} mm")

            # Check servo is below Middle_Plate
            middle_plate_bottom = self.MIDDLE_PLATE_SPECS["z_position"] - self.MIDDLE_PLATE_SPECS["thickness"] / 2.0
            middle_clearance = middle_plate_bottom - self.placement_data.z

            middle_passed = middle_clearance > 0
            self.validations.append(ValidationResult(
                check_name="Servo below Middle_Plate",
                passed=middle_passed,
                details=f"Offset: {middle_clearance:.2f} mm",
                value=middle_clearance
            ))
            status = "✓" if middle_passed else "✗"
            print(f"  {status} Servo below Middle_Plate surface: {middle_clearance:.2f} mm")

            return all(v.passed for v in self.validations if "Clearance" in v.check_name)

        except Exception as e:
            print(f"ERROR validating clearances: {e}")
            return False

    def save_assembly(self, output_path: Optional[str] = None) -> bool:
        """Save updated assembly with servo meshes"""
        try:
            if not self.doc:
                return False

            if output_path is None:
                try:
                    output_dir = Path(__file__).parent
                except NameError:
                    output_dir = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"
                output_path = output_dir / "plates_servo_assembled.FCStd"

            # Recompute before saving
            self.doc.recompute()

            # Save
            self.doc.saveAs(str(output_path))

            # Check file size
            file_size = Path(output_path).stat().st_size
            size_kb = file_size / 1024.0

            print(f"\n✓ Assembly saved: {output_path}")
            print(f"  File size: {size_kb:.1f} KB")

            return True

        except Exception as e:
            print(f"ERROR saving assembly: {e}")
            return False

    def save_link_config(self, output_path: Optional[str] = None) -> bool:
        """Save servo link configuration to JSON"""
        try:
            if output_path is None:
                try:
                    output_dir = Path(__file__).parent
                except NameError:
                    output_dir = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"
                output_path = output_dir / "servo_link_config.json"

            output_data = {
                "phase": 3,
                "title": "Servo Motor Assembly Link Configuration",
                "servo_meshes": [
                    {"name": name, "file": str(self.mechanical_dir / filename) if self.mechanical_dir else filename}
                    for name, filename in self.SERVO_MESHES
                ],
                "placement": self.placement_data.to_dict() if self.placement_data else None,
                "servo_part_name": self.servo_part.Name if self.servo_part else None,
                "mesh_object_names": [m.Name for m in self.mesh_objects],
                "linked_plates": list(self.plates.keys()),
                "validations": [v.to_dict() for v in self.validations],
                "all_validations_passed": all(v.passed for v in self.validations),
                "specifications": {
                    "servo": self.SERVO_SPECS,
                    "middle_plate": self.MIDDLE_PLATE_SPECS,
                    "tolerances": {
                        "placement_tolerance_mm": self.PLACEMENT_TOLERANCE,
                        "clearance_min_mm": self.CLEARANCE_MIN,
                    },
                },
            }

            with open(output_path, 'w') as f:
                json.dump(output_data, f, indent=2)

            print(f"✓ Saved link configuration: {output_path}")
            return True

        except Exception as e:
            print(f"ERROR saving link config: {e}")
            return False

    def run(self) -> bool:
        """Execute full servo mesh assembly process"""
        print("=" * 70)
        print("PHASE 3: SERVO MOTOR MESH ASSEMBLY (STS3032_Mount)")
        print("=" * 70)
        print()

        try:
            script_dir = Path(__file__).parent
        except NameError:
            script_dir = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"

        doc_path = script_dir / "plates_servo_assembled.FCStd"

        # Resolve Mechanical directory
        print("Step 1: Resolve servo STL mesh directory")
        print("-" * 70)
        if not self.resolve_mechanical_dir():
            print("WARNING: Could not resolve Mechanical directory")
        print()

        # Load document
        print("Step 2: Load FreeCAD assembly document")
        print("-" * 70)
        if not self.load_document(str(doc_path)):
            return False
        print()

        # Verify plates
        print("Step 3: Verify plate objects")
        print("-" * 70)
        if not self.verify_plates():
            return False
        print()

        # Load placement data
        print("Step 4: Load servo placement data")
        print("-" * 70)
        if not self.load_placement_json(script_dir / "servo_placement.json"):
            print("WARNING: Could not load placement JSON, using verified defaults")
        print()

        # Create servo Part + import meshes
        print("Step 5: Create STS3032_Mount and import servo meshes")
        print("-" * 70)
        if not self.create_servo_body():
            return False
        print()

        # Apply placement
        print("Step 6: Apply servo placement to both meshes")
        print("-" * 70)
        if not self.apply_placement():
            return False
        print()

        # Validate link
        print("Step 7: Validate mesh import")
        print("-" * 70)
        if not self.validate_link_resolution():
            print("WARNING: Link validation had issues")
        print()

        # Validate placement
        print("Step 8: Validate placement accuracy")
        print("-" * 70)
        if not self.validate_placement():
            print("WARNING: Placement validation had issues")
        print()

        # Validate clearances
        print("Step 9: Validate clearances")
        print("-" * 70)
        if not self.validate_clearances():
            print("WARNING: Clearance validation had issues")
        print()

        # Save assembly
        print("Step 10: Save updated assembly")
        print("-" * 70)
        if not self.save_assembly(script_dir / "plates_servo_assembled.FCStd"):
            print("WARNING: Could not save assembly")
        print()

        # Save link config
        print("Step 11: Save servo link configuration")
        print("-" * 70)
        if not self.save_link_config(script_dir / "servo_link_config.json"):
            print("WARNING: Could not save link config")
        print()

        print("=" * 70)
        print("✓ Phase 3 Complete")
        print("=" * 70)

        return True


def main():
    """Main entry point"""
    manager = ServoLinkManager()
    success = manager.run()

    # Close document
    if manager.doc:
        try:
            App.closeDocument(manager.doc.Name)
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
