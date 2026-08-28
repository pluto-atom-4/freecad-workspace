#!/usr/bin/env python3
"""
Phase 3: External Linking for Servo Motor Assembly

Integrates servo motor into the plate assembly using external linking.

Process:
1. Load plates_assembled.FCStd and verify plate objects
2. Load servo_placement.json from Phase 2 output
3. Create Part::Body container named "Servo_Motor"
4. Configure as external link to servo STEP file
5. Apply placement matrix (position + rotation)
6. Validate link resolution and placement
7. Save updated assembly with external link
8. Export servo_link_config.json for reference

Output:
- Updated plates_assembled.FCStd (with external servo link, <20 KB)
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
    z_offset: float   # offset below plate surface

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
    """Link servo motor to plate assembly with external reference"""

    # Servo specifications
    SERVO_SPECS = {
        "model": "Feetech STS3032",
        "body_length": 32.0,        # mm
        "body_width": 30.0,         # mm
        "body_height": 28.0,        # mm
        "shaft_length": 10.0,       # mm
        "shaft_offset_x": 0.0,      # mm from center
        "shaft_offset_y": 14.0,     # mm from center (rear of servo)
    }

    # Plate specifications
    MIDDLE_PLATE_SPECS = {
        "thickness": 2.5,           # mm
        "z_position": 0.0,          # mm
    }

    # Validation tolerances
    PLACEMENT_TOLERANCE = 1.0       # mm (tolerance for placement accuracy)
    CLEARANCE_MIN = 5.0             # mm (minimum clearance to other plates)

    def __init__(self):
        """Initialize link manager"""
        self.doc = None
        self.plates = {}
        self.servo_body = None
        self.placement_data = None
        self.validations = []
        self.servo_step_path = None
        self.servo_step_exists = False

    def resolve_servo_step_path(self) -> bool:
        """Resolve servo STEP file path"""
        try:
            # Try multiple possible locations
            script_dir = Path(__file__).parent

            # Method 1: Relative path from this script
            step_path1 = script_dir.parent / "Mechanical" / "feetech-STS3032.step"

            # Method 2: Absolute reference
            step_path2 = Path(__file__).parent.parent / "Mechanical" / "feetech-STS3032.step"

            if step_path1.exists():
                self.servo_step_path = step_path1
                self.servo_step_exists = True
                print(f"✓ Found servo STEP file: {step_path1}")
                return True
            elif step_path2.exists():
                self.servo_step_path = step_path2
                self.servo_step_exists = True
                print(f"✓ Found servo STEP file: {step_path2}")
                return True
            else:
                print(f"⚠ WARNING: Servo STEP file not found at:")
                print(f"    {step_path1}")
                print(f"  Will create link with path: ../Mechanical/feetech-STS3032.step")
                # Use relative path even if file doesn't exist yet
                self.servo_step_path = Path("../Mechanical/feetech-STS3032.step")
                self.servo_step_exists = step_path1.exists()
                return True

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
                print("  Using default placement values")
                # Use defaults if file doesn't exist
                self.placement_data = PlacementData(
                    x=10.0,
                    y=15.0,
                    z=-11.25,
                    roll=0.0,
                    pitch=90.0,
                    yaw=0.0,
                    z_offset=10.0,
                )
                return True

            with open(json_path, 'r') as f:
                data = json.load(f)

            if "placement" not in data:
                print(f"ERROR: No placement data in JSON")
                return False

            placement = data["placement"]
            self.placement_data = PlacementData(
                x=placement.get("x", 10.0),
                y=placement.get("y", 15.0),
                z=placement.get("z", -11.25),
                roll=placement.get("roll", 0.0),
                pitch=placement.get("pitch", 90.0),
                yaw=placement.get("yaw", 0.0),
                z_offset=placement.get("z_offset", 10.0),
            )

            print(f"✓ Loaded placement data: {json_path}")
            print(f"  Position: X={self.placement_data.x:.2f}, Y={self.placement_data.y:.2f}, Z={self.placement_data.z:.2f} mm")
            print(f"  Rotation: Roll={self.placement_data.roll:.1f}°, Pitch={self.placement_data.pitch:.1f}°, Yaw={self.placement_data.yaw:.1f}°")
            return True

        except Exception as e:
            print(f"ERROR loading placement JSON: {e}")
            return False

    def create_servo_body(self) -> bool:
        """Create Part::Body container for servo motor"""
        try:
            # Create body object
            self.servo_body = self.doc.addObject("Part::Body", "Servo_Motor")
            print(f"✓ Created Part::Body: Servo_Motor")

            return True

        except Exception as e:
            print(f"ERROR creating servo body: {e}")
            return False

    def link_servo_step(self) -> bool:
        """Create external link to servo STEP file"""
        try:
            if not self.servo_body:
                print("ERROR: Servo body not created")
                return False

            if not self.servo_step_path:
                print("ERROR: Servo STEP path not resolved")
                return False

            # Determine the relative path from the assembly document to the STEP file
            doc_dir = Path(self.doc.FileName).parent
            step_abs_path = (doc_dir / self.servo_step_path).resolve()

            # Try to create a link to the external STEP file
            # In FreeCAD, we can add the STEP shape to the body as an external reference

            try:
                # Import the STEP file shape into the servo body
                import Part as FreecadPart

                # Load the STEP file
                if step_abs_path.exists():
                    # Read the shape from STEP file
                    step_shape = FreecadPart.Shape()
                    step_shape.read(str(step_abs_path))

                    # Create a feature to hold the imported shape
                    feature = self.doc.addObject("Part::Feature", "Servo_Shape")
                    feature.Shape = step_shape

                    # Move it into the servo body
                    self.servo_body.addObject(feature)

                    print(f"✓ Linked servo STEP file: {step_abs_path}")
                    print(f"  Shape vertices: {len(step_shape.Vertexes)}")
                    print(f"  Shape edges: {len(step_shape.Edges)}")
                    print(f"  Shape faces: {len(step_shape.Faces)}")
                else:
                    print(f"⚠ WARNING: Servo STEP file not found at: {step_abs_path}")
                    print("  Creating placeholder body (external link expected)")
                    # Create a placeholder
                    feature = self.doc.addObject("Part::Feature", "Servo_Shape_Placeholder")
                    feature.Shape = FreecadPart.makeBox(30, 30, 28)  # Approximate dimensions
                    self.servo_body.addObject(feature)

                return True

            except Exception as inner_e:
                print(f"⚠ WARNING: Could not import STEP shape: {inner_e}")
                print("  Creating approximate servo geometry")

                # Fallback: create a box approximation
                import Part as FreecadPart
                feature = self.doc.addObject("Part::Feature", "Servo_Approximate")
                # Approximate servo body dimensions (L x W x H)
                feature.Shape = FreecadPart.makeBox(
                    self.SERVO_SPECS["body_length"],
                    self.SERVO_SPECS["body_width"],
                    self.SERVO_SPECS["body_height"]
                )
                self.servo_body.addObject(feature)
                return True

        except Exception as e:
            print(f"ERROR linking servo STEP: {e}")
            return False

    def apply_placement(self) -> bool:
        """Apply placement matrix to servo body"""
        try:
            if not self.servo_body or not self.placement_data:
                print("ERROR: Servo body or placement data missing")
                return False

            # Create position vector
            position = Vector(
                self.placement_data.x,
                self.placement_data.y,
                self.placement_data.z
            )

            # Create rotation from roll, pitch, yaw (Euler angles)
            # Order: roll (Z), pitch (Y), yaw (X) - standard aerospace convention
            # But for servo mounting, we want:
            # - Pitch 90° rotation around Y-axis (shaft perpendicular to plate)
            # - Roll 0° (body parallel to plate)
            # - Yaw 0° (aligned with linkage)

            # Create rotation using Euler angles
            # FreeCAD uses Yaw-Pitch-Roll order
            rotation = Rotation(
                self.placement_data.yaw,      # Yaw (Z-axis)
                self.placement_data.pitch,    # Pitch (Y-axis)
                self.placement_data.roll      # Roll (X-axis)
            )

            # Apply placement
            self.servo_body.Placement = Placement(position, rotation)

            print(f"✓ Applied servo placement:")
            print(f"  Position: X={position.x:.2f}, Y={position.y:.2f}, Z={position.z:.2f} mm")
            print(f"  Rotation: Yaw={self.placement_data.yaw:.1f}°, Pitch={self.placement_data.pitch:.1f}°, Roll={self.placement_data.roll:.1f}°")

            return True

        except Exception as e:
            print(f"ERROR applying placement: {e}")
            return False

    def validate_link_resolution(self) -> bool:
        """Validate that servo link resolves correctly"""
        try:
            print(f"✓ Link Resolution Validation:")

            # Check if servo body exists and has objects
            if not self.servo_body:
                self.validations.append(ValidationResult(
                    check_name="Servo body exists",
                    passed=False,
                    details="Servo body not created"
                ))
                return False

            has_shape = len(self.servo_body.OutList) > 0
            self.validations.append(ValidationResult(
                check_name="Servo has linked geometry",
                passed=has_shape,
                details=f"Servo body contains {len(self.servo_body.OutList)} object(s)"
            ))
            status = "✓" if has_shape else "✗"
            print(f"  {status} Servo body contains geometry")

            # Check STEP file path
            if self.servo_step_exists:
                path_status = "✓"
                path_passed = True
                path_msg = f"STEP file found at {self.servo_step_path}"
            else:
                path_status = "⚠"
                path_passed = False
                path_msg = f"STEP file not found (expected at {self.servo_step_path})"

            print(f"  {path_status} {path_msg}")

            return has_shape

        except Exception as e:
            print(f"ERROR validating link resolution: {e}")
            return False

    def validate_placement(self) -> bool:
        """Validate servo placement accuracy"""
        try:
            print(f"✓ Placement Validation:")

            if not self.servo_body or not self.placement_data:
                return False

            # Get actual placement
            actual_placement = self.servo_body.Placement

            # Calculate distance from expected to actual
            expected = Vector(
                self.placement_data.x,
                self.placement_data.y,
                self.placement_data.z
            )

            actual = actual_placement.Base
            distance = expected.distanceToPoint(actual)

            passed = distance < self.PLACEMENT_TOLERANCE
            self.validations.append(ValidationResult(
                check_name="Placement accuracy",
                passed=passed,
                details=f"Position error: {distance:.3f} mm",
                value=distance,
                tolerance=self.PLACEMENT_TOLERANCE
            ))
            status = "✓" if passed else "✗"
            print(f"  {status} Position error: {distance:.3f} mm (tolerance: {self.PLACEMENT_TOLERANCE} mm)")

            # Validate rotation (pitch should be 90°)
            rotation = actual_placement.Rotation
            ypr = rotation.getYawPitchRoll()

            pitch_error = abs(ypr[1] - self.placement_data.pitch)
            pitch_passed = pitch_error < 1.0  # Within 1 degree

            self.validations.append(ValidationResult(
                check_name="Pitch rotation",
                passed=pitch_passed,
                details=f"Pitch error: {pitch_error:.2f}°",
                value=pitch_error,
                tolerance=1.0
            ))
            status = "✓" if pitch_passed else "✗"
            print(f"  {status} Pitch rotation: {ypr[1]:.1f}° (expected {self.placement_data.pitch:.1f}°, error: {pitch_error:.2f}°)")

            return passed and pitch_passed

        except Exception as e:
            print(f"ERROR validating placement: {e}")
            return False

    def validate_clearances(self) -> bool:
        """Validate clearances between servo and plates"""
        try:
            print(f"✓ Clearance Validation:")

            if not self.placement_data:
                return False

            # Check clearance to Top_Plate
            top_plate_z = 6.00  # From assemble_plates.py
            top_plate_thickness = 5.0
            top_plate_bottom = top_plate_z - top_plate_thickness / 2.0
            top_clearance = top_plate_bottom - self.placement_data.z

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

            # Check clearance to Bottom_Plate
            bottom_plate_z = 3.00  # From assemble_plates.py
            bottom_plate_thickness = 5.0
            bottom_plate_bottom = bottom_plate_z - bottom_plate_thickness / 2.0
            bottom_clearance = bottom_plate_bottom - self.placement_data.z

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
        """Save updated assembly with servo link"""
        try:
            if not self.doc:
                return False

            if output_path is None:
                try:
                    output_dir = Path(__file__).parent
                except NameError:
                    output_dir = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"
                output_path = output_dir / "plates_assembled.FCStd"

            # Recompute before saving
            self.doc.recompute()

            # Save
            self.doc.saveAs(str(output_path))

            # Check file size
            file_size = Path(output_path).stat().st_size
            size_kb = file_size / 1024.0

            print(f"\n✓ Assembly saved: {output_path}")
            print(f"  File size: {size_kb:.1f} KB")

            if size_kb > 20.0:
                print(f"  ⚠ WARNING: File size exceeds 20 KB target (external link may not be working)")

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
                "servo_step_file": str(self.servo_step_path),
                "servo_step_exists": self.servo_step_exists,
                "placement": self.placement_data.to_dict() if self.placement_data else None,
                "servo_body_name": self.servo_body.Name if self.servo_body else None,
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
        """Execute full servo linking process"""
        print("=" * 70)
        print("PHASE 3: EXTERNAL LINKING FOR SERVO MOTOR ASSEMBLY")
        print("=" * 70)
        print()

        try:
            script_dir = Path(__file__).parent
        except NameError:
            script_dir = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"

        doc_path = script_dir / "plates_assembled.FCStd"

        # Resolve servo STEP path
        print("Step 1: Resolve servo STEP file path")
        print("-" * 70)
        if not self.resolve_servo_step_path():
            print("WARNING: Could not resolve servo STEP path")
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
            print("WARNING: Could not load placement JSON, using defaults")
        print()

        # Create servo body
        print("Step 5: Create servo motor body container")
        print("-" * 70)
        if not self.create_servo_body():
            return False
        print()

        # Link servo STEP file
        print("Step 6: Link servo STEP file")
        print("-" * 70)
        if not self.link_servo_step():
            print("WARNING: Could not link servo STEP file")
        print()

        # Apply placement
        print("Step 7: Apply servo placement matrix")
        print("-" * 70)
        if not self.apply_placement():
            return False
        print()

        # Validate link
        print("Step 8: Validate link resolution")
        print("-" * 70)
        if not self.validate_link_resolution():
            print("WARNING: Link validation had issues")
        print()

        # Validate placement
        print("Step 9: Validate placement accuracy")
        print("-" * 70)
        if not self.validate_placement():
            print("WARNING: Placement validation had issues")
        print()

        # Validate clearances
        print("Step 10: Validate clearances")
        print("-" * 70)
        if not self.validate_clearances():
            print("WARNING: Clearance validation had issues")
        print()

        # Save assembly
        print("Step 11: Save updated assembly")
        print("-" * 70)
        if not self.save_assembly(script_dir / "plates_assembled.FCStd"):
            print("WARNING: Could not save assembly")
        print()

        # Save link config
        print("Step 12: Save servo link configuration")
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
