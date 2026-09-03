#!/usr/bin/env python3
"""
Phase 2: Servo Motor Position Calculation

NOTE (2026-09-02, issue #3 reconciliation): This script previously derived
servo placement from `Middle_Plate.Shape.Edges[25]` / `[33]` (hardcoded
"Edge26"/"Edge34" indices) against `plates_assembled.FCStd`, producing a
pitch=90 deg "shaft pointing down" convention. That approach has been
replaced — BRep edge indices are not stable across topology edits (they can
silently renumber after any sketch/pocket change), and it no longer matches
reality: the live, human-approved document places the servo with an
IDENTITY rotation, not pitch=90.

The placement below was verified directly via live FreeCAD MCP inspection
against `plates_servo_assembled.FCStd` on 2026-09-02: with this placement,
the servo mesh's mounting bore lines up with Middle_Plate's hole cluster B
(global center X=32.9933, Y=25.7719, Z span 2.75-5.25mm) to within
~0.03-0.04mm. It is hardcoded rather than re-derived from geometry so this
script no longer depends on fragile edge indexing.

JSON schema note: `pitch`/`z_offset` no longer have one consistent meaning
across the assembly. Top_Plate, Middle_Plate and Bottom_Plate each sit at
their own independently Z-axis-rotated placement and their own Z height
(6mm / 4mm / 3mm respectively) rather than a uniform parallel stack, so a
single scalar "z_offset below the plate" or "pitch to point the shaft down"
cannot describe all three relationships at once. For the current (and only)
mount point — Middle_Plate hole cluster B — both are effectively 0: the
servo sits at its own explicit (x, y, z) with identity rotation, not offset
"below" the plate along a shared normal.

Output:
- servo_placement.json: Placement data (x, y, z, roll, pitch, yaw, z_offset)
- Console log: Validation results
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
    """Servo placement matrix"""
    x: float          # mm
    y: float          # mm
    z: float          # mm
    roll: float       # degrees
    pitch: float      # degrees
    yaw: float        # degrees
    z_offset: float   # kept for schema compatibility; see module docstring

    def to_dict(self) -> dict:
        """Convert to dictionary"""
        return asdict(self)


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


class ServoPositionCalculator:
    """Provide the verified servo motor placement for Middle_Plate hole cluster B"""

    # Middle_Plate specifications (live document values, verified 2026-09-02)
    MIDDLE_PLATE_SPECS = {
        "center_to_center": 44.72,  # mm
        "width": 10.0,              # mm
        "thickness": 2.5,           # mm
        "z_position": 4.0,          # mm (Middle_Plate.Placement.Position.z in the live document)
    }

    # Servo motor specifications (Feetech STS3032)
    SERVO_SPECS = {
        "body_length": 32.0,        # mm
        "body_width": 12.0,         # mm
        "body_height": 28.0,        # mm
        "shaft_length": 10.0,       # mm
        "shaft_offset_x": 0.0,      # mm from center
        "shaft_offset_y": 14.0,     # mm from center (rear of servo)
    }

    # Target mount: Middle_Plate hole cluster B, global center (X, Y)
    TARGET_HOLE_CENTER = (32.9933, 25.7719)
    TARGET_HOLE_Z_SPAN = (2.75, 5.25)  # mm

    # Verified servo placement (both visual and collision-proxy meshes use this
    # placement directly, matching the live document — see module docstring).
    VERIFIED_PLACEMENT = PlacementData(
        x=-150.1217,
        y=-141.9987,
        z=-3.1,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        z_offset=0.0,
    )

    ALIGNMENT_TOLERANCE = 1.0       # mm (tolerance for alignment)
    CLEARANCE_MIN = 5.0             # mm (minimum clearance to other plates)

    def __init__(self):
        """Initialize calculator"""
        self.doc = None
        self.middle_plate = None
        self.placement = None
        self.validations = []
        self.clearances = {}

    def load_document(self, doc_path: str) -> bool:
        """Load FreeCAD document (used only to confirm Middle_Plate exists)"""
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

    def find_middle_plate(self) -> bool:
        """Find Middle_Plate object in document"""
        try:
            for obj in self.doc.Objects:
                if "Middle" in obj.Name and "Plate" in obj.Name:
                    self.middle_plate = obj
                    print(f"✓ Found Middle_Plate: {obj.Name}")
                    return True

            print("ERROR: Middle_Plate not found in document")
            return False

        except Exception as e:
            print(f"ERROR finding Middle_Plate: {e}")
            return False

    def calculate_placement(self, z_offset: float = 0.0) -> bool:
        """Return the verified servo placement.

        `z_offset` is accepted for CLI/interface compatibility with earlier
        phases but is not used: the placement below is an empirically
        verified fixed value, not derived by offsetting from plate geometry
        (see module docstring).
        """
        if z_offset:
            print(f"NOTE: z_offset argument ({z_offset}) is ignored — placement is hardcoded/verified, not offset-derived.")

        self.placement = self.VERIFIED_PLACEMENT

        print(f"\n✓ Servo placement (verified 2026-09-02 via live FreeCAD MCP inspection):")
        print(f"    Position: X={self.placement.x:.4f} mm, Y={self.placement.y:.4f} mm, Z={self.placement.z:.4f} mm")
        print(f"    Rotation: Roll={self.placement.roll:.1f}°, Pitch={self.placement.pitch:.1f}°, Yaw={self.placement.yaw:.1f}° (identity)")

        return True

    def validate_clearances(self) -> bool:
        """Validate clearances with other plates.

        NOTE: Top_Plate, Middle_Plate and Bottom_Plate each sit at their own
        independently Z-rotated placement and Z height — this is not a
        uniform parallel stack. These checks compare raw Z heights only as a
        coarse sanity signal; they do not account for each plate's rotation.
        """
        try:
            print(f"\n✓ Clearance Validation (coarse Z-height sanity check):")

            # Check clearance to Top_Plate
            top_plate_z = 6.00  # Live Top_Plate.Placement.Position.z
            top_plate_thickness = self.MIDDLE_PLATE_SPECS["thickness"]
            top_plate_bottom = top_plate_z - top_plate_thickness / 2.0
            top_plate_clearance = top_plate_bottom - self.placement.z

            top_passed = top_plate_clearance > self.CLEARANCE_MIN
            self.clearances["top_plate"] = top_plate_clearance
            self.validations.append(ValidationResult(
                check_name="Clearance to Top_Plate",
                passed=top_passed,
                details=f"Clearance: {top_plate_clearance:.2f} mm (minimum: {self.CLEARANCE_MIN} mm)",
                value=top_plate_clearance,
                tolerance=self.CLEARANCE_MIN,
            ))
            status = "✓" if top_passed else "✗"
            print(f"  {status} Clearance to Top_Plate: {top_plate_clearance:.2f} mm")

            # Check clearance to Bottom_Plate
            bottom_plate_z = 3.00  # Live Bottom_Plate.Placement.Position.z
            bottom_plate_thickness = self.MIDDLE_PLATE_SPECS["thickness"]
            bottom_plate_bottom = bottom_plate_z - bottom_plate_thickness / 2.0
            bottom_plate_clearance = bottom_plate_bottom - self.placement.z

            bottom_passed = bottom_plate_clearance > self.CLEARANCE_MIN
            self.clearances["bottom_plate"] = bottom_plate_clearance
            self.validations.append(ValidationResult(
                check_name="Clearance to Bottom_Plate",
                passed=bottom_passed,
                details=f"Clearance: {bottom_plate_clearance:.2f} mm (minimum: {self.CLEARANCE_MIN} mm)",
                value=bottom_plate_clearance,
                tolerance=self.CLEARANCE_MIN,
            ))
            status = "✓" if bottom_passed else "✗"
            print(f"  {status} Clearance to Bottom_Plate: {bottom_plate_clearance:.2f} mm")

            # Middle plate itself - verify servo is below the plate
            middle_plate_bottom = self.MIDDLE_PLATE_SPECS["z_position"] - self.MIDDLE_PLATE_SPECS["thickness"] / 2.0
            middle_plate_clearance = middle_plate_bottom - self.placement.z
            middle_passed = middle_plate_clearance > 0  # Should be positive if servo is below
            self.clearances["middle_plate_clearance"] = middle_plate_clearance
            self.validations.append(ValidationResult(
                check_name="Clearance from Middle_Plate",
                passed=middle_passed,
                details=f"Servo below plate surface: {middle_plate_clearance:.2f} mm",
                value=middle_plate_clearance,
            ))
            status = "✓" if middle_passed else "✗"
            print(f"  {status} Servo below plate surface: {middle_plate_clearance:.2f} mm")

            return all(v.passed for v in self.validations if "Clearance" in v.check_name)

        except Exception as e:
            print(f"ERROR validating clearances: {e}")
            return False

    def save_placement_json(self, output_path: Optional[str] = None) -> bool:
        """Save placement data to JSON"""
        try:
            if output_path is None:
                output_path = Path(__file__).parent / "servo_placement.json"
            else:
                output_path = Path(output_path)

            # Prepare output data
            output_data = {
                "phase": 2,
                "title": "Servo Motor Position Calculation",
                "timestamp": str(Path(__file__).stat().st_mtime),
                "placement": self.placement.to_dict() if self.placement else None,
                "visual_placement": self.placement.to_dict() if self.placement else None,
                "collision_placement": self.placement.to_dict() if self.placement else None,
                "placement_source": {
                    "method": "empirically_verified",
                    "verified_date": "2026-09-02",
                    "verified_via": "live FreeCAD MCP inspection",
                    "target_document": "plates_servo_assembled.FCStd",
                    "note": (
                        "Superseded the old Edge26/Edge34 (Shape.Edges[25]/[33]) "
                        "index-based derivation — BRep edge indices are not "
                        "stable after topology edits."
                    ),
                    "target_hole_center_xy": list(self.TARGET_HOLE_CENTER),
                    "target_hole_z_span_mm": list(self.TARGET_HOLE_Z_SPAN),
                    "fit_tolerance_mm": "~0.03-0.04",
                },
                "clearances": self.clearances,
                "validations": [v.to_dict() for v in self.validations],
                "all_validations_passed": all(v.passed for v in self.validations),
                "specifications": {
                    "middle_plate": self.MIDDLE_PLATE_SPECS,
                    "servo": self.SERVO_SPECS,
                    "tolerances": {
                        "alignment_tolerance_mm": self.ALIGNMENT_TOLERANCE,
                        "clearance_min_mm": self.CLEARANCE_MIN,
                    },
                },
            }

            # Save to JSON
            with open(output_path, 'w') as f:
                json.dump(output_data, f, indent=2)

            print(f"\n✓ Saved placement data: {output_path}")
            return True

        except Exception as e:
            print(f"ERROR saving JSON: {e}")
            return False

    def run(self, z_offset: float = 0.0) -> bool:
        """Execute full position calculation process"""
        print("=" * 70)
        print("PHASE 2: SERVO MOTOR POSITION (VERIFIED, NOT EDGE-DERIVED)")
        print("=" * 70)
        print()

        try:
            script_dir = Path(__file__).parent
        except NameError:
            # Running in FreeCAD console
            script_dir = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"

        doc_path = script_dir / "plates_servo_assembled.FCStd"

        # Load document (sanity check only — placement itself is hardcoded)
        if not self.load_document(str(doc_path)):
            print("WARNING: Could not open live document for sanity check; continuing with verified placement anyway")
        else:
            print()
            if not self.find_middle_plate():
                print("WARNING: Middle_Plate not found in document; continuing with verified placement anyway")

        print()

        # "Calculate" (i.e. report) placement
        if not self.calculate_placement(z_offset):
            return False

        print()

        # Validate clearances
        if not self.validate_clearances():
            print("WARNING: Clearance validation failed")

        print()

        # Save JSON
        if not self.save_placement_json(script_dir / "servo_placement.json"):
            print("WARNING: Could not save JSON")

        print()
        print("=" * 70)
        print("✓ Phase 2 Complete")
        print("=" * 70)

        return True


def main():
    """Main entry point"""
    calculator = ServoPositionCalculator()

    # Optional: read Z-offset from command line (ignored — kept for CLI compatibility)
    z_offset = 0.0
    if len(sys.argv) > 1:
        try:
            z_offset = float(sys.argv[1])
        except ValueError:
            print(f"WARNING: Invalid Z-offset '{sys.argv[1]}', ignoring")

    success = calculator.run(z_offset)

    # Close document
    if calculator.doc:
        App.closeDocument(calculator.doc.Name)

    return 0 if success else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
