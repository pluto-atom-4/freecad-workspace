#!/usr/bin/env python3
"""
Phase 2: Servo Motor Position Calculation

Calculates servo motor placement matrix based on Middle_Plate edge geometry.

Process:
1. Extract Edge26 and Edge34 from Middle_Plate
2. Compute edge midpoints and normal vectors
3. Calculate servo placement (position + rotation)
4. Validate alignment tolerance and clearances
5. Export placement matrix to JSON

Output:
- servo_placement.json: Placement data (x, y, z, roll, pitch, yaw)
- Console log: Detailed calculations and validation results
"""

import sys
import json
import math
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Optional, Tuple, Dict, Any

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
class EdgeData:
    """Edge geometry data"""
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

    def to_dict(self) -> dict:
        """Convert to dictionary"""
        return asdict(self)


@dataclass
class PlacementData:
    """Servo placement matrix"""
    x: float          # mm
    y: float          # mm
    z: float          # mm
    roll: float       # degrees
    pitch: float      # degrees
    yaw: float        # degrees
    z_offset: float   # offset below plate surface

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
    """Calculate servo motor placement based on plate geometry"""

    # Middle_Plate specifications (from feat-idea.md)
    MIDDLE_PLATE_SPECS = {
        "center_to_center": 44.72,  # mm
        "width": 10.0,              # mm
        "thickness": 2.5,           # mm (actual thickness from assembly)
        "z_position": 0.0,          # mm (assembly reference)
    }

    # Servo motor specifications (Feetech STS3032)
    SERVO_SPECS = {
        "body_length": 32.0,        # mm
        "body_width": 30.0,         # mm
        "body_height": 28.0,        # mm
        "shaft_length": 10.0,       # mm
        "shaft_offset_x": 0.0,      # mm from center
        "shaft_offset_y": 14.0,     # mm from center (rear of servo)
    }

    # Placement parameters
    Z_OFFSET_DEFAULT = 10.0         # mm below plate surface (default)
    PITCH_ROTATION = 90.0           # degrees (shaft perpendicular to plate)
    ALIGNMENT_TOLERANCE = 1.0       # mm (tolerance for alignment)
    CLEARANCE_MIN = 5.0             # mm (minimum clearance to other plates)

    def __init__(self):
        """Initialize calculator"""
        self.doc = None
        self.middle_plate = None
        self.edge26 = None
        self.edge34 = None
        self.edge26_data = None
        self.edge34_data = None
        self.placement = None
        self.validations = []
        self.clearances = {}

    def load_document(self, doc_path: str) -> bool:
        """Load FreeCAD document"""
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

    def extract_edge_data(self, edge_index: int) -> Optional[EdgeData]:
        """Extract geometry data from an edge"""
        try:
            if not self.middle_plate or not hasattr(self.middle_plate, 'Shape'):
                print(f"ERROR: No shape for edge extraction")
                return None

            shape = self.middle_plate.Shape
            if edge_index < 0 or edge_index >= len(shape.Edges):
                print(f"WARNING: Edge{edge_index + 1} not found (only {len(shape.Edges)} edges)")
                return None

            edge = shape.Edges[edge_index]

            # Get edge vertices
            start = edge.Vertexes[0].Point
            end = edge.Vertexes[-1].Point

            # Calculate midpoint
            mid_x = (start.x + end.x) / 2.0
            mid_y = (start.y + end.y) / 2.0
            mid_z = (start.z + end.z) / 2.0

            # Calculate length
            length = math.sqrt(
                (end.x - start.x) ** 2 +
                (end.y - start.y) ** 2 +
                (end.z - start.z) ** 2
            )

            return EdgeData(
                edge_id=edge_index + 1,
                start_x=start.x,
                start_y=start.y,
                start_z=start.z,
                end_x=end.x,
                end_y=end.y,
                end_z=end.z,
                mid_x=mid_x,
                mid_y=mid_y,
                mid_z=mid_z,
                length=length,
            )

        except Exception as e:
            print(f"ERROR extracting edge {edge_index + 1} data: {e}")
            return None

    def extract_edges(self) -> bool:
        """Extract Edge26 and Edge34 from Middle_Plate"""
        try:
            # Edge26 (index 25) and Edge34 (index 33)
            self.edge26_data = self.extract_edge_data(25)
            if not self.edge26_data:
                print("WARNING: Could not extract Edge26")
            else:
                print(f"✓ Extracted Edge26:")
                print(f"    Start: ({self.edge26_data.start_x:.2f}, {self.edge26_data.start_y:.2f}, {self.edge26_data.start_z:.2f})")
                print(f"    End: ({self.edge26_data.end_x:.2f}, {self.edge26_data.end_y:.2f}, {self.edge26_data.end_z:.2f})")
                print(f"    Midpoint: ({self.edge26_data.mid_x:.2f}, {self.edge26_data.mid_y:.2f}, {self.edge26_data.mid_z:.2f})")
                print(f"    Length: {self.edge26_data.length:.2f} mm")

            self.edge34_data = self.extract_edge_data(33)
            if not self.edge34_data:
                print("WARNING: Could not extract Edge34")
            else:
                print(f"✓ Extracted Edge34:")
                print(f"    Start: ({self.edge34_data.start_x:.2f}, {self.edge34_data.start_y:.2f}, {self.edge34_data.start_z:.2f})")
                print(f"    End: ({self.edge34_data.end_x:.2f}, {self.edge34_data.end_y:.2f}, {self.edge34_data.end_z:.2f})")
                print(f"    Midpoint: ({self.edge34_data.mid_x:.2f}, {self.edge34_data.mid_y:.2f}, {self.edge34_data.mid_z:.2f})")
                print(f"    Length: {self.edge34_data.length:.2f} mm")

            return self.edge26_data is not None or self.edge34_data is not None

        except Exception as e:
            print(f"ERROR extracting edges: {e}")
            return False

    def calculate_edge_normal(self, edge_data: EdgeData) -> Tuple[float, float, float]:
        """Calculate normal vector for an edge"""
        # For edges on the plate, the normal points perpendicular to the edge
        # in the XY plane

        dx = edge_data.end_x - edge_data.start_x
        dy = edge_data.end_y - edge_data.start_y

        # Perpendicular to edge in XY plane (rotate 90 degrees)
        # Normal = (-dy, dx) normalized
        norm = math.sqrt(dx**2 + dy**2)
        if norm > 0:
            normal_x = -dy / norm
            normal_y = dx / norm
        else:
            normal_x = 0.0
            normal_y = 0.0

        # Normal points in Z direction for plate-mounted servo
        normal_z = 1.0

        return (normal_x, normal_y, normal_z)

    def calculate_placement(self, z_offset: float = Z_OFFSET_DEFAULT) -> bool:
        """Calculate servo placement matrix"""
        try:
            if not self.edge26_data or not self.edge34_data:
                print("ERROR: Missing edge data for placement calculation")
                return False

            # Use midpoint of Edge26 as primary servo position
            servo_x = self.edge26_data.mid_x
            servo_y = self.edge26_data.mid_y

            # Z position: below Middle_Plate surface
            middle_plate_z = self.MIDDLE_PLATE_SPECS["z_position"]
            middle_plate_thickness = self.MIDDLE_PLATE_SPECS["thickness"]
            servo_z = middle_plate_z - middle_plate_thickness / 2.0 - z_offset

            # Calculate rotation angles
            # Pitch: 90 degrees (shaft perpendicular to plate, pointing down)
            pitch = self.PITCH_ROTATION

            # Roll and Yaw: 0 degrees (body parallel to plate)
            roll = 0.0
            yaw = 0.0

            self.placement = PlacementData(
                x=servo_x,
                y=servo_y,
                z=servo_z,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                z_offset=z_offset,
            )

            print(f"\n✓ Calculated servo placement:")
            print(f"    Position: X={self.placement.x:.2f} mm, Y={self.placement.y:.2f} mm, Z={self.placement.z:.2f} mm")
            print(f"    Rotation: Roll={self.placement.roll:.1f}°, Pitch={self.placement.pitch:.1f}°, Yaw={self.placement.yaw:.1f}°")
            print(f"    Z-offset: {self.placement.z_offset:.2f} mm below plate surface")

            return True

        except Exception as e:
            print(f"ERROR calculating placement: {e}")
            return False

    def validate_alignment(self) -> bool:
        """Validate servo alignment with edge midpoints"""
        try:
            print(f"\n✓ Alignment Validation:")

            if self.edge26_data and self.placement:
                # Distance from servo position to Edge26 midpoint
                dist26 = math.sqrt(
                    (self.placement.x - self.edge26_data.mid_x) ** 2 +
                    (self.placement.y - self.edge26_data.mid_y) ** 2
                )

                passed = dist26 < self.ALIGNMENT_TOLERANCE
                self.validations.append(ValidationResult(
                    check_name="Servo alignment to Edge26",
                    passed=passed,
                    details=f"Distance to Edge26 midpoint: {dist26:.3f} mm",
                    value=dist26,
                    tolerance=self.ALIGNMENT_TOLERANCE,
                ))
                status = "✓" if passed else "✗"
                print(f"  {status} Distance to Edge26 midpoint: {dist26:.3f} mm (tolerance: {self.ALIGNMENT_TOLERANCE} mm)")

            if self.edge34_data and self.placement:
                # Distance from servo position to Edge34 midpoint
                dist34 = math.sqrt(
                    (self.placement.x - self.edge34_data.mid_x) ** 2 +
                    (self.placement.y - self.edge34_data.mid_y) ** 2
                )

                passed = dist34 < self.ALIGNMENT_TOLERANCE
                self.validations.append(ValidationResult(
                    check_name="Servo alignment to Edge34",
                    passed=passed,
                    details=f"Distance to Edge34 midpoint: {dist34:.3f} mm",
                    value=dist34,
                    tolerance=self.ALIGNMENT_TOLERANCE,
                ))
                status = "✓" if passed else "✗"
                print(f"  {status} Distance to Edge34 midpoint: {dist34:.3f} mm (tolerance: {self.ALIGNMENT_TOLERANCE} mm)")

            return all(v.passed for v in self.validations)

        except Exception as e:
            print(f"ERROR validating alignment: {e}")
            return False

    def validate_clearances(self) -> bool:
        """Validate clearances with other plates"""
        try:
            print(f"\n✓ Clearance Validation:")

            # Check clearance to Top_Plate
            top_plate_z = 6.00  # From assemble_plates.py
            top_plate_thickness = 5.0  # Standard thickness
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
            bottom_plate_z = 3.00  # From assemble_plates.py
            bottom_plate_thickness = 5.0  # Standard thickness
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
                "edge_data": {
                    "edge26": self.edge26_data.to_dict() if self.edge26_data else None,
                    "edge34": self.edge34_data.to_dict() if self.edge34_data else None,
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

    def run(self, z_offset: float = Z_OFFSET_DEFAULT) -> bool:
        """Execute full position calculation process"""
        print("=" * 70)
        print("PHASE 2: SERVO MOTOR POSITION CALCULATION")
        print("=" * 70)
        print()

        try:
            script_dir = Path(__file__).parent
        except NameError:
            # Running in FreeCAD console
            script_dir = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"

        doc_path = script_dir / "plates_assembled.FCStd"

        # Load document
        if not self.load_document(str(doc_path)):
            return False

        print()

        # Find Middle_Plate
        if not self.find_middle_plate():
            return False

        print()

        # Extract edges
        if not self.extract_edges():
            print("WARNING: Edge extraction had issues, continuing with available data")

        print()

        # Calculate placement
        if not self.calculate_placement(z_offset):
            return False

        print()

        # Validate alignment
        if not self.validate_alignment():
            print("WARNING: Alignment validation failed")

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

    # Optional: read Z-offset from command line
    z_offset = ServoPositionCalculator.Z_OFFSET_DEFAULT
    if len(sys.argv) > 1:
        try:
            z_offset = float(sys.argv[1])
        except ValueError:
            print(f"WARNING: Invalid Z-offset '{sys.argv[1]}', using default {z_offset} mm")

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
