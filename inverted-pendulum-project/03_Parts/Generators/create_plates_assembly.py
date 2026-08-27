#!/usr/bin/env python3
"""
3-Plate Assembly Generator for Inverted Pendulum Robot

Creates three aluminum linkage plates with servo integration:
  - Top Plate (60mm): connects upper joint
  - Middle Plate (~54.72mm): servo shaft coupling point
  - Bottom Plate (~52.43mm): connects lower joint

Servo: FeeTech STS3032 (integrated via STL mesh)

Usage:
  python3 create_plates_assembly.py

  Or with FreeCAD directly:
  freecad --gui && then run in Python console
"""

import sys
import os
from pathlib import Path

try:
    import FreeCAD as App
    import Part
    import Mesh
    from FreeCAD import Vector, Placement, Rotation
except ImportError:
    print("ERROR: FreeCAD Python modules not available. Run inside FreeCAD or activate FreeCAD environment.")
    sys.exit(1)


class PlateConfig:
    """Plate dimensions and parameters"""
    def __init__(self, name: str, overall_length: float, center_to_center: float):
        self.name = name
        self.overall_length = overall_length
        self.center_to_center = center_to_center
        self.width = 10.0  # mm
        self.thickness = 5.0  # mm
        self.hole_diameter = 5.0  # mm (M5)
        self.end_radius = 5.0  # mm


class PlateGenerator:
    """Generate 3D models for aluminum linkage plates"""

    def __init__(self):
        self.plates_config = [
            PlateConfig("Top_Plate", overall_length=60.0, center_to_center=50.0),
            PlateConfig("Middle_Plate", overall_length=54.72, center_to_center=44.72),
            PlateConfig("Bottom_Plate", overall_length=52.43, center_to_center=42.43),
        ]
        self.doc = None
        self.servo_path = Path(__file__).parent.parent.parent.parent / "Downloads" / "feetech-STS3032_20190118_ASM.stl"

    def create_plate_sketch(self, doc, name: str, config: PlateConfig):
        """Create 2D sketch for plate profile"""
        body = doc.addObject("PartDesign::Body", f"{name}_Body")
        sketch = doc.addObject("Sketcher::SketchObject", f"{name}_Sketch")
        body.addObject(sketch)

        # Set sketch plane to XY
        sketch.MapReversed = False
        sketch.Placement = Placement(Vector(0, 0, 0), Rotation(Vector(0, 0, 1), 0))

        # Create elongated profile with rounded ends
        # Center at origin, extends from -half_length/2 to +half_length/2
        half_length = config.center_to_center / 2.0
        half_width = config.width / 2.0

        # Geometry: rectangle with semi-circular ends
        # Line top
        sketch.addGeometry(Part.LineSegment(
            Vector(-half_length, half_width, 0),
            Vector(half_length, half_width, 0)
        ), False)

        # Line bottom
        sketch.addGeometry(Part.LineSegment(
            Vector(-half_length, -half_width, 0),
            Vector(half_length, -half_width, 0)
        ), False)

        # Left semi-circle (end radius)
        left_center = Vector(-half_length, 0, 0)
        left_circle = Part.Circle(left_center, Vector(0, 0, 1), config.end_radius)
        sketch.addGeometry(left_circle, False)

        # Right semi-circle
        right_center = Vector(half_length, 0, 0)
        right_circle = Part.Circle(right_center, Vector(0, 0, 1), config.end_radius)
        sketch.addGeometry(right_circle, False)

        # Constraints: circles at correct positions
        sketch.addConstraint(Part.Constraint("Coincident", 0, 2, 2, 3))  # Top line right end
        sketch.addConstraint(Part.Constraint("Coincident", 1, 2, 2, 3))  # Bottom line right end
        sketch.addConstraint(Part.Constraint("Coincident", 0, 1, 3, 3))  # Top line left end
        sketch.addConstraint(Part.Constraint("Coincident", 1, 1, 3, 3))  # Bottom line left end

        # Add holes (circles at specific positions)
        hole_radius = config.hole_diameter / 2.0
        hole_offset = config.center_to_center / 2.0

        # Left hole
        left_hole = Part.Circle(Vector(-hole_offset, 0, 0), Vector(0, 0, 1), hole_radius)
        sketch.addGeometry(left_hole, False)
        sketch.addConstraint(Part.Constraint("Equal", 4, 1, 4, 3))

        # Right hole
        right_hole = Part.Circle(Vector(hole_offset, 0, 0), Vector(0, 0, 1), hole_radius)
        sketch.addGeometry(right_hole, False)
        sketch.addConstraint(Part.Constraint("Equal", 5, 1, 5, 3))

        sketch.recompute()
        return body, sketch

    def create_plate_solid(self, doc, name: str, config: PlateConfig):
        """Create complete plate solid with holes"""
        body, sketch = self.create_plate_sketch(doc, name, config)

        # Pad sketch to thickness
        pad = doc.addObject("PartDesign::Pad", f"{name}_Pad")
        pad.Profile = sketch
        pad.Length = config.thickness
        pad.Midplane = True
        body.addObject(pad)
        body.Tip = pad

        # Pocket holes
        hole_sketch = doc.addObject("Sketcher::SketchObject", f"{name}_Holes")
        body.addObject(hole_sketch)

        hole_offset = config.center_to_center / 2.0
        hole_radius = config.hole_diameter / 2.0

        hole_sketch.addGeometry(Part.Circle(
            Vector(-hole_offset, 0, 0), Vector(0, 0, 1), hole_radius
        ), False)
        hole_sketch.addGeometry(Part.Circle(
            Vector(hole_offset, 0, 0), Vector(0, 0, 1), hole_radius
        ), False)
        hole_sketch.recompute()

        pocket = doc.addObject("PartDesign::Pocket", f"{name}_Pocket")
        pocket.Profile = hole_sketch
        pocket.Length = config.thickness
        pocket.Midplane = True
        body.addObject(pocket)
        body.Tip = pocket

        doc.recompute()
        return body

    def import_servo(self, doc):
        """Import servo STL file"""
        if not self.servo_path.exists():
            print(f"WARNING: Servo STL not found at {self.servo_path}")
            return None

        mesh_obj = doc.addObject("Mesh::Feature", "Servo_STS3032")
        mesh = Mesh.Mesh(str(self.servo_path))
        mesh_obj.Mesh = mesh

        # Position servo with middle plate
        # Servo shaft couples to middle plate center
        mesh_obj.Placement = Placement(Vector(0, 0, -10), Rotation(Vector(0, 0, 1), 0))

        doc.recompute()
        return mesh_obj

    def assembly_plates(self, doc):
        """Position plates in assembly configuration"""
        # Vertical spacing between plates
        spacing = 20.0  # mm

        # Top plate (positive Z)
        top_body = [o for o in doc.Objects if "Top_Plate" in o.Name and hasattr(o, 'Tip')][0]
        top_body.Placement = Placement(Vector(0, 0, spacing), Rotation(Vector(0, 0, 1), 0))

        # Middle plate (origin) - connects to servo
        middle_body = [o for o in doc.Objects if "Middle_Plate" in o.Name and hasattr(o, 'Tip')][0]
        middle_body.Placement = Placement(Vector(0, 0, 0), Rotation(Vector(0, 0, 1), 0))

        # Bottom plate (negative Z)
        bottom_body = [o for o in doc.Objects if "Bottom_Plate" in o.Name and hasattr(o, 'Tip')][0]
        bottom_body.Placement = Placement(Vector(0, 0, -spacing), Rotation(Vector(0, 0, 1), 0))

        doc.recompute()

    def generate(self, output_path: str = None):
        """Generate complete assembly"""
        # Create document
        self.doc = App.newDocument("Plates_Assembly")

        print("Creating plates...")
        for config in self.plates_config:
            print(f"  - {config.name} ({config.overall_length}mm)")
            try:
                self.create_plate_solid(self.doc, config.name, config)
            except Exception as e:
                print(f"    ERROR: {e}")
                continue

        print("Assembling plates...")
        self.assembly_plates(self.doc)

        print("Importing servo...")
        self.import_servo(self.doc)

        self.doc.recompute()

        # Save document
        if output_path is None:
            output_path = Path(__file__).parent / "plates_assembly.FCStd"

        self.doc.saveAs(str(output_path))
        print(f"✓ Assembly saved to {output_path}")

        return self.doc


def main():
    """Main entry point"""
    generator = PlateGenerator()

    output_dir = Path(__file__).parent
    output_path = output_dir / "plates_assembly.FCStd"

    print(f"Generating 3-plate assembly for inverted pendulum robot...")
    print(f"Output: {output_path}")
    print()

    doc = generator.generate(str(output_path))

    print()
    print("Assembly complete. Objects created:")
    for obj in doc.Objects:
        print(f"  - {obj.Name} ({obj.TypeId})")

    return doc


if __name__ == "__main__":
    main()
