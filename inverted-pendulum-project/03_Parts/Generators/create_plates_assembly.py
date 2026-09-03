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
        self.thickness = 2.5  # mm (live document value, verified via FreeCAD MCP 2026-09-02)
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

        # Handle both file-based and interactive (console) execution
        try:
            script_dir = Path(__file__).parent
        except NameError:
            # Running in FreeCAD console - __file__ not defined
            script_dir = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"

        # Servo mesh sources live in-repo under 03_Parts/Mechanical (not ~/Downloads).
        mechanical_dir = script_dir.parent / "Mechanical"
        self.servo_visual_path = mechanical_dir / "feetech-STS3032-visual-1.0mm.stl"
        self.servo_collision_path = mechanical_dir / "feetech-STS3032-collision-proxy.stl"

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
        """Import servo STL meshes (visual + collision proxy), grouped under an
        App::Part "STS3032_Mount", matching the live assembled document.

        Placement (identity rotation, position below) was verified via live
        FreeCAD MCP inspection on 2026-09-02: it fits Middle_Plate's hole
        cluster B (global center 32.9933, 25.7719) to within ~0.03-0.04mm.
        """
        servo_part = doc.addObject("App::Part", "STS3032_Mount")

        servo_placement = Placement(Vector(-150.1217, -141.9987, -3.1), Rotation(Vector(0, 0, 1), 0))

        mesh_specs = [
            ("feetech_STS3032_visual_1_0mm", self.servo_visual_path),
            ("feetech_STS3032_collision_proxy", self.servo_collision_path),
        ]

        mesh_objects = []
        for obj_name, stl_path in mesh_specs:
            if not stl_path.exists():
                print(f"WARNING: Servo STL not found at {stl_path}")
                continue

            mesh_obj = doc.addObject("Mesh::Feature", obj_name)
            mesh_obj.Mesh = Mesh.Mesh(str(stl_path))
            # Both visual and collision-proxy meshes carry the same placement
            # directly (they were synced to match this session) rather than
            # relying on the parent Part's placement.
            mesh_obj.Placement = servo_placement
            servo_part.addObject(mesh_obj)
            mesh_objects.append(mesh_obj)

        doc.recompute()
        return servo_part

    def group_plates(self, doc):
        """Group the three plate bodies under an App::Part "PlateStack",
        matching the live assembled document's structure."""
        plate_stack = doc.addObject("App::Part", "PlateStack")

        for config in self.plates_config:
            body = doc.getObject(f"{config.name}_Body")
            if body:
                plate_stack.addObject(body)
            else:
                print(f"WARNING: {config.name}_Body not found, cannot add to PlateStack")

        doc.recompute()
        return plate_stack

    def assembly_plates(self, doc):
        """Position plates in an initial/default layout.

        NOTE: these placements are a rough starting layout only, not the
        final assembly geometry. `assemble_plates.py` is the downstream step
        that applies the real, hole-aligned placements (Top/Middle/Bottom at
        their individually-derived positions and Z-axis rotations) — see
        that script's PlateAssembly.PLATE_ROTATIONS / assemble_with_alignment().
        Do not treat this function's output as final placement.
        """
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

        print("Grouping plates into PlateStack...")
        self.group_plates(self.doc)

        print("Assembling plates...")
        self.assembly_plates(self.doc)

        print("Importing servo...")
        self.import_servo(self.doc)

        self.doc.recompute()

        # Save document
        if output_path is None:
            try:
                output_dir = Path(__file__).parent
            except NameError:
                # Running in FreeCAD console - use default location
                output_dir = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"
            output_path = output_dir / "plates_assembly.FCStd"

        self.doc.saveAs(str(output_path))
        print(f"✓ Assembly saved to {output_path}")

        return self.doc


def main():
    """Main entry point"""
    generator = PlateGenerator()

    try:
        output_dir = Path(__file__).parent
    except NameError:
        # Running in FreeCAD console - use default location
        output_dir = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"
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
