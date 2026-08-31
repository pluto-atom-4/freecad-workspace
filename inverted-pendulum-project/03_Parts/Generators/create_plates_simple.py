#!/usr/bin/env python3
"""
Simple Plate Generator using Part Module (No PartDesign/Sketches)

Creates three aluminum linkage plates using basic Part geometry:
  - Top Plate (60mm): connects upper joint
  - Middle Plate (~54.72mm): servo shaft coupling point
  - Bottom Plate (~52.43mm): connects lower joint

This version uses Part module directly for better compatibility.

Usage:
  freecad --gui
  Then: File > Open Console > paste this file

  Or (headless — this script imports FreeCAD directly, it does NOT run in
  the pendulum-tools mamba env):
  "${FREECAD_BIN:-freecadcmd}" create_plates_simple.py
"""

import sys
import math
from pathlib import Path

try:
    import FreeCAD as App
    import Part
    from FreeCAD import Vector, Placement, Rotation
except ImportError:
    print("ERROR: FreeCAD Python modules not available.")
    print("Run inside FreeCAD environment or activate FreeCAD Python.")
    sys.exit(1)


class PlateGeometry:
    """Geometries for elongated plate with rounded ends and holes"""

    @staticmethod
    def create_rounded_plate(length: float, width: float, thickness: float) -> Part.Shape:
        """
        Create elongated plate with rounded ends.

        Args:
            length: Center-to-center distance between holes
            width: Width of plate (minor dimension)
            thickness: Thickness (Z-direction)

        Returns:
            Part.Shape of elongated rectangle with semi-circular ends
        """
        half_length = length / 2.0
        half_width = width / 2.0
        half_thickness = thickness / 2.0

        # Create main rectangle
        rect = Part.makeBox(length, width, thickness, Vector(-half_length, -half_width, -half_thickness))

        # Create semi-cylinders for rounded ends
        radius = half_width

        # Left semi-cylinder
        left_cyl = Part.makeCylinder(
            radius, thickness,
            Vector(-half_length, 0, -half_thickness),
            Vector(0, 0, 1)
        )

        # Right semi-cylinder
        right_cyl = Part.makeCylinder(
            radius, thickness,
            Vector(half_length, 0, -half_thickness),
            Vector(0, 0, 1)
        )

        # Union all parts (fuse = union in Part module)
        shape = rect.fuse(left_cyl)
        shape = shape.fuse(right_cyl)

        return shape

    @staticmethod
    def create_hole(diameter: float, thickness: float) -> Part.Shape:
        """Create cylindrical hole"""
        radius = diameter / 2.0
        half_thickness = thickness / 2.0
        hole = Part.makeCylinder(
            radius, thickness,
            Vector(0, 0, -half_thickness),
            Vector(0, 0, 1)
        )
        return hole

    @staticmethod
    def create_plate_with_holes(length: float, width: float, thickness: float,
                                hole_diameter: float = 5.0) -> Part.Shape:
        """Create complete plate with two mounting holes"""
        # Base shape
        plate = PlateGeometry.create_rounded_plate(length, width, thickness)

        # Holes positioned at center-to-center distance
        hole_offset = length / 2.0

        # Left hole
        left_hole = PlateGeometry.create_hole(hole_diameter, thickness)
        left_hole.Placement = Placement(Vector(-hole_offset, 0, 0), Rotation(Vector(0, 0, 1), 0))

        # Right hole
        right_hole = PlateGeometry.create_hole(hole_diameter, thickness)
        right_hole.Placement = Placement(Vector(hole_offset, 0, 0), Rotation(Vector(0, 0, 1), 0))

        # Cut holes from plate
        plate = plate.cut(left_hole)
        plate = plate.cut(right_hole)

        return plate


class PlateAssembly:
    """Generate and assemble all three plates"""

    PLATE_SPECS = [
        {
            "name": "Top_Plate",
            "overall_length": 60.0,
            "center_to_center": 50.0,
            "z_position": 20.0,
        },
        {
            "name": "Middle_Plate",
            "overall_length": 54.72,
            "center_to_center": 44.72,
            "z_position": 0.0,
        },
        {
            "name": "Bottom_Plate",
            "overall_length": 60.0,
            "center_to_center": 50.0,
            "z_position": -20.0,
        },
    ]

    PLATE_PARAMS = {
        "width": 10.0,
        "thickness": 2.5,  # Reduced to 50% (was 5.0mm)
        "hole_diameter": 5.0,
    }

    def __init__(self):
        self.doc = None

        # Handle both file-based and interactive (console) execution
        try:
            script_dir = Path(__file__).parent
        except NameError:
            # Running in FreeCAD console - __file__ not defined
            script_dir = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"

        self.servo_stl_path = script_dir.parent.parent.parent / "Downloads" / "feetech-STS3032_20190118_ASM.stl"

    def create_plates(self):
        """Create all three plate objects in FreeCAD document"""
        print("Creating plate geometries...")

        for spec in self.PLATE_SPECS:
            name = spec["name"]
            length = spec["center_to_center"]
            z_pos = spec["z_position"]

            print(f"  • {name} (length: {spec['overall_length']}mm, C-C: {length}mm)")

            # Bottom plate has special geometry: 2 holes left side + 1 hole right side
            if "Bottom" in name:
                # Create base plate
                plate = PlateGeometry.create_rounded_plate(
                    length=length,
                    width=self.PLATE_PARAMS["width"],
                    thickness=self.PLATE_PARAMS["thickness"],
                )

                # Cut 2 holes on LEFT side: leftmost hole at center of left rounded end
                # 10mm apart along length
                left_end = -length / 2.0
                # Leftmost hole: centered at left rounded end
                left_hole_1 = PlateGeometry.create_hole(self.PLATE_PARAMS["hole_diameter"],
                                                       self.PLATE_PARAMS["thickness"])
                left_hole_1.Placement = Placement(Vector(left_end, 0, 0),
                                                 Rotation(Vector(0, 0, 1), 0))
                plate = plate.cut(left_hole_1)

                # Second hole: 12.5mm from left end
                left_hole_2 = PlateGeometry.create_hole(self.PLATE_PARAMS["hole_diameter"],
                                                       self.PLATE_PARAMS["thickness"])
                left_hole_2.Placement = Placement(Vector(left_end + 12.5, 0, 0),
                                                 Rotation(Vector(0, 0, 1), 0))
                plate = plate.cut(left_hole_2)

                # Cut 1 hole on RIGHT side (centered)
                right_x = length / 2.0
                right_hole = PlateGeometry.create_hole(self.PLATE_PARAMS["hole_diameter"],
                                                      self.PLATE_PARAMS["thickness"])
                right_hole.Placement = Placement(Vector(right_x, 0, 0),
                                                Rotation(Vector(0, 0, 1), 0))
                plate = plate.cut(right_hole)

                shape = plate
                print(f"    └─ Custom: 2 holes left (10mm apart), 1 hole right (center)")
            else:
                # Top and Middle plates: standard 2 holes (center-to-center)
                shape = PlateGeometry.create_plate_with_holes(
                    length=length,
                    width=self.PLATE_PARAMS["width"],
                    thickness=self.PLATE_PARAMS["thickness"],
                    hole_diameter=self.PLATE_PARAMS["hole_diameter"],
                )

            # Add to document
            plate_obj = self.doc.addObject("Part::Feature", name)
            plate_obj.Shape = shape
            plate_obj.Placement = Placement(Vector(0, 0, z_pos), Rotation(Vector(0, 0, 1), 0))

            # Set appearance
            plate_obj.ViewObject.ShapeColor = (0.8, 0.8, 0.8, 0.0)  # Light gray
            plate_obj.ViewObject.LineColor = (0.5, 0.5, 0.5, 0.0)

    def import_servo(self):
        """Import and position servo mesh"""
        if not self.servo_stl_path.exists():
            print(f"⚠ Servo STL not found: {self.servo_stl_path}")
            return

        print(f"Importing servo mesh: {self.servo_stl_path.name}")

        # Import as mesh object
        servo = self.doc.addObject("Mesh::Feature", "Servo_STS3032")

        # Load STL
        try:
            import Mesh
            mesh_data = Mesh.Mesh(str(self.servo_stl_path))
            servo.Mesh = mesh_data

            # Position with middle plate
            # Servo shaft couples at origin
            servo.Placement = Placement(
                Vector(0, 0, -8),  # Slightly below middle plate
                Rotation(Vector(0, 0, 1), 0)
            )

            servo.ViewObject.ShapeColor = (0.2, 0.2, 0.2, 0.0)  # Dark gray
            print("  ✓ Servo positioned at origin (Z = -8mm)")

        except Exception as e:
            print(f"  ✗ Error importing servo: {e}")

    def generate(self, output_path: str = None) -> App.Document:
        """Generate complete assembly"""
        # Create new document
        self.doc = App.newDocument("Plates_Assembly")
        print(f"Created FreeCAD document: {self.doc.Name}\n")

        # Create all plates
        self.create_plates()

        # Import servo
        print()
        self.import_servo()

        # Recompute document
        self.doc.recompute()

        # Save
        if output_path is None:
            try:
                output_dir = Path(__file__).parent
            except NameError:
                # Running in FreeCAD console - use default location
                output_dir = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"
            output_path = output_dir / "plates_assembly.FCStd"

        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        self.doc.saveAs(str(output_path))
        print(f"\n✓ Assembly saved: {output_path}")

        return self.doc


def main():
    """Main entry point"""
    print("=" * 60)
    print("3-Plate Assembly Generator (Inverted Pendulum Robot)")
    print("=" * 60)
    print()

    generator = PlateAssembly()
    doc = generator.generate()

    print()
    print("Objects created:")
    for obj in doc.Objects:
        if hasattr(obj, 'Shape'):
            bounds = obj.Shape.BoundBox
            print(f"  • {obj.Name:20s} | Bounds: {bounds.XLength:.1f}×{bounds.YLength:.1f}×{bounds.ZLength:.1f} mm")
        else:
            print(f"  • {obj.Name:20s}")

    return doc


if __name__ == "__main__":
    try:
        doc = main()
        print("\n✓ Generation complete!")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
