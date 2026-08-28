#!/usr/bin/env python3
"""
Assemble Three Plates - Position with Explicit Coordinates

Creates an assembly with three plates positioned at precise coordinates
and rotated according to image analysis:
- Plate 1 (Top): [-1.98, 33.30, 6.00] mm, rotated 53.1°
- Plate 2 (Middle): [13.00, 15.76, 4.00] mm, rotated 26.6°
- Plate 3 (Bottom): [3.01, -1.73, 3.00] mm, rotated -36.9°

All positions and rotations derived from mechanical linkage diagram.
"""

import sys
from pathlib import Path

try:
    import FreeCAD as App
    import Part
    from FreeCAD import Vector, Placement, Rotation
except ImportError:
    print("ERROR: FreeCAD Python modules not available.")
    sys.exit(1)


class PlateHoleCalculator:
    """Calculate hole positions for plate alignment"""

    # Plate hole specifications (for reference only - positions are hardcoded)
    PLATE_SPECS = {
        "Top_Plate": 50.0,      # Holes ±25mm from center
        "Middle_Plate": 44.72,  # Holes ±22.36mm from center
        "Bottom_Plate": 50.0,   # Special: left_1=-21.215mm, left_2=-8.715mm, right=28.0mm
    }

    @staticmethod
    def get_plate_holes(plate_name: str) -> dict:
        """Get hole positions for a plate relative to its center"""
        cc_dist = PlateHoleCalculator.PLATE_SPECS.get(plate_name)

        if not cc_dist:
            return {}

        if "Bottom" in plate_name:
            # Special configuration: 3 holes (right side stretched)
            # Left holes fixed, right hole extended beyond original center-to-center
            return {
                "left_1": -21.215,        # Fixed at rounded end center
                "left_2": -8.715,         # Fixed at 12.5mm from left end
                "right": 28.0,            # Extended right (stretched from original 21.215mm)
            }
        else:
            # Standard: 2 holes at center-to-center distance
            half_cc = cc_dist / 2.0
            return {
                "left": -half_cc,
                "right": half_cc,
            }

    @staticmethod
    def calculate_plate_offset(plate_name: str, target_hole_x: float, target_hole_name: str = "left") -> float:
        """
        Calculate offset needed to align plate hole with target position.

        Args:
            plate_name: Name of plate to position
            target_hole_x: X position of target hole on reference plate
            target_hole_name: Which hole to align ("left", "right", "left_2")

        Returns:
            X offset for plate center to align with target
        """
        holes = PlateHoleCalculator.get_plate_holes(plate_name)

        if target_hole_name not in holes:
            print(f"ERROR: {plate_name} has no hole '{target_hole_name}'")
            return 0.0

        hole_offset = holes[target_hole_name]
        plate_center_offset = target_hole_x - hole_offset

        return plate_center_offset


class PlateAssembly:
    """Assemble plates with hole alignment and rotation"""

    # Plate rotation angles (degrees) derived from grid analysis
    PLATE_ROTATIONS = {
        "Top_Plate": 53.1,      # arctan(4/3) = 53.1°
        "Middle_Plate": 26.6,   # arctan(2/4) = 26.6°
        "Bottom_Plate": -36.9,  # arctan(-3/4) = -36.9°
    }

    def __init__(self):
        self.doc = None
        self.plates = {}

    def load_plates(self) -> bool:
        """Load existing plates from generated assembly"""
        try:
            # Handle both file-based and interactive (console) execution
            try:
                script_dir = Path(__file__).parent
            except NameError:
                # Running in FreeCAD console - __file__ not defined
                script_dir = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"

            # Try to open existing assembly
            plates_file = script_dir / "plates_assembly.FCStd"
            if not plates_file.exists():
                print(f"⚠ Plates file not found: {plates_file}")
                return False

            # Open document
            self.doc = App.openDocument(str(plates_file))
            print(f"✓ Loaded: {plates_file.name}")

            # Find plate objects
            for obj in self.doc.Objects:
                if "Plate" in obj.Name:
                    self.plates[obj.Name] = obj
                    print(f"  • {obj.Name}")

            return len(self.plates) >= 3

        except Exception as e:
            print(f"ERROR loading plates: {e}")
            return False

    def assemble_with_alignment(self) -> bool:
        """Position plates with hole alignment"""
        if len(self.plates) < 3:
            print("ERROR: Need at least 3 plate objects")
            return False

        print("\nPositioning plates...")

        # Get plate objects
        bottom_plate = self.plates.get("Bottom_Plate")
        if not bottom_plate:
            print("ERROR: Bottom_Plate not found")
            return False

        # Position Plate 1 (Top) at explicit position
        top_plate = self.plates.get("Top_Plate")
        if top_plate:
            # Position from image measurement: [-1.98, 33.30, 6.00] mm
            position = Vector(-1.98, 33.30, 6.00)
            angle = self.PLATE_ROTATIONS.get("Top_Plate", 0)
            top_plate.Placement = Placement(
                position,
                Rotation(Vector(0, 0, 1), angle)  # Rotation around Z-axis
            )

            print(f"\nTop Plate positioning:")
            print(f"  Position: X={position.x:.2f}mm, Y={position.y:.2f}mm, Z={position.z:.2f}mm")
            print(f"  Rotation: {angle}° around Z-axis")

        # Position Plate 2 (Middle) at explicit position
        middle_plate = self.plates.get("Middle_Plate")
        if middle_plate:
            # Position from image measurement: [13.00, 15.76, 4.00] mm
            position = Vector(13.00, 15.76, 4.00)
            angle = self.PLATE_ROTATIONS.get("Middle_Plate", 0)
            middle_plate.Placement = Placement(
                position,
                Rotation(Vector(0, 0, 1), angle)  # Rotation around Z-axis
            )

            print(f"\nMiddle Plate positioning:")
            print(f"  Position: X={position.x:.2f}mm, Y={position.y:.2f}mm, Z={position.z:.2f}mm")
            print(f"  Rotation: {angle}° around Z-axis")

        # Position Bottom Plate at explicit position with rotation
        if bottom_plate:
            # Position from image measurement: [3.01, -1.73, 3.00] mm
            position = Vector(3.01, -1.73, 3.00)
            angle = self.PLATE_ROTATIONS.get("Bottom_Plate", 0)
            bottom_plate.Placement = Placement(
                position,
                Rotation(Vector(0, 0, 1), angle)  # Rotation around Z-axis
            )
            print(f"\nBottom Plate positioning:")
            print(f"  Position: X={position.x:.2f}mm, Y={position.y:.2f}mm, Z={position.z:.2f}mm")
            print(f"  Rotation: {angle}° around Z-axis")

        return True

    def save_assembly(self, output_path: str = None) -> bool:
        """Save assembled configuration"""
        if not self.doc:
            return False

        if output_path is None:
            try:
                output_dir = Path(__file__).parent
            except NameError:
                # Running in FreeCAD console - __file__ not defined
                output_dir = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"
            output_path = output_dir / "plates_assembled.FCStd"

        try:
            self.doc.saveAs(str(output_path))
            print(f"\n✓ Assembly saved: {output_path}")
            return True
        except Exception as e:
            print(f"ERROR saving assembly: {e}")
            return False

    def run(self):
        """Execute full assembly process"""
        print("=" * 70)
        print("PLATE ASSEMBLY - HOLE ALIGNMENT")
        print("=" * 70)
        print()

        # Load plates
        if not self.load_plates():
            print("ERROR: Could not load plates")
            return False

        # Assemble with alignment
        if not self.assemble_with_alignment():
            print("ERROR: Assembly failed")
            return False

        # Recompute
        self.doc.recompute()

        # Save
        if not self.save_assembly():
            print("ERROR: Could not save assembly")
            return False

        print()
        print("=" * 70)
        print("✓ Assembly complete with hole alignment")
        print("=" * 70)

        return True


def main():
    """Main entry point"""
    assembler = PlateAssembly()
    success = assembler.run()
    return 0 if success else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
