#!/usr/bin/env python3
"""Test script for plate generator - includes debug output"""

import sys
import os
from pathlib import Path

print("=" * 60)
print("PLATE GENERATOR TEST")
print("=" * 60)
print()

print(f"Python: {sys.version}")
print(f"CWD: {os.getcwd()}")
print(f"Script: {__file__}")
print()

# Try importing FreeCAD
print("Attempting FreeCAD imports...")
try:
    import FreeCAD as App
    print(f"✓ FreeCAD {App.Version()[0]}.{App.Version()[1]}")
except ImportError as e:
    print(f"✗ FreeCAD import failed: {e}")
    sys.exit(1)

try:
    import Part
    print("✓ Part module")
except ImportError as e:
    print(f"✗ Part import failed: {e}")
    sys.exit(1)

try:
    from FreeCAD import Vector, Placement, Rotation
    print("✓ Vector, Placement, Rotation")
except ImportError as e:
    print(f"✗ Transform imports failed: {e}")
    sys.exit(1)

try:
    import Mesh
    print("✓ Mesh module")
except ImportError as e:
    print(f"✗ Mesh import failed: {e}")
    # Non-fatal

print()
print("Creating test plate...")

try:
    # Create new document
    doc = App.newDocument("TEST_Plates")
    print(f"✓ Document created: {doc.Name}")

    # Create simple box
    box_shape = Part.makeBox(50, 10, 5, Vector(-25, -5, -2.5))
    print(f"✓ Box shape created: {box_shape.Volume:.1f} mm³")

    # Create cylinder (hole)
    cyl_shape = Part.makeCylinder(2.5, 5, Vector(-25, 0, -2.5), Vector(0, 0, 1))
    print(f"✓ Cylinder shape created: {cyl_shape.Volume:.1f} mm³")

    # Boolean cut
    plate_shape = box_shape.cut(cyl_shape)
    print(f"✓ Boolean cut successful: {plate_shape.Volume:.1f} mm³")

    # Add to document
    plate_obj = doc.addObject("Part::Feature", "TestPlate")
    plate_obj.Shape = plate_shape
    print(f"✓ Object added to document: {plate_obj.Name}")

    # Recompute
    doc.recompute()
    print("✓ Document recomputed")

    # Save
    output_path = Path(__file__).parent / "test_plates.FCStd"
    doc.saveAs(str(output_path))
    print(f"✓ Document saved: {output_path}")
    print(f"  File size: {output_path.stat().st_size} bytes")

    print()
    print("=" * 60)
    print("✓ TEST PASSED - All operations successful!")
    print("=" * 60)

except Exception as e:
    print(f"✗ ERROR: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
