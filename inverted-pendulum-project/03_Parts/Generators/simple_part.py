#!/usr/bin/env python3
"""
Simple Part Generator - FreeCAD Model Creation

Creates a parametric mechanical part with features:
- 40×20×15mm base block
- 4mm diameter through-hole at (12, 10)
- 3mm fillet on back edge
- 5mm chamfer on top-front edge
- Exports to FCStd and STEP formats

USAGE:
    Standard run:
        ./run_part.sh

    Or manually with AppImage Python:
        ~/tmp/squashfs-root/usr/bin/python simple_part.py

OUTPUT:
    - inverted-pendulum-project/03_Parts/Mechanical/T101pwb01_02_Part.FCStd
    - inverted-pendulum-project/03_Parts/Mechanical/T101pwb01_02_Part.step

REFERENCES:
    - FreeCAD Part Module: https://wiki.freecadweb.org/Part_Module
    - Best Practices: https://wiki.freecadweb.org/Scripting_basics
"""

import os
import sys
from pathlib import Path
from typing import Optional

# ============================================================================
# SETUP: FreeCAD AppImage Environment
# ============================================================================

def setup_appimage_environment() -> None:
    """Configure Python path for FreeCAD AppImage."""
    appimage_path = os.path.expanduser("~/.local/bin/FreeCAD_1.1.3-Linux-x86_64-py311.AppImage")
    if not os.path.exists(appimage_path):
        raise FileNotFoundError(f"FreeCAD AppImage not found: {appimage_path}")

    appimage_root = os.path.expanduser("~/tmp/squashfs-root")
    os.makedirs(os.path.expanduser("~/tmp"), exist_ok=True)

    if not os.path.exists(appimage_root):
        os.system(f"cd {os.path.expanduser('~/tmp')} && {appimage_path} --appimage-extract > /dev/null 2>&1")

    os.environ['APPIMAGE'] = appimage_path

    site_packages = f"{appimage_root}/usr/lib/python3.11/site-packages"
    lib_path = f"{appimage_root}/usr/lib"

    if os.path.exists(site_packages):
        sys.path.insert(0, site_packages)
    if os.path.exists(lib_path):
        sys.path.insert(0, lib_path)

    ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    os.environ['LD_LIBRARY_PATH'] = f"{lib_path}:{ld_path}"


setup_appimage_environment()

import FreeCAD as App
import Part

# ============================================================================
# CONSTANTS
# ============================================================================

# Main block dimensions (mm)
BLOCK_LENGTH = 40.0
BLOCK_WIDTH = 20.0
BLOCK_HEIGHT = 15.0

# Feature dimensions
HOLE_RADIUS = 4.0
FILLET_RADIUS = 3.0
CHAMFER_SIZE = 5.0

# Hole position (X, Y from block corner)
HOLE_X = 12.0
HOLE_Y = 10.0

# Part naming
PART_NAME = "T101pwb01_02_Part"
PART_LABEL = "T101pwb01_02_Part"

# Output paths
SCRIPT_DIR = Path(__file__).parent
OUTPUT_DIR = SCRIPT_DIR.parent / "Mechanical"
FCSTD_FILE = OUTPUT_DIR / f"{PART_NAME}.FCStd"
STEP_FILE = OUTPUT_DIR / f"{PART_NAME}.step"


# ============================================================================
# PART GENERATION FUNCTIONS
# ============================================================================

def create_base_block() -> 'Part.Part':
    """Create the main rectangular block.

    Returns:
        Part.Part: Block solid with dimensions (L × W × H)
    """
    return Part.makeBox(BLOCK_LENGTH, BLOCK_WIDTH, BLOCK_HEIGHT)


def apply_fillet(block: 'Part.Part') -> 'Part.Part':
    """Apply fillet to back edge (X=0, Z=0, parallel to Y-axis).

    Args:
        block: Input solid shape

    Returns:
        Part.Part: Filleted block shape

    Raises:
        RuntimeError: If fillet edge not found
    """
    fillet_edges = []
    for edge in block.Edges:
        bbox = edge.BoundBox
        # Back edge: X=0, Z=0, spans Y-direction
        if (abs(bbox.XMin) < 0.001 and abs(bbox.XMax) < 0.001 and
            abs(bbox.ZMin) < 0.001 and abs(bbox.ZMax) < 0.001):
            fillet_edges.append(edge)

    if not fillet_edges:
        raise RuntimeError("Fillet edge not found")

    return block.makeFillet(FILLET_RADIUS, fillet_edges)


def apply_chamfer(block: 'Part.Part') -> 'Part.Part':
    """Apply chamfer to top-front edge (X=40, Z=15, parallel to Y-axis).

    Args:
        block: Input solid shape

    Returns:
        Part.Part: Chamfered block shape

    Raises:
        RuntimeError: If chamfer edge not found
    """
    chamfer_edges = []
    for edge in block.Edges:
        bbox = edge.BoundBox
        # Top-front edge: X=BLOCK_LENGTH, Z=BLOCK_HEIGHT, spans Y-direction
        if (abs(bbox.XMin - BLOCK_LENGTH) < 0.001 and
            abs(bbox.XMax - BLOCK_LENGTH) < 0.001 and
            abs(bbox.ZMin - BLOCK_HEIGHT) < 0.001 and
            abs(bbox.ZMax - BLOCK_HEIGHT) < 0.001):
            chamfer_edges.append(edge)

    if not chamfer_edges:
        raise RuntimeError("Chamfer edge not found")

    return block.makeChamfer(CHAMFER_SIZE, chamfer_edges)


def create_through_hole() -> 'Part.Part':
    """Create cylindrical through-hole.

    Returns:
        Part.Part: Cylinder positioned and oriented for Z-axis through-hole
    """
    hole_center = App.Vector(HOLE_X, HOLE_Y, 0)
    hole_axis = App.Vector(0, 0, 1)
    # Cylinder height > block height to ensure clean cut through top and bottom
    cylinder = Part.makeCylinder(HOLE_RADIUS, BLOCK_HEIGHT + 2.0, hole_center, hole_axis)
    # Shift down to completely cut through
    cylinder.translate(App.Vector(0, 0, -1.0))
    return cylinder


def generate_part() -> bool:
    """Generate complete mechanical part with all features.

    Returns:
        bool: True if successful, False otherwise
    """
    try:
        # Ensure output directory exists
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

        # Create document
        doc = App.newDocument(PART_NAME)

        # Build geometry step-by-step
        print("Creating base block...")
        block = create_base_block()

        print("Applying fillet...")
        block = apply_fillet(block)

        print("Applying chamfer...")
        block = apply_chamfer(block)

        print("Creating through-hole...")
        hole = create_through_hole()

        print("Performing boolean cut...")
        final_shape = block.cut(hole)

        # Add to document and recompute
        print("Adding to FreeCAD document...")
        part_obj = doc.addObject("Part::Feature", PART_NAME)
        part_obj.Shape = final_shape
        doc.recompute()

        # Save FCStd format
        print(f"Saving FCStd: {FCSTD_FILE}")
        doc.saveAs(str(FCSTD_FILE))

        # Export STEP format
        print(f"Exporting STEP: {STEP_FILE}")
        Part.export([part_obj], str(STEP_FILE))

        # Print summary
        print("\n" + "=" * 60)
        print("✓ Part generated successfully")
        print("=" * 60)
        print(f"\nPart specifications:")
        print(f"  Block: {BLOCK_LENGTH}×{BLOCK_WIDTH}×{BLOCK_HEIGHT} mm")
        print(f"  Hole: Ø{HOLE_RADIUS*2} mm at ({HOLE_X}, {HOLE_Y})")
        print(f"  Fillet: {FILLET_RADIUS} mm radius")
        print(f"  Chamfer: {CHAMFER_SIZE} mm size")
        print(f"\nGenerated files:")
        print(f"  • {FCSTD_FILE}")
        print(f"  • {STEP_FILE}")
        print("\nNext steps:")
        print("  1. Open STEP file in FreeCAD")
        print("  2. Run in FreeCAD Python console for isometric view:")
        print("     import FreeCADGui as Gui")
        print("     Gui.activeView().viewIsometric()")
        print("     Gui.activeView().fitAll()")
        print("     Gui.activeView().redraw()")

        return True

    except Exception as e:
        print(f"\n✗ Error generating part: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc(file=sys.stderr)
        return False
    finally:
        # Clean up: close document properly
        try:
            if 'doc' in locals():
                App.closeDocument(doc.Name)
        except Exception as e:
            print(f"Warning: Failed to close document: {e}", file=sys.stderr)


if __name__ == '__main__':
    success = generate_part()
    sys.exit(0 if success else 1)

# 1. Create a new document or use the active one
doc = App.newDocument("SimplePartModel") if App.activeDocument() is None else App.activeDocument()

# --- Dimensions from the drawing ---
# Main block dimensions
BLOCK_L = 40.0   # Length (X)
BLOCK_W = 20.0   # Width (Y)
BLOCK_H = 15.0   # Height (Z)

# Top chamfer dimensions
CHAMFER_SIZE = 5.0

# Back edge fillet radius
FILLET_RADIUS = 3.0

# Hole dimensions and location
HOLE_RADIUS = 4.0
HOLE_X = 12.0
HOLE_Y = 10.0

# 2. Create the main solid block
block = Part.makeBox(BLOCK_L, BLOCK_W, BLOCK_H)

# 3. Apply the 3mm Fillet
# Find the specific edge along the back (at X=0, Z=0 parallel to Y-axis)
fillet_edges = []
for edge in block.Edges:
    # Identify the correct edge by looking for one spanning Y while sitting at X=0, Z=0
    bbox = edge.BoundBox
    if abs(bbox.XMin) < 0.001 and abs(bbox.XMax) < 0.001 and abs(bbox.ZMin) < 0.001 and abs(bbox.ZMax) < 0.001:
        fillet_edges.append(edge)

if fillet_edges:
    block = block.makeFillet(FILLET_RADIUS, fillet_edges)
else:
    print("Warning: Fillet edge not found.")

# 4. Apply the 5mm Chamfer
# Find the top-front edge (at X=40, Z=15 parallel to Y-axis)
chamfer_edges = []
for edge in block.Edges:
    bbox = edge.BoundBox
    if abs(bbox.XMin - BLOCK_L) < 0.001 and abs(bbox.XMax - BLOCK_L) < 0.001 and abs(bbox.ZMin - BLOCK_H) < 0.001 and abs(bbox.ZMax - BLOCK_H) < 0.001:
        chamfer_edges.append(edge)

if chamfer_edges:
    block = block.makeChamfer(CHAMFER_SIZE, chamfer_edges)
else:
    print("Warning: Chamfer edge not found.")

# 5. Create the through-hole cylinder
# Position it at (12, 10, 0) and orient its axis vertically (Z)
hole_center = App.Vector(HOLE_X, HOLE_Y, 0)
hole_axis = App.Vector(0, 0, 1)
# Make the cylinder slightly taller than the block to ensure a clean cut
cylinder = Part.makeCylinder(HOLE_RADIUS, BLOCK_H + 2.0, hole_center, hole_axis)

# Shift the cylinder down slightly on Z to completely cut through the top and bottom faces
cylinder.translate(App.Vector(0, 0, -1.0))

# 6. Perform the boolean cut to finalize the part
final_shape = block.cut(cylinder)

# 7. Load the final shape into the FreeCAD document GUI
part_obj = doc.addObject("Part::Feature", "T101pwb01_02_Part")
part_obj.Shape = final_shape

# Recompute the document to refresh the viewport
doc.recompute()

# 8. Save the document
fcstd_file = os.path.join(output_dir, "T101pwb01_02_Part.FCStd")
doc.saveAs(fcstd_file)
print(f"✓ Part generated: Mechanical/T101pwb01_02_Part.FCStd")
print(f"  Block dimensions: {BLOCK_L}x{BLOCK_W}x{BLOCK_H} mm")
print(f"  Hole radius: {HOLE_RADIUS} mm at ({HOLE_X}, {HOLE_Y})")
print(f"  Fillet radius: {FILLET_RADIUS} mm, Chamfer size: {CHAMFER_SIZE} mm")

# 9. Export to STEP format
step_file = os.path.join(output_dir, "T101pwb01_02_Part.step")
Part.export([part_obj], step_file)
print(f"✓ Exported to STEP: Mechanical/T101pwb01_02_Part.step")

# 10. Set isometric view and center model
# Note: View setup works in FreeCAD Python console but not when script runs separately
# To set view after opening STEP file in FreeCAD:
# 1. Open Python console (View → Panels → Python console)
# 2. Paste this code:
#
#    import FreeCADGui as Gui
#    view = Gui.activeView()
#    view.viewIsometric()
#    view.fitAll()
#    view.redraw()
#    print("✓ View: Isometric centered")
#

print("✓ Model files generated successfully")
print("→ To set isometric view: open STEP file in FreeCAD, run view code in Python console")
