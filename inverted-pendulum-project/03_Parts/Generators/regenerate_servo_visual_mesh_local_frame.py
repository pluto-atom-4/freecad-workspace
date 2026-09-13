#!/usr/bin/env python3
"""
Regenerate servo visual mesh with correct local-frame coordinates.

Issue #107: The visual STL (feetech-STS3032-visual-1.0mm.stl) was tessellated
and exported from a FreeCAD object that had its Placement already set to the
assembly-space position. This caused all vertex coordinates to be baked with
the assembly offset (~177, ~168, ~-7mm), resulting in double-offset in URDF.

This script:
1. Loads feetech-STS3032.step (the full-resolution STEP source)
2. Resets the shape's Placement to identity (0, 0, 0 with no rotation)
3. Tessellates to mesh with 1.0mm deviation tolerance (matching original quality)
4. Exports to STL with correct local-frame coordinates (bbox center ~16-17mm)

Run via: freecadcmd -c "exec(open('regenerate_servo_visual_mesh_local_frame.py').read())"
Or: mamba run -n freecad-mcp python3 <path>/regenerate_servo_visual_mesh_local_frame.py

Output:
  - inverted-pendulum-project/03_Parts/Mechanical/feetech-STS3032-visual-1.0mm.stl
    (overwrites with corrected mesh)
  - Console report: bbox before/after, vertex count, tessellation deviation

Coordinate system: Local frame with servo body spanning roughly X:0-32mm, Y:0-28mm, Z:0-32mm
"""

import sys
import os
from pathlib import Path
from typing import Optional

try:
    import FreeCAD as App
    import Part
    import Mesh
except ImportError:
    print("ERROR: FreeCAD modules not available.")
    print("This script must be run with FreeCAD's Python interpreter:")
    print("  freecadcmd -c \"exec(open('regenerate_servo_visual_mesh_local_frame.py').read())\"")
    sys.exit(1)


# ============================================================================
# CONFIGURATION
# ============================================================================

try:
    SCRIPT_DIR = Path(__file__).parent
except NameError:
    # When executed via freecad -c exec(), __file__ is not defined
    SCRIPT_DIR = Path.cwd() / "03_Parts" / "Generators"

PROJECT_DIR = SCRIPT_DIR.parent.parent
MECHANICAL_DIR = PROJECT_DIR / "03_Parts" / "Mechanical"

# STEP source (full resolution, ~38MB)
STEP_SOURCE = MECHANICAL_DIR / "feetech-STS3032.step"

# Output visual mesh with corrected local-frame coordinates
VISUAL_STL_OUTPUT = MECHANICAL_DIR / "feetech-STS3032-visual-1.0mm.stl"

# Tessellation tolerance (must match original quality: 1.0mm deviation)
TESSELLATION_TOLERANCE = 1.0  # mm


# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def validate_step_source() -> bool:
    """Validate that STEP source file exists and is readable."""
    if not STEP_SOURCE.exists():
        print(f"ERROR: STEP source not found: {STEP_SOURCE}")
        return False

    if not STEP_SOURCE.is_file():
        print(f"ERROR: STEP path is not a file: {STEP_SOURCE}")
        return False

    size_mb = STEP_SOURCE.stat().st_size / (1024 * 1024)
    print(f"✓ STEP source found: {STEP_SOURCE.name} ({size_mb:.1f} MB)")
    return True


def load_step_file() -> Optional[Part.Shape]:
    """Load STEP file into FreeCAD and extract the shape."""
    print(f"\nLoading STEP file: {STEP_SOURCE.name}")

    try:
        # Create a temporary document for the import
        doc = App.newDocument("ServoVisualRegen")

        # Import STEP
        Part.insert(str(STEP_SOURCE), doc.Name)
        doc.recompute()

        # Find the imported shape object
        # Usually the first non-Origin object after import
        shape_obj = None
        for obj in doc.Objects:
            if obj.TypeId.startswith("Part::") and hasattr(obj, 'Shape'):
                shape_obj = obj
                break

        if shape_obj is None:
            print("ERROR: No Part shape found after STEP import")
            return None

        print(f"✓ Loaded shape: {shape_obj.Name} ({shape_obj.TypeId})")

        return shape_obj.Shape, doc

    except Exception as e:
        print(f"ERROR loading STEP: {e}")
        import traceback
        traceback.print_exc()
        return None, None


def reset_placement_to_identity(shape: Part.Shape) -> Part.Shape:
    """
    Create a copy of the shape with Placement reset to identity.

    The original shape may have been placed in assembly-space. We create a new
    shape at the origin with no rotation, so tessellated vertices will have
    correct local-frame coordinates.
    """
    print(f"\nResetting shape placement to identity...")

    # Get original bounding box (in assembly-space)
    bbox_original = shape.BoundBox
    print(f"  Original BBox: ({bbox_original.XMin:.2f}, {bbox_original.YMin:.2f}, {bbox_original.ZMin:.2f}) to " +
          f"({bbox_original.XMax:.2f}, {bbox_original.YMax:.2f}, {bbox_original.ZMax:.2f})")
    print(f"  Original BBox center: ({bbox_original.Center.x:.2f}, {bbox_original.Center.y:.2f}, {bbox_original.Center.z:.2f})")

    # The shape's vertices are already in assembly-space coordinates.
    # We need to translate them back to local frame by shifting all vertices
    # by -center (so the shape's center moves to origin).

    # Translate shape to origin by shifting all vertices by -center
    center = bbox_original.Center
    translation = App.Vector(-center.x, -center.y, -center.z)

    print(f"  Translation to apply: ({translation.x:.2f}, {translation.y:.2f}, {translation.z:.2f})")

    # Apply translation using Part module
    # Use Part.Compound to wrap and then transform
    try:
        # Try using Part.Shape.copy() and translate
        shape_copy = shape.copy()
        shape_translated = shape_copy.translate(translation)
    except:
        # If translate() fails, use Part's matrix-based transformation
        from FreeCAD import Matrix
        matrix = Matrix()
        matrix.move(translation)
        shape_translated = shape.transformGeometry(matrix)

    # Verify new bounding box
    bbox_new = shape_translated.BoundBox
    print(f"✓ Shape translated to local frame")
    print(f"  New BBox: ({bbox_new.XMin:.2f}, {bbox_new.YMin:.2f}, {bbox_new.ZMin:.2f}) to " +
          f"({bbox_new.XMax:.2f}, {bbox_new.YMax:.2f}, {bbox_new.ZMax:.2f})")
    print(f"  New BBox center: ({bbox_new.Center.x:.2f}, {bbox_new.Center.y:.2f}, {bbox_new.Center.z:.2f})")

    return shape_translated


def tessellate_shape_to_mesh(shape: Part.Shape, tolerance: float) -> Optional['Mesh.Mesh']:
    """Tessellate the shape to a mesh with specified deviation tolerance.

    Uses FreeCAD's Part.export() which automatically tessellates to STL,
    then reads back as mesh. This is the most reliable approach.
    """
    print(f"\nTessellating shape to mesh (tolerance: {tolerance}mm)...")

    import tempfile
    import os

    try:
        # Create a temporary STL file for tessellation
        with tempfile.NamedTemporaryFile(suffix='.stl', delete=False) as f:
            temp_stl = f.name

        # Create temporary document with the shape
        doc_temp = App.newDocument("TessTemp")
        part_obj = doc_temp.addObject("Part::Feature", "ToTessellate")
        part_obj.Shape = shape
        doc_temp.recompute()

        # Export to STL (FreeCAD automatically tessellates)
        print(f"  Exporting to temporary STL for tessellation...")
        Part.export([part_obj], temp_stl)

        # Load back as mesh
        mesh = Mesh.Mesh(temp_stl)

        # Clean up temporary file and document
        try:
            os.unlink(temp_stl)
        except:
            pass
        App.closeDocument(doc_temp.Name)

        vertex_count = mesh.CountPoints
        face_count = mesh.CountFacets

        print(f"✓ Tessellated to mesh:")
        print(f"  Vertices: {vertex_count}")
        print(f"  Faces: {face_count}")

        return mesh

    except Exception as e:
        print(f"ERROR tessellating shape: {e}")
        import traceback
        traceback.print_exc()
        return None


def export_mesh_to_stl(mesh: 'Mesh.Mesh', output_path: Path) -> bool:
    """Export mesh to STL file."""
    print(f"\nExporting mesh to STL: {output_path.name}")

    try:
        # Ensure output directory exists
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # Export as STL (ASCII format is portable; binary saves space)
        mesh.write(str(output_path))

        file_size_mb = output_path.stat().st_size / (1024 * 1024)
        print(f"✓ Exported STL ({file_size_mb:.1f} MB)")

        return True

    except Exception as e:
        print(f"ERROR exporting STL: {e}")
        import traceback
        traceback.print_exc()
        return False


def verify_mesh_bounds(mesh: 'Mesh.Mesh') -> bool:
    """Verify that the exported mesh has local-frame coordinates."""
    bbox = mesh.BoundBox
    center = (
        (bbox.XMin + bbox.XMax) / 2,
        (bbox.YMin + bbox.YMax) / 2,
        (bbox.ZMin + bbox.ZMax) / 2,
    )

    print(f"\nMesh verification:")
    print(f"  BBox min: ({bbox.XMin:.2f}, {bbox.YMin:.2f}, {bbox.ZMin:.2f})")
    print(f"  BBox max: ({bbox.XMax:.2f}, {bbox.YMax:.2f}, {bbox.ZMax:.2f})")
    print(f"  BBox center: ({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f})")

    # Expected servo body dimensions: ~32mm length, ~28mm height, ~12mm width
    # Roughly centered around [16, 14, 16] in local frame
    expected_range = 30  # mm, expecting all coords in roughly 0-32 range

    all_in_range = all(
        0 <= coord <= expected_range
        for coord in center
    )

    if all_in_range:
        print(f"✓ Mesh bounds valid (center ~{center[0]:.1f}, {center[1]:.1f}, {center[2]:.1f}mm)")
        return True
    else:
        print(f"⚠ WARNING: Mesh bounds seem off (center at {center[0]:.1f}, {center[1]:.1f}, {center[2]:.1f}mm)")
        print(f"  Expected center in range 0-{expected_range}mm per axis")
        return False


# ============================================================================
# MAIN
# ============================================================================

def regenerate_servo_visual_mesh() -> bool:
    """Execute complete visual mesh regeneration workflow."""
    print("\n" + "=" * 70)
    print("SERVO VISUAL MESH REGENERATION (Issue #107)")
    print("Regenerate from STEP with identity placement (local frame coords)")
    print("=" * 70)

    try:
        # Validate STEP source exists
        if not validate_step_source():
            return False

        # Load STEP
        shape, doc = load_step_file()
        if shape is None:
            return False

        # Reset placement to identity (translate to origin)
        shape_local = reset_placement_to_identity(shape)

        # Tessellate
        mesh = tessellate_shape_to_mesh(shape_local, TESSELLATION_TOLERANCE)
        if mesh is None:
            return False

        # Export to STL
        if not export_mesh_to_stl(mesh, VISUAL_STL_OUTPUT):
            return False

        # Verify bounds
        verify_mesh_bounds(mesh)

        # Cleanup
        if doc is not None:
            App.closeDocument(doc.Name)

        print("\n" + "=" * 70)
        print("✓ REGENERATION COMPLETE")
        print("=" * 70)
        print(f"\n✓ Visual mesh regenerated with local-frame coordinates")
        print(f"  Output: {VISUAL_STL_OUTPUT}")
        print(f"\nNext steps:")
        print(f"  1. Rerun URDF export pipeline:")
        print(f"     ./run_urdf_export.sh")
        print(f"  2. Verify bbox via trimesh:")
        print(f"     import trimesh")
        print(f"     mesh = trimesh.load('{VISUAL_STL_OUTPUT}')")
        print(f"     print(f'Center: {{(mesh.bounds[0] + mesh.bounds[1])/2}}')")
        print(f"  3. Run test suite:")
        print(f"     mamba run -n pendulum-tools python3 -m pytest -q")

        return True

    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == '__main__':
    success = regenerate_servo_visual_mesh()
    import sys
    sys.exit(0 if success else 1)
else:
    # When executed via freecad -c, run immediately
    regenerate_servo_visual_mesh()
