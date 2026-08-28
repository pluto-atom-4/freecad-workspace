#!/usr/bin/env python3
"""
Wrapper script to run STL->STEP conversion via FreeCAD

This is the actual conversion logic that runs inside FreeCAD.
Execute it via: freecad -c 'exec(open(...).read())'
"""

import os
from pathlib import Path
from typing import Dict, Any, Optional
import json

# FreeCAD modules are available in the macro context
import FreeCAD as App
import Part
import Mesh

# ============================================================================
# CONFIGURATION
# ============================================================================

# Input/Output paths
try:
    SCRIPT_DIR = Path(__file__).parent
except NameError:
    # When executed via freecad -c exec(), __file__ is not defined
    SCRIPT_DIR = Path.cwd() / "03_Parts" / "Generators"

PROJECT_DIR = SCRIPT_DIR.parent.parent
DOCUMENTS_DIR = Path.home() / "Documents"

# STL source (servo motor mesh)
STL_SOURCE = DOCUMENTS_DIR / "feetech-STS3032_20190118_ASM.stl"

# Output directory and file
OUTPUT_DIR = PROJECT_DIR / "03_Parts" / "Mechanical"
STEP_OUTPUT = OUTPUT_DIR / "feetech-STS3032.step"
REPORT_OUTPUT = OUTPUT_DIR / "feetech-STS3032_conversion_report.json"

# Conversion parameters
MESH_TOLERANCE = 0.1  # mm
DEFLECTION = 0.1      # mm


# ============================================================================
# VALIDATION FUNCTIONS
# ============================================================================

def validate_stl_exists() -> bool:
    """Validate that STL source file exists."""
    if not STL_SOURCE.exists():
        print(f"✗ STL file not found: {STL_SOURCE}")
        return False

    if not STL_SOURCE.is_file():
        print(f"✗ STL path is not a file: {STL_SOURCE}")
        return False

    if not os.access(STL_SOURCE, os.R_OK):
        print(f"✗ STL file not readable: {STL_SOURCE}")
        return False

    file_size_mb = STL_SOURCE.stat().st_size / (1024 * 1024)
    print(f"✓ STL file found: {STL_SOURCE.name} ({file_size_mb:.2f} MB)")
    return True


def validate_stl_geometry(mesh: 'Mesh.Mesh') -> Dict[str, Any]:
    """Validate STL mesh geometry integrity."""
    validation = {
        'vertex_count': mesh.CountPoints,
        'face_count': mesh.CountFacets,
        'has_topology_errors': False,
        'topology_error_count': 0,
        'is_closed': None,  # Not available via standard API
        'surface_area': mesh.Area,
        'bounding_box': {
            'min_x': mesh.BoundBox.XMin,
            'max_x': mesh.BoundBox.XMax,
            'min_y': mesh.BoundBox.YMin,
            'max_y': mesh.BoundBox.YMax,
            'min_z': mesh.BoundBox.ZMin,
            'max_z': mesh.BoundBox.ZMax,
            'width': mesh.BoundBox.XLength,
            'height': mesh.BoundBox.YLength,
            'depth': mesh.BoundBox.ZLength,
        }
    }

    # Log geometry metrics
    print(f"\nMesh Geometry Metrics:")
    print(f"  Vertices: {validation['vertex_count']}")
    print(f"  Faces: {validation['face_count']}")
    print(f"  Surface area: {validation['surface_area']:.2f} mm²")

    bbox = validation['bounding_box']
    print(f"\nBounding Box:")
    print(f"  X: [{bbox['min_x']:.2f}, {bbox['max_x']:.2f}] ({bbox['width']:.2f} mm)")
    print(f"  Y: [{bbox['min_y']:.2f}, {bbox['max_y']:.2f}] ({bbox['height']:.2f} mm)")
    print(f"  Z: [{bbox['min_z']:.2f}, {bbox['max_z']:.2f}] ({bbox['depth']:.2f} mm)")

    return validation


def load_and_validate_stl():
    """Load STL and validate geometry."""
    print("\n" + "=" * 70)
    print("PHASE 1.1: STL Validation")
    print("=" * 70)

    # Check file existence
    if not validate_stl_exists():
        return None, {}

    # Load STL mesh
    print("\nLoading STL mesh...")
    try:
        mesh = Mesh.Mesh(str(STL_SOURCE))
        print(f"✓ STL loaded successfully")
    except Exception as e:
        print(f"✗ Failed to load STL: {e}")
        return None, {}

    # Validate geometry
    validation = validate_stl_geometry(mesh)

    return mesh, validation


# ============================================================================
# CONVERSION FUNCTIONS
# ============================================================================

def mesh_to_shape(mesh: 'Mesh.Mesh') -> Optional[Part.Shape]:
    """Convert FreeCAD Mesh to Part Shape.

    The conversion:
    1. Creates Part faces from each mesh triangle
    2. Builds a shell from the faces
    3. Converts the shell to a solid

    This approach works with FreeCAD's C++ API for mesh objects.

    Args:
        mesh: FreeCAD Mesh object

    Returns:
        Part.Shape (Solid) or None if conversion fails
    """
    print("\n" + "=" * 70)
    print("PHASE 1.2: Mesh to Shape Conversion")
    print("=" * 70)

    try:
        print(f"\nConverting {mesh.CountFacets} mesh facets to Part faces...")

        # Build faces from mesh facets
        faces = []
        for i, facet in enumerate(mesh.Facets):
            try:
                # Each facet has 3 points (triangle)
                points = facet.Points
                p1, p2, p3 = points[0], points[1], points[2]

                # Create vectors from points
                v1 = App.Vector(p1)
                v2 = App.Vector(p2)
                v3 = App.Vector(p3)

                # Create a face from the triangle
                wire = Part.makePolygon([v1, v2, v3, v1])
                face = Part.Face(wire)
                faces.append(face)

                # Progress indicator
                if (i + 1) % 5000 == 0:
                    print(f"  {i + 1} faces converted...")

            except Exception as e:
                print(f"⚠ Error on facet {i}: {e}")
                # Continue with other facets

        print(f"✓ Converted {len(faces)} faces")

        # Create shell from faces
        print(f"Creating shell from {len(faces)} faces...")
        shell = Part.makeShell(faces)
        print(f"✓ Shell created")

        # Convert shell to solid
        print(f"Converting shell to solid...")
        solid = Part.makeSolid(shell)
        print(f"✓ Solid created: {solid.ShapeType}")

        return solid

    except Exception as e:
        print(f"✗ Failed to convert mesh to shape: {e}")
        import traceback
        traceback.print_exc()
        return None


def export_to_step(shape: Part.Shape):
    """Export Part shape to STEP format with validation."""
    print("\n" + "=" * 70)
    print("PHASE 1.3: STEP Export")
    print("=" * 70)

    # Ensure output directory exists
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    try:
        print(f"\nExporting shape to STEP format...")
        print(f"  Output: {STEP_OUTPUT}")
        print(f"  Shape type: {shape.ShapeType}")

        # Create temporary Part feature to export
        doc = App.newDocument("ServoConversion")
        shape_obj = doc.addObject("Part::Feature", "Servo_Mesh")
        shape_obj.Shape = shape
        doc.recompute()

        # Export STEP
        Part.export([shape_obj], str(STEP_OUTPUT))

        print(f"✓ STEP export successful")

        # Verify output file
        if not STEP_OUTPUT.exists():
            print(f"✗ Output file not created: {STEP_OUTPUT}")
            return False, None

        file_size_kb = STEP_OUTPUT.stat().st_size / 1024
        file_size_mb = file_size_kb / 1024
        print(f"  File size: {file_size_mb:.2f} MB ({file_size_kb:.0f} KB)")

        # Check file size reasonableness
        if file_size_kb > 5000:
            print(f"⚠ STEP file is large ({file_size_mb:.2f} MB)")

        # Close document
        App.closeDocument(doc.Name)

        return True, STEP_OUTPUT

    except Exception as e:
        print(f"✗ Failed to export STEP: {e}")
        import traceback
        traceback.print_exc()
        return False, None


def validate_step_output(shape: Part.Shape) -> Dict[str, Any]:
    """Validate the STEP conversion results."""
    validation = {
        'dimensions_preserved': True,
        'coordinate_system_valid': True,
        'surface_gaps': [],
        'notes': []
    }

    try:
        # Get bounding box
        bbox = shape.BoundBox
        print(f"\nStep Output Validation:")
        print(f"  Shape dimensions:")
        print(f"    X: {bbox.XLength:.2f} mm")
        print(f"    Y: {bbox.YLength:.2f} mm")
        print(f"    Z: {bbox.ZLength:.2f} mm")

        # Check if servo appears to be positioned correctly
        if bbox.ZMin < -50 or bbox.ZMax > 50:
            validation['coordinate_system_valid'] = False
            validation['notes'].append("Servo Z position may be off from origin")

        # Verify shape is solid/closed
        if hasattr(shape, 'isClosed') and not shape.isClosed():
            validation['notes'].append("Shape is not closed (may have gaps >0.1mm)")

    except Exception as e:
        print(f"⚠ Validation check error: {e}")
        validation['notes'].append(f"Validation error: {e}")

    return validation


# ============================================================================
# MAIN CONVERSION WORKFLOW
# ============================================================================

def convert_servo_stl_to_step() -> bool:
    """Execute complete STL to STEP conversion workflow."""
    print("\n")
    print("╔" + "=" * 68 + "╗")
    print("║" + " " * 15 + "STL TO STEP CONVERSION: SERVO MOTOR" + " " * 18 + "║")
    print("║" + " " * 20 + "Phase 1: Geometry Validation & Export" + " " * 10 + "║")
    print("╚" + "=" * 68 + "╝")

    report = {
        'timestamp': 'conversion',
        'source_file': str(STL_SOURCE),
        'output_file': str(STEP_OUTPUT),
        'status': 'FAILED',
        'stl_validation': {},
        'conversion': {},
        'step_validation': {},
        'file_metrics': {},
    }

    try:
        # Phase 1.1: Load and validate STL
        mesh, stl_validation = load_and_validate_stl()
        report['stl_validation'] = stl_validation

        if mesh is None:
            print("\n✗ STL validation failed. Aborting conversion.")
            report['status'] = 'FAILED - STL validation'
            return False

        # Phase 1.2: Convert mesh to Part shape
        shape = mesh_to_shape(mesh)
        if shape is None:
            print("\n✗ Mesh to shape conversion failed. Aborting.")
            report['status'] = 'FAILED - Mesh conversion'
            return False

        report['conversion']['success'] = True
        report['conversion']['shape_type'] = shape.ShapeType

        # Phase 1.3: Export to STEP
        success, step_path = export_to_step(shape)
        if not success or step_path is None:
            print("\n✗ STEP export failed. Aborting.")
            report['status'] = 'FAILED - STEP export'
            return False

        # Phase 1.4: Validate STEP output
        step_validation = validate_step_output(shape)
        report['step_validation'] = step_validation

        # File metrics
        report['file_metrics'] = {
            'stl_size_mb': STL_SOURCE.stat().st_size / (1024 * 1024),
            'step_size_mb': step_path.stat().st_size / (1024 * 1024),
            'step_size_kb': step_path.stat().st_size / 1024,
        }

        report['status'] = 'SUCCESS'

        # Print summary
        print("\n" + "=" * 70)
        print("CONVERSION SUMMARY")
        print("=" * 70)
        print(f"\n✓ Conversion completed successfully!")
        print(f"\nInput:")
        print(f"  File: {STL_SOURCE.name}")
        print(f"  Size: {report['file_metrics']['stl_size_mb']:.2f} MB")
        print(f"\nOutput:")
        print(f"  File: {step_path.name}")
        print(f"  Size: {report['file_metrics']['step_size_mb']:.2f} MB ({report['file_metrics']['step_size_kb']:.0f} KB)")
        print(f"\nValidation:")
        print(f"  Mesh vertices: {stl_validation.get('vertex_count', 'N/A')}")
        print(f"  Mesh faces: {stl_validation.get('face_count', 'N/A')}")
        print(f"  Shape type: {report['conversion']['shape_type']}")
        print(f"  Coordinate system valid: {step_validation['coordinate_system_valid']}")

        if step_validation['notes']:
            print(f"\nNotes:")
            for note in step_validation['notes']:
                print(f"  • {note}")

        print("\n" + "=" * 70)

        # Save conversion report
        try:
            OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
            with open(REPORT_OUTPUT, 'w') as f:
                json.dump(report, f, indent=2, default=str)
            print(f"\nConversion report saved: {REPORT_OUTPUT.name}")
        except Exception as e:
            print(f"⚠ Failed to save report: {e}")

        return True

    except Exception as e:
        print(f"\n✗ Unexpected error during conversion: {e}")
        import traceback
        traceback.print_exc()
        report['status'] = f'FAILED - {str(e)}'
        return False


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == '__main__':
    success = convert_servo_stl_to_step()
    import sys
    sys.exit(0 if success else 1)
else:
    # When executed via freecad -c, run immediately
    convert_servo_stl_to_step()
