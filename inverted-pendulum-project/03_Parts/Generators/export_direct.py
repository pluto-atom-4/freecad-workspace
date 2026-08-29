#!/usr/bin/env python3
"""
Direct Python execution of Phase 4 export without FreeCAD --python wrapper.
This bypasses FreeCAD's output buffering issues.
"""

import sys
import os

# Ensure output is unbuffered
sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', buffering=1)
sys.stderr = os.fdopen(sys.stderr.fileno(), 'w', buffering=1)

# Add current directory to path
sys.path.insert(0, os.path.dirname(__file__))

print("=" * 70, flush=True)
print("PHASE 4: EXPORT/MERGE SERVO MOTOR ASSEMBLY (Direct Python)", flush=True)
print("=" * 70, flush=True)
print(f"Python: {sys.version}", flush=True)
print(f"Executable: {sys.executable}", flush=True)
print(f"Working directory: {os.getcwd()}", flush=True)
print(f"Script directory: {os.path.dirname(__file__)}", flush=True)
print()

# Try to import FreeCAD
try:
    print("Importing FreeCAD modules...", flush=True)
    import FreeCAD as App
    import Part
    import Mesh
    from FreeCAD import Vector, Placement, Rotation
    print(f"✓ FreeCAD version: {App.Version()}", flush=True)
    print()
except ImportError as e:
    print(f"ERROR: FreeCAD modules not available: {e}", flush=True)
    print("This script requires FreeCAD to be installed with Python support.", flush=True)
    sys.exit(1)

# Now import and run the exporter
try:
    print("Importing AssemblyExporter...", flush=True)
    from _04_export_assembly_merged import AssemblyExporter
    print("✓ AssemblyExporter imported successfully", flush=True)
    print()

    # Create and run the exporter
    print("Creating AssemblyExporter instance...", flush=True)
    exporter = AssemblyExporter()
    print("✓ AssemblyExporter instance created", flush=True)
    print()

    # Run the export
    print("Starting export process...", flush=True)
    success = exporter.run()
    print()
    print(f"Export result: {'SUCCESS' if success else 'FAILED'}", flush=True)

    # Close document
    if exporter.doc:
        try:
            App.closeDocument(exporter.doc.Name)
            print("✓ Document closed", flush=True)
        except:
            pass

    sys.exit(0 if success else 1)

except Exception as e:
    print(f"ERROR during export: {e}", flush=True)
    import traceback
    traceback.print_exc()
    sys.exit(1)
