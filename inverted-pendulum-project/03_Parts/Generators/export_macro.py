#!/usr/bin/env python3
"""
FreeCAD Macro for Phase 4 Export
Run with: freecad -M export_macro.py or freecad --console --script export_macro.py
"""

import sys
import os

# Set script directory
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
os.chdir(SCRIPT_DIR)
sys.path.insert(0, SCRIPT_DIR)

print("=" * 70)
print("FREECAD MACRO: PHASE 4 EXPORT/MERGE SERVO ASSEMBLY")
print("=" * 70)
print(f"Working directory: {os.getcwd()}")
print()

# Import the exporter
try:
    from _04_export_assembly_merged import AssemblyExporter
    print("✓ AssemblyExporter imported successfully")
except ImportError as e:
    print(f"ERROR: Could not import AssemblyExporter: {e}")
    sys.exit(1)

# Create and run the exporter
try:
    exporter = AssemblyExporter()
    success = exporter.run()

    # Close document
    if exporter.doc:
        try:
            App.closeDocument(exporter.doc.Name)
        except:
            pass

    print()
    print("=" * 70)
    print(f"Export result: {'SUCCESS' if success else 'FAILED'}")
    print("=" * 70)

    sys.exit(0 if success else 1)

except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
