#!/usr/bin/env python3
"""Debug version with file logging"""

import sys
import os
from pathlib import Path

LOG_FILE = Path(__file__).parent / "generator_debug.log"

def log(msg):
    with open(LOG_FILE, 'a') as f:
        f.write(msg + '\n')
    print(msg)

log("="*60)
log("PLATE GENERATOR DEBUG")
log("="*60)

try:
    log(f"Python: {sys.version}")
    log(f"CWD: {os.getcwd()}")
    
    log("Importing FreeCAD...")
    import FreeCAD as App
    log(f"✓ FreeCAD {App.Version()[0]}.{App.Version()[1]}")
    
    log("Importing Part...")
    import Part
    log("✓ Part module")
    
    log("Importing Vector, Placement, Rotation...")
    from FreeCAD import Vector, Placement, Rotation
    log("✓ Transforms")
    
    log("Importing generator...")
    from create_plates_simple import PlateAssembly
    log("✓ Generator loaded")
    
    log("Creating assembly...")
    generator = PlateAssembly()
    doc = generator.generate()
    log("✓ Generation complete!")
    
    log("Checking output...")
    output_path = Path(__file__).parent / "plates_assembly.FCStd"
    if output_path.exists():
        size = output_path.stat().st_size
        log(f"✓ Output file: {output_path.name} ({size} bytes)")
    else:
        log("✗ Output file not found")
    
except Exception as e:
    log(f"✗ Error: {e}")
    import traceback
    log(traceback.format_exc())
    sys.exit(1)

log("="*60)
log("DEBUG LOG COMPLETE")
log("="*60)
