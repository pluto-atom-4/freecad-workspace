#!/usr/bin/env python3
"""
Debug script to test assemble_plates.py and capture actual errors.
Run in FreeCAD Python console.
"""

import sys
import traceback
from pathlib import Path

# Ensure plates exist first
print("=" * 70)
print("STEP 1: Generate plates")
print("=" * 70)

try:
    exec(open('/home/pluto-atom-4/freecad-workspace/inverted-pendulum-project/03_Parts/Generators/create_plates_simple.py').read())
    print("✓ Plates generated successfully\n")
except Exception as e:
    print(f"✗ Error generating plates: {e}")
    traceback.print_exc()
    sys.exit(1)

# Now test assembler with detailed error capture
print("\n" + "=" * 70)
print("STEP 2: Assemble plates (with error capture)")
print("=" * 70)

try:
    exec(open('/home/pluto-atom-4/freecad-workspace/inverted-pendulum-project/03_Parts/Generators/assemble_plates.py').read())
    print("✓ Assembly completed successfully")
except Exception as e:
    print(f"\n✗ ERROR during assembly:")
    print(f"  Type: {type(e).__name__}")
    print(f"  Message: {e}")
    print(f"\nFull traceback:")
    traceback.print_exc()
    sys.exit(1)

print("\n" + "=" * 70)
print("✓ All tests passed")
print("=" * 70)
