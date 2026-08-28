#!/bin/bash
# FreeCAD Plate Generator Wrapper

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

echo "=========================================="
echo "FreeCAD Plate Generator"
echo "=========================================="
echo ""
echo "Script: ${SCRIPT_DIR}/create_plates_simple.py"
echo "Output: ${SCRIPT_DIR}/plates_assembly.FCStd"
echo ""

# Find FreeCAD
FREECAD_BIN=""
if command -v freecad &> /dev/null; then
    FREECAD_BIN="freecad"
elif [ -f "/home/pluto-atom-4/.local/bin/FreeCAD_1.1.3-Linux-x86_64-py311.AppImage" ]; then
    FREECAD_BIN="/home/pluto-atom-4/.local/bin/FreeCAD_1.1.3-Linux-x86_64-py311.AppImage"
else
    echo "ERROR: FreeCAD not found"
    exit 1
fi

echo "Using FreeCAD: ${FREECAD_BIN}"
echo ""

# Run with timeout
timeout 120 "${FREECAD_BIN}" --python "${SCRIPT_DIR}/create_plates_simple.py" 2>&1 || {
    EXIT_CODE=$?
    if [ $EXIT_CODE -eq 124 ]; then
        echo "WARNING: Command timed out (120s)"
    else
        echo "ERROR: FreeCAD exited with code $EXIT_CODE"
    fi
}

echo ""
echo "Checking for output..."
if [ -f "${SCRIPT_DIR}/plates_assembly.FCStd" ]; then
    SIZE=$(ls -lh "${SCRIPT_DIR}/plates_assembly.FCStd" | awk '{print $5}')
    echo "✓ Output file created: ${SIZE}"
    ls -lh "${SCRIPT_DIR}/plates_assembly.FCStd"
else
    echo "⚠ Output file not created yet"
    echo "  Check if FreeCAD python modules are available"
fi

echo ""
echo "=========================================="
