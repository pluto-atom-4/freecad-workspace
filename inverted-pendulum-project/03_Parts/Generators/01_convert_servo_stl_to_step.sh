#!/bin/bash
# Wrapper script to run STL->STEP conversion via FreeCAD
#
# Usage: ./01_convert_servo_stl_to_step.sh
#
# This script runs the Python conversion logic inside FreeCAD,
# which provides access to the required Part, Mesh, and FreeCAD modules.

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONVERSION_SCRIPT="$SCRIPT_DIR/01_convert_servo_stl_to_step_via_freecad.py"

# Verify the conversion script exists
if [ ! -f "$CONVERSION_SCRIPT" ]; then
    echo "✗ Conversion script not found: $CONVERSION_SCRIPT"
    exit 1
fi

# Run the conversion via FreeCAD
echo "Starting STL to STEP conversion..."
echo "Script: $CONVERSION_SCRIPT"
echo ""

freecad -c "exec(open('$CONVERSION_SCRIPT').read())" 2>&1 | grep -v "Wayland\|Qt\|EGL" | grep -v "^Recompute"

# Check if output was created
OUTPUT_STEP="$SCRIPT_DIR/../Mechanical/feetech-STS3032.step"
REPORT_JSON="$SCRIPT_DIR/../Mechanical/feetech-STS3032_conversion_report.json"

if [ -f "$OUTPUT_STEP" ]; then
    echo ""
    echo "✓ Conversion completed successfully!"
    echo "  Output: $OUTPUT_STEP"

    # Show file size
    SIZE_KB=$(du -k "$OUTPUT_STEP" | cut -f1)
    SIZE_MB=$(echo "scale=2; $SIZE_KB / 1024" | bc)
    echo "  Size: ${SIZE_MB} MB"
else
    echo ""
    echo "✗ Output file not created"
    exit 1
fi

if [ -f "$REPORT_JSON" ]; then
    echo "  Report: $REPORT_JSON"
fi
