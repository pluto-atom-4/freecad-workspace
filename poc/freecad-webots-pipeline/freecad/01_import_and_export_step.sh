#!/bin/bash
# Wrapper to run Stage 1 (FreeCAD import & STEP export) headless.
#
# Usage: ./01_import_and_export_step.sh
#
# Set FREECAD_BIN to choose a specific headless FreeCAD binary (defaults to
# freecadcmd on PATH), e.g.:
#   export FREECAD_BIN=/home/pluto-atom-4/.local/opt/freecad-1.1.3/usr/bin/freecadcmd

set -e

FREECAD_BIN="${FREECAD_BIN:-freecadcmd}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONVERSION_SCRIPT="$SCRIPT_DIR/01_import_and_export_step.py"

if [ ! -f "$CONVERSION_SCRIPT" ]; then
    echo "Conversion script not found: $CONVERSION_SCRIPT"
    exit 1
fi

echo "Starting Stage 1: FreeCAD import & STEP export..."
echo "Script: $CONVERSION_SCRIPT"
echo ""

"$FREECAD_BIN" -c "__file__=r'$CONVERSION_SCRIPT'; exec(open(__file__).read())" 2>&1 | grep -v "Wayland\|Qt\|EGL"

REPORT_JSON="$SCRIPT_DIR/output/stage1_conversion_report.json"
if [ -f "$REPORT_JSON" ]; then
    echo ""
    echo "Report: $REPORT_JSON"
else
    echo ""
    echo "Report not created — Stage 1 likely failed before writing it."
    exit 1
fi
