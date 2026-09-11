#!/bin/bash
# URDF Export Pipeline (Phases 7-11)
#
# Runs the complete URDF export pipeline:
#   Phase 7: Create body + wheel geometry (freecadcmd)
#   Phase 8: Configure assembly joints (REQUIRES MCP BRIDGE -- see below)
#   Phase 9: Compute mass properties (freecadcmd)
#   Phase 10: Export URDF (pure Python)
#   Phase 11: Validate inertia (pure Python)
#
# Prerequisites:
#   - freecadcmd available (set via FREECAD_BIN or on PATH)
#   - plates_servo_assembled.FCStd in this directory
#   - robot_parameters.yaml in ../../02_Design_Inputs/
#
# KNOWN LIMITATION - Phase 8 (Assembly Joints):
#   Phase 8 (08_configure_assembly_joints.py) cannot run in plain headless
#   freecadcmd 1.1.3 -- Assembly::JointGroup/Joint creation segfaults.
#   See 08_configure_assembly_joints.py's docstring for the full explanation.
#
#   To run Phase 8:
#   a) Via the FreeCAD MCP bridge (if running):
#      Use the bridge's execute_python tool with the script (see CLAUDE.md)
#   b) Manually via freecadcmd with a GUI:
#      (requires a GUI-capable FreeCAD installation, not headless)
#   c) Use pre-generated robot_assembly.FCStd if available
#
# Usage:
#   ./run_urdf_export.sh
#   # or with a specific FreeCAD build:
#   FREECAD_BIN=/path/to/freecadcmd ./run_urdf_export.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
FREECAD_BIN="${FREECAD_BIN:-freecadcmd}"

echo "=== Phase 7: Body + wheel geometry ==="
# freecadcmd -c enters interactive REPL after script, so wrap with timeout to force exit
# We check for output files as success indicator (freecadcmd's exit code is unreliable)
timeout 120 bash -c "echo \"exec(open('07_create_body_and_wheels.py').read())\" | '$FREECAD_BIN' -c" 2>&1 || {
    EXIT_CODE=$?
    # Timeout (124) is expected; non-timeout failures may still have created outputs
    if [ $EXIT_CODE -ne 124 ] && [ $EXIT_CODE -ne 1 ]; then
        echo "ERROR: Unexpected exit code $EXIT_CODE"
        exit $EXIT_CODE
    fi
}
# Verify output was created (success indicator for this phase)
if [ ! -f "robot_body_wheels.FCStd" ] || [ ! -f "07_body_wheels_metadata.json" ]; then
    echo "ERROR: Phase 7 output files not created"
    exit 1
fi
echo "✓ Phase 7 complete - output files created"

echo ""
echo "!!! PHASE 8 (ASSEMBLY JOINTS) REQUIRES MCP BRIDGE !!!"
echo ""
echo "Phase 8 cannot run in plain headless freecadcmd (segfault on Assembly::JointGroup creation)."
echo "See 08_configure_assembly_joints.py docstring for details."
echo ""
echo "Phases 9-11 depend on robot_assembly.FCStd (Phase 8 output)."
echo ""

if [ ! -f "robot_assembly.FCStd" ]; then
    echo "ERROR: robot_assembly.FCStd not found."
    echo ""
    echo "To complete the pipeline, you must first run Phase 8 via the FreeCAD MCP bridge:"
    echo "  1. Start the FreeCAD MCP bridge (see root CLAUDE.md for instructions)"
    echo "  2. Run Phase 8 via the bridge's execute_python tool"
    echo "  3. Or use a pre-generated robot_assembly.FCStd if available"
    echo ""
    echo "After Phase 8 completes, run this script again to continue with phases 9-11."
    exit 1
fi

echo "=== Phase 9: Mass properties ==="
echo "exec(open('09_compute_mass_properties.py').read())" | "$FREECAD_BIN" -c

echo "=== Phase 10: URDF export ==="
python3 10_export_urdf.py

echo "=== Phase 11: Inertia validation ==="
python3 11_validate_inertia.py

echo "=== Pipeline complete (Phase 8 requires MCP bridge -- see notes above) ==="
