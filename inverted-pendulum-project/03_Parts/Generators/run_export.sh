#!/bin/bash
# Wrapper script to run Phase 4 export with explicit output handling

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGFILE="$SCRIPT_DIR/export_run.log"

# Kill any existing FreeCAD processes
pkill -f "freecad --python 04_export" 2>/dev/null || true
sleep 2

# Run with explicit output redirection
cd "$SCRIPT_DIR"

echo "Starting Phase 4 export at $(date)" | tee "$LOGFILE"
echo "Script: $SCRIPT_DIR/04_export_assembly_merged.py" | tee -a "$LOGFILE"
echo "Working directory: $(pwd)" | tee -a "$LOGFILE"
echo "---" | tee -a "$LOGFILE"

# Run FreeCAD with the export script
xvfb-run -a freecad --python 04_export_assembly_merged.py 2>&1 | tee -a "$LOGFILE"

EXIT_CODE=$?

echo "---" | tee -a "$LOGFILE"
echo "Export finished with exit code: $EXIT_CODE" | tee -a "$LOGFILE"
echo "Finished at $(date)" | tee -a "$LOGFILE"

# Check for output files
echo "" | tee -a "$LOGFILE"
echo "Output files:" | tee -a "$LOGFILE"
ls -lh plates_assembled_with_servo.* export_metadata.json 2>&1 | tee -a "$LOGFILE"

exit $EXIT_CODE
