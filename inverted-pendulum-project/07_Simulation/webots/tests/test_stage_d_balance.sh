#!/bin/bash
# Stage D: Balance validation harness — CI-runnable test.
#
# Runs the pendulum controller against the undisturbed resting state
# (robot starts upright, no initial rotation offset) and validates that
# the IMU pitch angle stays within acceptable bounds, confirming the
# pendulum remains balanced and the control loop functions correctly.
#
# Requirements (from #210):
#   - Run Stage D controller ~20 seconds on default pendulum_robot.wbt
#   - Parse controller.log and extract IMU_Pitch column (3rd field)
#   - Compute maximum absolute tilt angle across all logged cycles
#   - Assert max |tilt| <= 0.15rad (comfortable margin above true resting
#     equilibrium of ~-0.1086rad, catches regressions like PID sign flips)
#   - Fail test if tilt exceeds threshold
#
# Usage:
#   cd inverted-pendulum-project/07_Simulation/webots
#   bash tests/test_stage_d_balance.sh
#
# Exit code:
#   0 if max |tilt| <= 0.15rad and test PASS
#   1 if max |tilt| > 0.15rad or test FAIL

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEBOTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONTROLLER_DIR="$WEBOTS_DIR/controllers/pendulum_controller"
CONTROLLER_LOG="$CONTROLLER_DIR/controller.log"
WORLD="$WEBOTS_DIR/worlds/pendulum_robot.wbt"

# Test parameters
THRESHOLD_RAD=0.15
TIMEOUT_S=20

echo "========================================"
echo "Stage D: Balance Validation Harness"
echo "========================================"
echo "Test: Verify robot stays balanced (max |tilt| <= ${THRESHOLD_RAD}rad)"
echo "Timeout: ${TIMEOUT_S}s"
echo "World: $WORLD"
echo "Controller log: $CONTROLLER_LOG"
echo ""

# Validate world exists
if [ ! -f "$WORLD" ]; then
    echo "FAIL: World file not found: $WORLD"
    exit 1
fi

# Clear old controller log
if [ -f "$CONTROLLER_LOG" ]; then
    rm -f "$CONTROLLER_LOG"
fi

# Run the simulation (batch mode, headless)
echo "Running Webots (batch, headless)..."

# Run with timeout. Webots will exit 0 (success), 124 (timeout), or non-zero (error).
# We allow both 0 and 124 to be acceptable; 124 means it ran the full 20s.
# Any other non-zero code is a real failure.
export TEST_MAX_SIM_TIME_S=30
set +e
(cd "$WEBOTS_DIR" && timeout "${TIMEOUT_S}s" ./run_batch.sh "$WORLD" > /dev/null 2>&1)
rc=$?
set -e

if [ "$rc" -ne 0 ] && [ "$rc" -ne 124 ]; then
    echo "FAIL: Webots exited with error code $rc (expected 0 or 124)"
    tail -n 20 "$CONTROLLER_LOG" 2>/dev/null || echo "  (no controller log found)"
    exit 1
fi

echo "Webots completed (exit code: $rc, timeout or normal completion)."
echo ""

# Wait for controller log to be flushed (safety margin)
sleep 1

# Validate controller log exists
if [ ! -f "$CONTROLLER_LOG" ]; then
    echo "FAIL: Controller log not found: $CONTROLLER_LOG"
    exit 1
fi

# Parse IMU_Pitch column (3rd field) from data lines in controller.log.
# Data lines start with a numeric timestamp; skip header/status lines.
# Compute maximum absolute tilt angle across all cycles.

# Use awk to:
#   1. Filter lines where first field is numeric (matches timestamp pattern)
#   2. Extract IMU_Pitch (3rd field) from each data line
#   3. Track maximum absolute value across all samples
max_tilt=$(awk '
  $1 ~ /^[0-9]+\.?[0-9]*$/ {
    pitch = $3
    abs_pitch = (pitch < 0) ? -pitch : pitch
    if (abs_pitch > max) {
      max = abs_pitch
    }
  }
  END {
    if (max == "") {
      # No valid data lines found
      print "NONE"
    } else {
      printf "%.4f\n", max
    }
  }
' "$CONTROLLER_LOG")

if [ "$max_tilt" = "NONE" ]; then
    echo "FAIL: No valid IMU data rows found in controller log"
    echo "Last 20 lines of log:"
    tail -n 20 "$CONTROLLER_LOG"
    exit 1
fi

echo "IMU data parsed from controller.log"
echo "Max |IMU_Pitch|: ${max_tilt}rad"
echo "Threshold: ${THRESHOLD_RAD}rad"
echo ""

# Compare using floating point arithmetic
pass=$(awk -v tilt="$max_tilt" -v threshold="$THRESHOLD_RAD" 'BEGIN { print (tilt <= threshold) ? 1 : 0 }')

if [ "$pass" -eq 1 ]; then
    echo "PASS: Max |tilt| ${max_tilt}rad <= ${THRESHOLD_RAD}rad threshold"
    echo ""
    echo "Robot remained balanced throughout the ${TIMEOUT_S}s run."
    exit 0
else
    echo "FAIL: Max |tilt| ${max_tilt}rad exceeds ${THRESHOLD_RAD}rad threshold"
    echo ""
    echo "Last 20 lines of log:"
    tail -n 20 "$CONTROLLER_LOG"
    exit 1
fi
