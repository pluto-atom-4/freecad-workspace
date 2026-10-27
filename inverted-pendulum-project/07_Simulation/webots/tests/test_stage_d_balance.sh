#!/bin/bash
# Stage D: Balance validation harness — CI-runnable test.
#
# Runs the pendulum controller against the undisturbed resting state
# (robot starts upright, no initial rotation offset) and validates that
# the IMU pitch angle stays within acceptable bounds, confirming the
# pendulum remains balanced and the control loop functions correctly.
#
# Requirements (from #210):
#   - Run Stage D controller for 30 s of simulated time (TEST_MAX_SIM_TIME_S=30)
#     on default pendulum_robot.wbt; the 20 s wall-clock timeout is a safety cap
#   - Parse controller.log and extract IMU_Pitch column (3rd field)
#   - Compute maximum absolute tilt angle across all logged cycles
#   - Assert max |tilt| <= 0.15rad (comfortable margin above true resting
#     equilibrium of ~-0.1086rad, catches regressions like PID sign flips)
#   - Fail test if tilt exceeds threshold
#
# Informational control-off pass (#405): after the normal run passes, the same
# world is run again with CONTROLLER_DISABLED=1 (wheels commanded to 0 rad/s).
# Max |pitch|, last log time and wheel drift (t >= 10 s) are printed for both
# runs. The control-off run does NOT change the exit status, except when the
# switch itself is broken (wheels still drift, or the run is truncated/crashes).
#
# Usage:
#   cd inverted-pendulum-project/07_Simulation/webots
#   bash tests/test_stage_d_balance.sh
#
# Exit code:
#   0 if max |tilt| <= 0.15rad and test PASS (and the control-off switch works)
#   1 if max |tilt| > 0.15rad or test FAIL

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEBOTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONTROLLER_DIR="$WEBOTS_DIR/controllers/pendulum_controller"
CONTROLLER_LOG="$CONTROLLER_DIR/controller.log"
WORLD="$WEBOTS_DIR/worlds/pendulum_robot.wbt"
TMP_DIR="$(mktemp -d)"
NORMAL_LOG_COPY="$TMP_DIR/normal_controller.log"
trap 'rm -rf "$TMP_DIR"' EXIT

# Test parameters
THRESHOLD_RAD=0.15
TIMEOUT_S=20          # wall-clock safety cap per Webots run
SIM_TIME_S=30         # simulated time per run (TEST_MAX_SIM_TIME_S)
MIN_LAST_T_S=25       # a run whose last log row is earlier than this is truncated
DRIFT_LIMIT=0.01     # rad/s; control-off wheel drift must stay below this
DRIFT_FROM_T=10       # drift window starts at first row with t >= this

echo "========================================"
echo "Stage D: Balance Validation Harness"
echo "========================================"
echo "Test: Verify robot stays balanced (max |tilt| <= ${THRESHOLD_RAD}rad)"
echo "Sim time: ${SIM_TIME_S}s (TEST_MAX_SIM_TIME_S); wall-clock cap: ${TIMEOUT_S}s"
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
# We allow both 0 and 124 to be acceptable; 124 means the wall-clock cap was hit.
# Any other non-zero code is a real failure.
# CONTROLLER_DISABLED=0 is passed explicitly so an exported value cannot disable the normal run.
export TEST_MAX_SIM_TIME_S=$SIM_TIME_S
set +e
(cd "$WEBOTS_DIR" && CONTROLLER_DISABLED=0 timeout "${TIMEOUT_S}s" ./run_batch.sh "$WORLD" > /dev/null 2>&1)
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

# Last logged simulation time (first field of the last data row).
last_t=$(awk '$1 ~ /^[0-9]+\.?[0-9]*$/ { t = $1 } END { if (t == "") print "NONE"; else printf "%.3f\n", t }' "$CONTROLLER_LOG")

echo "IMU data parsed from controller.log"
echo "Max |IMU_Pitch|: ${max_tilt}rad"
echo "Last logged time: ${last_t}s"
echo "Threshold: ${THRESHOLD_RAD}rad"
echo ""

# Compare using floating point arithmetic
pass=$(awk -v tilt="$max_tilt" -v threshold="$THRESHOLD_RAD" 'BEGIN { print (tilt <= threshold) ? 1 : 0 }')

if [ "$pass" -ne 1 ]; then
    echo "FAIL: Max |tilt| ${max_tilt}rad exceeds ${THRESHOLD_RAD}rad threshold"
    echo ""
    echo "Last 20 lines of log:"
    tail -n 20 "$CONTROLLER_LOG"
    exit 1
fi

# Truncation guard: the normal run must reach the full simulated time.
if [ "$last_t" = "NONE" ] || [ "$(awk -v t="$last_t" -v m="$MIN_LAST_T_S" 'BEGIN { print (t >= m) ? 1 : 0 }')" -ne 1 ]; then
    echo "FAIL: Normal run truncated: last logged time ${last_t}s < ${MIN_LAST_T_S}s"
    echo "Last 20 lines of log:"
    tail -n 20 "$CONTROLLER_LOG"
    exit 1
fi

echo "PASS: Max |tilt| ${max_tilt}rad <= ${THRESHOLD_RAD}rad threshold"
echo ""
echo "Robot remained balanced throughout the ${SIM_TIME_S}s simulated run."
echo ""

# ---------------------------------------------------------------------------
# Informational control-off pass (#405). Does not affect the exit status above.
# ---------------------------------------------------------------------------

# Keep the normal run's log: the control-off run overwrites controller.log.
cp "$CONTROLLER_LOG" "$NORMAL_LOG_COPY"

# Print per-run metrics from a controller log:
#   max |pitch|, last logged time, WheelL drift rate, WheelR drift rate (rad/s),
#   and the max absolute drift of the two wheels. Drift uses rows with t >= DRIFT_FROM_T.
# Output: "max_pitch last_t drift_L drift_R drift_max" or "NONE" if no usable rows.
run_metrics() {
    awk -v from_t="$DRIFT_FROM_T" '
      $1 ~ /^[0-9]+\.?[0-9]*$/ {
        n++
        t = $1 + 0
        p = ($3 < 0) ? -$3 : $3
        if (p > max_p) max_p = p
        last_t = t
        last_l = $8 + 0
        last_r = $9 + 0
        if (!have_w && t >= from_t) {
          have_w = 1
          w_t = t; w_l = $8 + 0; w_r = $9 + 0
        }
      }
      END {
        if (n == 0 || !have_w || last_t == w_t) { print "NONE"; exit }
        dl = (last_l - w_l) / (last_t - w_t)
        dr = (last_r - w_r) / (last_t - w_t)
        dm = (dl < 0 ? -dl : dl)
        if ((dr < 0 ? -dr : dr) > dm) dm = (dr < 0 ? -dr : dr)
        printf "%.4f %.3f %.5f %.5f %.5f\n", max_p, last_t, dl, dr, dm
      }
    ' "$1"
}

normal_metrics=$(run_metrics "$NORMAL_LOG_COPY")

# Control-off run: same command, CONTROLLER_DISABLED=1.
rm -f "$CONTROLLER_LOG"
echo "Running Webots control-off pass (CONTROLLER_DISABLED=1)..."
set +e
(cd "$WEBOTS_DIR" && CONTROLLER_DISABLED=1 timeout "${TIMEOUT_S}s" ./run_batch.sh "$WORLD" > /dev/null 2>&1)
off_rc=$?
set -e
sleep 1

if [ "$off_rc" -ne 0 ] && [ "$off_rc" -ne 124 ]; then
    echo "FAIL: control-off Webots run exited with error code $off_rc (expected 0 or 124)"
    exit 1
fi
if [ ! -f "$CONTROLLER_LOG" ]; then
    echo "FAIL: control-off run produced no controller log: $CONTROLLER_LOG"
    exit 1
fi

off_metrics=$(run_metrics "$CONTROLLER_LOG")
cp "$CONTROLLER_LOG" "$TMP_DIR/off_controller.log"

echo ""
echo "Control-off comparison (t >= ${DRIFT_FROM_T}s drift window):"
echo "  run          max|pitch|(rad)  last_t(s)  driftL(rad/s)  driftR(rad/s)  drift_max(rad/s)"
if [ "$normal_metrics" = "NONE" ]; then
    echo "  normal       NONE (no usable rows)"
else
    read -r n_p n_t n_dl n_dr n_dm <<< "$normal_metrics"
    echo "  normal       ${n_p}           ${n_t}      ${n_dl}       ${n_dr}       ${n_dm}"
fi
if [ "$off_metrics" = "NONE" ]; then
    echo "  control-off  NONE (no usable rows)"
    echo "FAIL: control-off run has no usable data rows (switch check cannot be evaluated)"
    exit 1
fi
read -r o_p o_t o_dl o_dr o_dm <<< "$off_metrics"
echo "  control-off  ${o_p}           ${o_t}      ${o_dl}       ${o_dr}       ${o_dm}"
echo ""

# Switch check: with wheels commanded to 0, the wheel angles must not drift.
switch_ok=$(awk -v d="$o_dm" -v lim="$DRIFT_LIMIT" 'BEGIN { a = (d < 0 ? -d : d); print (a < lim) ? 1 : 0 }')
if [ "$switch_ok" -ne 1 ]; then
    echo "FAIL: CONTROLLER_DISABLED=1 switch not effective: wheel drift ${o_dm} rad/s >= ${DRIFT_LIMIT} rad/s"
    exit 1
fi
echo "Switch check: control-off wheel drift ${o_dm} rad/s < ${DRIFT_LIMIT} rad/s (switch works)"

# Truncation guard for the control-off run.
if [ "$(awk -v t="$o_t" -v m="$MIN_LAST_T_S" 'BEGIN { print (t >= m) ? 1 : 0 }')" -ne 1 ]; then
    echo "FAIL: control-off run truncated: last logged time ${o_t}s < ${MIN_LAST_T_S}s"
    exit 1
fi

# Informational: does the pitch check also pass with the controller off?
off_pass=$(awk -v tilt="$o_p" -v threshold="$THRESHOLD_RAD" 'BEGIN { print (tilt <= threshold) ? 1 : 0 }')
if [ "$off_pass" -eq 1 ]; then
    echo "NOTE: pitch check is blind to the controller (control-off run passes too)"
fi

echo ""
echo "Stage D normal run: PASS (control-off pass is informational)."
exit 0
