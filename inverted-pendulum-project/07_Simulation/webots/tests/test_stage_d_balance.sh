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
# Control-off pass (#405): after the normal run passes, the same world is run
# again with CONTROLLER_DISABLED=1 (wheels commanded to 0 rad/s). The pitch
# check cannot tell the two apart (both pass), so the wheel drift is the
# switch check: the normal run must move the wheels (drift >= 0.002 rad/s, else
# the switch cannot be tested) and the control-off run must hold them
# (drift < 0.001 rad/s and < half the normal drift).
#
# Usage:
#   cd inverted-pendulum-project/07_Simulation/webots
#   bash tests/test_stage_d_balance.sh
#
# Exit code:
#   0 if the normal run passes and the control-off switch check passes
#   1 otherwise

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEBOTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONTROLLER_DIR="$WEBOTS_DIR/controllers/pendulum_controller"
CONTROLLER_LOG="$CONTROLLER_DIR/controller.log"
WORLD="$WEBOTS_DIR/worlds/pendulum_robot.wbt"

# Test parameters
THRESHOLD_RAD=0.15
# Wall-clock cap per Webots run. Known limitation (pre-existing, not fixed here):
# this outer timeout kills only run_batch.sh, so a Webots process can outlive it.
TIMEOUT_S=20
SIM_TIME_S=30         # simulated time per run (TEST_MAX_SIM_TIME_S)
MIN_LAST_T_S=25       # a run whose last log row is earlier than this is truncated
DRIFT_FROM_T=10       # drift window starts at first row with t >= this
# Normal-run floor: today's PID drift is 0.2086 rad/s and the expected drift after
# the setpoint fix (#408) is about 0.0034 rad/s, so 0.002 is below both.
NORMAL_DRIFT_FLOOR=0.002
# Disabled wheels hold exactly 0 rad/s; 0.001 leaves float margin. The control-off
# drift must also be under half the normal drift.
OFF_DRIFT_MAX=0.001

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

# Per-run metrics from a controller log. A data row is a line whose first field is
# a number (or nan/inf). Header and status lines are ignored. A data row is used only
# when fields 1, 3, 8 and 9 (time, pitch, WheelL, WheelR) are plain decimal numbers;
# any other data row is counted as malformed and makes the test FAIL.
#   max |pitch|, last logged time, drift_max (larger absolute WheelL/WheelR rate over
#   rows with t >= DRIFT_FROM_T), malformed row count, usable row count.
# Output: "max_pitch last_t drift_max bad_rows n_rows", or "NONE ..." with no usable rows.
run_metrics() {
    awk -v from_t="$DRIFT_FROM_T" '
      function isnum(s) { return s ~ /^[-+]?[0-9]+(\.[0-9]+)?([eE][-+]?[0-9]+)?$/ }
      $1 ~ /^[-+]?[0-9]/ || tolower($1) ~ /^[-+]?(nan|inf)$/ {
        if (!(isnum($1) && isnum($3) && isnum($8) && isnum($9))) { bad++; next }
        n++
        t = $1 + 0
        p = ($3 < 0) ? -$3 : $3
        if (!have_p || p > max_p) { max_p = p; have_p = 1 }
        last_t = t
        last_l = $8 + 0
        last_r = $9 + 0
        if (!have_w && t >= from_t) {
          have_w = 1
          w_t = t; w_l = $8 + 0; w_r = $9 + 0
        }
      }
      END {
        if (n == 0 || !have_w || last_t == w_t) { print "NONE NONE NONE " bad+0 " " n+0; exit }
        dl = (last_l - w_l) / (last_t - w_t)
        dr = (last_r - w_r) / (last_t - w_t)
        dm = (dl < 0 ? -dl : dl)
        if ((dr < 0 ? -dr : dr) > dm) dm = (dr < 0 ? -dr : dr)
        printf "%.4f %.3f %.5f %d %d\n", max_p, last_t, dm, bad+0, n
      }
    ' "$1"
}

# Succeeds when the last logged time $1 reaches MIN_LAST_T_S (guards truncated runs).
reached_min_t() {
    [ "$(awk -v t="$1" -v m="$MIN_LAST_T_S" 'BEGIN { print (t >= m) ? 1 : 0 }')" -eq 1 ]
}

# Sets PITCH_MAX, LAST_T, DRIFT_MAX from controller log $2 (label $1). Exits 1 when the
# log is unreadable, has no usable rows, or has any malformed data row.
load_stats() {
    local label="$1" file="$2" out
    out=$(run_metrics "$file") || { echo "FAIL: ${label} controller log unreadable: $file"; exit 1; }
    read -r PITCH_MAX LAST_T DRIFT_MAX BAD_ROWS N_ROWS <<< "$out" || true
    if [ -z "$PITCH_MAX" ] || [ -z "$LAST_T" ] || [ -z "$DRIFT_MAX" ] || [ -z "$BAD_ROWS" ] || [ -z "$N_ROWS" ] || [ "$PITCH_MAX" = NONE ]; then
        echo "FAIL: ${label} controller log has no usable data rows (drift window t >= ${DRIFT_FROM_T}s)"
        exit 1
    fi
    if [ "$BAD_ROWS" -ne 0 ]; then
        echo "FAIL: ${label} controller log has ${BAD_ROWS} malformed data row(s) (non-numeric or nan/inf field)"
        exit 1
    fi
}

load_stats normal "$CONTROLLER_LOG"
NORM_PITCH=$PITCH_MAX
NORM_T=$LAST_T
NORM_DRIFT=$DRIFT_MAX

echo "IMU data parsed from controller.log"
echo "Max |IMU_Pitch|: ${NORM_PITCH}rad"
echo "Last logged time: ${NORM_T}s"
echo "Threshold: ${THRESHOLD_RAD}rad"
echo ""

# Compare using floating point arithmetic
pass=$(awk -v tilt="$NORM_PITCH" -v threshold="$THRESHOLD_RAD" 'BEGIN { print (tilt <= threshold) ? 1 : 0 }')

if [ "$pass" -ne 1 ]; then
    echo "FAIL: Max |tilt| ${NORM_PITCH}rad exceeds ${THRESHOLD_RAD}rad threshold"
    echo ""
    echo "Last 20 lines of log:"
    tail -n 20 "$CONTROLLER_LOG"
    exit 1
fi

# The normal run must reach the full simulated time.
if ! reached_min_t "$NORM_T"; then
    echo "FAIL: Normal run truncated: last logged time ${NORM_T}s < ${MIN_LAST_T_S}s"
    echo "Last 20 lines of log:"
    tail -n 20 "$CONTROLLER_LOG"
    exit 1
fi

echo "PASS: Max |tilt| ${NORM_PITCH}rad <= ${THRESHOLD_RAD}rad threshold"
echo ""
echo "Robot remained balanced through t=${NORM_T}s (measured last logged time)."
echo ""

# ---------------------------------------------------------------------------
# Control-off pass (#405). Runs after the normal result above is final.
# ---------------------------------------------------------------------------

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

load_stats control-off "$CONTROLLER_LOG"
OFF_PITCH=$PITCH_MAX
OFF_T=$LAST_T
OFF_DRIFT=$DRIFT_MAX

echo ""
echo "Control-off comparison (drift window t >= ${DRIFT_FROM_T}s):"
echo "  run          max|pitch|(rad)  last_t(s)  drift_max(rad/s)"
echo "  normal       ${NORM_PITCH}           ${NORM_T}      ${NORM_DRIFT}"
echo "  control-off  ${OFF_PITCH}           ${OFF_T}      ${OFF_DRIFT}"
echo ""

# Floor: the normal run must move the wheels, or the switch cannot be tested.
if [ "$(awk -v d="$NORM_DRIFT" -v f="$NORMAL_DRIFT_FLOOR" 'BEGIN { print (d >= f) ? 1 : 0 }')" -ne 1 ]; then
    echo "FAIL: normal run's wheels did not move (drift ${NORM_DRIFT} rad/s < ${NORMAL_DRIFT_FLOOR}); the switch cannot be tested"
    exit 1
fi

# Switch check: disabled wheels must hold, i.e. drift < OFF_DRIFT_MAX and < half the normal drift.
switch_ok=$(awk -v o="$OFF_DRIFT" -v n="$NORM_DRIFT" -v lim="$OFF_DRIFT_MAX" 'BEGIN { print (o < lim && o < n / 2) ? 1 : 0 }')
if [ "$switch_ok" -ne 1 ]; then
    echo "FAIL: CONTROLLER_DISABLED=1 switch not effective: control-off drift ${OFF_DRIFT} rad/s (need < ${OFF_DRIFT_MAX} and < half of normal ${NORM_DRIFT})"
    exit 1
fi
echo "Switch check: control-off drift ${OFF_DRIFT} rad/s < ${OFF_DRIFT_MAX} and < half of normal ${NORM_DRIFT} rad/s (switch works)"

if ! reached_min_t "$OFF_T"; then
    echo "FAIL: control-off run truncated: last logged time ${OFF_T}s < ${MIN_LAST_T_S}s"
    exit 1
fi

# Informational: the pitch check does not separate the two runs.
if [ "$(awk -v tilt="$OFF_PITCH" -v threshold="$THRESHOLD_RAD" 'BEGIN { print (tilt <= threshold) ? 1 : 0 }')" -eq 1 ]; then
    echo "NOTE: pitch check is blind to the controller (control-off run passes too)"
fi

echo ""
echo "Stage D normal run: PASS (control-off switch check passed)."
exit 0
