#!/bin/bash
# Stage E: LQR balance + compatibility validation harness — CI-runnable test.
#
# Part 1: Balance validation (mirrors test_stage_d_balance.sh's pattern).
# Runs the LQR controller against the undisturbed resting state and validates
# the IMU pitch angle stays within acceptable bounds, and that the run reached
# LAST_TIME_MIN_S of simulated time.
#
# Part 2: Compatibility validation. Asserts LQR's controller reads/commands
# the same core device set as PID's controller (imu, both wheel position
# sensors, both pivot sensors, both wheel motors) plus the Gyro added in E1,
# and that the LQR world file differs from the PID world file only in the
# controller field (no undocumented PROTO/world drift).
#
# Part 3: Informational control-off run (issue #406, follows #404 S2).
# Runs the same world again with CONTROLLER_DISABLED=1 (the LQR core still runs
# and logs, but both wheel commands are forced to 0.0 rad/s). The switch check
# asserts that it works: the max wheel drift over both wheels for t >= 10 s must
# be < 0.001 rad/s (and under half the normal drift when that is >= 0.002), and
# the run must reach LAST_TIME_MIN_S of simulated time before the wall-clock
# timeout. If the control-off run also passes the pitch check, a NOTE says the
# pitch check cannot tell control from no control.
#
# Data-row rule (both runs): a row whose first field is numeric is a data row.
# It is accepted only if fields 1, 3, 8 and 9 are plain finite-looking numbers
# and the row has at least 9 fields. Any other data-looking row is counted as
# malformed and FAILS the run (nan/inf, truncated last row, READING_ERROR lines).
# Header and info lines (first field non-numeric) are ignored.
#
# Requirements (from #221):
#   - Run LQR controller for 30 s of SIMULATED time on pendulum_robot_lqr.wbt
#     (TEST_MAX_SIM_TIME_S=30). The Webots process also has a 20 s WALL-CLOCK
#     timeout (TIMEOUT_S); the sim time reached within that window is what the
#     checks read from the log.
#   - Parse controller.log and extract IMU_Pitch column (3rd field).
#   - Assert max |tilt| <= 0.15rad. Independently justified from LQR's own
#     empirical logs (not copy-pasted from PID): LQR settles at the same
#     ~-0.1086rad physical resting tilt PID does (shared robot geometry/
#     asymmetry, not a PID-specific number), comfortable margin above it.
#   - Assert compatibility: same device set + only expected world diff.
#
# Usage:
#   cd inverted-pendulum-project/07_Simulation/webots
#   bash tests/test_stage_e_lqr_balance.sh
#
# Exit code:
#   0 if balance, compatibility and control-off switch checks PASS
#   1 if any of them FAILs

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEBOTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONTROLLER_DIR="$WEBOTS_DIR/controllers/lqr_controller"
CONTROLLER_LOG="$CONTROLLER_DIR/controller.log"
WORLD="$WEBOTS_DIR/worlds/pendulum_robot_lqr.wbt"
PID_WORLD="$WEBOTS_DIR/worlds/pendulum_robot.wbt"
PID_CONTROLLER="$WEBOTS_DIR/controllers/pendulum_controller/pendulum_controller.py"
LQR_CONTROLLER="$CONTROLLER_DIR/lqr_controller.py"

# Test parameters
THRESHOLD_RAD=0.15
TIMEOUT_S=20                 # WALL-CLOCK timeout (real s). Pre-existing limit: this kills run_batch.sh only; a webots-bin child can outlive it (Stage C too).
SIM_LIMIT_S=30               # SIMULATED-time limit passed to the controller (TEST_MAX_SIM_TIME_S)
DRIFT_WINDOW_START_S=10      # simulated time from which wheel drift is measured
LAST_TIME_MIN_S=25           # both runs must log up to at least this simulated time
# Control-off drift bounds (rad/s, max over BOTH wheels). With the switch on, both
# wheel commands are exactly 0.0, so the drift is float noise only; 0.001 leaves
# margin over that. The normal LQR run drifts ~0.0042 rad/s on this world, so a dead
# switch would sit above 0.001.
# Defence in depth: when the normal drift is >= 0.002 rad/s, the control-off drift
# must also be under half of it. At these constants that clause is implied by the
# absolute 0.001 bound (0.5 x 0.002 = 0.001), so it adds no separate constraint today.
DRIFT_MAX_RAD_S=0.001
DRIFT_NORMAL_MIN_RAD_S=0.002

echo "========================================"
echo "Stage E: LQR Balance + Compatibility Harness"
echo "========================================"
echo "Test: Verify robot stays balanced (max |tilt| <= ${THRESHOLD_RAD}rad)"
echo "Sim time limit: ${SIM_LIMIT_S}s (TEST_MAX_SIM_TIME_S); wall-clock timeout: ${TIMEOUT_S}s"
echo "World: $WORLD"
echo "Controller log: $CONTROLLER_LOG"
echo ""

# --- Part 2a: Compatibility check -- device set ---
echo "--- Compatibility check: device set ---"
for device in '"imu"' '"wheel_left_joint_sensor"' '"wheel_right_joint_sensor"' '"pendulum_pivot_joint_sensor"' '"pendulum_pivot_right_joint_sensor"' '"wheel_left_joint"' '"wheel_right_joint"'; do
    if ! grep -q -- "$device" "$PID_CONTROLLER"; then
        echo "FAIL: expected device $device not found in PID controller (test assumption broken, update this test)"
        exit 1
    fi
    if ! grep -q -- "$device" "$LQR_CONTROLLER"; then
        echo "FAIL: compatibility broken -- LQR controller missing device $device that PID uses"
        exit 1
    fi
done
if ! grep -q '"gyro"' "$LQR_CONTROLLER"; then
    echo "FAIL: LQR controller missing expected Gyro device"
    exit 1
fi
echo "PASS: LQR reads/commands the same devices as PID, plus Gyro"
echo ""

# --- Part 2b: Compatibility check -- world file diff ---
echo "--- Compatibility check: world file diff ---"
if [ ! -f "$PID_WORLD" ] || [ ! -f "$WORLD" ]; then
    echo "FAIL: world file(s) not found: $PID_WORLD or $WORLD"
    exit 1
fi
world_diff=$(diff "$PID_WORLD" "$WORLD" || true)
diff_line_count=$(echo "$world_diff" | grep -c '^[<>]' || true)
if [ "$diff_line_count" -ne 2 ]; then
    echo "FAIL: expected exactly 2 differing lines (controller field only) between $PID_WORLD and $WORLD, found $diff_line_count:"
    echo "$world_diff"
    exit 1
fi
if ! echo "$world_diff" | grep -q 'pendulum_controller'; then
    echo "FAIL: expected PID world's controller field \"pendulum_controller\" not found in diff:"
    echo "$world_diff"
    exit 1
fi
if ! echo "$world_diff" | grep -q 'lqr_controller'; then
    echo "FAIL: expected LQR world's controller field \"lqr_controller\" not found in diff:"
    echo "$world_diff"
    exit 1
fi
echo "PASS: world files identical except the controller field"
echo ""

# --- Helpers ---

# Run Webots once on $WORLD with CONTROLLER_DISABLED=$1 ("0" or "1").
# Sets RUN_RC. Exits 1 on a real Webots error (anything but 0 or 124).
run_webots_pass() {
    local disabled="$1"
    if [ ! -f "$WORLD" ]; then
        echo "FAIL: World file not found: $WORLD"
        exit 1
    fi
    if [ -f "$CONTROLLER_LOG" ]; then
        rm -f "$CONTROLLER_LOG"
    fi
    echo "Running Webots (batch, headless, CONTROLLER_DISABLED=${disabled})..."
    export TEST_MAX_SIM_TIME_S=$SIM_LIMIT_S
    export CONTROLLER_DISABLED="$disabled"
    set +e
    (cd "$WEBOTS_DIR" && timeout "${TIMEOUT_S}s" ./run_batch.sh "$WORLD" > /dev/null 2>&1)
    RUN_RC=$?
    set -e
    if [ "$RUN_RC" -ne 0 ] && [ "$RUN_RC" -ne 124 ]; then
        echo "FAIL: Webots exited with error code $RUN_RC (expected 0 or 124)"
        tail -n 20 "$CONTROLLER_LOG" 2>/dev/null || echo "  (no controller log found)"
        exit 1
    fi
    echo "Webots completed (exit code: $RUN_RC, timeout or normal completion)."
    echo ""
    sleep 1
    if [ ! -f "$CONTROLLER_LOG" ]; then
        echo "FAIL: Controller log not found: $CONTROLLER_LOG"
        exit 1
    fi
}

# awk over $CONTROLLER_LOG. Prints one line: "max drift last good bad".
#   max   = max |IMU_Pitch| over good rows ("NONE" if no good rows)
#   drift = larger |least-squares slope| of WheelL and WheelR over t >= W
#           ("NONE" if fewer than 2 good rows in that window)
#   last  = time of the last good row
#   good  = rows accepted; bad = data-looking rows rejected (see header rule)
log_stats() {
    awk -v W="$DRIFT_WINDOW_START_S" '
      function isnum(x) { return (x ~ /^-?[0-9]+(\.[0-9]+)?([eE][-+]?[0-9]+)?$/) }
      function absf(x) { return (x < 0) ? -x : x }
      $1 ~ /^[0-9]+\.?[0-9]*$/ {
        if (NF < 9 || !isnum($1) || !isnum($3) || !isnum($8) || !isnum($9)) { bad++; next }
        good++
        t = $1 + 0
        p = absf($3 + 0)
        if (good == 1 || p > maxp) maxp = p
        last = t
        if (t >= W) {
          k++; st += t; stt += t * t
          sl += $8; stl += t * $8
          sr += $9; str += t * $9
        }
      }
      END {
        if (good == 0) { printf "NONE NONE NONE 0 %d\n", bad; exit }
        printf "%.4f ", maxp
        den = k * stt - st * st
        if (k >= 2 && den != 0) {
          slope_l = (k * stl - st * sl) / den
          slope_r = (k * str - st * sr) / den
          drift = (absf(slope_l) > absf(slope_r)) ? absf(slope_l) : absf(slope_r)
          printf "%.5f ", drift
        } else {
          printf "NONE "
        }
        printf "%.3f %d %d\n", last, good, bad
      }
    ' "$CONTROLLER_LOG"
}

# Parse the current log into STAT_MAX STAT_DRIFT STAT_LAST STAT_GOOD STAT_BAD.
# Returns 1 if awk fails (e.g. unreadable log) or the output is not 5 fields.
parse_log() {
    local out
    out=$(log_stats) || return 1
    read -r STAT_MAX STAT_DRIFT STAT_LAST STAT_GOOD STAT_BAD <<< "$out" || return 1
    if [ -z "$STAT_MAX" ] || [ -z "$STAT_DRIFT" ] || [ -z "$STAT_LAST" ] || [ -z "$STAT_GOOD" ] || [ -z "$STAT_BAD" ]; then
        return 1
    fi
    return 0
}

# Hard failure if the log cannot be parsed (never report PASS on an empty parse).
parse_or_die() {
    if ! parse_log; then
        echo "FAIL: $1: could not parse $CONTROLLER_LOG (log_stats failed or log unreadable)"
        exit 1
    fi
}

# Print a stats value with a unit suffix, or "no data" when it is NONE.
show() {
    if [ "$1" = "NONE" ]; then
        echo "no data"
    else
        echo "$1$2"
    fi
}

# --- Part 1: Balance validation (normal controller) ---
run_webots_pass 0
parse_or_die "normal run"
normal_max=$STAT_MAX
normal_drift=$STAT_DRIFT
normal_last=$STAT_LAST
normal_good=$STAT_GOOD
normal_bad=$STAT_BAD

BALANCE_OK=1
if [ "$normal_bad" -gt 0 ]; then
    echo "FAIL: normal run: ${normal_bad} malformed or non-finite data row(s) in controller.log"
    BALANCE_OK=0
fi
if [ "$normal_good" -eq 0 ]; then
    echo "FAIL: No valid IMU data rows found in controller log"
    echo "Last 20 lines of log:"
    tail -n 20 "$CONTROLLER_LOG"
    BALANCE_OK=0
else
    echo "IMU data parsed from controller.log (${normal_good} rows, ${normal_bad} malformed)"
    echo "Max |IMU_Pitch|: ${normal_max}rad"
    echo "Threshold: ${THRESHOLD_RAD}rad"
    echo ""
    pass=$(awk -v tilt="$normal_max" -v threshold="$THRESHOLD_RAD" 'BEGIN { print (tilt <= threshold) ? 1 : 0 }')
    if [ "$pass" -eq 1 ]; then
        echo "PASS: Max |tilt| ${normal_max}rad <= ${THRESHOLD_RAD}rad threshold"
    else
        echo "FAIL: Max |tilt| ${normal_max}rad exceeds ${THRESHOLD_RAD}rad threshold"
        echo ""
        echo "Last 20 lines of log:"
        tail -n 20 "$CONTROLLER_LOG"
        BALANCE_OK=0
    fi
    if [ "$(awk -v t="$normal_last" -v m="$LAST_TIME_MIN_S" 'BEGIN { print (t >= m) ? 1 : 0 }')" -ne 1 ]; then
        echo "FAIL: normal run last log time ${normal_last}s < ${LAST_TIME_MIN_S}s (run did not reach the simulated time)"
        BALANCE_OK=0
    fi
fi
echo ""

# --- Part 3: Informational control-off run (CONTROLLER_DISABLED=1) ---
echo "--- Control-off run: CONTROLLER_DISABLED=1 ---"
run_webots_pass 1
parse_or_die "control-off run"
off_max=$STAT_MAX
off_drift=$STAT_DRIFT
off_last=$STAT_LAST
off_good=$STAT_GOOD
off_bad=$STAT_BAD
echo "Control-off run: max |IMU_Pitch| $(show "$off_max" rad), wheel drift $(show "$off_drift" ' rad/s') (t >= ${DRIFT_WINDOW_START_S}s), last log t $(show "$off_last" s)"
echo ""

SWITCH_OK=1
if [ "$off_bad" -gt 0 ]; then
    echo "FAIL: control-off run: ${off_bad} malformed or non-finite data row(s) in controller.log"
    SWITCH_OK=0
fi
if [ "$off_drift" = "NONE" ]; then
    echo "FAIL: control-off run has too few rows at t >= ${DRIFT_WINDOW_START_S}s to measure wheel drift"
    SWITCH_OK=0
elif [ "$(awk -v d="$off_drift" -v m="$DRIFT_MAX_RAD_S" 'BEGIN { print (d < m) ? 1 : 0 }')" -ne 1 ]; then
    echo "FAIL: control-off run wheel drift ${off_drift} rad/s >= ${DRIFT_MAX_RAD_S} rad/s (CONTROLLER_DISABLED did not stop both wheels)"
    SWITCH_OK=0
elif [ "$normal_drift" != "NONE" ] && [ "$(awk -v n="$normal_drift" -v nmin="$DRIFT_NORMAL_MIN_RAD_S" -v d="$off_drift" 'BEGIN { print (n >= nmin && d >= n / 2) ? 1 : 0 }')" -eq 1 ]; then
    echo "FAIL: control-off run wheel drift ${off_drift} rad/s is not under half the normal run's ${normal_drift} rad/s (CONTROLLER_DISABLED did not stop the wheels)"
    SWITCH_OK=0
fi
if [ "$off_last" = "NONE" ] || [ "$(awk -v t="$off_last" -v m="$LAST_TIME_MIN_S" 'BEGIN { print (t >= m) ? 1 : 0 }')" -ne 1 ]; then
    echo "FAIL: control-off run last log time ${off_last}s < ${LAST_TIME_MIN_S}s"
    SWITCH_OK=0
fi
if [ "$SWITCH_OK" -eq 1 ]; then
    echo "PASS: control-off switch works (drift ${off_drift} rad/s < ${DRIFT_MAX_RAD_S} rad/s, last t ${off_last}s >= ${LAST_TIME_MIN_S}s)"
fi

off_pitch_pass=$(awk -v tilt="$off_max" -v threshold="$THRESHOLD_RAD" 'BEGIN { print (tilt != "NONE" && tilt <= threshold) ? 1 : 0 }')
if [ "$off_pitch_pass" -eq 1 ]; then
    echo "NOTE: pitch check is blind to the controller (control-off run passes too)"
fi
echo ""

# --- Summary ---
echo "--- Summary (normal vs control-off, simulated time) ---"
printf "%-12s %-14s %-22s %-12s\n" "run" "max|pitch|" "drift(t>=${DRIFT_WINDOW_START_S}s)" "last t"
printf "%-12s %-14s %-22s %-12s\n" "normal" "$(show "$normal_max" rad)" "$(show "$normal_drift" ' rad/s')" "$(show "$normal_last" s)"
printf "%-12s %-14s %-22s %-12s\n" "disabled" "$(show "$off_max" rad)" "$(show "$off_drift" ' rad/s')" "$(show "$off_last" s)"
echo ""

if [ "$BALANCE_OK" -ne 1 ]; then
    echo "FAIL: Balance check failed (see above)"
    echo ""
    echo "========================================"
    echo "CHECKS FAILED (balance)"
    echo "========================================"
    exit 1
fi

if [ "$SWITCH_OK" -ne 1 ]; then
    echo "========================================"
    echo "CHECKS FAILED (control-off switch)"
    echo "========================================"
    exit 1
fi

echo "Robot remained balanced for the measured ${normal_last}s of simulated time (wall-clock timeout ${TIMEOUT_S}s)."
echo ""
echo "========================================"
echo "ALL CHECKS PASSED (balance + compatibility + control-off switch)"
echo "========================================"
exit 0
