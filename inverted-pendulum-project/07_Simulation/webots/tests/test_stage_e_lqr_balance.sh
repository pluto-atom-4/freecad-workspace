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
# Log rules (both runs, one awk parser, log_stats): the header line "Time(s) ..."
# sets the column count. A data-looking row (first field starts with a digit, sign
# or dot, or is nan/inf) is accepted only if it has exactly that many fields, a
# plain time, finite-looking pitch and wheel values, and a time strictly greater
# than the previous row. Anything else is malformed and FAILs the run. Lines with
# CR characters are malformed. READING_ERROR rows (the controller could not read a
# sensor) are reported separately and FAIL the run. Other info lines are ignored.
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
# Rounding: the pitch and drift checks compare the printed values (pitch %.4f, error
# up to 5e-5 rad; drift %.5f, error up to 5e-6 rad/s).
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
# The 1 s sleep before parsing is pre-existing; it is not a proven fix for a log
# that is still being flushed when Webots is killed (not reproduced).
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

# The single log parser, used for both runs. Prints one line:
#   "max drift last good bad rerr first_bad_line first_READING_ERROR_line"
#   max   = max |IMU_Pitch| over accepted rows ("NONE" if none)
#   drift = larger |least-squares slope| of WheelL and WheelR over t >= W
#           ("NONE" if fewer than 2 accepted rows in that window)
#   last  = time of the last accepted row (the largest, since times must increase)
#   good / bad / rerr = accepted rows / malformed rows / READING_ERROR rows
#   line numbers are 0 when there is none
log_stats() {
    awk -v W="$DRIFT_WINDOW_START_S" '
      function num(x)      { return (x ~ /^[-+]?[0-9]+(\.[0-9]*)?([eE][-+]?[0-9]+)?$/) }
      function tnum(x)     { return (x ~ /^[0-9]+(\.[0-9]+)?$/) }
      function datalike(f) { return (f ~ /^[-+.0-9]/ || tolower(f) ~ /^[-+]?(nan|inf)/) }
      function absf(x)     { return (x < 0) ? -x : x }
      index($0, "\r") > 0  { bad++; if (bad == 1) fb = NR; next }
      index($0, "READING_ERROR") > 0 { rerr++; if (rerr == 1) fr = NR; next }
      $1 == "Time(s)"      { ncol = NF; next }
      !datalike($1)        { next }
      {
        if (ncol == 0 || NF != ncol || !tnum($1) || !num($3) || !num($8) || !num($9) || (good > 0 && $1 + 0 <= prevt)) {
          bad++; if (bad == 1) fb = NR; next
        }
        good++
        t = $1 + 0; prevt = t
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
        if (good == 0) { printf "NONE NONE NONE 0 %d %d %d %d\n", bad+0, rerr+0, fb+0, fr+0; exit }
        printf "%.4f ", maxp
        den = k * stt - st * st
        if (k >= 2 && den != 0) {
          sL = (k * stl - st * sl) / den
          sR = (k * str - st * sr) / den
          printf "%.5f ", (absf(sL) > absf(sR)) ? absf(sL) : absf(sR)
        } else {
          printf "NONE "
        }
        printf "%.3f %d %d %d %d %d\n", last, good, bad+0, rerr+0, fb+0, fr+0
      }
    ' "$CONTROLLER_LOG"
}

# Parse the current log into STAT_* variables.
# Returns 1 if awk fails (e.g. unreadable log) or the output is not 8 fields.
parse_log() {
    local out
    out=$(log_stats) || return 1
    read -r STAT_MAX STAT_DRIFT STAT_LAST STAT_GOOD STAT_BAD STAT_RERR STAT_FB STAT_FR <<< "$out" || return 1
    if [ -z "$STAT_MAX" ] || [ -z "$STAT_DRIFT" ] || [ -z "$STAT_LAST" ] || [ -z "$STAT_GOOD" ] \
        || [ -z "$STAT_BAD" ] || [ -z "$STAT_RERR" ] || [ -z "$STAT_FB" ] || [ -z "$STAT_FR" ]; then
        return 1
    fi
    return 0
}

# Hard failure if the log cannot be parsed (never report PASS on an empty parse).
parse_or_die() {
    if ! parse_log; then
        echo "FAIL: $1: could not parse $CONTROLLER_LOG (log_stats failed or log unreadable)"
        tail -n 20 "$CONTROLLER_LOG" 2>/dev/null || echo "  (log unreadable)"
        exit 1
    fi
}

# Print the malformed-row diagnostics for one run.
# $1 label, $2 malformed count, $3 READING_ERROR count, $4 first malformed line, $5 first READING_ERROR line.
row_diagnostics() {
    if [ "$3" -gt 0 ]; then
        echo "FAIL: $1: ${3} sensor READING_ERROR row(s) in the controller log: the controller could not read a sensor"
        echo "  first READING_ERROR at line $5: $(sed -n "$5p" "$CONTROLLER_LOG")"
    fi
    if [ "$2" -gt 0 ]; then
        echo "FAIL: $1: ${2} malformed or non-finite data row(s) in controller.log"
        echo "  first malformed row at line $4: $(sed -n "$4p" "$CONTROLLER_LOG")"
    fi
}

# Print the last 20 log lines at a FAIL point.
show_tail() {
    echo "Last 20 lines of log:"
    tail -n 20 "$CONTROLLER_LOG" 2>/dev/null || echo "  (log unreadable)"
}

# Numeric comparison, fail-closed: prints nothing, returns 0 if "A OP B" is true,
# 1 if false. An awk failure or an unexpected result is a test failure (exit 1).
num_test() {
    local r
    r=$(awk -v a="$1" -v b="$3" "BEGIN { print (a $2 b) ? 1 : 0 }") || { echo "FAIL: awk could not evaluate '$1 $2 $3'"; exit 1; }
    case "$r" in
        1) return 0 ;;
        0) return 1 ;;
        *) echo "FAIL: comparison '$1 $2 $3' gave '$r'"; exit 1 ;;
    esac
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
normal_rerr=$STAT_RERR
normal_fb=$STAT_FB
normal_fr=$STAT_FR

BALANCE_OK=1
if [ "$normal_bad" -gt 0 ] || [ "$normal_rerr" -gt 0 ]; then
    row_diagnostics "normal run" "$normal_bad" "$normal_rerr" "$normal_fb" "$normal_fr"
    show_tail
    BALANCE_OK=0
fi
if [ "$normal_good" -eq 0 ]; then
    echo "FAIL: no usable data rows in controller log (malformed=${normal_bad}, READING_ERROR=${normal_rerr})"
    [ "$normal_bad" -gt 0 ] || [ "$normal_rerr" -gt 0 ] || show_tail
    BALANCE_OK=0
else
    echo "IMU data parsed from controller.log (${normal_good} rows, ${normal_bad} malformed, ${normal_rerr} READING_ERROR)"
    echo "Max |IMU_Pitch|: ${normal_max}rad"
    echo "Threshold: ${THRESHOLD_RAD}rad"
    echo ""
    if num_test "$normal_max" "<=" "$THRESHOLD_RAD"; then
        echo "PASS: Max |tilt| ${normal_max}rad <= ${THRESHOLD_RAD}rad threshold"
    else
        echo "FAIL: Max |tilt| ${normal_max}rad exceeds ${THRESHOLD_RAD}rad threshold"
        show_tail
        BALANCE_OK=0
    fi
    if ! num_test "$normal_last" ">=" "$LAST_TIME_MIN_S"; then
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
off_rerr=$STAT_RERR
off_fb=$STAT_FB
off_fr=$STAT_FR
echo "Control-off run: max |IMU_Pitch| $(show "$off_max" rad), wheel drift $(show "$off_drift" ' rad/s') (t >= ${DRIFT_WINDOW_START_S}s), last log t $(show "$off_last" s)"
echo ""

SWITCH_OK=1
if [ "$off_bad" -gt 0 ] || [ "$off_rerr" -gt 0 ]; then
    row_diagnostics "control-off run" "$off_bad" "$off_rerr" "$off_fb" "$off_fr"
    show_tail
    SWITCH_OK=0
fi
if [ "$off_good" -eq 0 ]; then
    echo "FAIL: control-off run: no usable data rows (malformed=${off_bad}, READING_ERROR=${off_rerr})"
    [ "$off_bad" -gt 0 ] || [ "$off_rerr" -gt 0 ] || show_tail
    SWITCH_OK=0
elif [ "$off_drift" = "NONE" ]; then
    echo "FAIL: control-off run has too few rows at t >= ${DRIFT_WINDOW_START_S}s to measure wheel drift"
    SWITCH_OK=0
elif ! num_test "$off_drift" "<" "$DRIFT_MAX_RAD_S"; then
    echo "FAIL: control-off run wheel drift ${off_drift} rad/s >= ${DRIFT_MAX_RAD_S} rad/s (CONTROLLER_DISABLED did not stop both wheels)"
    SWITCH_OK=0
elif [ "$normal_drift" != "NONE" ] && num_test "$normal_drift" ">=" "$DRIFT_NORMAL_MIN_RAD_S" \
    && num_test "$off_drift" ">=" "$(awk -v n="$normal_drift" 'BEGIN { printf "%.8f", n / 2 }')"; then
    echo "FAIL: control-off run wheel drift ${off_drift} rad/s is not under half the normal run's ${normal_drift} rad/s (CONTROLLER_DISABLED did not stop the wheels)"
    SWITCH_OK=0
fi
if [ "$off_good" -gt 0 ] && ! num_test "$off_last" ">=" "$LAST_TIME_MIN_S"; then
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
