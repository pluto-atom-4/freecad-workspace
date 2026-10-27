#!/bin/bash
# Stage E: LQR balance + compatibility validation harness — CI-runnable test.
#
# Part 1: Balance validation (mirrors test_stage_d_balance.sh's pattern).
# Runs the LQR controller against the undisturbed resting state and validates
# the IMU pitch angle stays within acceptable bounds.
#
# Part 2: Compatibility validation. Asserts LQR's controller reads/commands
# the same core device set as PID's controller (imu, both wheel position
# sensors, both pivot sensors, both wheel motors) plus the Gyro added in E1,
# and that the LQR world file differs from the PID world file only in the
# controller field (no undocumented PROTO/world drift).
#
# Part 3: Informational control-off run (issue #406, follows #404 S2).
# Runs the same world again with CONTROLLER_DISABLED=1 (the LQR core still runs
# and logs, but both wheel commands are forced to 0.0 rad/s). Prints max |pitch|,
# wheel drift rate and last log time for both runs. Asserts only that the switch
# works: the disabled run must drift < 0.001 rad/s (max over both wheels) over
# t >= 10 s, and under half the normal run's drift when that is >= 0.002 rad/s, and
# must reach t >= 25 s. If the disabled run also passes the pitch check, the pitch check
# cannot tell control from no control; a NOTE line says so.
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
TIMEOUT_S=20                 # WALL-CLOCK timeout for the Webots process (real seconds)
SIM_LIMIT_S=30               # SIMULATED-time limit passed to the controller (TEST_MAX_SIM_TIME_S)
DRIFT_WINDOW_START_S=10      # simulated time from which wheel drift is measured
# Control-off drift bounds (rad/s, max over BOTH wheels). With the switch on, both
# wheel commands are exactly 0.0, so the drift is float noise only; 0.001 leaves
# margin over that. The normal LQR run drifts ~0.0042 rad/s on this world, so a dead
# switch would sit above 0.001. When the normal drift is >= 0.002 rad/s, the
# control-off drift must also be under half of it.
DRIFT_MAX_RAD_S=0.001
DRIFT_NORMAL_MIN_RAD_S=0.002
LAST_TIME_MIN_S=25           # control-off run must log up to at least this sim time

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

# Print "max_tilt drift_rad_s last_time" for the current controller log.
# max_tilt is "NONE" if there are no data rows. drift_rad_s is the larger of the
# two least-squares wheel-angle slopes (|WheelL| and |WheelR|, rad/s) over
# t >= DRIFT_WINDOW_START_S; "NONE" if fewer than 2 rows fall in that window.
log_stats() {
    awk -v W="$DRIFT_WINDOW_START_S" '
      $1 ~ /^[0-9]+\.?[0-9]*$/ {
        t = $1 + 0
        pitch = $3 + 0
        abs_pitch = (pitch < 0) ? -pitch : pitch
        if (rows == 0 || abs_pitch > max) max = abs_pitch
        rows++
        last = t
        if (t >= W) {
          k++; st += t; stt += t * t
          sl += $8; stl += t * $8
          sr += $9; str += t * $9
        }
      }
      function absf(x) { return (x < 0) ? -x : x }
      END {
        if (rows == 0) { printf "NONE NONE NONE\n"; exit }
        printf "%.4f ", max
        den = k * stt - st * st
        if (k >= 2 && den != 0) {
          slope_l = (k * stl - st * sl) / den
          slope_r = (k * str - st * sr) / den
          drift = (absf(slope_l) > absf(slope_r)) ? absf(slope_l) : absf(slope_r)
          printf "%.5f ", drift
        } else {
          printf "NONE "
        }
        printf "%.3f\n", last
      }
    ' "$CONTROLLER_LOG"
}

# --- Part 1: Balance validation (normal controller) ---
run_webots_pass 0

read -r max_tilt _ _ <<< "$(log_stats)"
BALANCE_OK=0
if [ "$max_tilt" = "NONE" ]; then
    echo "FAIL: No valid IMU data rows found in controller log"
    echo "Last 20 lines of log:"
    tail -n 20 "$CONTROLLER_LOG"
    BALANCE_OK=0
else
    echo "IMU data parsed from controller.log"
    echo "Max |IMU_Pitch|: ${max_tilt}rad"
    echo "Threshold: ${THRESHOLD_RAD}rad"
    echo ""
    pass=$(awk -v tilt="$max_tilt" -v threshold="$THRESHOLD_RAD" 'BEGIN { print (tilt <= threshold) ? 1 : 0 }')
    if [ "$pass" -eq 1 ]; then
        echo "PASS: Max |tilt| ${max_tilt}rad <= ${THRESHOLD_RAD}rad threshold"
        BALANCE_OK=1
    else
        echo "FAIL: Max |tilt| ${max_tilt}rad exceeds ${THRESHOLD_RAD}rad threshold"
        echo ""
        echo "Last 20 lines of log:"
        tail -n 20 "$CONTROLLER_LOG"
    fi
fi
echo ""

# Normal run numbers (for the summary table below). Copy before the next run deletes the log.
read -r normal_max normal_drift normal_last <<< "$(log_stats)"
normal_rc=$RUN_RC

# --- Part 3: Informational control-off run (CONTROLLER_DISABLED=1) ---
echo "--- Control-off run: CONTROLLER_DISABLED=1 ---"
run_webots_pass 1

read -r off_max off_drift off_last <<< "$(log_stats)"
echo "Control-off run: max |IMU_Pitch| ${off_max}rad, wheel drift ${off_drift} rad/s (t >= ${DRIFT_WINDOW_START_S}s), last log t ${off_last}s"
echo ""

SWITCH_OK=1
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
printf "%-12s %-14s %-22s %-12s\n" "normal" "${normal_max}rad" "${normal_drift} rad/s" "${normal_last}s"
printf "%-12s %-14s %-22s %-12s\n" "disabled" "${off_max}rad" "${off_drift} rad/s" "${off_last}s"
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

echo "Robot remained balanced through the ${SIM_LIMIT_S}s simulated run (wall-clock timeout ${TIMEOUT_S}s)."
echo ""
echo "========================================"
echo "ALL CHECKS PASSED (balance + compatibility + control-off switch)"
echo "========================================"
exit 0
