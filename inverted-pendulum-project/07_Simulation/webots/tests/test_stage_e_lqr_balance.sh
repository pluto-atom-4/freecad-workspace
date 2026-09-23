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
# Requirements (from #221):
#   - Run LQR controller ~20 seconds on pendulum_robot_lqr.wbt (sim-time-bound
#     via TEST_MAX_SIM_TIME_S, not a wall-clock-only timeout -- see #219/D5's
#     floor-edge lesson).
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
#   0 if both balance and compatibility checks PASS
#   1 if either check FAILs

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
TIMEOUT_S=20

echo "========================================"
echo "Stage E: LQR Balance + Compatibility Harness"
echo "========================================"
echo "Test: Verify robot stays balanced (max |tilt| <= ${THRESHOLD_RAD}rad)"
echo "Timeout: ${TIMEOUT_S}s"
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

# --- Part 1: Balance validation ---
if [ ! -f "$WORLD" ]; then
    echo "FAIL: World file not found: $WORLD"
    exit 1
fi

if [ -f "$CONTROLLER_LOG" ]; then
    rm -f "$CONTROLLER_LOG"
fi

echo "Running Webots (batch, headless)..."

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

sleep 1

if [ ! -f "$CONTROLLER_LOG" ]; then
    echo "FAIL: Controller log not found: $CONTROLLER_LOG"
    exit 1
fi

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

pass=$(awk -v tilt="$max_tilt" -v threshold="$THRESHOLD_RAD" 'BEGIN { print (tilt <= threshold) ? 1 : 0 }')

if [ "$pass" -eq 1 ]; then
    echo "PASS: Max |tilt| ${max_tilt}rad <= ${THRESHOLD_RAD}rad threshold"
    echo ""
    echo "Robot remained balanced throughout the ${TIMEOUT_S}s run."
    echo ""
    echo "========================================"
    echo "ALL CHECKS PASSED (balance + compatibility)"
    echo "========================================"
    exit 0
else
    echo "FAIL: Max |tilt| ${max_tilt}rad exceeds ${THRESHOLD_RAD}rad threshold"
    echo ""
    echo "Last 20 lines of log:"
    tail -n 20 "$CONTROLLER_LOG"
    exit 1
fi
