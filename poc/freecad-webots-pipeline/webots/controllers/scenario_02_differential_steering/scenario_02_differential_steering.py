#!/usr/bin/env python3
"""
Scenario 2: Differential Steering Controller
Demonstrates curved motion by setting different wheel velocities.

Expected behavior:
  - Left wheel: 2.0 rad/s (faster)
  - Right wheel: 1.0 rad/s (slower)
  - Result: Robot curves to the right in a smooth arc
  - Turning radius: proportional to velocity difference
"""

import math

# Initialize Webots robot instance
from controller import Robot, Motor, PositionSensor

TIME_STEP = 64  # milliseconds, must match world file
robot = Robot()

# Get time step from world
time_step = int(robot.getBasicTimeStep())

# Motor setup (left and right wheels)
motor_left = robot.getDevice("wheel_left_joint")
motor_right = robot.getDevice("wheel_right_joint")

if not motor_left:
    motor_left = robot.getDevice("wheel_left_joint_motor")
if not motor_right:
    motor_right = robot.getDevice("wheel_right_joint_motor")

# Position sensors for wheel rotation
sensor_left = robot.getDevice("wheel_left_joint_sensor")
sensor_right = robot.getDevice("wheel_right_joint_sensor")

if not sensor_left:
    sensor_left = robot.getDevice("wheel_left_joint")
if not sensor_right:
    sensor_right = robot.getDevice("wheel_right_joint")

# Motor configuration
motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

# Target velocities for differential steering
V_LEFT = 2.0    # rad/s (faster—left wheel)
V_RIGHT = 1.0   # rad/s (slower—right wheel)

motor_left.setVelocity(V_LEFT)
motor_right.setVelocity(V_RIGHT)

# Enable sensors
if sensor_left:
    sensor_left.enable(time_step)
if sensor_right:
    sensor_right.enable(time_step)

# Telemetry and test logic
simulation_time = 0.0
max_time = 10.0  # seconds
motor_velocity_left_recorded = False
motor_velocity_right_recorded = False

previous_position_left = 0.0
previous_position_right = 0.0

positions_left = []
positions_right = []

print("=== Scenario 2: Differential Steering ===")
print(f"Left wheel velocity: {V_LEFT} rad/s")
print(f"Right wheel velocity: {V_RIGHT} rad/s")
print("Expected: Curved motion (arc) to the right")

# Main simulation loop
while robot.step(time_step) != -1:
    simulation_time = robot.getTime()
    
    # Record telemetry
    if sensor_left:
        pos_left = sensor_left.getValue()
        positions_left.append(pos_left)
    
    if sensor_right:
        pos_right = sensor_right.getValue()
        positions_right.append(pos_right)
    
    # Print periodic telemetry
    if abs(simulation_time % 2.0) < 0.1:
        if sensor_left and sensor_right:
            print(f"T={simulation_time:.1f}s | L_pos={positions_left[-1]:.4f} rad | R_pos={positions_right[-1]:.4f} rad")
    
    # Stop simulation after max_time
    if simulation_time >= max_time:
        break

# Verdict logic
test_verdict = "PASS"
reasons = []

if len(positions_left) < 2 or len(positions_right) < 2:
    test_verdict = "FAIL"
    reasons.append("Insufficient sensor data collected")
else:
    # Check that left wheel rotated more than right wheel
    left_delta = positions_left[-1] - positions_left[0]
    right_delta = positions_right[-1] - positions_right[0]
    
    if left_delta <= right_delta:
        test_verdict = "FAIL"
        reasons.append(f"Left wheel rotation ({left_delta:.4f}) not greater than right ({right_delta:.4f})")
    
    # Check that velocity ratio is correct
    if left_delta > 0 and right_delta > 0:
        ratio = left_delta / right_delta
        expected_ratio = V_LEFT / V_RIGHT  # 2.0
        if abs(ratio - expected_ratio) > 0.2:
            test_verdict = "FAIL"
            reasons.append(f"Velocity ratio {ratio:.2f} deviates from expected {expected_ratio:.2f}")
    
    # Check monotonic increase (no reversal)
    for i in range(1, len(positions_left)):
        if positions_left[i] < positions_left[i-1]:
            test_verdict = "FAIL"
            reasons.append("Left wheel position decreased (reversal detected)")
            break
    
    for i in range(1, len(positions_right)):
        if positions_right[i] < positions_right[i-1]:
            test_verdict = "FAIL"
            reasons.append("Right wheel position decreased (reversal detected)")
            break

print(f"\n=== Test Verdict: {test_verdict} ===")
if reasons:
    for reason in reasons:
        print(f"  - {reason}")
if test_verdict == "PASS":
    print(f"  - Left wheel rotation: {positions_left[-1]:.4f} rad")
    print(f"  - Right wheel rotation: {positions_right[-1]:.4f} rad")
    print(f"  - Velocity ratio: {positions_left[-1] / positions_right[-1]:.2f} (expected ~2.0)")
    print(f"  - Curved motion verified (differential steering working)")

exit(0 if test_verdict == "PASS" else 1)
