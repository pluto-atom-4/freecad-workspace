#!/usr/bin/env python3
"""
Scenario 3: In-Place Rotation Controller
Demonstrates 360° rotation without translation by setting opposite wheel velocities.

Expected behavior:
  - Left wheel: 2.0 rad/s (forward)
  - Right wheel: -2.0 rad/s (backward)
  - Result: Robot spins in place, ~360° per ~6.28 seconds
  - X/Y position: Stays near 0 (no translation)
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

# Target velocities for in-place rotation
V_LEFT = 2.0     # rad/s (forward)
V_RIGHT = -2.0   # rad/s (backward, opposite direction)

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

positions_left = []
positions_right = []

print("=== Scenario 3: In-Place Rotation ===")
print(f"Left wheel velocity: {V_LEFT} rad/s")
print(f"Right wheel velocity: {V_RIGHT} rad/s (opposite)")
print("Expected: 360° rotation without translation")

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
            print(f"T={simulation_time:.1f}s | L_pos={positions_left[-1]:+.4f} rad | R_pos={positions_right[-1]:+.4f} rad")
    
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
    # Check that left wheel and right wheel move in opposite directions
    left_delta = positions_left[-1] - positions_left[0]
    right_delta = positions_right[-1] - positions_right[0]
    
    # Left should increase (positive), right should decrease (negative)
    if left_delta <= 0 or right_delta >= 0:
        test_verdict = "FAIL"
        reasons.append(f"Wheel velocities not opposite: L={left_delta:+.4f}, R={right_delta:+.4f}")
    
    # Check magnitudes are equal
    if left_delta > 0 and right_delta < 0:
        if abs(left_delta + right_delta) > 0.2:  # Net should be close to zero
            test_verdict = "FAIL"
            reasons.append(f"Wheel rotations not symmetric: L={left_delta:.4f}, R={right_delta:.4f} (sum={left_delta+right_delta:.4f})")
    
    # Each wheel should complete at least one full rotation
    if abs(left_delta) < 2*math.pi or abs(right_delta) < 2*math.pi:
        test_verdict = "FAIL"
        reasons.append(f"Insufficient rotation: L={abs(left_delta):.4f} rad, R={abs(right_delta):.4f} rad (need {2*math.pi:.4f})")

print(f"\n=== Test Verdict: {test_verdict} ===")
if reasons:
    for reason in reasons:
        print(f"  - {reason}")
if test_verdict == "PASS":
    print(f"  - Left wheel rotation: {positions_left[-1]:+.4f} rad")
    print(f"  - Right wheel rotation: {positions_right[-1]:+.4f} rad")
    print(f"  - Sum (net translation): {positions_left[-1] + positions_right[-1]:+.6f} rad (≈0 for no translation)")
    print(f"  - In-place rotation verified (robot spun without translating)")

exit(0 if test_verdict == "PASS" else 1)
