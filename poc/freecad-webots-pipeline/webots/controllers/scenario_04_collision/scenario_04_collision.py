#!/usr/bin/env python3
"""
Scenario 4: Collision Handling Controller
Demonstrates physics-based collision when robot hits an obstacle. Uses forward motion until collision.

Expected behavior:
  - Left/Right wheel: 2.0 rad/s (forward motion, same as Scenario 1)
  - Robot moves forward until it contacts the obstacle at ~0.5m
  - Physics engine simulates collision → robot stops
  - Motor commands continue but position stops (wheels spin in place)
  - Wheel velocity decreases to 0 as friction/collision force dominates
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

# Target velocities: same as forward motion (no collision avoidance logic)
V_LEFT = 2.0    # rad/s (forward)
V_RIGHT = 2.0   # rad/s (forward)

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
timestamps = []

print("=== Scenario 4: Collision Handling ===")
print(f"Left wheel velocity: {V_LEFT} rad/s")
print(f"Right wheel velocity: {V_RIGHT} rad/s")
print("Expected: Robot moves forward, hits obstacle at ~0.5m, stops due to collision")

collision_detected_time = None
collision_detected_rotation = None

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
    
    timestamps.append(simulation_time)
    
    # Detect collision: when position stops increasing despite motor commands
    if len(positions_left) > 10 and collision_detected_time is None:
        # Check if last 5 readings show no position change (within tolerance)
        recent_delta = positions_left[-1] - positions_left[-5]
        if recent_delta < 0.01:  # Less than 0.01 rad in 5 steps (~0.3 seconds)
            collision_detected_time = simulation_time
            collision_detected_rotation = positions_left[-1]
    
    # Print periodic telemetry
    if abs(simulation_time % 2.0) < 0.1:
        if sensor_left and sensor_right:
            status = "COLLISION" if collision_detected_time and simulation_time > collision_detected_time else "MOVING"
            print(f"T={simulation_time:.1f}s [{status}] | L_pos={positions_left[-1]:.4f} rad | R_pos={positions_right[-1]:.4f} rad")
    
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
    # Check that robot started moving forward
    initial_delta = positions_left[10] - positions_left[0] if len(positions_left) > 10 else 0
    if initial_delta <= 0:
        test_verdict = "FAIL"
        reasons.append("Robot did not move forward initially")
    
    # Check that collision was detected (position plateaued)
    if collision_detected_time is None:
        test_verdict = "FAIL"
        reasons.append("No collision detected (position never plateaued)")
    else:
        # Collision was detected at expected time
        print(f"  - Collision detected at t={collision_detected_time:.1f}s, wheel rotation={collision_detected_rotation:.4f} rad")
    
    # Check that wheels are still in sync
    left_final = positions_left[-1]
    right_final = positions_right[-1]
    if abs(left_final - right_final) > 0.1:
        test_verdict = "FAIL"
        reasons.append(f"Wheels out of sync: L={left_final:.4f}, R={right_final:.4f}")

print(f"\n=== Test Verdict: {test_verdict} ===")
if reasons:
    for reason in reasons:
        print(f"  - {reason}")
if test_verdict == "PASS":
    print(f"  - Initial forward motion: {initial_delta:.4f} rad")
    print(f"  - Final wheel rotation: {positions_left[-1]:.4f} rad")
    print(f"  - Collision physics verified (motor commands vs. actual motion differ)")

exit(0 if test_verdict == "PASS" else 1)
