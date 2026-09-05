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
from controller import Supervisor, Motor, PositionSensor

# Unused: the main loop reads the real value from robot.getBasicTimeStep()
# instead. World file's basicTimeStep is 32ms.
TIME_STEP = 32  # milliseconds (unused, see getBasicTimeStep() call below)
robot = Supervisor()

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

# Supervisor access to chassis (self) position for collision detection
self_node = robot.getSelf()
initial_position = self_node.getPosition()

# Telemetry and test logic
simulation_time = 0.0
max_time = 10.0  # seconds

positions_left = []
positions_right = []
positions_x = []
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

    # Record chassis position (ground truth for collision detection,
    # since wheel-joint rotation alone doesn't reflect whether the
    # chassis is actually translating - see issue #32)
    position = self_node.getPosition()
    positions_x.append(position[0])

    timestamps.append(simulation_time)

    # Detect collision: when chassis x-position stops advancing despite motor commands
    if len(positions_x) > 10 and collision_detected_time is None:
        # Check if last 5 readings show no chassis displacement (within tolerance)
        recent_delta = positions_x[-1] - positions_x[-5]
        if abs(recent_delta) < 0.001:  # Less than 1mm in 5 steps
            collision_detected_time = simulation_time
            collision_detected_rotation = positions_left[-1] if positions_left else None
    
    # Print periodic telemetry
    if abs(simulation_time % 2.0) < 0.1:
        status = "COLLISION" if collision_detected_time and simulation_time > collision_detected_time else "MOVING"
        chassis_dx = positions_x[-1] - positions_x[-5] if len(positions_x) > 5 else float('nan')
        print(f"T={simulation_time:.1f}s [{status}] | chassis_x={positions_x[-1]:.4f} m | dx(5steps)={chassis_dx:.5f} m", end="")
        if sensor_left and sensor_right:
            print(f" | L_pos={positions_left[-1]:.4f} rad | R_pos={positions_right[-1]:.4f} rad (diagnostic only)")
        else:
            print()
    
    # Stop simulation after max_time
    if simulation_time >= max_time:
        break

# Verdict logic - based on chassis position (ground truth), not wheel-joint
# rotation. Wheel-angle telemetry (positions_left/positions_right) is kept
# only as diagnostic output below; it no longer drives PASS/FAIL (see #32 -
# wheel rotation alone can keep advancing after a collision has stopped the
# chassis, e.g. wheels spinning in place, and would previously pass a stuck
# robot).
test_verdict = "PASS"
reasons = []

if len(positions_x) < 2:
    test_verdict = "FAIL"
    reasons.append("Insufficient chassis position data collected")
else:
    # Check that the chassis started moving forward
    initial_delta_x = positions_x[10] - positions_x[0] if len(positions_x) > 10 else 0
    if initial_delta_x <= 0:
        test_verdict = "FAIL"
        reasons.append("Robot chassis did not move forward initially")

    # Check that collision was detected (chassis x-position plateaued)
    if collision_detected_time is None:
        test_verdict = "FAIL"
        reasons.append("No collision detected (position never plateaued)")
    else:
        # Collision was detected at expected time
        print(f"  - Collision detected at t={collision_detected_time:.1f}s, chassis_x={positions_x[-1]:.4f} m "
              f"(wheel rotation at detection={collision_detected_rotation:.4f} rad, diagnostic only)"
              if collision_detected_rotation is not None else
              f"  - Collision detected at t={collision_detected_time:.1f}s, chassis_x={positions_x[-1]:.4f} m")

print(f"\n=== Test Verdict: {test_verdict} ===")
if reasons:
    for reason in reasons:
        print(f"  - {reason}")
if test_verdict == "PASS":
    print(f"  - Initial forward motion (chassis): {initial_delta_x:.4f} m")
    print(f"  - Final chassis x-position: {positions_x[-1]:.4f} m")
    print(f"  - Collision physics verified (chassis stopped translating despite continued motor commands)")

# Diagnostic-only: wheel-joint telemetry (does not affect verdict)
if positions_left and positions_right:
    print(f"  - [diagnostic] Final wheel rotation: L={positions_left[-1]:.4f} rad, R={positions_right[-1]:.4f} rad")
    if abs(positions_left[-1] - positions_right[-1]) > 0.1:
        print(f"  - [diagnostic] Wheels out of sync: L={positions_left[-1]:.4f}, R={positions_right[-1]:.4f} (not a verdict criterion)")

exit(0 if test_verdict == "PASS" else 1)
