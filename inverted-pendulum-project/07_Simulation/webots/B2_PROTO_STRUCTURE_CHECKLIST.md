# B2 PROTO Structure Checklist

**Phase 13 Validation Report** — Structural alignment between URDF and PROTO

## Validation Results Summary

### ✅ PASS joint_names_match_urdf
**Details:** Checked 4 URDF joints against 4 PROTO motors

### ✅ PASS position_sensors_auto_named
**Details:** Checked 4 expected sensors, found 4

### ✅ PASS physics_propagated
**Details:** Found 5 mass, 5 inertia, 5 centerOfMass, 4 solids (expected 5 physics blocks, 4 solids)

### ✅ PASS mesh_urls_resolve
**Details:** Checked 1 mesh files, 1 resolved, 0 missing

### ✅ PASS vrml_syntax
**Details:** Braces: 90 open, 90 close; VRML header present: True

## Links (Expected: 5)

| # | Link Name | Status | Notes |
|---|-----------|--------|-------|
| 1 | `Base_Link` | ✓ | Present in URDF |
| 2 | `Wheel_Left` | ✓ | Present in URDF |
| 3 | `Wheel_Right` | ✓ | Present in URDF |
| 4 | `Pendulum_Link` | ✓ | Present in URDF |
| 5 | `Pendulum_Link_Right` | ✓ | Present in URDF |

## Joints (Expected: 4)

| # | Joint Name | RotationalMotor | PositionSensor | Status |
|---|------------|-----------------|----------------|--------|
| 1 | `wheel_left_joint` | `wheel_left_joint` | `wheel_left_joint_sensor` | ✓ |
| 2 | `wheel_right_joint` | `wheel_right_joint` | `wheel_right_joint_sensor` | ✓ |
| 3 | `pendulum_pivot_joint` | `pendulum_pivot_joint` | `pendulum_pivot_joint_sensor` | ✓ |
| 4 | `pendulum_pivot_right_joint` | `pendulum_pivot_right_joint` | `pendulum_pivot_right_joint_sensor` | ✓ |

## Next Steps

1. Review validation results above
2. If all checks pass, proceed to **Stage B3** (IMU node + live Webots visual check)
3. If any check fails, inspect the PROTO file and URDF reference, then regenerate
