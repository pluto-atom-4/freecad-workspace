#!/usr/bin/env python3
"""
Linearized LQR (Linear Quadratic Regulator) controller for the inverted pendulum plant.

Pure Python, no Webots dependencies.
Implements a reduced 2-state linearization around the upright equilibrium (theta=0).

The model represents an inverted pendulum on a wheeled cart with:
- State vector: [theta, theta_dot] (pitch angle in radians, pitch rate in rad/s)
- Linearization: around theta=0 (ideal upright equilibrium)
- Input: commanded cart linear acceleration (m/s^2)

Note on simplification: The empirical PID resting tilt (~-0.1086 rad) observed in
closed-loop operation is a controller-loop artifact, not part of this physics-derived
linearized model, which assumes ideal upright equilibrium at theta=0. This linearization
is valid for small deviations around theta=0.
"""

import numpy as np
from scipy.linalg import solve_continuous_are

# Physical constants derived from InvertedPendulumRobot.proto's baked physics blocks
# (base Robot physics block + Pendulum_Link + Pendulum_Link_Right physics blocks;
#  robot_parameters.yaml is NOT used -- it is a stale placeholder, see issue #218).
M_BODY_KG = 0.6            # base (0.25kg) + both pendulum links (0.175kg each)
L_EFF_M = 0.034745875      # combined COM height above wheel axle (parallel-axis derived)
I_PIVOT_KG_M2 = 0.0009663818266249999  # moment of inertia about axle Y-line
GRAVITY_M_S2 = 9.81


def build_state_space():
    """Build linearized state-space matrices A and B.

    Returns:
        tuple: (A, B) where
            - A is the 2x2 state transition matrix
            - B is the 2x1 input matrix

    The linearized model is:
        x_dot = A @ x + B @ u
    where:
        - x = [theta, theta_dot]^T (state: pitch angle and rate)
        - u = cart linear acceleration (m/s^2)

    Matrix entries:
        - A[0][1] = 1.0: theta_dot contributes to theta rate (identity)
        - A[1][0] = (M_BODY_KG * g * L_EFF_M) / I_PIVOT_KG_M2:
          Gravitational restoring torque coefficient (positive for inverted pendulum,
          unstable pole at origin)
        - A[1][1] = 0.0: No damping in linearized model
        - B[0][0] = 0.0: Cart acceleration does not directly affect angle
        - B[1][0] = -(M_BODY_KG * L_EFF_M) / I_PIVOT_KG_M2:
          Negative because cart acceleration left → pendulum tilts right
    """
    A = np.array([
        [0.0, 1.0],
        [M_BODY_KG * GRAVITY_M_S2 * L_EFF_M / I_PIVOT_KG_M2, 0.0]
    ])

    B = np.array([
        [0.0],
        [-M_BODY_KG * L_EFF_M / I_PIVOT_KG_M2]
    ])

    return A, B


def compute_lqr_gain(A, B, Q, R):
    """Compute optimal LQR feedback gain using continuous-time Riccati equation.

    Solves the continuous-time algebraic Riccati equation (CARE):
        0 = A^T @ P + P @ A - P @ B @ R^{-1} @ B^T @ P + Q

    and returns the optimal feedback gain:
        K = R^{-1} @ B^T @ P

    Args:
        A (np.ndarray): 2x2 state transition matrix
        B (np.ndarray): 2x1 input matrix
        Q (np.ndarray): 2x2 positive semidefinite state cost matrix
        R (np.ndarray): 1x1 positive definite input cost matrix

    Returns:
        np.ndarray: Optimal feedback gain K (1x2 shape)

    Raises:
        ValueError: If Q or R have incorrect shape, or if CARE solver fails
    """
    # Validate shapes
    if Q.shape != (2, 2):
        raise ValueError(
            f"Q must be 2x2 for this 2-state system, got shape {Q.shape}"
        )
    if R.shape != (1, 1):
        raise ValueError(
            f"R must be 1x1 for this 1-input system, got shape {R.shape}"
        )

    # Solve continuous-time algebraic Riccati equation
    try:
        P = solve_continuous_are(A, B, Q, R)
    except Exception as e:
        raise ValueError(
            f"LQR gain computation failed: {e}. "
            f"Check Q/R are positive (semi-)definite."
        ) from e

    # Compute gain K = R^{-1} @ B^T @ P
    K = np.linalg.inv(R) @ B.T @ P

    return K


def default_gain():
    """Compute LQR gain using default Q and R cost matrices.

    Default costs (expected to be tuned in a later issue):
        Q = diag([10.0, 1.0]): prioritize angle stabilization over rate (10:1 ratio)
        R = [[0.1]]: moderate control effort penalty

    These are reasonable starting defaults for the inverted pendulum balancing problem.

    Returns:
        np.ndarray: Optimal feedback gain K (1x2 shape)
    """
    A, B = build_state_space()
    Q = np.diag([10.0, 1.0])
    R = np.array([[0.1]])
    return compute_lqr_gain(A, B, Q, R)
