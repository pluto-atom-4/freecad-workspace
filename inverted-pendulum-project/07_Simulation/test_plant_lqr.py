#!/usr/bin/env python3
"""
Unit tests for LQR controller and linearized plant model.

Pure Python / pytest, no external dependencies beyond pytest and numpy/scipy.

Usage:
    cd inverted-pendulum-project
    mamba run -n pendulum-tools python3 -m pytest 07_Simulation/test_plant_lqr.py -v
"""

import sys
from pathlib import Path

import numpy as np
import pytest

# Add the script directory to path for imports
sys.path.insert(0, str(Path(__file__).resolve().parent))

from plant_lqr import build_state_space, compute_lqr_gain, default_gain


class TestBuildStateSpace:
    """Test: state-space matrix construction"""

    def test_state_space_shapes(self):
        """build_state_space() returns correct shapes: A is 2x2, B is 2x1."""
        A, B = build_state_space()

        assert A.shape == (2, 2), f"A should be 2x2, got {A.shape}"
        assert B.shape == (2, 1), f"B should be 2x1, got {B.shape}"

    def test_state_space_correct_values(self):
        """build_state_space() returns correct sign structure.

        For an inverted pendulum:
        - A[0][1] = 1.0 (theta_dot effect on theta)
        - A[1][0] > 0.0 (positive unstable pole, gravitational restoring torque)
        - A[1][1] = 0.0 (no damping term)
        - B[0][0] = 0.0 (cart accel doesn't directly affect angle)
        - B[1][0] < 0.0 (negative: cart accel left → pendulum tilts right)
        """
        A, B = build_state_space()

        assert A[0, 0] == 0.0
        assert A[0, 1] == 1.0
        assert A[1, 0] > 0.0, "A[1][0] should be positive (unstable pole)"
        assert A[1, 1] == 0.0

        assert B[0, 0] == 0.0
        assert B[1, 0] < 0.0, "B[1][0] should be negative"


class TestComputeLqrGain:
    """Test: LQR gain computation with continuous Riccati solver"""

    def test_lqr_gain_default_values(self):
        """compute_lqr_gain() with default Q/R produces expected gain.

        Using Q=diag([10, 1]), R=[[0.1]], the gain should be close to
        K ≈ [[-23.81842951, -3.49402271]] (pre-verified numerically).
        """
        A, B = build_state_space()
        Q = np.diag([10.0, 1.0])
        R = np.array([[0.1]])

        K = compute_lqr_gain(A, B, Q, R)

        expected_K = np.array([[-23.81842951, -3.49402271]])

        assert K.shape == (1, 2), f"K should be 1x2, got {K.shape}"
        assert np.allclose(K, expected_K, atol=1e-3), \
            f"K={K} does not match expected {expected_K}"

    def test_lqr_gain_closed_loop_stability(self):
        """LQR gain stabilizes closed-loop: all eigenvalues of (A - B@K) are negative real.

        This is the actual control-theory correctness check: the feedback gain
        must shift the unstable poles of the open-loop system into the left half-plane.
        """
        A, B = build_state_space()
        Q = np.diag([10.0, 1.0])
        R = np.array([[0.1]])

        K = compute_lqr_gain(A, B, Q, R)

        # Compute closed-loop A matrix: A_cl = A - B @ K
        A_cl = A - B @ K

        # Get eigenvalues
        eigvals = np.linalg.eigvals(A_cl)

        # All eigenvalues must have negative real parts for stability
        assert np.all(np.real(eigvals) < 0), \
            f"Closed-loop eigenvalues {eigvals} not all in left half-plane"

    def test_compute_lqr_gain_wrong_q_shape(self):
        """compute_lqr_gain() raises ValueError for wrong Q shape (e.g. 3x3)."""
        A, B = build_state_space()
        Q_wrong = np.eye(3)  # 3x3 instead of 2x2
        R = np.array([[0.1]])

        with pytest.raises(ValueError, match="Q must be 2x2"):
            compute_lqr_gain(A, B, Q_wrong, R)

    def test_compute_lqr_gain_wrong_r_shape(self):
        """compute_lqr_gain() raises ValueError for wrong R shape (e.g. 2x2)."""
        A, B = build_state_space()
        Q = np.diag([10.0, 1.0])
        R_wrong = np.eye(2)  # 2x2 instead of 1x1

        with pytest.raises(ValueError, match="R must be 1x1"):
            compute_lqr_gain(A, B, Q, R_wrong)


class TestDefaultGain:
    """Test: convenience wrapper default_gain()"""

    def test_default_gain_returns_valid_shape(self):
        """default_gain() returns 1x2 shaped array without error."""
        K = default_gain()

        assert K.shape == (1, 2), f"K should be 1x2, got {K.shape}"

    def test_default_gain_stabilizes_system(self):
        """default_gain() produces a gain that stabilizes the closed-loop system."""
        A, B = build_state_space()
        K = default_gain()

        A_cl = A - B @ K
        eigvals = np.linalg.eigvals(A_cl)

        assert np.all(np.real(eigvals) < 0), \
            f"Closed-loop with default_gain() eigenvalues {eigvals} not stable"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
