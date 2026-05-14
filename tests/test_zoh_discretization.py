"""Tests for the public zoh_discretize helper in state_space_model."""
from __future__ import annotations

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

scipy = pytest.importorskip('scipy')

from pi_nodes.control.state_space_model import zoh_discretize  # noqa: E402


def test_zoh_scalar_known_values():
    """Continuous A=[[-10]], B=[[10]], Ts=0.02 →
    Ad = exp(-0.2) ≈ 0.818731, Bd = (1-exp(-0.2)) ≈ 0.181269."""
    Ad, Bd = zoh_discretize(np.array([[-10.0]]), np.array([[10.0]]), 0.02)
    np.testing.assert_allclose(Ad, [[0.8187307530779818]], atol=1e-9)
    np.testing.assert_allclose(Bd, [[0.1812692469220182]], atol=1e-9)


def test_zoh_eigenvalue_mapping():
    """ZOH maps continuous eigenvalues by λ_d = exp(λ_c · Ts)."""
    tau_v, tau_w, Ts = 0.15, 0.10, 0.02
    A_c = np.array([
        [0.0,  1.0,       0.0,  0.0,       0.0],
        [0.0, -1.0/tau_v, 0.0,  0.0,       0.0],
        [0.0,  0.0,       0.0,  1.0,       0.0],
        [0.0,  0.0,       0.0, -1.0/tau_w, 0.0],
        [0.0, -1.0,       0.0,  0.0,       0.0],
    ])
    B_c = np.array([
        [0.0,       0.0],
        [1.0/tau_v, 0.0],
        [0.0,       0.0],
        [0.0,       1.0/tau_w],
        [0.0,       0.0],
    ])
    Ad, _ = zoh_discretize(A_c, B_c, Ts)
    lam_c = np.sort_complex(np.linalg.eigvals(A_c))
    lam_d = np.sort_complex(np.linalg.eigvals(Ad))
    np.testing.assert_allclose(lam_d, np.exp(lam_c * Ts), atol=1e-9)


def test_zoh_shapes():
    Ad, Bd = zoh_discretize(np.zeros((5, 5)), np.ones((5, 2)), 0.05)
    assert Ad.shape == (5, 5)
    assert Bd.shape == (5, 2)
