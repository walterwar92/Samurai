"""Unit-tests для pi_nodes/control/mps_reference.py — генератор r(t)."""
import math
import numpy as np
import pytest

from pi_nodes.control.mps_reference import (
    ReferenceTrajectory,
    build_reference,
)


def test_build_reference_returns_trajectory():
    traj = build_reference(distance=0.30, v_target=0.15, target_heading=math.pi,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    assert isinstance(traj, ReferenceTrajectory)
    assert traj.t_drive > 0
    assert traj.t_end > traj.t_drive
