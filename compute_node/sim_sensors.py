"""
SimSensors — ultrasonic ray-cast + IMU model for the Flask simulator
(#44 phase 4).

Extracted from compute_node/simulator.py. Pure geometry + Gaussian
noise; no Flask, no NumPy. The accelerometer derivative needs the
robot's previous-tick linear velocity, which we read via attribute
(SimRobot._prev_v_linear) — the contract is that the caller updates
sensors AFTER ticking the robot, so _prev_v_linear holds the previous
frame's value.

For unit tests, an `inject_noise` flag disables the random noise so
ray-cast results are deterministic.
"""
from __future__ import annotations

import math
import random
from typing import Optional

DEFAULT_ULTRASONIC_MAX = 2.0      # m
DEFAULT_ULTRASONIC_MIN = 0.02     # m
DEFAULT_ULTRASONIC_CONE = 0.05    # m — half-width of ray for ball intersect
DEFAULT_SIM_DT = 0.05             # s — needed for accel derivative

# Noise stddevs — Gaussian. Match the original simulator's hardcoded values.
DEFAULT_RANGE_NOISE = 0.005       # m
DEFAULT_IMU_YAW_NOISE = 0.3       # deg
DEFAULT_IMU_PITCH_ROLL_NOISE = 0.2   # deg


class SimSensors:
    """Forward-pointing ultrasonic + body-frame IMU model.

    Ultrasonic: ray from robot pose along heading; nearest hit among
    arena walls and unsifted balls within the half-cone, plus Gaussian
    noise. Returns reading in (MIN, MAX) range.

    IMU: yaw is the robot's heading in degrees, gyro_z is angular
    velocity, accel_x is the discrete derivative of linear velocity.
    Pitch/roll are pure noise (the simulator runs on a flat floor).
    """

    def __init__(self,
                 ultrasonic_min: float = DEFAULT_ULTRASONIC_MIN,
                 ultrasonic_max: float = DEFAULT_ULTRASONIC_MAX,
                 cone_half_width: float = DEFAULT_ULTRASONIC_CONE,
                 dt: float = DEFAULT_SIM_DT,
                 inject_noise: bool = True,
                 noise_seed: Optional[int] = None):
        self.range_m = ultrasonic_max
        self.imu_yaw = 0.0
        self.imu_pitch = 0.0
        self.imu_roll = 0.0
        self.imu_gyro_z = 0.0
        self.accel_x = 0.0

        # Configuration
        self.ultrasonic_min = ultrasonic_min
        self.ultrasonic_max = ultrasonic_max
        self.cone_half_width = cone_half_width
        self.dt = dt
        self.inject_noise = inject_noise
        # Seedable RNG so tests get reproducible noise patterns.
        self._rng = random.Random(noise_seed) if noise_seed is not None else random

    # ── Update ─────────────────────────────────────────────────────────
    def update(self, robot, arena) -> None:
        """Refresh both sub-sensors from the current robot/arena state."""
        self._update_ultrasonic(robot, arena)
        self._update_imu(robot)

    def _update_ultrasonic(self, robot, arena) -> None:
        rx, ry = robot.x, robot.y
        dx = math.cos(robot.theta)
        dy = math.sin(robot.theta)
        best = self.ultrasonic_max

        # Walls — solve for the parametric distance t along the ray
        # until x or y hits an arena boundary.
        if dx > 0:
            t = (arena.width - rx) / dx
            if self.ultrasonic_min < t < best:
                best = t
        elif dx < 0:
            t = -rx / dx
            if self.ultrasonic_min < t < best:
                best = t
        if dy > 0:
            t = (arena.height - ry) / dy
            if self.ultrasonic_min < t < best:
                best = t
        elif dy < 0:
            t = -ry / dy
            if self.ultrasonic_min < t < best:
                best = t

        # Balls — intersection with a finite-width ray cone.
        for ball in arena.balls:
            if ball.get('grabbed', False):
                continue
            bx, by = ball['x'], ball['y']
            to_ball_x = bx - rx
            to_ball_y = by - ry
            proj = to_ball_x * dx + to_ball_y * dy
            if proj < self.ultrasonic_min or proj > best:
                continue
            perp = abs(to_ball_x * dy - to_ball_y * dx)
            if perp < ball.get('radius', 0.02) + self.cone_half_width:
                best = proj

        if self.inject_noise:
            best += self._rng.gauss(0.0, DEFAULT_RANGE_NOISE)
        self.range_m = max(self.ultrasonic_min, min(self.ultrasonic_max, best))

    def _update_imu(self, robot) -> None:
        if self.inject_noise:
            self.imu_yaw = math.degrees(robot.theta) + \
                self._rng.gauss(0.0, DEFAULT_IMU_YAW_NOISE)
            self.imu_pitch = self._rng.gauss(0.0, DEFAULT_IMU_PITCH_ROLL_NOISE)
            self.imu_roll = self._rng.gauss(0.0, DEFAULT_IMU_PITCH_ROLL_NOISE)
        else:
            self.imu_yaw = math.degrees(robot.theta)
            self.imu_pitch = 0.0
            self.imu_roll = 0.0
        self.imu_gyro_z = robot.v_angular
        prev_v = getattr(robot, '_prev_v_linear', robot.v_linear)
        self.accel_x = (robot.v_linear - prev_v) / self.dt
