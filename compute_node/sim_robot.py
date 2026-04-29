"""
SimRobot — kinematic 2-D physics for the Flask simulator (#44 phase 3).

Extracted from compute_node/simulator.py. Pure kinematics: forward
velocity / yaw integration, wall clamp, forbidden-zone reject. No Flask,
no rendering, no NumPy. The robot reads its sensors via an injected
`range_provider` callable so the SimRobot ↔ SimSensors edge stays loose
and unit-testable.

The remaining classes (SimSensors, SimDetector, SimFSM, MapRenderer)
still live in simulator.py — see backlog #44 for the migration plan.
"""
from __future__ import annotations

import math
from typing import Callable, Optional

# Defaults match simulator.py — calling code can override per instance.
DEFAULT_ROBOT_RADIUS = 0.12        # m
DEFAULT_MAX_LINEAR = 0.30          # m/s
DEFAULT_MAX_ANGULAR = 2.0          # rad/s
DEFAULT_COLLISION_STOP = 0.20      # m
DEFAULT_COLLISION_SLOW = 0.40      # m


class SimRobot:
    """Tracked-base kinematics with collision guard.

    `arena` is duck-typed: needs `.width`, `.height`, `.forbidden_zones`.
    `range_provider`, if supplied, is called with no args and must return
    a dataclass-like object with a `.range_m` attribute (the ultrasonic
    reading); if None, the collision guard is silently disabled. This
    keeps SimRobot independent of SimSensors at import time — they're
    wired up by the caller after construction.
    """

    def __init__(self, arena,
                 robot_radius: float = DEFAULT_ROBOT_RADIUS,
                 max_linear: float = DEFAULT_MAX_LINEAR,
                 max_angular: float = DEFAULT_MAX_ANGULAR,
                 collision_stop: float = DEFAULT_COLLISION_STOP,
                 collision_slow: float = DEFAULT_COLLISION_SLOW,
                 range_provider: Optional[Callable[[], object]] = None):
        self.arena = arena
        self.x = arena.width / 2.0
        self.y = arena.height / 2.0
        self.theta = 0.0          # heading (radians)
        self.v_linear = 0.0
        self.v_angular = 0.0

        # Configuration
        self.robot_radius = robot_radius
        self.max_speed = max_linear   # mutated by speed_profile
        self.max_angular = max_angular
        self.collision_stop = collision_stop
        self.collision_slow = collision_slow

        # Actuator state — kept for status snapshots, not part of kinematics.
        self.claw_open = False
        self.head_angle = 0.0
        self.arm_joints = [0.0, 120.0, 0.0, 0.0]

        # Hooks the simulator wires up after construction
        self._prev_v_linear = 0.0
        self.collision_guard = False
        self._range_provider = range_provider

    def attach_range_provider(self,
                              provider: Optional[Callable[[], object]]) -> None:
        """Wire up sensors after construction. Old simulator.py used a
        `_range_m_ref` attribute set by the sim loop; the provider
        indirection is functionally equivalent and easier to mock."""
        self._range_provider = provider

    # ── Velocity API ───────────────────────────────────────────────────
    def set_velocity(self, linear: float, angular: float) -> None:
        # Collision guard — limit forward motion when obstacle ahead.
        if self.collision_guard and linear > 0 and self._range_provider is not None:
            sensors = self._range_provider()
            r = getattr(sensors, 'range_m', None)
            if r is not None:
                if r < self.collision_stop:
                    linear = 0.0
                elif r < self.collision_slow:
                    factor = (r - self.collision_stop) / \
                             (self.collision_slow - self.collision_stop)
                    linear *= max(0.0, factor)
        self.v_linear = max(-self.max_speed, min(self.max_speed, linear))
        self.v_angular = max(-self.max_angular, min(self.max_angular, angular))

    def stop(self) -> None:
        self.v_linear = 0.0
        self.v_angular = 0.0

    # ── Kinematics integrator ──────────────────────────────────────────
    def tick(self, dt: float) -> None:
        self._prev_v_linear = self.v_linear

        prev_x, prev_y = self.x, self.y

        # Forward integration: x' = v cos θ; y' = v sin θ; θ' = ω
        self.x += self.v_linear * math.cos(self.theta) * dt
        self.y += self.v_linear * math.sin(self.theta) * dt
        self.theta += self.v_angular * dt
        # Normalise θ ∈ (-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Wall clamp (Minkowski-shrunk arena)
        m = self.robot_radius
        self.x = max(m, min(self.arena.width - m, self.x))
        self.y = max(m, min(self.arena.height - m, self.y))

        # Forbidden zone reject — if the new position lands inside any
        # inflated zone, snap back to the previous position and zero
        # forward velocity. Faithful to the original behaviour: the
        # robot just sits there until the operator changes course.
        for z in self.arena.forbidden_zones:
            zx1 = z['x1'] - m
            zy1 = z['y1'] - m
            zx2 = z['x2'] + m
            zy2 = z['y2'] + m
            if zx1 <= self.x <= zx2 and zy1 <= self.y <= zy2:
                self.x = prev_x
                self.y = prev_y
                self.v_linear = 0.0
                break
