"""
SimArena — 2D world model for the Flask simulator (#44 cont.).

Extracted from compute_node/simulator.py — pure data-model class with
no Flask, no NumPy, no rendering coupling. Lets the rest of the
simulator stack (and tests) work with arena state in isolation.

The simulator's other classes (SimRobot, SimSensors, SimDetector,
SimFSM, MapRenderer) still live in simulator.py for now — they're
much more entangled and need a careful pass each.
"""
from __future__ import annotations

# Default arena size and ball radius come from config.yaml's `simulator`
# section — caller can override via constructor params for tests / alternate
# arenas. The defaults match simulator.py's values so behaviour is identical
# when used through the existing Flask routes.
DEFAULT_ARENA_W = 3.0       # metres
DEFAULT_ARENA_H = 3.0
DEFAULT_BALL_RADIUS = 0.02

# BGR-tuple per ball colour name. Matches simulator.py's COLOUR_BGR.
DEFAULT_BALL_COLOURS = ('red', 'blue', 'green', 'yellow', 'orange')


class SimArena:
    """2D rectangular arena holding balls and forbidden zones.

    Pure model — no rendering, no physics. Methods are intentionally
    side-effect-free except for state mutation on this instance, so unit
    tests can exercise zone/ball logic without spinning a simulator loop.
    """

    def __init__(self,
                 width: float = DEFAULT_ARENA_W,
                 height: float = DEFAULT_ARENA_H,
                 ball_radius: float = DEFAULT_BALL_RADIUS,
                 colours=DEFAULT_BALL_COLOURS):
        self.width = width
        self.height = height
        self.ball_radius = ball_radius
        self._colours = tuple(colours)
        self.balls: list[dict] = []
        # Zones: list of {id, x1, y1, x2, y2}. id auto-incremented, never
        # reused (mirrors the dashboard zone API).
        self.forbidden_zones: list[dict] = []
        self._zone_counter = 0
        self._spawn_balls()

    # ── Balls ──────────────────────────────────────────────────────────
    def _spawn_balls(self) -> None:
        positions = [
            (0.8, 0.6), (2.2, 0.8), (1.5, 2.0), (0.5, 2.3), (2.5, 1.8),
        ]
        self.balls = [
            {
                'x': x, 'y': y,
                'colour': colour,
                'radius': self.ball_radius,
                'grabbed': False,
            }
            for (x, y), colour in zip(positions, self._colours)
        ]

    # ── Zones ──────────────────────────────────────────────────────────
    def add_zone(self, x1: float, y1: float, x2: float, y2: float) -> dict:
        """Add a forbidden zone (axis-aligned rectangle). Returns the zone."""
        self._zone_counter += 1
        zone = {
            'id': self._zone_counter,
            'x1': min(x1, x2), 'y1': min(y1, y2),
            'x2': max(x1, x2), 'y2': max(y1, y2),
        }
        self.forbidden_zones.append(zone)
        return zone

    def remove_zone(self, zone_id: int) -> bool:
        """Remove a zone by id. Returns True if a matching zone existed."""
        for i, z in enumerate(self.forbidden_zones):
            if z['id'] == zone_id:
                self.forbidden_zones.pop(i)
                return True
        return False

    def clear_zones(self) -> None:
        """Remove all forbidden zones (counter is NOT reset — ids stay unique)."""
        self.forbidden_zones.clear()

    def point_in_zone(self, x: float, y: float) -> bool:
        """True iff (x, y) sits inside any forbidden zone."""
        for z in self.forbidden_zones:
            if z['x1'] <= x <= z['x2'] and z['y1'] <= y <= z['y2']:
                return True
        return False

    # ── Reset ──────────────────────────────────────────────────────────
    def reset(self) -> None:
        """Restore balls to their spawn positions; KEEP forbidden zones."""
        self._spawn_balls()
