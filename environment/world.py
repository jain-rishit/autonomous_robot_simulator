"""The simulated environment: arena, boundary walls and interior obstacles.

`World` is deliberately boring: it stores solid rectangles and answers the
two spatial questions the rest of the system asks every tick:

1. "Does this circle overlap anything solid?"   -> collide_circle()
2. "How far until a ray hits something solid?"  -> raycast()

All actual geometry lives in algorithms/geometry.py; the World is just the
map. Units are metres, world frame is y-up.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from algorithms.geometry import Collision, Rect, circle_rect_collision, ray_rect_distance

# Arena size (metres). 24 x 16 m at 40 px/m -> a 960 x 640 px window.
ARENA_WIDTH = 24.0
ARENA_HEIGHT = 16.0
WALL_THICKNESS = 0.4


@dataclass
class World:
    width: float
    height: float
    walls: tuple[Rect, ...]
    obstacles: tuple[Rect, ...]

    @property
    def solids(self) -> tuple[Rect, ...]:
        """Everything the robot body and the sonar beams can hit."""
        return self.walls + self.obstacles

    def collide_circle(self, cx: float, cy: float, radius: float) -> Collision | None:
        """Deepest overlap between a circle and any solid, or None.

        Returning only the deepest contact is enough in practice: the robot
        resolves it, and the next iteration picks up any remaining contact.
        That is how corner cases converge without a full physics engine.
        """
        deepest: Collision | None = None
        for rect in self.solids:
            hit = circle_rect_collision(cx, cy, radius, rect)
            if hit is not None and (deepest is None or hit.depth > deepest.depth):
                deepest = hit
        return deepest

    def raycast(self, ox: float, oy: float, angle: float, max_range: float) -> float:
        """Distance from (ox, oy) along `angle` to the nearest solid.

        Mirrors what an ultrasonic sensor reports: max_range when the beam
        hits nothing (the echo never comes back).
        """
        dx = math.cos(angle)
        dy = math.sin(angle)
        best = max_range
        for rect in self.solids:
            t = ray_rect_distance(ox, oy, dx, dy, rect, best)
            if t is not None and t < best:
                best = t
        return best


def build_default_world() -> World:
    """The Week-1 test arena: a walled room with a few obstacle blocks."""
    t = WALL_THICKNESS
    w, h = ARENA_WIDTH, ARENA_HEIGHT
    walls = (
        Rect(0.0, -t, w, t),  # bottom
        Rect(0.0, h, w, t),  # top
        Rect(-t, 0.0, t, h),  # left
        Rect(w, 0.0, t, h),  # right
    )
    obstacles = (
        Rect(5.0, 4.0, 1.2, 6.0),  # tall slab, left half
        Rect(9.0, 10.5, 7.0, 1.2),  # horizontal beam, upper middle
        Rect(15.5, 3.0, 1.2, 5.0),  # pillar, right half
        Rect(18.0, 10.0, 3.0, 3.0),  # large block, upper right
        Rect(6.5, 1.5, 3.0, 1.2),  # kerb, bottom left
    )
    return World(width=w, height=h, walls=walls, obstacles=obstacles)
