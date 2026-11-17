"""The environment: an arena with border walls and interior obstacles.

World is intentionally boring. It stores solid rectangles and answers the
two questions the rest of the system asks every single tick:

1. does this circle overlap anything solid?   -> collide_circle()
2. how far until a ray hits something solid?  -> raycast()

All the tricky geometry lives in algorithms/geometry.py. This file is just
the map. Metres, y-up.
"""

import math

from algorithms.geometry import Rect, circle_rect_collision, ray_rect_distance

# Arena size in metres. 24 x 16 m at 40 px per metre -> a 960 x 640 window.
ARENA_WIDTH = 24.0
ARENA_HEIGHT = 16.0
WALL_THICKNESS = 0.4


class World:
    def __init__(self, width, height, walls, obstacles):
        self.width = width
        self.height = height
        self.walls = walls
        self.obstacles = obstacles

    @property
    def solids(self):
        """Everything the robot body and the sonar beams can hit."""
        return self.walls + self.obstacles

    def collide_circle(self, cx, cy, radius):
        """Deepest overlap between a circle and any solid, or None.

        Only the deepest contact is reported. That's enough in practice:
        the robot pushes out of it, and the next pass picks up any contact
        still left. Corners settle down in a couple of iterations without
        needing a real physics engine.
        """
        deepest = None
        for rect in self.solids:
            hit = circle_rect_collision(cx, cy, radius, rect)
            if hit is not None and (deepest is None or hit.depth > deepest.depth):
                deepest = hit
        return deepest

    def raycast(self, ox, oy, angle, max_range):
        """Distance from (ox, oy) along `angle` to the nearest solid.

        This mirrors what an ultrasonic sensor actually reports: if the
        beam hits nothing, the echo never comes back, so we return
        max_range.
        """
        dx = math.cos(angle)
        dy = math.sin(angle)
        best = max_range
        for rect in self.solids:
            t = ray_rect_distance(ox, oy, dx, dy, rect, best)
            if t is not None and t < best:
                best = t
        return best


def build_default_world():
    """The Week 1 arena: a walled room with a handful of obstacle blocks."""
    t = WALL_THICKNESS
    w, h = ARENA_WIDTH, ARENA_HEIGHT
    walls = (
        Rect(0.0, -t, w, t),  # bottom
        Rect(0.0, h, w, t),   # top
        Rect(-t, 0.0, t, h),  # left
        Rect(w, 0.0, t, h),   # right
    )
    obstacles = (
        Rect(5.0, 4.0, 1.2, 6.0),    # tall slab on the left
        Rect(9.0, 10.5, 7.0, 1.2),   # horizontal beam up top
        Rect(15.5, 3.0, 1.2, 5.0),   # pillar on the right
        Rect(18.0, 10.0, 3.0, 3.0),  # big block, upper right
        Rect(6.5, 1.5, 3.0, 1.2),    # kerb, bottom left
    )
    return World(width=w, height=h, walls=walls, obstacles=obstacles)
