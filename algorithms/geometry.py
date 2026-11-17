"""Reusable 2D computational geometry (no pygame, no simulation state).

Conventions
-----------
* Rectangles are axis-aligned (AABB): bottom-left corner (x, y) + w, h.
* Rays are parametric: p(t) = origin + t * direction, t >= 0. Directions
  are unit vectors, so t == distance in metres.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

_EPS = 1e-9


@dataclass(frozen=True)
class Rect:
    """Axis-aligned rectangle; (x, y) is the bottom-left corner (y up)."""

    x: float
    y: float
    w: float
    h: float

    @property
    def min_x(self) -> float:
        return self.x

    @property
    def max_x(self) -> float:
        return self.x + self.w

    @property
    def min_y(self) -> float:
        return self.y

    @property
    def max_y(self) -> float:
        return self.y + self.h


@dataclass(frozen=True)
class Collision:
    """Circle-vs-rect overlap result.

    (nx, ny): unit normal pointing from the rectangle surface TOWARD the
              circle centre (the direction to push the circle to resolve
              the overlap).
    depth:    how far the circle currently overlaps, in metres.
    """

    nx: float
    ny: float
    depth: float
    rect: Rect


def ray_rect_distance(
    ox: float, oy: float, dx: float, dy: float, rect: Rect, max_t: float
) -> float | None:
    """Distance along a ray to its first intersection with an AABB.

    Classic "slab" method: the ray is inside the rectangle exactly when it
    is simultaneously inside the x-slab [min_x, max_x] and the y-slab
    [min_y, max_y]. For each axis compute the t-interval spent inside that
    slab, then intersect the intervals.

    Returns None when the ray misses or the hit lies beyond max_t.
    Returns 0.0 when the origin already lies inside the rectangle.
    """
    t_enter = 0.0
    t_exit = max_t

    for o, d, lo, hi in (
        (ox, dx, rect.min_x, rect.max_x),
        (oy, dy, rect.min_y, rect.max_y),
    ):
        if abs(d) < _EPS:
            # Parallel to this axis: inside the slab only if the origin is.
            if o < lo or o > hi:
                return None
        else:
            t1 = (lo - o) / d
            t2 = (hi - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            t_enter = max(t_enter, t1)
            t_exit = min(t_exit, t2)
            if t_enter > t_exit:
                return None

    return t_enter


def circle_rect_collision(
    cx: float, cy: float, radius: float, rect: Rect
) -> Collision | None:
    """Circle vs axis-aligned rectangle (closest-point test).

    The closest point on the rect to the circle centre is the centre
    clamped into the rectangle. If that point is farther than the radius,
    there is no overlap.
    """
    closest_x = min(max(cx, rect.min_x), rect.max_x)
    closest_y = min(max(cy, rect.min_y), rect.max_y)
    dx = cx - closest_x
    dy = cy - closest_y
    dist_sq = dx * dx + dy * dy

    if dist_sq > radius * radius:
        return None

    if dist_sq > _EPS * _EPS:
        dist = math.sqrt(dist_sq)
        return Collision(nx=dx / dist, ny=dy / dist, depth=radius - dist, rect=rect)

    # Degenerate case: the centre is inside (or exactly on the edge of) the
    # rectangle. Push out through the nearest face.
    left = cx - rect.min_x
    right = rect.max_x - cx
    bottom = cy - rect.min_y
    top = rect.max_y - cy
    m = min(left, right, bottom, top)
    if m == left:
        return Collision(-1.0, 0.0, radius + left, rect)
    if m == right:
        return Collision(1.0, 0.0, radius + right, rect)
    if m == bottom:
        return Collision(0.0, -1.0, radius + bottom, rect)
    return Collision(0.0, 1.0, radius + top, rect)
