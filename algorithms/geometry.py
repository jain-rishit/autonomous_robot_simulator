"""2D computational geometry, shared by the robot, the sonar and the world.

Conventions:
- rectangles are axis-aligned, stored as bottom-left corner (x, y) + w, h
- rays are p(t) = origin + t * direction with t >= 0; directions are unit
  vectors, so t is also the distance in metres
"""

import math

_EPS = 1e-9


class Rect:
    """An axis-aligned rectangle. (x, y) is the bottom-left corner (y up)."""

    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    @property
    def min_x(self):
        return self.x

    @property
    def max_x(self):
        return self.x + self.w

    @property
    def min_y(self):
        return self.y

    @property
    def max_y(self):
        return self.y + self.h


class Collision:
    """The result of a circle-vs-rect overlap test.

    (nx, ny) is a unit normal pointing from the wall surface back toward
    the circle centre -- i.e. the direction we push the circle to resolve
    the overlap. depth is how far it currently overlaps, in metres.
    """

    def __init__(self, nx, ny, depth, rect):
        self.nx = nx
        self.ny = ny
        self.depth = depth
        self.rect = rect


def ray_rect_distance(ox, oy, dx, dy, rect, max_t):
    """How far along the ray until it first hits an axis-aligned rect?

    Uses the classic slab method: a point is inside the rectangle exactly
    when it's inside the x-slab AND the y-slab. For each axis we work out
    the interval of t spent inside that slab, then intersect the intervals.

    Returns None if the ray misses (or the hit is past max_t), and 0.0 if
    the origin already sits inside the rectangle.
    """
    t_enter = 0.0
    t_exit = max_t

    for o, d, lo, hi in (
        (ox, dx, rect.min_x, rect.max_x),
        (oy, dy, rect.min_y, rect.max_y),
    ):
        if abs(d) < _EPS:
            # parallel to this axis: inside the slab only if we started there
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


def circle_rect_collision(cx, cy, radius, rect):
    """Circle vs axis-aligned rectangle, via the closest-point trick.

    The closest point on the rect to the circle centre is just the centre
    clamped into the rectangle. If that point is farther away than the
    radius, we're clear.
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

    # Corner case (literally): the centre is inside the rect or exactly on
    # its edge, so there's no meaningful normal. Push out through the
    # nearest face instead.
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
