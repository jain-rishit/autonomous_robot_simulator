"""Unit tests for collision geometry and the world queries."""

from __future__ import annotations

import math

from algorithms.geometry import Rect, circle_rect_collision, ray_rect_distance
from environment.world import build_default_world

_BOX = Rect(4.0, 4.0, 2.0, 2.0)  # x in [4, 6], y in [4, 6]


def test_ray_hits_rect_at_expected_distance():
    d = ray_rect_distance(0.0, 5.0, 1.0, 0.0, _BOX, 100.0)
    assert abs(d - 4.0) < 1e-9


def test_ray_missing_the_rect_returns_none():
    d = ray_rect_distance(0.0, 5.0, 0.0, 1.0, _BOX, 100.0)  # straight up at x=0
    assert d is None


def test_ray_origin_inside_the_rect_returns_zero():
    assert ray_rect_distance(5.0, 5.0, 1.0, 0.0, _BOX, 100.0) == 0.0


def test_ray_hit_beyond_max_t_returns_none():
    assert ray_rect_distance(0.0, 5.0, 1.0, 0.0, _BOX, 3.0) is None


def test_circle_rect_collision_depth_and_normal():
    hit = circle_rect_collision(3.5, 5.0, 1.0, _BOX)
    assert hit is not None
    assert abs(hit.nx + 1.0) < 1e-9  # normal points back toward the circle
    assert abs(hit.ny) < 1e-9
    assert abs(hit.depth - 0.5) < 1e-9


def test_circle_far_from_rect_does_not_collide():
    assert circle_rect_collision(0.0, 0.0, 0.5, _BOX) is None


def test_circle_centre_inside_rect_pushes_out():
    hit = circle_rect_collision(5.0, 5.0, 0.5, _BOX)
    assert hit is not None
    assert hit.depth > 0.5  # deeper than the radius itself


def test_default_world_raycast_distances():
    world = build_default_world()
    # Due right from the arena centre: the pillar's top edge lies exactly on
    # y = 8, so the ray strikes its x = 15.5 face at 3.5 m.
    assert abs(world.raycast(12.0, 8.0, 0.0, 100.0) - 3.5) < 1e-6
    # Due left: the tall slab's right face (x = 6.2) intercepts first;
    # its y-range [4, 10] covers y = 8.
    assert abs(world.raycast(12.0, 8.0, math.pi, 100.0) - 5.8) < 1e-6
    # Due down: open floor until the bottom border wall face at y = 0.
    assert abs(world.raycast(12.0, 8.0, -math.pi / 2, 100.0) - 8.0) < 1e-6


def test_default_world_detects_border_overlap():
    world = build_default_world()
    hit = world.collide_circle(23.8, 8.0, 0.5)
    assert hit is not None
    assert abs(hit.nx + 1.0) < 1e-9
    assert abs(hit.depth - 0.3) < 1e-9


def test_default_world_clear_space_has_no_collision():
    world = build_default_world()
    assert world.collide_circle(12.0, 8.0, 0.5) is None
