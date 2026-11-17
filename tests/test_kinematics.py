"""Unit tests for the pure kinematics math."""

from __future__ import annotations

import math

from algorithms.kinematics import (
    body_to_world,
    integrate_pose,
    ramp,
    world_to_body,
    wrap_angle,
)


def test_wrap_angle_maps_into_half_open_range():
    for angle in (-10.0, -3.5, 0.2, 7.0, 100.0):
        assert -math.pi - 1e-9 < wrap_angle(angle) <= math.pi + 1e-9


def test_wrap_angle_equivalent_angles_agree():
    assert abs(wrap_angle(3 * math.pi)) - math.pi < 1e-9
    assert abs(wrap_angle(-3 * math.pi / 2) - math.pi / 2) < 1e-9


def test_forward_body_velocity_becomes_world_y_at_90_degrees():
    vx_w, vy_w = body_to_world(1.0, 0.0, math.pi / 2)
    assert abs(vx_w) < 1e-9
    assert abs(vy_w - 1.0) < 1e-9


def test_world_to_body_is_the_inverse_rotation():
    wx, wy, theta = 2.0, -1.0, 0.7
    bx, by = world_to_body(wx, wy, theta)
    wx2, wy2 = body_to_world(bx, by, theta)
    assert abs(wx2 - wx) < 1e-9
    assert abs(wy2 - wy) < 1e-9


def test_ramp_limits_the_step_size():
    assert ramp(0.0, 10.0, 2.0, 0.1) == 0.2
    assert ramp(0.0, -10.0, 2.0, 0.1) == -0.2


def test_ramp_never_overshoots_the_target():
    assert ramp(0.0, 0.1, 2.0, 0.1) == 0.1


def test_integrate_straight_line():
    x, y, theta = integrate_pose(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1)
    assert abs(x - 0.1) < 1e-9
    assert abs(y) < 1e-9
    assert abs(theta) < 1e-9


def test_integrate_pure_rotation_keeps_position():
    x, y, theta = integrate_pose(1.0, 2.0, 0.0, 0.0, 0.0, math.pi / 2, 0.5)
    assert abs(x - 1.0) < 1e-9
    assert abs(y - 2.0) < 1e-9
    assert abs(theta - math.pi / 4) < 1e-9
