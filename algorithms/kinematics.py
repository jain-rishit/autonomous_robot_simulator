"""Pure math for a planar (2D) mobile robot.

Coordinate frames
-----------------
* World frame: right-handed, metres, origin at the arena's bottom-left
  corner, x -> right, y -> UP, angles counter-clockwise from +x.
* Body frame: fixed to the robot; x points FORWARD, y points to the LEFT.

All functions are pure: no pygame, no global state. That makes them trivial
to unit-test and lets the exact same math run headless (CI) or in the GUI.

Units: metres, seconds, radians, m/s, rad/s, m/s^2, rad/s^2.
"""

from __future__ import annotations

import math

TWO_PI = 2.0 * math.pi


def wrap_angle(angle: float) -> float:
    """Wrap an angle into (-pi, pi].

    Keeps the heading bounded so it never loses precision as the robot
    spins, and makes angle comparisons sane.
    """
    return (angle + math.pi) % TWO_PI - math.pi


def body_to_world(vx_body: float, vy_body: float, theta: float) -> tuple[float, float]:
    """Rotate a body-frame velocity into the world frame (2D rotation matrix).

    R(theta) = [[cos t, -sin t], [sin t, cos t]]
    """
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    return (
        vx_body * cos_t - vy_body * sin_t,
        vx_body * sin_t + vy_body * cos_t,
    )


def world_to_body(vx_world: float, vy_world: float, theta: float) -> tuple[float, float]:
    """Rotate a world-frame velocity into the body frame (inverse rotation)."""
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    return (
        vx_world * cos_t + vy_world * sin_t,
        -vx_world * sin_t + vy_world * cos_t,
    )


def ramp(current: float, target: float, max_rate: float, dt: float) -> float:
    """Step `current` toward `target`, moving at most `max_rate * dt`.

    This is the whole actuator model: commands are *targets* that the
    robot can only approach at limited acceleration, which is what makes
    motion feel like real hardware instead of a teleporting sprite.
    """
    max_step = max_rate * dt
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


def integrate_pose(
    x: float,
    y: float,
    theta: float,
    vx_body: float,
    vy_body: float,
    omega: float,
    dt: float,
) -> tuple[float, float, float]:
    """Advance the pose one fixed timestep (semi-implicit Euler).

    Unicycle (and, with vy free, omnidirectional) kinematics:

        x_dot = vx cos(theta) - vy sin(theta)
        y_dot = vx sin(theta) + vy cos(theta)
        theta_dot = omega

    We update theta FIRST, then integrate the position with the new
    heading: on turning arcs this tracks the true curve noticeably better
    than updating everything with the stale heading.
    """
    theta_new = wrap_angle(theta + omega * dt)
    vx_w, vy_w = body_to_world(vx_body, vy_body, theta_new)
    return x + vx_w * dt, y + vy_w * dt, theta_new
