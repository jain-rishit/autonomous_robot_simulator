"""Pure math for a planar (2D) mobile robot.

Frames we use:
- world frame: metres, origin at the arena's bottom-left corner, x right,
  y up, angles counter-clockwise from +x
- body frame: bolted to the robot, x pointing forward, y pointing left

Nothing in here touches pygame or keeps state, which is exactly why these
functions are easy to test and easy to reuse.

Units: metres, seconds, radians, m/s, rad/s, m/s^2, rad/s^2.
"""

import math

TWO_PI = 2.0 * math.pi


def wrap_angle(angle):
    """Keep an angle inside (-pi, pi] so headings never drift out of range."""
    return (angle + math.pi) % TWO_PI - math.pi


def body_to_world(vx_body, vy_body, theta):
    """Rotate a body-frame velocity into the world frame.

    Plain old 2D rotation matrix:
        R(theta) = [[cos t, -sin t], [sin t, cos t]]
    """
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    return (
        vx_body * cos_t - vy_body * sin_t,
        vx_body * sin_t + vy_body * cos_t,
    )


def world_to_body(vx_world, vy_world, theta):
    """Rotate a world-frame velocity back into the body frame."""
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    return (
        vx_world * cos_t + vy_world * sin_t,
        -vx_world * sin_t + vy_world * cos_t,
    )


def ramp(current, target, max_rate, dt):
    """Move `current` toward `target`, but no faster than max_rate * dt.

    This one function IS the actuator model. The keyboard asks for a
    velocity; the robot can only approach it at a limited acceleration.
    That gap between "asked" and "managed" is what makes motion feel like
    hardware instead of a teleporting sprite.
    """
    max_step = max_rate * dt
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


def integrate_pose(x, y, theta, vx_body, vy_body, omega, dt):
    """Advance the pose by one fixed timestep (semi-implicit Euler).

    The unicycle model (and, once vy is allowed to move, the omni model):

        x_dot = vx cos(theta) - vy sin(theta)
        y_dot = vx sin(theta) + vy cos(theta)
        theta_dot = omega

    We update theta first and then move with the *new* heading. On turning
    arcs that follows the true curve noticeably better than the naive
    order of updates.
    """
    theta_new = wrap_angle(theta + omega * dt)
    vx_w, vy_w = body_to_world(vx_body, vy_body, theta_new)
    return x + vx_w * dt, y + vy_w * dt, theta_new
