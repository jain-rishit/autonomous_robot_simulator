"""The virtual mobile robot: state, actuator model, physics update.

Velocity state is kept in the *body frame*:

    vx    forward speed  (m/s)
    vy    lateral speed  (m/s)
    omega turn rate      (rad/s)

One model covers both drive types we simulate:

* "diff" -- differential drive (two driven wheels + a caster). The classic
  nonholonomic mobile robot: it can drive and turn but CANNOT slide
  sideways, so vy is pinned to 0 no matter what you command. TurtleBots,
  Roombas and most AGVs behave this way.
* "omni" -- omni/mecanum wheels, which allow true sideways strafing (vy free).

Keyboard input only ever sets *velocity targets*; the actuator model ramps
the actual velocities toward them at limited acceleration. That is what
makes the robot feel like heavy hardware instead of a teleporting sprite.
"""

from __future__ import annotations

from dataclasses import dataclass

from algorithms.geometry import Collision
from algorithms.kinematics import body_to_world, integrate_pose, ramp, world_to_body


@dataclass
class Command:
    """Body-frame velocity targets for one tick (what the driver 'wants')."""

    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0


class Robot:
    """A circular diff/omni-drive mobile robot with collision handling."""

    def __init__(
        self,
        x: float,
        y: float,
        theta: float = 0.0,
        radius: float = 0.5,
        drive: str = "diff",
    ) -> None:
        if drive not in ("diff", "omni"):
            raise ValueError(f"unknown drive type: {drive!r}")
        if radius <= 0.0:
            raise ValueError("radius must be positive")

        self.home = (x, y, theta)  # remembered for reset()
        self.x = x
        self.y = y
        self.theta = theta

        # Body-frame velocity and acceleration (accel is for the HUD).
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.alpha = 0.0

        self.radius = radius
        self.drive = drive

        # Actuator limits, tuned to feel like a small indoor robot.
        self.v_max = 3.0          # m/s, top forward speed
        self.v_reverse_max = 1.5  # m/s, reverse is slower (like real robots)
        self.strafe_max = 1.5     # m/s, sideways speed in omni mode
        self.omega_max = 2.5      # rad/s (~143 deg/s)
        self.a_max = 4.0          # m/s^2 linear acceleration limit
        self.alpha_max = 8.0      # rad/s^2 angular acceleration limit

        self.restitution = 0.0    # 0 = dead stop into walls (slides along)
        self.colliding = False    # True on ticks where a wall contact exists
        self.bumps = 0
        self._was_colliding = False

    # ------------------------------------------------------------------ #
    # Simulation step
    # ------------------------------------------------------------------ #
    def update(self, cmd: Command, world, dt: float) -> None:
        """One fixed-timestep tick: actuate, integrate, resolve collisions."""
        if dt <= 0.0:
            raise ValueError(f"dt must be positive, got {dt}")

        prev_vx, prev_vy, prev_omega = self.vx, self.vy, self.omega

        # 1) Actuator model: ramp actual velocity toward the command targets.
        vx_target = max(-self.v_reverse_max, min(self.v_max, cmd.vx))
        omega_target = max(-self.omega_max, min(self.omega_max, cmd.omega))
        if self.drive == "diff":
            vy_target = 0.0  # nonholonomic constraint: no sideways sliding
        else:
            vy_target = max(-self.strafe_max, min(self.strafe_max, cmd.vy))

        self.vx = ramp(self.vx, vx_target, self.a_max, dt)
        self.vy = ramp(self.vy, vy_target, self.a_max, dt)
        self.omega = ramp(self.omega, omega_target, self.alpha_max, dt)

        # 2) Integrate the pose (semi-implicit Euler, heading first).
        self.x, self.y, self.theta = integrate_pose(
            self.x, self.y, self.theta, self.vx, self.vy, self.omega, dt
        )

        # 3) Resolve collisions: push out of any solid, then keep only the
        #    velocity component parallel to the surface (slide, don't stick).
        contact = False
        for _ in range(4):
            hit = world.collide_circle(self.x, self.y, self.radius)
            if hit is None:
                break
            contact = True
            self.x += hit.nx * hit.depth
            self.y += hit.ny * hit.depth
            self._kill_normal_velocity(hit)
        self.colliding = contact
        if contact and not self._was_colliding:
            self.bumps += 1
        self._was_colliding = contact

        # 4) Acceleration actually applied this tick (HUD display).
        self.ax = (self.vx - prev_vx) / dt
        self.ay = (self.vy - prev_vy) / dt
        self.alpha = (self.omega - prev_omega) / dt

    def _kill_normal_velocity(self, hit: Collision) -> None:
        """Remove (or bounce, via restitution) velocity into the surface."""
        vx_w, vy_w = body_to_world(self.vx, self.vy, self.theta)
        vn = vx_w * hit.nx + vy_w * hit.ny
        if vn < 0.0:  # moving INTO the surface
            vx_w -= (1.0 + self.restitution) * vn * hit.nx
            vy_w -= (1.0 + self.restitution) * vn * hit.ny
            self.vx, self.vy = world_to_body(vx_w, vy_w, self.theta)
        self.omega *= 0.5  # damp spin so we don't grind against corners

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    @property
    def speed(self) -> float:
        return (self.vx * self.vx + self.vy * self.vy) ** 0.5

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.theta

    def reset(self) -> None:
        """Restore the spawn pose and clear all motion state."""
        self.x, self.y, self.theta = self.home
        self.vx = self.vy = self.omega = 0.0
        self.ax = self.ay = self.alpha = 0.0
        self.colliding = False
        self.bumps = 0
        self._was_colliding = False
