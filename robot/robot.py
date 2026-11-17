"""The virtual mobile robot: state, actuator model and collision response.

Velocities live in the body frame:
    vx    forward speed  (m/s)
    vy    sideways speed (m/s)
    omega turn rate      (rad/s)

Two drive types share one model:

- "diff": a classic differential-drive robot (two driven wheels + a
  caster). It can drive and turn, but it cannot slide sideways, so vy is
  pinned to 0 no matter what you command. TurtleBots and Roombas work
  exactly like this.
- "omni": omni/mecanum wheels, which can genuinely strafe, so vy is free.

Note that the keyboard never sets velocity directly. It asks for velocity
targets, and update() walks the real velocity toward them at a limited
acceleration. That ramp is why the robot feels like it has mass.
"""

from algorithms.kinematics import body_to_world, integrate_pose, ramp, world_to_body


class Command:
    """Body-frame velocity targets for one tick -- what the driver wants."""

    def __init__(self, vx=0.0, vy=0.0, omega=0.0):
        self.vx = vx
        self.vy = vy
        self.omega = omega


class Robot:
    """A circular diff/omni-drive robot with sonar and bump handling."""

    def __init__(self, x, y, theta=0.0, radius=0.5, drive="diff"):
        if drive not in ("diff", "omni"):
            raise ValueError(f"unknown drive type: {drive!r}")
        if radius <= 0.0:
            raise ValueError("radius must be positive")

        self.home = (x, y, theta)  # remembered for reset()
        self.x = x
        self.y = y
        self.theta = theta

        # body-frame velocity, plus acceleration for the HUD
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.alpha = 0.0

        self.radius = radius
        self.drive = drive

        # actuator limits, tuned to feel like a small indoor robot
        self.v_max = 3.0          # top forward speed
        self.v_reverse_max = 1.5  # reverse is slower, like real robots
        self.strafe_max = 1.5     # sideways speed, omni mode only
        self.omega_max = 2.5      # about 143 deg/s
        self.a_max = 4.0          # linear acceleration limit
        self.alpha_max = 8.0      # angular acceleration limit

        self.restitution = 0.0    # 0 = dead stop into walls, slide along
        self.colliding = False    # True on ticks with a wall contact
        self.bumps = 0
        self._was_colliding = False

    def update(self, cmd, world, dt):
        """One tick: actuate, integrate, resolve collisions."""
        if dt <= 0.0:
            raise ValueError(f"dt must be positive, got {dt}")

        prev_vx = self.vx
        prev_vy = self.vy
        prev_omega = self.omega

        # 1) actuator model: walk the real velocity toward the targets
        vx_target = max(-self.v_reverse_max, min(self.v_max, cmd.vx))
        omega_target = max(-self.omega_max, min(self.omega_max, cmd.omega))
        if self.drive == "diff":
            vy_target = 0.0  # nonholonomic: no sideways sliding allowed
        else:
            vy_target = max(-self.strafe_max, min(self.strafe_max, cmd.vy))

        self.vx = ramp(self.vx, vx_target, self.a_max, dt)
        self.vy = ramp(self.vy, vy_target, self.a_max, dt)
        self.omega = ramp(self.omega, omega_target, self.alpha_max, dt)

        # 2) integrate the pose
        self.x, self.y, self.theta = integrate_pose(
            self.x, self.y, self.theta, self.vx, self.vy, self.omega, dt
        )

        # 3) collisions: push out of whatever we're inside, then keep only
        #    the velocity component parallel to the surface (slide, don't
        #    stick). A few passes so corners settle properly.
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

        # 4) acceleration actually applied this tick, for the HUD
        self.ax = (self.vx - prev_vx) / dt
        self.ay = (self.vy - prev_vy) / dt
        self.alpha = (self.omega - prev_omega) / dt

    def _kill_normal_velocity(self, hit):
        """Remove (or bounce off, given restitution) velocity into the wall."""
        vx_w, vy_w = body_to_world(self.vx, self.vy, self.theta)
        vn = vx_w * hit.nx + vy_w * hit.ny
        if vn < 0.0:  # moving INTO the surface
            vx_w -= (1.0 + self.restitution) * vn * hit.nx
            vy_w -= (1.0 + self.restitution) * vn * hit.ny
            self.vx, self.vy = world_to_body(vx_w, vy_w, self.theta)
        self.omega *= 0.5  # damp the spin so we don't grind on corners

    @property
    def speed(self):
        return (self.vx * self.vx + self.vy * self.vy) ** 0.5

    @property
    def pose(self):
        return self.x, self.y, self.theta

    def reset(self):
        """Back to the spawn pose, motion state cleared."""
        self.x, self.y, self.theta = self.home
        self.vx = self.vy = self.omega = 0.0
        self.ax = self.ay = self.alpha = 0.0
        self.colliding = False
        self.bumps = 0
        self._was_colliding = False
