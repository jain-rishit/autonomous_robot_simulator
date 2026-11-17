"""Simulated ultrasonic (sonar) range sensors.

A real sonar (think HC-SR04) fires an ultrasonic chirp, times the echo and
reports d = c * t / 2 with c ~ 343 m/s, out to about 4 m, through a wide
beam (15-30 degrees). We skip the wave physics and model what the sensor
actually reports: the distance to the nearest surface inside the cone,
with a bit of Gaussian noise on top, because real sonar is never clean.

Beam model: fire a few rays across the cone and keep the shortest return.
That's what a wide beam physically does -- the nearest reflecting surface
wins the echo.
"""

import math

import numpy as np


class UltrasonicSensor:
    """One sonar, mounted at a fixed offset on the robot body."""

    def __init__(
        self,
        name,
        mount_angle,
        mount_x=0.0,
        mount_y=0.0,
        max_range=4.0,
        beam_width=math.radians(25.0),
        noise_std=0.02,
        rays_per_beam=5,
    ):
        self.name = name
        self.mount_angle = mount_angle  # relative to the robot heading
        self.mount_x = mount_x
        self.mount_y = mount_y
        self.max_range = max_range
        self.beam_width = beam_width
        self.noise_std = noise_std
        self.rays_per_beam = rays_per_beam

    def read(self, world, pose, rng):
        """Return (noisy distance, per-ray geometry for drawing).

        Each ray is a (origin_x, origin_y, angle, distance) tuple.
        """
        x, y, theta = pose

        # mount offset rotated into the world frame
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        sx = x + self.mount_x * cos_t - self.mount_y * sin_t
        sy = y + self.mount_x * sin_t + self.mount_y * cos_t

        if self.rays_per_beam <= 1:
            offsets = (0.0,)
        else:
            step = self.beam_width / (self.rays_per_beam - 1)
            offsets = tuple(
                -self.beam_width / 2.0 + i * step for i in range(self.rays_per_beam)
            )

        best = self.max_range
        rays = []
        for off in offsets:
            angle = theta + self.mount_angle + off
            dist = world.raycast(sx, sy, angle, self.max_range)
            rays.append((sx, sy, angle, dist))
            if dist < best:
                best = dist

        # measurement noise, clipped so readings stay physically plausible
        noisy = best + float(rng.normal(0.0, self.noise_std))
        return min(max(noisy, 0.0), self.max_range), rays


class UltrasonicArray:
    """A ring of sonars around the robot, like a real avoidance stack."""

    def __init__(self, robot_radius=0.5, seed=7):
        self.rng = np.random.default_rng(seed)
        self.last_rays = []
        r = robot_radius
        d = r / math.sqrt(2.0)  # how far out a 45-degree mount actually sits
        half = math.pi / 2.0
        quarter = math.pi / 4.0
        self.sensors = [
            UltrasonicSensor("FRONT", 0.0, mount_x=r),
            UltrasonicSensor("FRONT_LEFT", quarter, mount_x=d, mount_y=d),
            UltrasonicSensor("FRONT_RIGHT", -quarter, mount_x=d, mount_y=-d),
            UltrasonicSensor("LEFT", half, mount_y=r),
            UltrasonicSensor("RIGHT", -half, mount_y=-r),
            UltrasonicSensor("BACK", math.pi, mount_x=-r),
        ]

    def read_all(self, world, pose):
        """Fresh readings from every sonar, keyed by sensor name."""
        readings = {}
        self.last_rays = []
        for sensor in self.sensors:
            dist, rays = sensor.read(world, pose, self.rng)
            readings[sensor.name] = dist
            self.last_rays.extend(rays)
        return readings
