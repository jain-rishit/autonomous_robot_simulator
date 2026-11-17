"""Simulated ultrasonic (sonar) range sensors.

Real sonar (e.g. an HC-SR04) emits an ultrasonic chirp, times the echo and
reports d = c * t / 2 with c ~ 343 m/s, out to ~4 m, through a wide beam
(~15-30 degrees cone). We skip the wave physics and model what the sensor
*reports*: the distance to the nearest surface inside the cone, corrupted
by small Gaussian noise -- real sonar readings are never perfectly clean.

Beam model: fire several rays across the cone and keep the SHORTEST
return, which is what a wide-beam echo physically corresponds to (the
nearest reflecting surface wins).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class UltrasonicSensor:
    """One sonar mounted at a fixed offset on the robot body."""

    name: str
    mount_angle: float  # rad, relative to the robot heading
    mount_x: float = 0.0  # mounting offset in the body frame (m)
    mount_y: float = 0.0
    max_range: float = 4.0  # m, beyond this the echo never returns
    beam_width: float = math.radians(25.0)  # cone opening
    noise_std: float = 0.02  # m, Gaussian measurement noise
    rays_per_beam: int = 5

    def read(
        self, world, pose: tuple[float, float, float], rng: np.random.Generator
    ) -> tuple[float, list[tuple[float, float, float, float]]]:
        """Return (noisy distance, per-ray geometry for drawing).

        Each ray tuple is (origin_x, origin_y, angle, distance).
        """
        x, y, theta = pose

        # Mounting offset rotated into the world frame (rigid-body transform).
        cos_t, sin_t = math.cos(theta), math.sin(theta)
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
        rays: list[tuple[float, float, float, float]] = []
        for off in offsets:
            angle = theta + self.mount_angle + off
            dist = world.raycast(sx, sy, angle, self.max_range)
            rays.append((sx, sy, angle, dist))
            if dist < best:
                best = dist

        # Measurement noise, clipped so readings stay physically plausible.
        noisy = best + float(rng.normal(0.0, self.noise_std))
        return min(max(noisy, 0.0), self.max_range), rays


class UltrasonicArray:
    """A ring of sonars around the robot, like a real avoidance stack."""

    def __init__(self, robot_radius: float = 0.5, seed: int = 7) -> None:
        self.rng = np.random.default_rng(seed)
        self.last_rays: list[tuple[float, float, float, float]] = []
        r = robot_radius
        d = r / math.sqrt(2.0)  # 45-degree mount offset distance
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

    def read_all(self, world, pose: tuple[float, float, float]) -> dict[str, float]:
        """Fresh readings from every sonar, keyed by sensor name."""
        readings: dict[str, float] = {}
        self.last_rays = []
        for sensor in self.sensors:
            dist, rays = sensor.read(world, pose, self.rng)
            readings[sensor.name] = dist
            self.last_rays.extend(rays)
        return readings
