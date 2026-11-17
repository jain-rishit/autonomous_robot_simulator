"""Integration tests: robot motion, collision response, sonar, reset."""

from __future__ import annotations

import math

from environment.world import build_default_world
from robot.robot import Command, Robot
from robot.sensors import UltrasonicArray


def step(robot: Robot, world, cmd: Command, n: int = 100, dt: float = 0.01) -> None:
    for _ in range(n):
        robot.update(cmd, world, dt)


def test_forward_motion_accumulates_with_acceleration_limit():
    world = build_default_world()
    robot = Robot(12.0, 8.0, 0.0)
    step(robot, world, Command(vx=1.0))
    # Ramping 0 -> 1 m/s at 4 m/s^2 then cruising: analytic distance 0.88 m.
    assert 0.87 < robot.x - 12.0 < 0.89
    assert robot.y == 8.0  # heading was 0, so no lateral drift at all


def test_differential_drive_cannot_strafe():
    world = build_default_world()
    robot = Robot(12.0, 8.0, 0.0, drive="diff")
    step(robot, world, Command(vy=1.0), n=10)
    assert robot.vy == 0.0
    assert robot.y == 8.0


def test_omni_drive_can_strafe():
    world = build_default_world()
    robot = Robot(12.0, 8.0, 0.0, drive="omni")
    step(robot, world, Command(vy=1.0), n=10)
    assert robot.vy > 0.0
    assert robot.y > 8.0
    assert robot.x == 12.0


def test_turning_changes_heading_not_position():
    world = build_default_world()
    robot = Robot(12.0, 8.0, 0.0)
    step(robot, world, Command(omega=math.pi / 2), n=100)
    assert robot.theta > 0.0
    assert abs(robot.x - 12.0) < 1e-9 and abs(robot.y - 8.0) < 1e-9


def test_collision_stops_robot_at_the_wall():
    world = build_default_world()
    robot = Robot(21.0, 8.0, 0.0)  # right border wall face is at x = 24
    step(robot, world, Command(vx=3.0), n=200)  # 2 s at up to 3 m/s
    assert robot.x <= 23.5 + 1e-6  # centre never penetrates: 24 - radius
    assert abs(robot.vx) < 0.05  # velocity into the wall was removed
    assert robot.bumps >= 1


def test_robot_never_leaves_the_arena():
    world = build_default_world()
    robot = Robot(12.0, 8.0, 0.7)
    # Slammed into every direction for 20 simulated seconds.
    for cmd in (
        Command(vx=3.0),
        Command(vx=-1.5, omega=1.0),
        Command(vx=3.0, omega=-1.0),
        Command(vx=-1.5),
        Command(omega=2.0),
    ):
        step(robot, world, cmd, n=200)
    assert robot.radius <= robot.x <= world.width - robot.radius
    assert robot.radius <= robot.y <= world.height - robot.radius


def test_sonar_reads_expected_wall_distance():
    world = build_default_world()
    robot = Robot(2.0, 8.0, 0.0)
    sonar = UltrasonicArray(robot.radius, seed=1)
    readings = sonar.read_all(world, robot.pose)
    # FRONT sonar sits on the nose (x = 2.5) facing +x; the tall slab
    # (x = 5.0) is straight ahead -> 2.5 m plus small noise.
    assert abs(readings["FRONT"] - 2.5) < 0.15


def test_sonar_returns_max_range_in_open_space():
    world = build_default_world()
    robot = Robot(12.0, 8.0, 0.0)
    sonar = UltrasonicArray(robot.radius, seed=1)
    readings = sonar.read_all(world, robot.pose)
    # RIGHT sonar looks at open floor: no echo within 4 m.
    assert 3.9 <= readings["RIGHT"] <= 4.0


def test_sonar_keys_and_bounds():
    world = build_default_world()
    sonar = UltrasonicArray(0.5, seed=3)
    readings = sonar.read_all(world, (12.0, 8.0, 0.0))
    assert set(readings) == {
        "FRONT", "FRONT_LEFT", "FRONT_RIGHT", "LEFT", "RIGHT", "BACK",
    }
    for dist in readings.values():
        assert 0.0 <= dist <= 4.0


def test_sonar_noise_is_deterministic_for_a_seed():
    world = build_default_world()
    a = UltrasonicArray(0.5, seed=42).read_all(world, (12.0, 8.0, 0.0))
    b = UltrasonicArray(0.5, seed=42).read_all(world, (12.0, 8.0, 0.0))
    assert a == b


def test_reset_restores_spawn_state():
    world = build_default_world()
    robot = Robot(12.0, 8.0, 0.0)
    step(robot, world, Command(vx=2.0, omega=1.0), n=50)
    robot.reset()
    assert robot.pose == (12.0, 8.0, 0.0)
    assert robot.vx == robot.vy == robot.omega == 0.0
    assert robot.bumps == 0


def test_invalid_drive_type_is_rejected():
    try:
        Robot(0.0, 0.0, drive="hovercraft")
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for unknown drive type")
