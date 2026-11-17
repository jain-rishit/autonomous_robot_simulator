"""Week 1 entry point: a keyboard-teleoperated 2D mobile robot.

Run:            uv run main.py
Deterministic:  uv run python main.py --frames 600
"""

import argparse

import pygame

from environment.world import build_default_world
from robot.robot import Command, Robot
from robot.sensors import UltrasonicArray
from visualization.analysis import save_run_plot
from visualization.renderer import Renderer

SIM_HZ = 60  # fixed timestep: deterministic physics, stable tests
SIM_DT = 1.0 / SIM_HZ
SPAWN = (12.0, 8.0, 0.0)  # x [m], y [m], heading [rad] -- arena centre
PLOT_PATH = "run_analysis.png"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--frames",
        type=int,
        default=0,
        help="run exactly N frames then exit (0 = run until quit); handy "
        "for smoke-testing without a display",
    )
    parser.add_argument(
        "--seed", type=int, default=7, help="sonar noise seed (reproducibility)"
    )
    return parser.parse_args()


def keyboard_command(robot):
    """Map held keys to body-frame velocity targets.

    These are TARGETS only -- the actuator model inside Robot.update()
    walks the real velocity toward them at limited acceleration.
    """
    keys = pygame.key.get_pressed()
    forward = keys[pygame.K_w] or keys[pygame.K_UP]
    backward = keys[pygame.K_s] or keys[pygame.K_DOWN]
    turn_left = keys[pygame.K_a] or keys[pygame.K_LEFT]
    turn_right = keys[pygame.K_d] or keys[pygame.K_RIGHT]
    strafe_left = keys[pygame.K_q]
    strafe_right = keys[pygame.K_e]
    brake = keys[pygame.K_SPACE]

    cmd = Command()  # zero targets = coast to a stop
    if not brake:
        if forward:
            cmd.vx = robot.v_max
        elif backward:
            cmd.vx = -robot.v_reverse_max
        if turn_left:
            cmd.omega = robot.omega_max
        elif turn_right:
            cmd.omega = -robot.omega_max
        if strafe_left:
            cmd.vy = robot.strafe_max  # ignored by the diff-drive model
        elif strafe_right:
            cmd.vy = -robot.strafe_max
    return cmd


def main():
    args = parse_args()

    try:
        pygame.init()
        world = build_default_world()
        robot = Robot(*SPAWN)
        sonar = UltrasonicArray(robot.radius, seed=args.seed)
        renderer = Renderer(world)
        screen = pygame.display.set_mode((renderer.width_px, renderer.height_px))
    except pygame.error as exc:
        print(
            "Could not open a window (%s).\n"
            "Run this on the Mac itself, not over a headless SSH session." % exc
        )
        return 1

    pygame.display.set_caption("AI Autonomous Robot Simulator -- Week 1")
    clock = pygame.time.Clock()

    history = []
    t_sim = 0.0
    frame = 0
    running = True

    try:
        while running:
            # -- events (edge-triggered actions) --------------------------
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_r:
                        robot.reset()
                    elif event.key == pygame.K_o:
                        robot.drive = "omni" if robot.drive == "diff" else "diff"
                        print(f"drive mode -> {robot.drive}")
                    elif event.key == pygame.K_p:
                        path = save_run_plot(history, world, PLOT_PATH)
                        print(f"analysis plot -> {path}" if path else "nothing recorded yet")

            # -- control + physics (level-triggered, every tick) ----------
            robot.update(keyboard_command(robot), world, SIM_DT)
            readings = sonar.read_all(world, robot.pose)
            t_sim += SIM_DT
            frame += 1

            # record at ~10 Hz for the post-run matplotlib analysis
            if frame % 6 == 0:
                history.append(
                    {
                        "t": t_sim,
                        "x": robot.x,
                        "y": robot.y,
                        "theta": robot.theta,
                        "sonar": dict(readings),
                    }
                )

            # -- render ----------------------------------------------------
            renderer.draw(screen, robot, sonar, readings, clock.get_fps())
            pygame.display.flip()
            clock.tick(SIM_HZ)

            if args.frames and frame >= args.frames:
                running = False
    except KeyboardInterrupt:
        pass  # Ctrl-C in the terminal just stops the sim cleanly
    finally:
        pygame.quit()
        path = save_run_plot(history, world, PLOT_PATH)
        if path:
            print(f"run analysis saved -> {path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
