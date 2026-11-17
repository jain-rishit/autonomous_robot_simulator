"""The pygame half of the visualization. It only ever *draws* the state
that robot/ and environment/ produce -- no physics in here, ever.

One thing to keep straight: the world is y-up in metres, the screen is
y-down in pixels. Every draw goes through to_screen(), which flips y and
scales. Keeping sim frame and screen frame separate is a discipline that
pays off the moment you add a camera or export data.
"""

import math

import pygame

SCALE = 40.0  # pixels per metre -> 24 m x 16 m arena = 960 x 640 px window

_BG = (16, 18, 24)
_GRID = (30, 34, 42)
_WALL = (58, 66, 74)
_OBSTACLE = (104, 114, 126)
_ROBOT_FILL = (0, 190, 170)
_ROBOT_EDGE = (0, 110, 100)
_HEADING = (255, 200, 60)
_RAY = (90, 170, 255)
_HIT = (255, 90, 70)
_DANGER = (255, 70, 70)
_TEXT = (222, 226, 235)
_DIM = (150, 158, 170)


class Renderer:
    def __init__(self, world, scale=SCALE):
        self.world = world
        self.scale = scale
        self.width_px = int(world.width * scale)
        self.height_px = int(world.height * scale)
        self.font = self._load_font(15)
        self.font_small = self._load_font(13)

    @staticmethod
    def _load_font(size):
        """Monospace font so HUD numbers line up, with a safe fallback."""
        for name in ("menlo", "monaco", "couriernew"):
            font = pygame.font.SysFont(name, size)
            if font is not None:
                return font
        return pygame.font.Font(None, size)

    def to_screen(self, x, y):
        """World metres (y-up) -> screen pixels (y-down)."""
        return int(x * self.scale), int(self.height_px - y * self.scale)

    def _rect_px(self, rect):
        left, top = self.to_screen(rect.min_x, rect.max_y)
        return pygame.Rect(
            left, top, int(rect.w * self.scale), int(rect.h * self.scale)
        )

    def draw(self, screen, robot, sonar, readings, fps):
        screen.fill(_BG)
        self._draw_grid(screen)
        self._draw_solids(screen)
        self._draw_sonar(screen, sonar)
        self._draw_robot(screen, robot)
        self._draw_hud(screen, robot, sonar, readings, fps)

    def _draw_grid(self, screen):
        """1 m grid, so distances are readable at a glance."""
        for gx in range(int(self.world.width) + 1):
            sx, _ = self.to_screen(gx, 0)
            pygame.draw.line(screen, _GRID, (sx, 0), (sx, self.height_px), 1)
        for gy in range(int(self.world.height) + 1):
            _, sy = self.to_screen(0, gy)
            pygame.draw.line(screen, _GRID, (0, sy), (self.width_px, sy), 1)

    def _draw_solids(self, screen):
        for rect in self.world.walls:
            pygame.draw.rect(screen, _WALL, self._rect_px(rect))
        for rect in self.world.obstacles:
            pygame.draw.rect(screen, _OBSTACLE, self._rect_px(rect))

    def _draw_sonar(self, screen, sonar):
        max_range = sonar.sensors[0].max_range
        for sx, sy, angle, dist in sonar.last_rays:
            start = self.to_screen(sx, sy)
            end = self.to_screen(
                sx + math.cos(angle) * dist, sy + math.sin(angle) * dist
            )
            pygame.draw.line(screen, _RAY, start, end, 1)
            if dist < max_range - 1e-6:
                pygame.draw.circle(screen, _HIT, end, 3)

    def _draw_robot(self, screen, robot):
        cx, cy = self.to_screen(robot.x, robot.y)
        r_px = robot.radius * self.scale

        # wheel axle: a thick line perpendicular to the heading
        perp_x = -math.sin(robot.theta)
        perp_y = -math.cos(robot.theta)
        axle_a = (int(cx + perp_x * r_px * 0.9), int(cy + perp_y * r_px * 0.9))
        axle_b = (int(cx - perp_x * r_px * 0.9), int(cy - perp_y * r_px * 0.9))
        pygame.draw.line(screen, _ROBOT_EDGE, axle_a, axle_b, 6)

        pygame.draw.circle(screen, _ROBOT_FILL, (cx, cy), int(r_px))
        pygame.draw.circle(screen, _ROBOT_EDGE, (cx, cy), int(r_px), 2)

        # heading arrow (screen y is flipped, hence the -sin)
        tip = (
            int(cx + math.cos(robot.theta) * r_px * 1.6),
            int(cy - math.sin(robot.theta) * r_px * 1.6),
        )
        pygame.draw.line(screen, _HEADING, (cx, cy), tip, 3)

        if robot.colliding:
            pygame.draw.circle(screen, _DANGER, (cx, cy), int(r_px) + 5, 2)

    def _draw_hud(self, screen, robot, sonar, readings, fps):
        lines = [
            "POSE   x=%6.2f m  y=%6.2f m  th=%6.1f deg"
            % (robot.x, robot.y, math.degrees(robot.theta)),
            "VEL    fwd=%5.2f  lat=%5.2f  yaw=%5.2f   (m/s, rad/s)"
            % (robot.vx, robot.vy, robot.omega),
            "ACC    fwd=%5.2f  lat=%5.2f  yaw=%5.2f   (m/s2, rad/s2)"
            % (robot.ax, robot.ay, robot.alpha),
            "DRIVE  %-12s BUMPS %d   FPS %d"
            % (
                "omni" if robot.drive == "omni" else "differential",
                robot.bumps,
                int(fps),
            ),
        ]
        y = 10
        for text in lines:
            screen.blit(self.font.render(text, True, _TEXT), (10, y))
            y += 20

        # sonar panel on the right: label + reading + distance bar
        max_range = sonar.sensors[0].max_range
        x0 = self.width_px - 250
        y = 10
        for name, dist in readings.items():
            frac = max(0.0, min(1.0, 1.0 - dist / max_range))  # 1 = touching
            label = self.font_small.render(f"{name:<11} {dist:4.2f} m", True, _TEXT)
            screen.blit(label, (x0, y))
            pygame.draw.rect(screen, _GRID, (x0, y + 16, 120, 6))
            colour = (int(255 * frac) + 30, int(200 * (1.0 - frac)) + 40, 70)
            pygame.draw.rect(screen, colour, (x0, y + 16, int(120 * frac), 6))
            y += 32

        help_text = (
            "W/S drive   A/D turn   Q/E strafe (omni)   O drive mode"
            "   SPACE brake   R reset   P plot   ESC quit"
        )
        screen.blit(
            self.font_small.render(help_text, True, _DIM),
            (10, self.height_px - 24),
        )
