# AI Autonomous Robot Simulator — Week 1

A 2D virtual mobile robot you drive from the keyboard: differential-drive
(or omni) kinematics, acceleration-limited actuators, wall collision
handling, and a six-sensor ultrasonic array — no hardware required.

## Quickstart (uv only)

```bash
uv sync                 # create/refresh .venv from uv.lock
uv run main.py          # launch the simulator window
uv run pytest           # run the 30 unit tests
```

First-time setup (already done in this repo):

```bash
uv init --bare --python 3.12
uv python pin 3.12
uv python install 3.12
uv add pygame numpy matplotlib
uv add --dev pytest
```

## Controls

| Key           | Action                                    |
| ------------- | ----------------------------------------- |
| `W` / `S`     | forward / backward                        |
| `A` / `D`     | turn left / right (CCW / CW)              |
| `Q` / `E`     | strafe left / right (omni mode only)      |
| `O`           | toggle differential ↔ omni drive          |
| `SPACE`       | brake (zero velocity targets)             |
| `R`           | reset robot to spawn                      |
| `P`           | save `run_analysis.png`                   |
| `ESC`         | quit (also saves the analysis plot)       |

## Project layout

```
main.py                  entry point: game loop, keyboard -> Command mapping
algorithms/kinematics.py pure math: pose integration, rotation, accel ramp
algorithms/geometry.py   pure math: ray-vs-AABB (slab method), circle-vs-AABB
environment/world.py     arena, border walls, obstacles; collide/raycast queries
robot/robot.py           robot state, actuator model, collision response
robot/sensors.py         ultrasonic sensor model (cone + ray casting + noise)
visualization/renderer.py  pygame drawing + HUD (no physics)
visualization/analysis.py  matplotlib post-run plots (no pygame)
ai/                      empty on purpose — Week 2 autonomy goes here
tests/                   pytest suite (headless, no pygame needed)
```

Architecture rule: **simulation core is pygame-free.** `main.py` and
`visualization/renderer.py` are the only files that touch pygame, so the
physics, sensors and future AI all run headless in CI and tests.

## Physics model

* Pose `(x, y, θ)` in metres/radians, world frame y-up.
* Body-frame velocity `(vx, vy, ω)`; commands are velocity *targets* ramped
  at limited acceleration (`a_max`, `alpha_max`).
* Differential drive is nonholonomic: lateral velocity pinned to 0.
* Integration: semi-implicit Euler at a fixed 60 Hz timestep.
* Collisions: circle-vs-AABB closest-point test, positional push-out along
  the contact normal, tangential velocity preserved (robot slides, not sticks).
* Sonar: 5 rays per 25° cone, shortest return wins, Gaussian noise,
  max range 4 m.

## Week 2 preview

`ai/` will host controllers that consume `readings` and emit `robot.Command`
objects — same contract as the keyboard driver. First up: reactive obstacle
avoidance (potential fields / bug algorithm).
