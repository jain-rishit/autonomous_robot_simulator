# AI Autonomous Robot Simulator — Week 1

> *"Hardware is just software that got too expensive to `git revert`."*

A 2D virtual mobile robot you drive from the keyboard: differential-drive
(or omni) kinematics, acceleration-limited actuators, wall collision
handling, and a six-sensor ultrasonic array — **no hardware required**.

What do you get when you combine a MacBook M4, a hardware budget of $0,
and an unhealthy amount of enthusiasm? This. Somewhere, a Roomba is doing
the same thing with 1/100th the code and zero unit tests. Respect.

> *"My robot never crashes into walls. It passionately hugs them — and
> then slides along them, because that's the physics I wrote."*

## Quickstart (uv only — we don't talk about pip here)

```bash
uv sync                 # create/refresh .venv from uv.lock
uv run main.py          # launch the simulator window
uv run pytest           # run the 30 unit tests (they pass, we checked)
```

> *"It works on my machine."*
> — Yes. That's the point. It's a *simulator*. The machine is the point.

First-time setup (already done in this repo, so you can feel superior):

```bash
uv init --bare --python 3.12
uv python pin 3.12
uv python install 3.12
uv add pygame numpy matplotlib
uv add --dev pytest
```

## Controls

Press keys. Robot moves. Physics does the rest, reluctantly.

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

> *"There are two hard problems in robotics: collision avoidance, naming
> things, and off-by-one errors."*

Pro tip: press `Q` in differential mode and nothing happens. That's not a
bug — that's a **nonholonomic constraint**, and now you sound smart at
parties.

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

> *"Premature optimization is the root of all evil — but premature
> abstraction is the root of all empty `ai/` folders."*

## Physics model

* Pose `(x, y, θ)` in metres/radians, world frame y-up. A robot is not a
  dot; a robot is a dot **with opinions about which way it's facing**.
* Body-frame velocity `(vx, vy, ω)`; commands are velocity *targets* ramped
  at limited acceleration (`a_max`, `alpha_max`) — because real motors have
  the audacity to obey the laws of physics.
* Differential drive is nonholonomic: lateral velocity pinned to 0. Wheels
  don't strafe. Neither should your code.
* Integration: semi-implicit Euler at a fixed 60 Hz timestep. Deterministic,
  so the unit tests can assert exact distances without hosting a séance.
* Collisions: circle-vs-AABB closest-point test, positional push-out along
  the contact normal, tangential velocity preserved (robot slides, not
  sticks — like a penguin on ice, but with more math).
* Sonar: 5 rays per 25° cone, shortest return wins, Gaussian noise, max
  range 4 m. The sensor lies a little (±2 cm), like all of us.

> *"In theory, theory and practice are the same. In practice, the wall is
> exactly where the ray cast said it would be."*

## Week 2 preview

`ai/` will host controllers that consume `readings` and emit `robot.Command`
objects — same contract as the keyboard driver. First up: reactive obstacle
avoidance (potential fields / bug algorithm).

That's the moment the robot stops needing you. We're proud and slightly
terrified.

> *"First, solve the problem. Then, write the code. Then, write the tests
> that prove the wall is still there."*

---

*No robots were harmed in the making of this simulator. The walls cannot
say the same.*
