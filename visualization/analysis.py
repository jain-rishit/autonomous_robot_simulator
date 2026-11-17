"""Post-run analysis plots (matplotlib, Agg backend -> writes PNG files).

pygame is for *looking* at the simulation in real time; matplotlib is for
*measuring* it afterwards. Same split as ROS bags + rqt_plot. This module
never touches pygame, so it also runs headless (CI, tests).
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")  # no window; render straight to file

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def save_run_plot(history, world, out_path: str) -> str | None:
    """Write trajectory + sonar-vs-time plots for a recorded run.

    history: list of dicts with keys t, x, y, theta, sonar (dict of ranges).
    Returns the output path, or None when there is nothing to plot.
    """
    if len(history) < 2:
        return None

    fig, (ax_path, ax_sonar) = plt.subplots(1, 2, figsize=(14, 6))

    # --- Trajectory -----------------------------------------------------
    xs = [h["x"] for h in history]
    ys = [h["y"] for h in history]
    ax_path.plot(xs, ys, color="#00beaa", lw=1.4, label="path")
    ax_path.plot(xs[0], ys[0], "o", color="#22c55e", ms=8, label="start")
    ax_path.plot(xs[-1], ys[-1], "X", color="#ef4444", ms=9, label="end")
    for rect in world.solids:
        ax_path.add_patch(
            Rectangle((rect.x, rect.y), rect.w, rect.h, facecolor="0.72",
                      edgecolor="0.4")
        )
    ax_path.set_xlim(-1.0, world.width + 1.0)
    ax_path.set_ylim(-1.0, world.height + 1.0)
    ax_path.set_aspect("equal")
    ax_path.set_xlabel("x [m]")
    ax_path.set_ylabel("y [m]")
    ax_path.set_title("Trajectory (world frame, y up)")
    ax_path.legend(loc="upper right", fontsize=8)

    # --- Sonar ranges vs time -------------------------------------------
    t = [h["t"] for h in history]
    for name in history[0]["sonar"]:
        ax_sonar.plot(
            t, [h["sonar"][name] for h in history], lw=1.0, label=name
        )
    ax_sonar.set_xlabel("t [s]")
    ax_sonar.set_ylabel("range [m]")
    ax_sonar.set_title("Ultrasonic readings vs time")
    ax_sonar.set_ylim(-0.2, None)
    ax_sonar.legend(loc="upper right", fontsize=8, ncol=2)

    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    return out_path
