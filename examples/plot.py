"""Plot a charqui trajectory (position, velocity, acceleration, jerk per DoF) with matplotlib."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from charqui import Problem, Trajectory


def plot_trajectory(path: str | Path, trajectory: Trajectory, problem: Problem, dt: float = 0.002) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available, skipping plot")
        return

    batch = trajectory.sample(dt)
    dofs = trajectory.dofs
    limits = problem.limits
    fig, axes = plt.subplots(4, dofs, figsize=(4.5 * dofs, 10), sharex=True, squeeze=False)
    rows = [("position", batch.position, None, None), ("velocity", batch.velocity, limits.max_velocity, limits.min_velocity),
            ("acceleration", batch.acceleration, limits.max_acceleration, limits.min_acceleration), ("jerk", batch.jerk, limits.max_jerk, None)]
    for row, (name, data, upper, lower) in enumerate(rows):
        for dof in range(dofs):
            ax = axes[row][dof]
            ax.plot(batch.time, data[:, dof], color="C0", linewidth=1.5)
            if upper is not None and len(upper) > dof and np.isfinite(upper[dof]):
                hi = upper[dof]
                lo = lower[dof] if lower is not None and len(lower) > dof else -hi
                ax.axhline(hi, color="C3", linestyle="--", linewidth=0.8)
                ax.axhline(lo, color="C3", linestyle="--", linewidth=0.8)
            for t in trajectory.section_end_times[:-1]:
                ax.axvline(t, color="0.6", linestyle=":", linewidth=0.8)
            ax.grid(True, alpha=0.3)
            if row == 0:
                ax.set_title(f"DoF {dof}")
            if dof == 0:
                ax.set_ylabel(name)
            if row == 3:
                ax.set_xlabel("time [s]")
    fig.suptitle(f"charqui trajectory, duration {trajectory.duration:.4f} s")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print(f"saved {path}")
