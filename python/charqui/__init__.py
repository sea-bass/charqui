"""charqui: jerk-limited online trajectory generation with intermediate waypoints.

A modern reimplementation of the Ruckig algorithm with a numpy-native API.
"""

from __future__ import annotations

from collections.abc import Iterator

from ._core import (
    Batch,
    Bound,
    BrakeProfile,
    CharquiError,
    ErrorCode,
    Interface,
    Limits,
    Online,
    Options,
    Planner,
    Problem,
    Profile,
    Sample,
    State,
    Step,
    Sync,
    Trajectory,
    Waypoints,
)

__version__ = "0.1.0"

__all__ = [
    "Batch",
    "Bound",
    "BrakeProfile",
    "CharquiError",
    "ErrorCode",
    "Interface",
    "Limits",
    "Online",
    "Options",
    "Planner",
    "Problem",
    "Profile",
    "Sample",
    "State",
    "Step",
    "Sync",
    "Trajectory",
    "Waypoints",
    "__version__",
]


def _run(self: Online, problem: Problem) -> Iterator[Step]:
    """Yield one Step per control cycle until the goal is reached (the final step included)."""
    while True:
        step = self.update(problem)
        yield step
        if step.finished:
            return


Online.run = _run
