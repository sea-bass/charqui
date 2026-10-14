from pathlib import Path

from charqui import Limits, Online, Planner, Problem, State, Options, Interface, Sync, Waypoints
from plot import plot_trajectory


def main() -> None:
    otg = Online(3, 0.01, max_waypoints=10)
    pb = Problem(
        start=State(position=[0.2, 0.0, -0.3], velocity=[0.0, 0.2, 0.0], acceleration=[0.0, 0.6, 0.0]),
        goal=State(position=[0.5, 1.0, 0.0], velocity=[0.2, 0.0, 0.3], acceleration=[0.0, 0.1, -0.1]),
        limits=Limits(max_velocity=[1.0, 2.0, 1.0], max_acceleration=[3.0, 2.0, 2.0], max_jerk=[6.0, 10.0, 20.0]),
        waypoints=Waypoints(positions=[[1.4, -1.6, 1.0], [-0.6, -0.5, 0.4], [-0.4, -0.35, 0.0], [0.8, 1.8, -0.1]]),
    )

    t = 0.0  # wall clock; otg.time restarts whenever a new trajectory is computed
    while not otg.finished:
        step = otg.update(pb)
        t += otg.delta_time
        if step.replanned:
            print(f"[t = {t:.2f}] new trajectory of {otg.trajectory.duration:.4f} s through {otg.trajectory.sections - 1} remaining waypoints, "
                  f"computed in {otg.calculation_duration_us:.1f} us")

        # After two seconds, move the goal: the generator replans from its current state through the remaining waypoints
        if t > 2.0 and pb.goal.position[2] == 0.0:
            pb.goal.position = [0.5, 1.0, 0.5]

    print(f"Reached {otg.state.position} at t = {t:.2f} s.")


if __name__ == "__main__":
    main()
