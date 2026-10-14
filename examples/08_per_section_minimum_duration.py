from pathlib import Path

from charqui import Limits, Online, Planner, Problem, State, Options, Interface, Sync, Waypoints
from plot import plot_trajectory


def main() -> None:
    planner = Planner(3, max_waypoints=10)
    pb = Problem(
        start=State(position=[0.8, 0.0, 0.5]),
        goal=State(position=[0.5, 1.2, 0.0]),
        limits=Limits(max_velocity=[3.0, 2.0, 2.0], max_acceleration=[6.0, 4.0, 4.0], max_jerk=[16.0, 10.0, 20.0]),
        waypoints=Waypoints(
            positions=[[1.4, -1.6, 1.0], [-0.6, -0.5, 0.4], [-0.4, -0.35, 0.0], [-0.2, 0.35, -0.1], [0.2, 0.5, -0.1], [0.8, 1.8, -0.1]],
            per_section_minimum_duration=[0.0, 2.0, 0.0, 1.0, 0.0, 2.0, 0.0],
        ),
    )

    traj = planner.plan(pb)
    print(f"Trajectory duration: {traj.duration:.6f} s")
    previous = 0.0
    for k, end in enumerate(traj.section_end_times):
        print(f"  section {k}: {end - previous:.4f} s (minimum {pb.waypoints.per_section_minimum_duration[k]:.1f} s)")
        previous = end

    plot_trajectory(Path(__file__).with_name("08_trajectory.png"), traj, pb)


if __name__ == "__main__":
    main()
