from pathlib import Path

from charqui import Limits, Online, Planner, Problem, State, Options, Interface, Sync, Waypoints
from plot import plot_trajectory


def main() -> None:
    planner = Planner(3)
    pb = Problem(
        start=State(position=[0.0, 0.0, 0.5], velocity=[0.0, -2.2, -0.5], acceleration=[0.0, 2.5, -0.5]),
        goal=State(position=[5.0, -2.0, -3.5], velocity=[0.0, -0.5, -2.0], acceleration=[0.0, 0.0, 0.5]),
        limits=Limits(
            max_velocity=[3.0, 1.0, 3.0], max_acceleration=[3.0, 2.0, 1.0], max_jerk=[4.0, 3.0, 2.0],
            min_velocity=[-2.0, -0.5, -3.0], min_acceleration=[-2.0, -2.0, -2.0],
        ),
    )

    traj = planner.plan(pb)
    print(f"Trajectory duration: {traj.duration:.6f} s")
    print(f"At t = 1.0 s: {traj.at(1.0)}")
    for dof, bound in enumerate(traj.position_extrema):
        print(f"DoF {dof}: {bound}")

    plot_trajectory(Path(__file__).with_name("02_trajectory.png"), traj, pb)


if __name__ == "__main__":
    main()
