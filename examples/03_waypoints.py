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

    print("t | section | position")
    for step in otg.run(pb):
        print(f"{step.time:.2f} | {step.section} | {step.position}")

    traj = otg.trajectory
    print(f"Reached the goal in {traj.duration:.6f} s (calculated in {otg.calculation_duration_us:.1f} us).")
    print(f"Section end times: {traj.section_end_times}")
    plot_trajectory(Path(__file__).with_name("03_trajectory.png"), traj, pb)


if __name__ == "__main__":
    main()
