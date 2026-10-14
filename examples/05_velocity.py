from pathlib import Path

from charqui import Limits, Online, Planner, Problem, State, Options, Interface, Sync, Waypoints
from plot import plot_trajectory


def main() -> None:
    otg = Online(3, 0.01)
    pb = Problem(
        start=State(position=[0.0, 0.0, 0.5], velocity=[3.0, -2.2, -0.5], acceleration=[0.0, 2.5, -0.5]),
        goal=State(velocity=[0.0, -0.5, -1.5], acceleration=[0.0, 0.0, 0.5]),
        limits=Limits(max_acceleration=[3.0, 2.0, 1.0], max_jerk=[6.0, 6.0, 4.0]),
        options=Options(interface=Interface.Velocity),
    )

    print("t | velocity")
    for step in otg.run(pb):
        print(f"{step.time:.3f} | {step.velocity}")

    print(f"Reached the goal velocity in {otg.trajectory.duration:.6f} s.")
    plot_trajectory(Path(__file__).with_name("05_trajectory.png"), otg.trajectory, pb)


if __name__ == "__main__":
    main()
