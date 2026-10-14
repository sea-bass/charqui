from pathlib import Path

from charqui import Limits, Online, Planner, Problem, State, Options, Interface, Sync, Waypoints
from plot import plot_trajectory


def main() -> None:
    otg = Online(3, 0.01)
    pb = Problem(
        start=State(position=[0.0, 0.0, 0.5], velocity=[0.0, -2.2, -0.5], acceleration=[0.0, 2.5, -0.5]),
        goal=State(position=[5.0, -2.0, -3.5], velocity=[0.0, -0.5, -2.0], acceleration=[0.0, 0.0, 0.5]),
        limits=Limits(max_velocity=[3.0, 1.0, 3.0], max_acceleration=[3.0, 2.0, 1.0], max_jerk=[4.0, 3.0, 2.0]),
    )

    print("t | position")
    stopping = False
    t = 0.0  # wall clock; otg.time restarts when the stop trajectory is computed
    while not otg.finished:
        step = otg.update(pb)
        t += otg.delta_time
        print(f"{t:.3f} | {step.position}")

        if not stopping and t >= 1.0:
            print("Stop!")
            stopping = True
            pb.options = Options(interface=Interface.Velocity, sync=Sync.Off)
            pb.goal = State(velocity=[0.0, 0.0, 0.0], acceleration=[0.0, 0.0, 0.0])
            pb.limits.max_jerk = [12.0, 10.0, 8.0]

    print(f"Stopped after {t:.3f} s (stop trajectory: {otg.trajectory.duration:.4f} s).")


if __name__ == "__main__":
    main()
