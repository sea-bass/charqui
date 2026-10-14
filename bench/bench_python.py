"""Python-level timing: charqui vs the ruckig wheel on Ruckig's examples (binding overhead included).

Usage: python bench/bench_python.py [repetitions]
"""

from __future__ import annotations

import sys
import timeit

import charqui
from charqui import Interface, Limits, Online, Options, Planner, Problem, State, Waypoints


def examples() -> list[tuple[str, Problem]]:
    ex01 = Problem(
        start=State(position=[0.0, 0.0, 0.5], velocity=[0.0, -2.2, -0.5], acceleration=[0.0, 2.5, -0.5]),
        goal=State(position=[5.0, -2.0, -3.5], velocity=[0.0, -0.5, -2.0], acceleration=[0.0, 0.0, 0.5]),
        limits=Limits(max_velocity=[3.0, 1.0, 3.0], max_acceleration=[3.0, 2.0, 1.0], max_jerk=[4.0, 3.0, 2.0]),
    )
    ex05 = Problem(
        start=State(position=[0.0, 0.0, 0.5], velocity=[3.0, -2.2, -0.5], acceleration=[0.0, 2.5, -0.5]),
        goal=State(velocity=[0.0, -0.5, -1.5], acceleration=[0.0, 0.0, 0.5]),
        limits=Limits(max_acceleration=[3.0, 2.0, 1.0], max_jerk=[6.0, 6.0, 4.0]),
        options=Options(interface=Interface.Velocity),
    )
    ex07 = Problem(
        start=State(position=[0.0, 0.0, 0.5], velocity=[0.0, -2.2, -0.5], acceleration=[0.0, 2.5, -0.5]),
        goal=State(position=[-5.0, -2.0, -3.5], velocity=[0.0, -0.5, -2.0], acceleration=[0.0, 0.0, 0.5]),
        limits=Limits(max_velocity=[3.0, 1.0, 3.0], max_acceleration=[3.0, 2.0, 1.0], max_jerk=[4.0, 3.0, 2.0]),
        options=Options(minimum_duration=5.0),
    )
    return [("01_position", ex01), ("05_velocity", ex05), ("07_minimum_duration", ex07)]


def waypoint_example() -> Problem:
    return Problem(
        start=State(position=[0.2, 0.0, -0.3], velocity=[0.0, 0.2, 0.0], acceleration=[0.0, 0.6, 0.0]),
        goal=State(position=[0.5, 1.0, 0.0], velocity=[0.2, 0.0, 0.3], acceleration=[0.0, 0.1, -0.1]),
        limits=Limits(max_velocity=[1.0, 2.0, 1.0], max_acceleration=[3.0, 2.0, 2.0], max_jerk=[6.0, 10.0, 20.0]),
        waypoints=Waypoints(positions=[[1.4, -1.6, 1.0], [-0.6, -0.5, 0.4], [-0.4, -0.35, 0.0], [0.8, 1.8, -0.1]]),
    )


def to_ruckig(pb: Problem, ruckig):
    inp = ruckig.InputParameter(pb.dofs)
    zeros = [0.0] * pb.dofs
    inf = [float("inf")] * pb.dofs
    inp.current_position = list(pb.start.position) or zeros
    inp.current_velocity = list(pb.start.velocity) or zeros
    inp.current_acceleration = list(pb.start.acceleration) or zeros
    inp.target_position = list(pb.goal.position) or zeros
    inp.target_velocity = list(pb.goal.velocity) or zeros
    inp.target_acceleration = list(pb.goal.acceleration) or zeros
    inp.max_velocity = list(pb.limits.max_velocity) or inf
    inp.max_acceleration = list(pb.limits.max_acceleration) or inf
    inp.max_jerk = list(pb.limits.max_jerk) or inf
    inp.control_interface = ruckig.ControlInterface.Position if pb.options.interface == Interface.Position else ruckig.ControlInterface.Velocity
    inp.minimum_duration = pb.options.minimum_duration
    return inp


def main() -> None:
    reps = int(sys.argv[1]) if len(sys.argv) > 1 else 20000
    try:
        import ruckig
    except ImportError:
        ruckig = None
        print("ruckig wheel not installed; showing charqui only")

    print(f"charqui {charqui.__version__}" + (f", ruckig {ruckig.__version__}" if ruckig else ""))
    print("| workload | ruckig calculate [µs] | charqui plan [µs] | ruckig update [µs] | charqui update [µs] |")
    print("|---|---|---|---|---|")
    for name, pb in examples():
        planner = Planner(pb.dofs)
        c_plan = timeit.timeit(lambda: planner.plan(pb), number=reps) / reps * 1e6

        online = Online(pb.dofs, 0.01)
        online.update(pb)
        c_update = timeit.timeit(lambda: online.update(pb), number=reps) / reps * 1e6

        if ruckig:
            inp = to_ruckig(pb, ruckig)
            otg = ruckig.Ruckig(pb.dofs, 0.01)
            traj = ruckig.Trajectory(pb.dofs)
            r_calc = timeit.timeit(lambda: otg.calculate(inp, traj), number=reps) / reps * 1e6
            out = ruckig.OutputParameter(pb.dofs)
            otg.update(inp, out)
            out.pass_to_input(inp)

            def ruckig_cycle():  # the usual Ruckig loop: feed the output back so no recalculation happens
                otg.update(inp, out)
                out.pass_to_input(inp)

            r_update = timeit.timeit(ruckig_cycle, number=reps) / reps * 1e6
            print(f"| {name} | {r_calc:.2f} | {c_plan:.2f} | {r_update:.2f} | {c_update:.2f} |")
        else:
            print(f"| {name} | - | {c_plan:.2f} | - | {c_update:.2f} |")

    pb = waypoint_example()
    planner = Planner(3, max_waypoints=10)
    n = max(reps // 20, 1)
    c_plan = timeit.timeit(lambda: planner.plan(pb), number=n) / n * 1e6
    print(f"| 03_waypoints (charqui only) | - | {c_plan:.1f} | - | - |")


if __name__ == "__main__":
    main()
