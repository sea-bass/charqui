import copy
import math

import numpy as np
import pytest

import charqui
from charqui import CharquiError, ErrorCode, Interface, Limits, Online, Options, Planner, Problem, State, Sync, Waypoints


def example_01() -> Problem:
    return Problem(
        start=State(position=[0.0, 0.0, 0.5], velocity=[0.0, -2.2, -0.5], acceleration=[0.0, 2.5, -0.5]),
        goal=State(position=[5.0, -2.0, -3.5], velocity=[0.0, -0.5, -2.0], acceleration=[0.0, 0.0, 0.5]),
        limits=Limits(max_velocity=[3.0, 1.0, 3.0], max_acceleration=[3.0, 2.0, 1.0], max_jerk=[4.0, 3.0, 2.0]),
    )


def example_03() -> Problem:
    return Problem(
        start=State(position=[0.2, 0.0, -0.3], velocity=[0.0, 0.2, 0.0], acceleration=[0.0, 0.6, 0.0]),
        goal=State(position=[0.5, 1.0, 0.0], velocity=[0.2, 0.0, 0.3], acceleration=[0.0, 0.1, -0.1]),
        limits=Limits(max_velocity=[1.0, 2.0, 1.0], max_acceleration=[3.0, 2.0, 2.0], max_jerk=[6.0, 10.0, 20.0]),
        waypoints=Waypoints(positions=[[1.4, -1.6, 1.0], [-0.6, -0.5, 0.4], [-0.4, -0.35, 0.0], [0.8, 1.8, -0.1]]),
    )


def test_version():
    assert charqui.__version__


def test_reference_trajectory():
    planner = Planner(3)
    pb = Problem(
        start=State(position=[0.0, -2.0, 0.0]),
        goal=State(position=[1.0, -3.0, 2.0], velocity=[0.0, 0.3, 0.0]),
        limits=Limits(max_velocity=[1.0, 1.0, 1.0], max_acceleration=[1.0, 1.0, 1.0], max_jerk=[1.0, 1.0, 1.0]),
    )
    traj = planner.plan(pb)
    assert traj.duration == pytest.approx(4.0)
    assert traj.dofs == 3
    assert traj.sections == 1

    s = traj.at(2.0)
    np.testing.assert_allclose(s.position, [0.5, -2.6871268303, 1.0], atol=1e-8)
    np.testing.assert_allclose(s.jerk, [0.0, 0.0, -1.0], atol=1e-12)
    assert s.section == 0

    np.testing.assert_allclose(traj.independent_min_durations, [3.1748021039, 3.6860977315, 4.0], atol=1e-8)
    assert traj.first_time_at_position(0, 0.5) == pytest.approx(2.0)
    assert traj.first_time_at_position(1, -3.4) is None

    extrema = traj.position_extrema
    assert extrema[2].max == pytest.approx(2.0)

    profile = traj.profile(0, 0)
    assert sum(profile.t) == pytest.approx(4.0)
    assert "Profile(" in repr(profile)


def test_numpy_inputs_and_batch_sampling():
    planner = Planner(3)
    pb = example_01()
    pb.start.position = np.array([0.0, 0.0, 0.5])
    pb.limits.max_jerk = np.array([4.0, 3.0, 2.0])
    traj = planner.plan(pb)
    assert traj.duration == pytest.approx(3.438710967, abs=1e-8)

    batch = traj.sample(0.01)
    assert batch.position.shape == (len(batch), 3)
    assert batch.time[0] == 0.0
    assert batch.time[-1] == pytest.approx(traj.duration)
    np.testing.assert_allclose(batch.position[-1], pb.goal.position, atol=1e-8)
    np.testing.assert_allclose(batch.velocity[-1], pb.goal.velocity, atol=1e-8)
    assert np.all(np.abs(batch.jerk) <= np.array(pb.limits.max_jerk) + 1e-12)

    at_times = traj.sample(np.linspace(0.0, traj.duration, 50))
    assert at_times.n == 50
    assert at_times.acceleration.shape == (50, 3)


def test_errors_are_exceptions_with_codes():
    pb = example_01()
    pb.goal.velocity = [5.0, 0.0, 0.0]
    with pytest.raises(CharquiError) as info:
        Planner(3).plan(pb)
    assert info.value.code == ErrorCode.InvalidInput
    assert "exceeds its maximum" in str(info.value)

    with pytest.raises(CharquiError):
        Planner.validate(pb)


def test_online_loop_reaches_goal():
    otg = Online(3, 0.01)
    pb = example_01()
    steps = list(otg.run(pb))
    assert steps[0].replanned
    assert sum(s.replanned for s in steps) == 1
    assert steps[-1].finished
    assert not steps[-2].finished
    assert otg.finished
    assert otg.time == pytest.approx(otg.trajectory.duration)
    np.testing.assert_allclose(steps[-1].position, pb.goal.position, atol=1e-9)
    np.testing.assert_allclose(otg.state.position, pb.goal.position, atol=1e-9)

    # Changing the goal triggers exactly one replan
    pb.goal.position = [4.0, -2.0, -3.5]
    step = otg.update(pb)
    assert step.replanned and not step.finished
    assert not otg.update(pb).replanned


def test_velocity_interface_and_options():
    pb = Problem(
        start=State(position=[0.0, 0.0, 0.5], velocity=[3.0, -2.2, -0.5], acceleration=[0.0, 2.5, -0.5]),
        goal=State(velocity=[0.0, -0.5, -1.5], acceleration=[0.0, 0.0, 0.5]),
        limits=Limits(max_acceleration=[3.0, 2.0, 1.0], max_jerk=[6.0, 6.0, 4.0]),
        options=Options(interface=Interface.Velocity),
    )
    traj = Planner(3).plan(pb)
    assert traj.duration == pytest.approx(1.5)
    np.testing.assert_allclose(traj.at(traj.duration).velocity, pb.goal.velocity, atol=1e-8)

    pb.options.sync = Sync.Off
    assert Planner(3).plan(pb).duration == pytest.approx(1.5)

    pb2 = example_01()
    pb2.options = Options(minimum_duration=5.0)
    assert Planner(3).plan(pb2).duration == pytest.approx(5.0)

    pb3 = example_01()
    pb3.options.duration_step = 0.07
    k = Planner(3).plan(pb3).duration / 0.07
    assert abs(k - round(k)) < 1e-9


def test_waypoints():
    planner = Planner(3, max_waypoints=10)
    pb = example_03()
    traj = planner.plan(pb)
    assert traj.sections == 5
    ends = traj.section_end_times
    assert ends.shape == (5,)
    assert ends[-1] == pytest.approx(traj.duration)
    for i, wp in enumerate(pb.waypoints.positions):
        np.testing.assert_allclose(traj.at(ends[i]).position, wp, atol=1e-7)

    otg = Online(3, 0.01, max_waypoints=10)
    steps = list(otg.run(pb))
    assert sum(s.replanned for s in steps) == 1
    assert max(s.section for s in steps) == 4
    np.testing.assert_allclose(steps[-1].position, pb.goal.position, atol=1e-9)

    filtered = Planner.filter_waypoints(pb, [0.5, 0.5, 0.5])
    assert len(filtered) <= 4


def test_problem_value_semantics_and_copy():
    pb = example_01()
    other = copy.deepcopy(pb)
    assert pb == other
    other.goal.position = [1.0, 2.0, 3.0]
    assert pb != other
    assert "Problem(3 DoFs" in repr(pb)


ruckig = pytest.importorskip("ruckig")


@pytest.mark.parametrize("seed", range(5))
def test_parity_with_ruckig_wheel(seed):
    rng = np.random.default_rng(seed)
    planner = Planner(3)
    otg = ruckig.Ruckig(3, 0.01)
    for _ in range(100):
        pb = Problem(
            start=State(position=rng.normal(0, 4, 3), velocity=rng.normal(0, 0.8, 3), acceleration=rng.normal(0, 0.8, 3)),
            goal=State(position=rng.normal(0, 4, 3), velocity=rng.normal(0, 0.8, 3), acceleration=rng.normal(0, 0.8, 3)),
            limits=Limits(
                max_velocity=rng.uniform(0.08, 16, 3) + np.abs(rng.normal(0, 0.8, 3)),
                max_acceleration=rng.uniform(0.08, 16, 3) + np.abs(rng.normal(0, 0.8, 3)),
                max_jerk=rng.uniform(0.08, 16, 3),
            ),
        )
        try:
            Planner.validate(pb)
        except CharquiError:
            continue

        inp = ruckig.InputParameter(3)
        inp.current_position = list(pb.start.position)
        inp.current_velocity = list(pb.start.velocity)
        inp.current_acceleration = list(pb.start.acceleration)
        inp.target_position = list(pb.goal.position)
        inp.target_velocity = list(pb.goal.velocity)
        inp.target_acceleration = list(pb.goal.acceleration)
        inp.max_velocity = list(pb.limits.max_velocity)
        inp.max_acceleration = list(pb.limits.max_acceleration)
        inp.max_jerk = list(pb.limits.max_jerk)
        rtraj = ruckig.Trajectory(3)
        result = otg.calculate(inp, rtraj)
        assert result in (ruckig.Result.Working, ruckig.Result.Finished)

        traj = planner.plan(pb)
        assert traj.duration == pytest.approx(rtraj.duration, abs=1e-8, rel=1e-8)
        for t in np.linspace(0.0, traj.duration, 5):
            p, v, a = rtraj.at_time(t)
            s = traj.at(t)
            np.testing.assert_allclose(s.position, p, atol=1e-6)
            np.testing.assert_allclose(s.velocity, v, atol=1e-6)
            np.testing.assert_allclose(s.acceleration, a, atol=1e-6)
