# charqui

> **Experimental, and mostly AI-generated.** This library was written largely by an AI coding assistant
> (Claude Code) under human direction, and has not yet been used on real hardware. The solver is validated
> against Ruckig on millions of random problems and by the test suites described below, but expect rough
> edges, API changes, and bugs. Review it as you would any unproven dependency before relying on it.

**Jerk-limited online trajectory generation with intermediate waypoints.** A modern C++23 recreation of the
[Ruckig](https://github.com/pantor/ruckig) algorithm with a redesigned API, a *local* waypoint solver, and
numpy-native Python bindings.

*Charqui* is jerky, and so is *Ruckig*: the library computes time-optimal trajectories from any initial
kinematic state (position, velocity, acceleration) to a target state while respecting velocity, acceleration
and jerk limits on every degree of freedom. Trajectories are computed in a few microseconds, so a new one can be
generated in every control cycle.

- **Bit-exact parity** with Ruckig's community solver on tens of thousands of random problems (identical
  durations, states within 1e-11), and the same per-call performance.
- **Intermediate waypoints computed locally.** Community Ruckig delegates waypoints to a cloud API; charqui
  selects a pass-through state at every waypoint with the bisection scheme of Kiemel & Kröger
  ([arXiv:2407.13423](https://arxiv.org/abs/2407.13423)) and synchronizes all DoFs per section.
- **A smaller, more modern API.** `std::expected` instead of integer result codes, `std::span` instead of a
  custom-vector template zoo, aggregates with designated initializers, an online generator that owns its state,
  and trajectories you can sample and iterate.
- **Python that feels like Python.** Keyword-constructible problems, numpy views for states, `(n, dofs)` batch
  sampling for plotting, exceptions with error codes, typed stubs.

## Quick start

### C++

```cpp
#include <charqui/charqui.hpp>
using namespace charqui;

Online otg(3, 0.01);  // 3 DoFs, 10 ms control cycle

Problem pb{
    .start  = {.position = {0.0, 0.0, 0.5}, .velocity = {0.0, -2.2, -0.5}, .acceleration = {0.0, 2.5, -0.5}},
    .goal   = {.position = {5.0, -2.0, -3.5}, .velocity = {0.0, -0.5, -2.0}, .acceleration = {0.0, 0.0, 0.5}},
    .limits = {.max_velocity = {3.0, 1.0, 3.0}, .max_acceleration = {3.0, 2.0, 1.0}, .max_jerk = {4.0, 3.0, 2.0}},
};

for (const auto& step : otg.run(pb)) {        // one Step per control cycle, until the goal is reached
    send_to_robot(step.position, step.velocity, step.acceleration);
}
```

Offline, with waypoints:

```cpp
Planner planner(3, /*max_waypoints=*/10);
pb.waypoints.positions = {{1.4, -1.6, 1.0}, {-0.6, -0.5, 0.4}};

auto traj = planner.plan(pb);                 // Result<Trajectory> = std::expected<Trajectory, Error>
if (!traj) { log(traj.error().message); return; }

Sample s = traj->at(1.0);                     // position, velocity, acceleration, jerk, section
for (const Sample& s : traj->sample(0.01)) { /* ... */ }
traj->section_end_times();                    // when each waypoint is reached
```

### Python

```python
from charqui import Online, Planner, Problem, State, Limits, Waypoints

pb = Problem(
    start=State(position=[0.2, 0.0, -0.3], velocity=[0.0, 0.2, 0.0]),
    goal=State(position=[0.5, 1.0, 0.0]),
    limits=Limits(max_velocity=[1.0, 2.0, 1.0], max_acceleration=[3.0, 2.0, 2.0], max_jerk=[6.0, 10.0, 20.0]),
    waypoints=Waypoints(positions=[[1.4, -1.6, 1.0], [-0.6, -0.5, 0.4]]),
)

otg = Online(3, 0.01, max_waypoints=10)
for step in otg.run(pb):                      # step.position etc. are numpy arrays
    ...

traj = Planner(3, max_waypoints=10).plan(pb)  # raises CharquiError on failure
batch = traj.sample(0.002)                    # batch.position has shape (n, dofs)
```

The `examples/` directory contains C++ and Python ports of Ruckig's examples 01–08 (same numbers), and
`examples/plot.py` renders a trajectory with matplotlib.

## Building

Requirements: a C++23 compiler (GCC 13+, Clang 17+), CMake ≥ 3.24, Ninja. The Python module needs
Python ≥ 3.9, [nanobind](https://github.com/wjakob/nanobind) and numpy.

```bash
cmake --preset release            # tests, examples, benchmark vs Ruckig (fetched), Python module
cmake --build --preset release
ctest --preset release            # C++ tests + parity against Ruckig
PYTHONPATH=build/release/python pytest tests/python

pip install .                     # Python package (scikit-build-core + nanobind)
```

CMake options: `CHARQUI_BUILD_TESTS`, `CHARQUI_BUILD_EXAMPLES`, `CHARQUI_BUILD_BENCHMARK` (fetches Ruckig for the
reference programs only; it is never linked into the library), `CHARQUI_BUILD_PYTHON`, `CHARQUI_INSTALL`. As a
dependency: `find_package(charqui)` and link `charqui::charqui`, or `add_subdirectory`.

## API overview

| Type | Role |
|---|---|
| `State` | position, velocity, acceleration per DoF. Velocity/acceleration may be left empty (= zero). |
| `Limits` | max velocity, acceleration, jerk; optional asymmetric minima. Empty acceleration/jerk = unlimited (second-/first-order profiles). |
| `Options` | `interface` (Position / Velocity), `sync` (Time / TimeIfNecessary / Phase / None), `minimum_duration`, `duration_step`, per-DoF overrides, `enabled` mask. |
| `Waypoints` | intermediate positions, optional per-section limits and per-section minimum durations. |
| `Problem` | `start`, `goal`, `limits`, `options`, `waypoints`. An aggregate: designated initializers in C++, keyword arguments in Python. |
| `Planner` | offline / one-shot planning. `plan(problem)` returns `Result<Trajectory>`; `plan(problem, trajectory&)` reuses storage and does not allocate. Also `validate` and `filter_waypoints`. |
| `Online` | cyclic generation. Owns the current state, advances it by `delta_time` per `update(problem)`, replans only when goal, limits, options or waypoints change, and holds the goal once reached. `run(problem)` iterates to the end. |
| `Trajectory` | `duration()`, `sections()`, `section_end_times()`, `at(t)`, `sample(times | dt)`, `position_extrema()`, `first_time_at_position()`, `profile(section, dof)`. |
| `Sample` / `Online::Step` | kinematic state of all DoFs at a time, plus the section index; a `Step` adds `finished` and `replanned`. |

Errors are values: `Result<T>` is `std::expected<T, Error>` with an `ErrorCode` and a message that names the
offending DoF and its inputs. `value_or_throw()` and the Python bindings turn them into `CharquiError`.

### Migrating from Ruckig

| Ruckig | charqui |
|---|---|
| `Ruckig<3> otg{0.01}; InputParameter<3> in; OutputParameter<3> out;` | `Online otg{3, 0.01}; Problem pb{...};` |
| `in.current_position`, `in.target_position`, `in.max_velocity` | `pb.start.position`, `pb.goal.position`, `pb.limits.max_velocity` |
| `while (otg.update(in, out) == Result::Working) { use(out.new_position); out.pass_to_input(in); }` | `for (const auto& step : otg.run(pb)) { use(step.position); }` |
| `otg.calculate(in, traj) == Result::Finished` | `auto traj = planner.plan(pb); if (!traj) handle(traj.error());` |
| `traj.at_time(t, p, v, a)` | `auto s = traj.at(t);` or the span overload |
| `in.intermediate_positions = {...}` (cloud) | `pb.waypoints.positions = {...}` (local) |
| `Synchronization::None` / `Synchronization.No` | `Sync::None` / `Sync.Off` |
| `DurationDiscretization::Discrete` | `options.duration_step = dt` |
| `Ruckig<DynamicDOFs, CustomVector>` | not needed: every input is a `std::span<const double>` |

Behavioural differences worth knowing: `Online` clamps to the trajectory end and holds the exact goal state
(Ruckig extrapolates past it); `Online::time()` is the time within the current trajectory and restarts on a
replan; `problem.start` is only read on the first `update()` (or after `reset()`), use `set_state()` to inject a
measured state.

## Intermediate waypoints

Given waypoints `P_1 … P_m` between the start and the goal, charqui chooses a kinematic state at every waypoint
and then solves and synchronizes each section like a normal state-to-state problem, so every DoF reaches each
waypoint at the same time and the result is exactly continuous in position, velocity and acceleration.

The pass-through states follow the paper's idea, per DoF and per waypoint, left to right:

- At a **turning point** (the DoF reverses direction), the velocity is zero and the acceleration points into the
  next section; its magnitude is the largest fraction of the limit found by bisection for which the segment from
  the previous waypoint and the segment to the next one are both feasible without overshoot.
- At a **pass-through** waypoint, the acceleration is zero and the velocity is the largest feasible fraction of
  the limit, with the same bisection.
- The feasibility test for a candidate solves the single-DoF problem with the closed-form solver and checks that
  the position stays between the two waypoints.

The resulting trajectory is guaranteed to be no slower than stopping at every waypoint (that baseline is always
computed and used as a fallback). Like Ruckig Pro, it is **not** globally time-optimal: the problem is
non-convex. Waypoints require the position interface, finite acceleration and jerk limits, a global
synchronization, and no global minimum duration or duration step (use `per_section_minimum_duration`).

## Benchmarks

Measured on an AMD Ryzen 9 9955HX 16-Core Processor (GCC 13.3, `-O3`, single core), against Ruckig v0.19.4 built from
source with identical flags. Durations are identical to all printed digits.

| workload | ruckig mean [µs] | ruckig worst [µs] | charqui mean [µs] | charqui worst [µs] | duration [s] |
|---|---|---|---|---|---|
| 01_position (3 DoF) | 1.65 | 37 | 1.48 | 24 | 3.438710967 |
| 02_position_offline | 1.67 | 5.0 | 1.56 | 12 | 3.841467602 |
| 05_velocity | 0.25 | 16 | 0.25 | 3.4 | 1.500000000 |
| 06_stop | 0.22 | 1.4 | 0.22 | 1.9 | 1.193750000 |
| 07_minimum_duration | 2.25 | 23 | 2.06 | 15 | 5.000000000 |
| random 7 DoF × 32768 (Ruckig's benchmark workload) | 5.35 | 89–123 | 5.08 | 26–27 | identical sums |

Worst-case numbers are single outliers out of 20 000 (or 32 768) calls and vary from run to run.

Waypoints (charqui only, community Ruckig cannot compute these locally):

| workload | charqui mean [µs] | charqui worst [µs] | duration [s] | stop-at-every-waypoint [s] |
|---|---|---|---|---|
| 03_waypoints (3 DoF, 4 waypoints) | 45 | 92 | 9.7550 | 9.7870 |
| 08_per_section_minimum_duration (3 DoF, 6 waypoints) | 79 | 104 | 10.0696 | – |

From Python (binding overhead included; `update` is one control cycle of the usual online loop):

| workload | ruckig calculate [µs] | charqui plan [µs] | ruckig update [µs] | charqui update [µs] |
|---|---|---|---|---|
| 01_position | 1.74 | 1.60 | 0.15 | 0.12 |
| 05_velocity | 0.32 | 0.43 | 0.15 | 0.12 |
| 07_minimum_duration | 2.29 | 2.19 | 0.15 | 0.12 |
| 03_waypoints (charqui only) | – | 44.6 | – | – |

Reproduce with `build/release/bench/bench_vs_ruckig` and `python bench/bench_python.py`. The parity program
`build/release/bench/parity N` runs N random problems per workload through both libraries and reports the largest
duration and state differences.

### Where the time goes, and what charqui does about it

A state-to-state solve is dominated by closed-form root finding: the quartics of step 1 (about a third of the
time), the safeguarded Newton iteration of step 2's sextic, and the profile checks. charqui keeps Ruckig's
case analysis and arithmetic bit for bit (the parity program reports a duration difference of exactly zero), and
gains its speed from work it can prove unnecessary:

- **Quartics are only solved when they can matter.** Every step-1 and step-2 quartic is followed by a time
  window that discards roots outside it. charqui first rewrites the quartic in the Bernstein basis over that
  window; if all coefficients share a sign (with a relative safety margin of 1e-6), the convex-hull property
  proves there is no root to find and the solve is skipped. On the random workload this removes about 40% of
  the quartic solves. A failed proof simply falls back to the solve, so results never change.
- **Order of work for targets at rest.** When the target velocity and acceleration are zero, Ruckig's
  enumeration stops at the first valid profile; charqui solves the remaining quartics lazily in that mode and
  eagerly otherwise (eager solves overlap their square roots and trigonometry in the CPU pipeline).
- **No copying of candidate profiles.** Step 1 writes its extremal profiles straight into the block's storage
  and the block refers to them by index; the candidate arrays are not zero-initialized either.
- **Polynomial evaluation** uses the power-form sum (two independent dependency chains) rather than Horner's
  serial multiply-add, and `sqrt` compiles to a single instruction (`-fno-math-errno`).

What is deliberately *not* done: no `-ffast-math`, no FMA contraction, no custom transcendental functions.
Those would change results in the last bits and break bit-exact parity with Ruckig. On the random workload,
libm calls (`acos`, `sincos`, `cbrt`) account for well under 10% of a solve, so there is little to gain there.

## ROS 2 / colcon

The repository is also a ROS 2 package (`package.xml`, `ament_cmake` build type, no ROS dependencies). Drop it
into a workspace and build as usual; when `ament_cmake` is found the package is registered in the ament index,
otherwise the very same `CMakeLists.txt` behaves as a plain CMake project. Under colcon the Python bindings are
built by default, with their `.pyi` stubs generated automatically, and installed into the workspace's
`site-packages` so `import charqui` works after `source install/setup.bash`. `colcon test` runs both the C++
suite and the pytest suite against the installed package.

```bash
cd ~/ros2_ws && rosdep install --from-paths src -i && colcon build --packages-select charqui && colcon test --packages-select charqui
```

Downstream packages use the exported CMake config in either world:

```cmake
find_package(charqui REQUIRED)
target_link_libraries(my_node PRIVATE charqui::charqui)
```

## Testing

- `tests/` (doctest): Ruckig's deterministic reference values, randomized suites following Ruckig's test
  distributions with a stricter checker (limits within 1e-12, goal within 1e-8 / 1e-10, phase continuity),
  waypoint invariants (exact pass-through, continuity across sections, per-section minimum durations, the
  stop-at-every-waypoint bound), and online stepping. `CHARQUI_TEST_CASES=N` scales the random suites.
- `bench/parity.cpp`: identical random inputs through Ruckig and charqui (registered with CTest).
- `tests/python`: bindings, numpy inputs, batch sampling, waypoints, and a cross-check against the `ruckig` wheel.

## License and attribution

MIT. The closed-form state-to-state solver recreates the case analysis of Ruckig (Lars Berscheid and Torsten
Kröger, MIT); the waypoint state selection follows Kiemel & Kröger, arXiv:2407.13423. See `NOTICE`.
