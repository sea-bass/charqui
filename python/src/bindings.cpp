// nanobind bindings: charqui._core

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <format>
#include <memory>
#include <sstream>

#include <charqui/charqui.hpp>

namespace nb = nanobind;
using namespace nb::literals;
using namespace charqui;

namespace {

using Array1 = nb::ndarray<nb::numpy, double, nb::ndim<1>>;
using Array2 = nb::ndarray<nb::numpy, double, nb::ndim<2>>;

//! A numpy view of a std::vector owned by a bound object (no copy; keeps the owner alive)
Array1 view(std::vector<double>& v, nb::handle owner) {
    return Array1(v.data(), {v.size()}, owner);
}

//! A fresh numpy array (copy) from any range of doubles
Array1 copy_array(std::span<const double> v) {
    double* data = new double[v.size()];
    std::copy(v.begin(), v.end(), data);
    nb::capsule owner(data, [](void* p) noexcept { delete[] static_cast<double*>(p); });
    return Array1(data, {v.size()}, owner);
}

template <class T>
T unwrap(Result<T>&& r) {
    if (!r) {
        throw CharquiError(std::move(r.error()));
    }
    if constexpr (!std::is_void_v<T>) {
        return std::move(*r);
    }
}

std::string repr_vec(const std::vector<double>& v) {
    std::string s = "[";
    for (std::size_t i = 0; i < v.size(); ++i) {
        s += std::format("{}{:.6g}", i ? ", " : "", v[i]);
    }
    return s + "]";
}

std::string enum_name(Interface i) {
    return i == Interface::Position ? "Position" : "Velocity";
}
std::string enum_name(Sync s) {
    switch (s) {
        case Sync::Time: return "Time";
        case Sync::TimeIfNecessary: return "TimeIfNecessary";
        case Sync::Phase: return "Phase";
        case Sync::None: return "Off";
    }
    return "?";
}

//! Column-major-free batch of samples for plotting and analysis
struct Batch {
    std::size_t n{0}, dofs{0};
    std::vector<double> time, position, velocity, acceleration, jerk;
    std::vector<std::size_t> section;

    explicit Batch(const std::vector<Sample>& samples, std::size_t dofs_) : n(samples.size()), dofs(dofs_) {
        time.resize(n);
        section.resize(n);
        position.resize(n * dofs);
        velocity.resize(n * dofs);
        acceleration.resize(n * dofs);
        jerk.resize(n * dofs);
        for (std::size_t i = 0; i < n; ++i) {
            time[i] = samples[i].time;
            section[i] = samples[i].section;
            std::copy(samples[i].position.begin(), samples[i].position.end(), position.begin() + static_cast<std::ptrdiff_t>(i * dofs));
            std::copy(samples[i].velocity.begin(), samples[i].velocity.end(), velocity.begin() + static_cast<std::ptrdiff_t>(i * dofs));
            std::copy(samples[i].acceleration.begin(), samples[i].acceleration.end(), acceleration.begin() + static_cast<std::ptrdiff_t>(i * dofs));
            std::copy(samples[i].jerk.begin(), samples[i].jerk.end(), jerk.begin() + static_cast<std::ptrdiff_t>(i * dofs));
        }
    }
};

Array2 view2(std::vector<double>& v, std::size_t n, std::size_t dofs, nb::handle owner) {
    return Array2(v.data(), {n, dofs}, owner);
}


//! Value semantics for Python: copy, deepcopy and pickling via a tuple of the public fields
template <class T, class Cls, class Get, class Set>
void add_value_semantics(Cls& cls, Get&& get, Set&& set) {
    cls.def("__copy__", [](const T& t) { return T(t); })
       .def("__deepcopy__", [](const T& t, nb::dict) { return T(t); }, "memo"_a)
       .def("__getstate__", get)
       .def("__setstate__", set);
}

}  // namespace

NB_MODULE(_core, m) {
    m.doc() = "charqui: jerk-limited online trajectory generation with intermediate waypoints";

    // ----- enums -----
    nb::enum_<Interface>(m, "Interface", "Which kinematic quantity the goal state constrains")
        .value("Position", Interface::Position)
        .value("Velocity", Interface::Velocity);

    nb::enum_<Sync>(m, "Sync", "How the degrees of freedom are synchronized in time")
        .value("Time", Sync::Time, "All DoFs finish at the same time (default)")
        .value("TimeIfNecessary", Sync::TimeIfNecessary, "Like Time, but DoFs with a zero goal velocity/acceleration are not slowed down")
        .value("Phase", Sync::Phase, "Time synchronization along a straight line, when the inputs allow it")
        .value("Off", Sync::None, "Every DoF is time-optimal on its own");

    nb::enum_<ErrorCode>(m, "ErrorCode")
        .value("InvalidInput", ErrorCode::InvalidInput)
        .value("TrajectoryDuration", ErrorCode::TrajectoryDuration)
        .value("PositionalLimits", ErrorCode::PositionalLimits)
        .value("ZeroLimits", ErrorCode::ZeroLimits)
        .value("ExecutionTime", ErrorCode::ExecutionTime)
        .value("Synchronization", ErrorCode::Synchronization);

    // ----- exception with a .code attribute -----
    static nb::object exc_type = nb::exception<CharquiError>(m, "CharquiError");
    nb::register_exception_translator([](const std::exception_ptr& p, void*) {
        try {
            std::rethrow_exception(p);
        } catch (const CharquiError& e) {
            nb::object exc = exc_type(e.what());
            exc.attr("code") = nb::cast(e.code());
            PyErr_SetObject(exc_type.ptr(), exc.ptr());
        }
    });

    // ----- problem description -----
    auto state_cls = nb::class_<State>(m, "State", "A kinematic state. Velocity and acceleration default to zero.");
    state_cls
        .def(nb::init<>())
        .def("__init__", [](State* s, std::vector<double> position, std::optional<std::vector<double>> velocity, std::optional<std::vector<double>> acceleration) {
            new (s) State{std::move(position), velocity.value_or(std::vector<double>{}), acceleration.value_or(std::vector<double>{})};
        }, "position"_a = std::vector<double>{}, "velocity"_a = nb::none(), "acceleration"_a = nb::none())
        .def_rw("position", &State::position)
        .def_rw("velocity", &State::velocity)
        .def_rw("acceleration", &State::acceleration)
        .def("__eq__", [](const State& a, const State& b) { return a == b; }, nb::is_operator())
        .def("__repr__", [](const State& s) { return std::format("State(position={}, velocity={}, acceleration={})", repr_vec(s.position), repr_vec(s.velocity), repr_vec(s.acceleration)); });
    add_value_semantics<State>(state_cls,
        [](const State& s) { return std::make_tuple(s.position, s.velocity, s.acceleration); },
        [](State& s, const std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>& t) { new (&s) State{std::get<0>(t), std::get<1>(t), std::get<2>(t)}; });

    auto limits_cls = nb::class_<Limits>(m, "Limits", "Kinematic limits per DoF. Empty max_acceleration / max_jerk mean unlimited; unset minima default to the negated maxima.");
    limits_cls
        .def(nb::init<>())
        .def("__init__", [](Limits* l, std::optional<std::vector<double>> max_velocity, std::optional<std::vector<double>> max_acceleration, std::optional<std::vector<double>> max_jerk,
                            std::optional<std::vector<double>> min_velocity, std::optional<std::vector<double>> min_acceleration) {
            new (l) Limits{max_velocity.value_or(std::vector<double>{}), max_acceleration.value_or(std::vector<double>{}), max_jerk.value_or(std::vector<double>{}), std::move(min_velocity), std::move(min_acceleration)};
        }, "max_velocity"_a = nb::none(), "max_acceleration"_a = nb::none(), "max_jerk"_a = nb::none(), "min_velocity"_a = nb::none(), "min_acceleration"_a = nb::none())
        .def_rw("max_velocity", &Limits::max_velocity)
        .def_rw("max_acceleration", &Limits::max_acceleration)
        .def_rw("max_jerk", &Limits::max_jerk)
        .def_rw("min_velocity", &Limits::min_velocity, nb::arg().none())
        .def_rw("min_acceleration", &Limits::min_acceleration, nb::arg().none())
        .def("__eq__", [](const Limits& a, const Limits& b) { return a == b; }, nb::is_operator())
        .def("__repr__", [](const Limits& l) { return std::format("Limits(max_velocity={}, max_acceleration={}, max_jerk={})", repr_vec(l.max_velocity), repr_vec(l.max_acceleration), repr_vec(l.max_jerk)); });
    using LimitsState = std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::optional<std::vector<double>>, std::optional<std::vector<double>>>;
    add_value_semantics<Limits>(limits_cls,
        [](const Limits& l) { return LimitsState{l.max_velocity, l.max_acceleration, l.max_jerk, l.min_velocity, l.min_acceleration}; },
        [](Limits& l, const LimitsState& t) { new (&l) Limits{std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t), std::get<4>(t)}; });

    auto options_cls = nb::class_<Options>(m, "Options");
    options_cls
        .def(nb::init<>())
        .def("__init__", [](Options* o, Interface interface, Sync sync, std::optional<double> minimum_duration, std::optional<double> duration_step,
                            std::optional<std::vector<Interface>> per_dof_interface, std::optional<std::vector<Sync>> per_dof_sync, std::optional<std::vector<bool>> enabled) {
            new (o) Options{interface, sync, minimum_duration, duration_step, std::move(per_dof_interface), std::move(per_dof_sync), std::move(enabled)};
        }, "interface"_a = Interface::Position, "sync"_a = Sync::Time, "minimum_duration"_a = nb::none(), "duration_step"_a = nb::none(),
           "per_dof_interface"_a = nb::none(), "per_dof_sync"_a = nb::none(), "enabled"_a = nb::none())
        .def_rw("interface", &Options::interface)
        .def_rw("sync", &Options::sync)
        .def_rw("minimum_duration", &Options::minimum_duration, nb::arg().none(), "Lower bound on the trajectory duration")
        .def_rw("duration_step", &Options::duration_step, nb::arg().none(), "If set, the duration is rounded up to a multiple of this step (e.g. the control cycle)")
        .def_rw("per_dof_interface", &Options::per_dof_interface, nb::arg().none())
        .def_rw("per_dof_sync", &Options::per_dof_sync, nb::arg().none())
        .def_rw("enabled", &Options::enabled, nb::arg().none())
        .def("__eq__", [](const Options& a, const Options& b) { return a == b; }, nb::is_operator())
        .def("__repr__", [](const Options& o) { return std::format("Options(interface={}, sync={}, minimum_duration={}, duration_step={})", enum_name(o.interface), enum_name(o.sync),
                                                                     o.minimum_duration ? std::format("{}", *o.minimum_duration) : "None", o.duration_step ? std::format("{}", *o.duration_step) : "None"); });
    using OptionsState = std::tuple<Interface, Sync, std::optional<double>, std::optional<double>, std::optional<std::vector<Interface>>, std::optional<std::vector<Sync>>, std::optional<std::vector<bool>>>;
    add_value_semantics<Options>(options_cls,
        [](const Options& o) { return OptionsState{o.interface, o.sync, o.minimum_duration, o.duration_step, o.per_dof_interface, o.per_dof_sync, o.enabled}; },
        [](Options& o, const OptionsState& t) { new (&o) Options{std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t), std::get<4>(t), std::get<5>(t), std::get<6>(t)}; });

    auto waypoints_cls = nb::class_<Waypoints>(m, "Waypoints", "Intermediate positions passed through in order. Per-section vectors have len(positions) + 1 entries.");
    waypoints_cls
        .def(nb::init<>())
        .def("__init__", [](Waypoints* w, std::vector<std::vector<double>> positions, std::optional<std::vector<Limits>> per_section_limits, std::optional<std::vector<double>> per_section_minimum_duration) {
            new (w) Waypoints{std::move(positions), std::move(per_section_limits), std::move(per_section_minimum_duration)};
        }, "positions"_a = std::vector<std::vector<double>>{}, "per_section_limits"_a = nb::none(), "per_section_minimum_duration"_a = nb::none())
        .def_rw("positions", &Waypoints::positions)
        .def_rw("per_section_limits", &Waypoints::per_section_limits, nb::arg().none())
        .def_rw("per_section_minimum_duration", &Waypoints::per_section_minimum_duration, nb::arg().none())
        .def_prop_ro("sections", &Waypoints::sections)
        .def("__len__", [](const Waypoints& w) { return w.positions.size(); })
        .def("__eq__", [](const Waypoints& a, const Waypoints& b) { return a == b; }, nb::is_operator())
        .def("__repr__", [](const Waypoints& w) { return std::format("Waypoints({} positions)", w.positions.size()); });
    using WaypointsState = std::tuple<std::vector<std::vector<double>>, std::optional<std::vector<Limits>>, std::optional<std::vector<double>>>;
    add_value_semantics<Waypoints>(waypoints_cls,
        [](const Waypoints& w) { return WaypointsState{w.positions, w.per_section_limits, w.per_section_minimum_duration}; },
        [](Waypoints& w, const WaypointsState& t) { new (&w) Waypoints{std::get<0>(t), std::get<1>(t), std::get<2>(t)}; });

    auto problem_cls = nb::class_<Problem>(m, "Problem", "A complete trajectory generation problem");
    problem_cls
        .def(nb::init<>())
        .def("__init__", [](Problem* p, State start, State goal, Limits limits, std::optional<Options> options, std::optional<Waypoints> waypoints) {
            new (p) Problem{std::move(start), std::move(goal), std::move(limits), options.value_or(Options{}), waypoints.value_or(Waypoints{})};
        }, "start"_a, "goal"_a, "limits"_a, "options"_a = nb::none(), "waypoints"_a = nb::none())
        .def_rw("start", &Problem::start)
        .def_rw("goal", &Problem::goal)
        .def_rw("limits", &Problem::limits)
        .def_rw("options", &Problem::options)
        .def_rw("waypoints", &Problem::waypoints)
        .def_prop_ro("dofs", &Problem::dofs)
        .def("__eq__", [](const Problem& a, const Problem& b) { return a == b; }, nb::is_operator())
        .def("__repr__", [](const Problem& p) { return std::format("Problem({} DoFs, {} waypoints)", p.dofs(), p.waypoints.positions.size()); });
    using ProblemState = std::tuple<State, State, Limits, Options, Waypoints>;
    add_value_semantics<Problem>(problem_cls,
        [](const Problem& p) { return ProblemState{p.start, p.goal, p.limits, p.options, p.waypoints}; },
        [](Problem& p, const ProblemState& t) { new (&p) Problem{std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t), std::get<4>(t)}; });

    // ----- results -----
    nb::class_<Bound>(m, "Bound", "Position extremum of one DoF and the times it is reached")
        .def_ro("min", &Bound::min)
        .def_ro("max", &Bound::max)
        .def_ro("t_min", &Bound::t_min)
        .def_ro("t_max", &Bound::t_max)
        .def("__repr__", [](const Bound& b) { return std::format("Bound(min={:.6g} at t={:.4g}, max={:.6g} at t={:.4g})", b.min, b.t_min, b.max, b.t_max); });

    nb::class_<Sample>(m, "Sample", "Kinematic state of all DoFs at one point in time (numpy views)")
        .def_prop_ro("position", [](nb::handle_t<Sample> self) { return view(nb::cast<Sample&>(self).position, self); }, nb::rv_policy::reference)
        .def_prop_ro("velocity", [](nb::handle_t<Sample> self) { return view(nb::cast<Sample&>(self).velocity, self); }, nb::rv_policy::reference)
        .def_prop_ro("acceleration", [](nb::handle_t<Sample> self) { return view(nb::cast<Sample&>(self).acceleration, self); }, nb::rv_policy::reference)
        .def_prop_ro("jerk", [](nb::handle_t<Sample> self) { return view(nb::cast<Sample&>(self).jerk, self); }, nb::rv_policy::reference)
        .def_ro("time", &Sample::time)
        .def_ro("section", &Sample::section)
        .def("state", &Sample::state, "The sample as a State (copy)")
        .def("__repr__", [](const Sample& s) { return std::format("Sample(t={:.4g}, position={}, velocity={}, acceleration={})", s.time, repr_vec(s.position), repr_vec(s.velocity), repr_vec(s.acceleration)); });

    nb::class_<Online::Step, Sample>(m, "Step", "One control-cycle result of Online.update()")
        .def_ro("finished", &Online::Step::finished, "The goal has been reached")
        .def_ro("replanned", &Online::Step::replanned, "A new trajectory was computed in this cycle");

    nb::class_<Batch>(m, "Batch", "Samples at several times, as (n, dofs) numpy arrays")
        .def_ro("n", &Batch::n)
        .def_ro("dofs", &Batch::dofs)
        .def_prop_ro("time", [](nb::handle_t<Batch> self) { return view(nb::cast<Batch&>(self).time, self); }, nb::rv_policy::reference)
        .def_prop_ro("position", [](nb::handle_t<Batch> self) { Batch& b = nb::cast<Batch&>(self); return view2(b.position, b.n, b.dofs, self); }, nb::rv_policy::reference)
        .def_prop_ro("velocity", [](nb::handle_t<Batch> self) { Batch& b = nb::cast<Batch&>(self); return view2(b.velocity, b.n, b.dofs, self); }, nb::rv_policy::reference)
        .def_prop_ro("acceleration", [](nb::handle_t<Batch> self) { Batch& b = nb::cast<Batch&>(self); return view2(b.acceleration, b.n, b.dofs, self); }, nb::rv_policy::reference)
        .def_prop_ro("jerk", [](nb::handle_t<Batch> self) { Batch& b = nb::cast<Batch&>(self); return view2(b.jerk, b.n, b.dofs, self); }, nb::rv_policy::reference)
        .def_prop_ro("section", [](nb::handle_t<Batch> self) {
            Batch& b = nb::cast<Batch&>(self);
            return nb::ndarray<nb::numpy, std::size_t, nb::ndim<1>>(b.section.data(), {b.n}, self);
        })
        .def("__len__", [](const Batch& b) { return b.n; });

    nb::class_<BrakeProfile>(m, "BrakeProfile")
        .def_ro("duration", &BrakeProfile::duration)
        .def_ro("t", &BrakeProfile::t)
        .def_ro("j", &BrakeProfile::j)
        .def_ro("a", &BrakeProfile::a)
        .def_ro("v", &BrakeProfile::v)
        .def_ro("p", &BrakeProfile::p);

    nb::class_<Profile>(m, "Profile", "Single-DoF profile of one section: seven constant-jerk phases (plus an optional brake pre-trajectory)")
        .def_ro("t", &Profile::t)
        .def_ro("t_sum", &Profile::t_sum)
        .def_ro("j", &Profile::j)
        .def_ro("a", &Profile::a)
        .def_ro("v", &Profile::v)
        .def_ro("p", &Profile::p)
        .def_ro("brake", &Profile::brake)
        .def_ro("pf", &Profile::pf)
        .def_ro("vf", &Profile::vf)
        .def_ro("af", &Profile::af)
        .def_prop_ro("total_duration", &Profile::total_duration)
        .def("__repr__", [](const Profile& p) { return "Profile(" + p.to_string() + ")"; });

    nb::class_<Trajectory>(m, "Trajectory", "A time-parameterized, jerk-limited trajectory for all DoFs")
        .def_prop_ro("dofs", &Trajectory::dofs)
        .def_prop_ro("sections", &Trajectory::sections)
        .def_prop_ro("duration", &Trajectory::duration)
        .def_prop_ro("section_end_times", [](const Trajectory& t) { return copy_array(t.section_end_times()); }, nb::rv_policy::reference)
        .def_prop_ro("independent_min_durations", [](const Trajectory& t) { return copy_array(t.independent_min_durations()); }, nb::rv_policy::reference)
        .def("at", [](const Trajectory& t, double time) { return t.at(time); }, "time"_a, "Kinematic state at a given time")
        .def("sample", [](const Trajectory& t, std::vector<double> times) { return Batch(t.sample(times), t.dofs()); }, "times"_a, "Kinematic states at the given times")
        .def("sample", [](const Trajectory& t, double dt) { return Batch(t.sample(dt), t.dofs()); }, "dt"_a, "Kinematic states every dt seconds, including the final time")
        .def_prop_ro("position_extrema", &Trajectory::position_extrema)
        .def("first_time_at_position", &Trajectory::first_time_at_position, "dof"_a, "position"_a, "after"_a = 0.0)
        .def("profile", &Trajectory::profile, "section"_a, "dof"_a, nb::rv_policy::reference_internal)
        .def("__repr__", [](const Trajectory& t) { return std::format("Trajectory(duration={:.6g} s, {} DoFs, {} sections)", t.duration(), t.dofs(), t.sections()); });

    // ----- planner -----
    nb::class_<Planner>(m, "Planner", "Offline / one-shot trajectory generation")
        .def(nb::init<std::size_t, std::size_t>(), "dofs"_a, "max_waypoints"_a = 0)
        .def_prop_ro("dofs", &Planner::dofs)
        .def("plan", [](Planner& p, const Problem& pb) { return unwrap(p.plan(pb)); }, "problem"_a, "Plan a trajectory (raises CharquiError on failure)")
        .def_static("validate", [](const Problem& pb, bool check_start, bool check_goal) { unwrap(Planner::validate(pb, check_start, check_goal)); },
                    "problem"_a, "check_start_within_limits"_a = false, "check_goal_within_limits"_a = true, "Raise CharquiError if the problem is inconsistent")
        .def_static("filter_waypoints", [](const Problem& pb, std::vector<double> threshold) { return Planner::filter_waypoints(pb, threshold); }, "problem"_a, "threshold"_a,
                    "Drop waypoints within `threshold` (per DoF) of the straight line between their neighbours");

    nb::class_<Online>(m, "Online", "Cyclic trajectory generation that owns the current state and advances it every control cycle")
        .def(nb::init<std::size_t, double, std::size_t>(), "dofs"_a, "delta_time"_a, "max_waypoints"_a = 0)
        .def_prop_ro("dofs", &Online::dofs)
        .def_prop_ro("delta_time", &Online::delta_time)
        .def("update", [](Online& o, const Problem& pb) { return unwrap(o.update(pb)); }, "problem"_a,
             "Advance one control cycle. problem.start is used only for the first update (or after reset()).")
        .def("set_state", &Online::set_state, "state"_a, "Override the tracked state (forces a new trajectory)")
        .def("reset", &Online::reset)
        .def_prop_ro("finished", &Online::finished)
        .def_prop_ro("time", &Online::time)
        .def_prop_ro("trajectory", &Online::trajectory, nb::rv_policy::reference_internal)
        .def_prop_ro("state", &Online::state)
        .def_prop_ro("calculation_duration_us", &Online::calculation_duration_us);
}
