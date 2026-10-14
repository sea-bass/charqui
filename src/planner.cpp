#include <charqui/planner.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>

#include <charqui/detail/math.hpp>
#include <charqui/detail/resolved.hpp>
#include <charqui/detail/target_solver.hpp>
#include <charqui/detail/waypoint_solver.hpp>

namespace charqui {

namespace {

using detail::v_at_a_zero;

std::unexpected<Error> invalid(std::string message) {
    return std::unexpected(Error{ErrorCode::InvalidInput, std::move(message)});
}

template <class T>
bool size_ok(const std::vector<T>& v, std::size_t dofs, bool optional) {
    return v.size() == dofs || (optional && v.empty());
}

template <class T>
bool size_ok(const std::optional<std::vector<T>>& v, std::size_t dofs) {
    return !v || v->size() == dofs;
}

Result<void> validate_limits(const Limits& limits, std::size_t dofs, std::string_view where) {
    if (!size_ok(limits.max_velocity, dofs, true) || !size_ok(limits.max_acceleration, dofs, true) || !size_ok(limits.max_jerk, dofs, true)
        || !size_ok(limits.min_velocity, dofs) || !size_ok(limits.min_acceleration, dofs)) {
        return invalid(std::format("{}: every limit vector must have {} entries (or be empty for an unlimited quantity)", where, dofs));
    }
    for (std::size_t dof = 0; dof < dofs; ++dof) {
        const double jMax = limits.max_jerk.empty() ? detail::kInf : limits.max_jerk[dof];
        if (std::isnan(jMax) || jMax < 0.0) {
            return invalid(std::format("{}: maximum jerk {} of DoF {} must be >= 0", where, jMax, dof));
        }
        const double aMax = limits.max_acceleration.empty() ? detail::kInf : limits.max_acceleration[dof];
        if (std::isnan(aMax) || aMax < 0.0) {
            return invalid(std::format("{}: maximum acceleration {} of DoF {} must be >= 0", where, aMax, dof));
        }
        const double aMin = limits.min_acceleration ? (*limits.min_acceleration)[dof] : -aMax;
        if (std::isnan(aMin) || aMin > 0.0) {
            return invalid(std::format("{}: minimum acceleration {} of DoF {} must be <= 0", where, aMin, dof));
        }
        if (!limits.max_velocity.empty()) {
            const double vMax = limits.max_velocity[dof];
            if (std::isnan(vMax) || vMax < 0.0) {
                return invalid(std::format("{}: maximum velocity {} of DoF {} must be >= 0", where, vMax, dof));
            }
            const double vMin = limits.min_velocity ? (*limits.min_velocity)[dof] : -vMax;
            if (std::isnan(vMin) || vMin > 0.0) {
                return invalid(std::format("{}: minimum velocity {} of DoF {} must be <= 0", where, vMin, dof));
            }
        }
    }
    return {};
}

}  // namespace

Planner::Planner(std::size_t dofs, std::size_t max_waypoints)
    : dofs_(dofs),
      resolved_(std::make_unique<detail::ResolvedProblem>()),
      target_(std::make_unique<detail::TargetSolver>(dofs)),
      waypoints_(std::make_unique<detail::WaypointSolver>(dofs)) {
    (void)max_waypoints;
    resolved_->resize(dofs);
}

Planner::~Planner() = default;
Planner::Planner(Planner&&) noexcept = default;
Planner& Planner::operator=(Planner&&) noexcept = default;

namespace {

//! Structural checks: sizes, required fields, option ranges
Result<void> validate_sizes(const Problem& problem) {
    const std::size_t dofs = problem.dofs();
    if (dofs == 0) {
        return invalid("start.position must not be empty (it defines the number of DoFs)");
    }

    const Options& o = problem.options;
    if (!size_ok(o.per_dof_interface, dofs) || !size_ok(o.per_dof_sync, dofs) || !size_ok(o.enabled, dofs)) {
        return invalid(std::format("per-DoF options must have {} entries", dofs));
    }
    const bool any_position = o.per_dof_interface
        ? std::any_of(o.per_dof_interface->begin(), o.per_dof_interface->end(), [](Interface i) { return i == Interface::Position; })
        : o.interface == Interface::Position;

    if (!size_ok(problem.start.velocity, dofs, true) || !size_ok(problem.start.acceleration, dofs, true)) {
        return invalid(std::format("start.velocity and start.acceleration must have {} entries or be empty", dofs));
    }
    if (!size_ok(problem.goal.position, dofs, !any_position) || !size_ok(problem.goal.velocity, dofs, true) || !size_ok(problem.goal.acceleration, dofs, true)) {
        return invalid(std::format("goal vectors must have {} entries (velocity and acceleration may be empty)", dofs));
    }
    if (any_position && problem.limits.max_velocity.empty()) {
        return invalid("limits.max_velocity is required for the position interface");
    }
    if (auto r = validate_limits(problem.limits, dofs, "limits"); !r) {
        return r;
    }
    if (o.minimum_duration && (std::isnan(*o.minimum_duration) || *o.minimum_duration < 0.0)) {
        return invalid("options.minimum_duration must be >= 0");
    }
    if (o.duration_step && !(*o.duration_step > 0.0)) {
        return invalid("options.duration_step must be > 0");
    }
    return {};
}

//! Numeric checks on a resolved problem (NaNs, limit signs, reachability of the boundary states)
Result<void> validate_values(const Problem& problem, const detail::ResolvedProblem& rp, bool check_start_within_limits, bool check_goal_within_limits) {
    const std::size_t dofs = rp.dofs;
    const Options& o = problem.options;

    for (std::size_t dof = 0; dof < dofs; ++dof) {
        const detail::DofLimits& l = rp.limits[dof];
        const double a0 = rp.a0[dof], af = rp.af[dof], v0 = rp.v0[dof], vf = rp.vf[dof];
        if (std::isnan(a0) || std::isnan(v0)) {
            return invalid(std::format("start velocity/acceleration of DoF {} must be a valid number", dof));
        }
        if (std::isnan(af) || std::isnan(vf)) {
            return invalid(std::format("goal velocity/acceleration of DoF {} must be a valid number", dof));
        }
        if (check_start_within_limits) {
            if (a0 > l.a_max) {
                return invalid(std::format("start acceleration {} of DoF {} exceeds its maximum {}", a0, dof, l.a_max));
            }
            if (a0 < l.a_min) {
                return invalid(std::format("start acceleration {} of DoF {} undercuts its minimum {}", a0, dof, l.a_min));
            }
        }
        if (check_goal_within_limits) {
            if (af > l.a_max) {
                return invalid(std::format("goal acceleration {} of DoF {} exceeds its maximum {}", af, dof, l.a_max));
            }
            if (af < l.a_min) {
                return invalid(std::format("goal acceleration {} of DoF {} undercuts its minimum {}", af, dof, l.a_min));
            }
        }

        if (rp.interface[dof] != Interface::Position) {
            continue;
        }

        if (std::isnan(rp.p0[dof]) || std::isnan(rp.pf[dof])) {
            return invalid(std::format("start/goal position of DoF {} must be a valid number", dof));
        }
        if (check_start_within_limits) {
            if (v0 > l.v_max) {
                return invalid(std::format("start velocity {} of DoF {} exceeds its maximum {}", v0, dof, l.v_max));
            }
            if (v0 < l.v_min) {
                return invalid(std::format("start velocity {} of DoF {} undercuts its minimum {}", v0, dof, l.v_min));
            }
            if (a0 > 0 && l.j_max > 0 && v_at_a_zero(v0, a0, l.j_max) > l.v_max) {
                return invalid(std::format("DoF {} will inevitably reach velocity {} from its start state, exceeding its maximum {}", dof, v_at_a_zero(v0, a0, l.j_max), l.v_max));
            }
            if (a0 < 0 && l.j_max > 0 && v_at_a_zero(v0, a0, -l.j_max) < l.v_min) {
                return invalid(std::format("DoF {} will inevitably reach velocity {} from its start state, undercutting its minimum {}", dof, v_at_a_zero(v0, a0, -l.j_max), l.v_min));
            }
        }
        if (check_goal_within_limits) {
            if (vf > l.v_max) {
                return invalid(std::format("goal velocity {} of DoF {} exceeds its maximum {}", vf, dof, l.v_max));
            }
            if (vf < l.v_min) {
                return invalid(std::format("goal velocity {} of DoF {} undercuts its minimum {}", vf, dof, l.v_min));
            }
            if (af < 0 && l.j_max > 0 && v_at_a_zero(vf, af, l.j_max) > l.v_max) {
                return invalid(std::format("DoF {} will inevitably have reached velocity {} before its goal state, exceeding its maximum {}", dof, v_at_a_zero(vf, af, l.j_max), l.v_max));
            }
            if (af > 0 && l.j_max > 0 && v_at_a_zero(vf, af, -l.j_max) < l.v_min) {
                return invalid(std::format("DoF {} will inevitably have reached velocity {} before its goal state, undercutting its minimum {}", dof, v_at_a_zero(vf, af, -l.j_max), l.v_min));
            }
        }
    }

    const Waypoints& w = problem.waypoints;
    if (!w.empty()) {
        if (o.interface != Interface::Position || o.per_dof_interface || o.per_dof_sync) {
            return invalid("waypoints require the position interface and a global synchronization");
        }
        if (o.sync == Sync::None) {
            return invalid("waypoints cannot be used with Sync::None");
        }
        if (o.minimum_duration || o.duration_step) {
            return invalid("waypoints cannot be combined with a global minimum duration or a duration step; use per_section_minimum_duration");
        }
        if (problem.limits.max_jerk.empty() || problem.limits.max_acceleration.empty()) {
            return invalid("waypoints require finite jerk and acceleration limits");
        }
        for (std::size_t i = 0; i < w.positions.size(); ++i) {
            if (w.positions[i].size() != dofs) {
                return invalid(std::format("waypoint {} must have {} entries", i, dofs));
            }
            for (double x : w.positions[i]) {
                if (std::isnan(x)) {
                    return invalid(std::format("waypoint {} contains NaN", i));
                }
            }
        }
        if (w.per_section_minimum_duration) {
            if (w.per_section_minimum_duration->size() != w.sections()) {
                return invalid(std::format("per_section_minimum_duration must have {} entries (one per section)", w.sections()));
            }
            for (double d : *w.per_section_minimum_duration) {
                if (std::isnan(d) || d < 0.0) {
                    return invalid("per_section_minimum_duration entries must be >= 0");
                }
            }
        }
        if (w.per_section_limits) {
            if (w.per_section_limits->size() != w.sections()) {
                return invalid(std::format("per_section_limits must have {} entries (one per section)", w.sections()));
            }
            for (std::size_t s = 0; s < w.sections(); ++s) {
                const Limits& sl = (*w.per_section_limits)[s];
                if (sl.max_velocity.empty() || sl.max_acceleration.empty() || sl.max_jerk.empty()) {
                    return invalid(std::format("per_section_limits[{}] must specify velocity, acceleration and jerk limits", s));
                }
                if (auto r = validate_limits(sl, dofs, std::format("per_section_limits[{}]", s)); !r) {
                    return r;
                }
            }
        }
    }

    return {};
}

}  // namespace

Result<void> Planner::validate(const Problem& problem, bool check_start_within_limits, bool check_goal_within_limits) {
    if (auto r = validate_sizes(problem); !r) {
        return r;
    }
    detail::ResolvedProblem rp;
    rp.assign(problem);
    return validate_values(problem, rp, check_start_within_limits, check_goal_within_limits);
}

Result<void> Planner::plan(const Problem& problem, Trajectory& trajectory) {
    if (problem.dofs() != dofs_) {
        return invalid(std::format("problem has {} DoFs but the planner was created for {}", problem.dofs(), dofs_));
    }
    if (auto r = validate_sizes(problem); !r) {
        return r;
    }
    resolved_->assign(problem);
    if (auto r = validate_values(problem, *resolved_, false, true); !r) {
        return r;
    }

    if (!problem.waypoints.empty()) {
        return waypoints_->solve(problem, trajectory);
    }

    trajectory.resize(dofs_, 1);
    double duration = 0.0;
    if (auto r = target_->solve(*resolved_, trajectory.section_profiles(0), trajectory.independent_min_durations_, duration); !r) {
        return r;
    }
    trajectory.duration_ = duration;
    trajectory.section_end_times_[0] = duration;
    return {};
}

Result<Trajectory> Planner::plan(const Problem& problem) {
    Trajectory trajectory(dofs_, problem.waypoints.sections());
    if (auto r = plan(problem, trajectory); !r) {
        return std::unexpected(std::move(r.error()));
    }
    return trajectory;
}

std::vector<std::vector<double>> Planner::filter_waypoints(const Problem& problem, std::span<const double> threshold) {
    const auto& positions = problem.waypoints.positions;
    if (positions.empty()) {
        return positions;
    }

    const std::size_t dofs = problem.dofs();
    const std::size_t n = positions.size();
    std::vector<bool> is_active(n, true);

    std::size_t start = 0;
    for (std::size_t end = start + 2; end < n + 2; ++end) {
        const std::vector<double>& pos_start = (start == 0) ? problem.start.position : positions[start - 1];
        const std::vector<double>& pos_end = (end == n + 1) ? problem.goal.position : positions[end - 1];

        bool are_all_below{true};
        for (std::size_t current = start + 1; current < end; ++current) {
            const std::vector<double>& pos_current = positions[current - 1];

            // Is there a point t on the line within the threshold for every DoF?
            double t_start_max = 0.0;
            double t_end_min = 1.0;
            for (std::size_t dof = 0; dof < dofs; ++dof) {
                const double h0 = (pos_current[dof] - pos_start[dof]) / (pos_end[dof] - pos_start[dof]);
                const double t_start = h0 - threshold[dof] / std::abs(pos_end[dof] - pos_start[dof]);
                const double t_end = h0 + threshold[dof] / std::abs(pos_end[dof] - pos_start[dof]);

                t_start_max = std::max(t_start, t_start_max);
                t_end_min = std::min(t_end, t_end_min);

                if (t_start_max > t_end_min) {
                    are_all_below = false;
                    break;
                }
            }
            if (!are_all_below) {
                break;
            }
        }

        is_active[end - 2] = !are_all_below;
        if (!are_all_below) {
            start = end - 1;
        }
    }

    std::vector<std::vector<double>> filtered;
    filtered.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (is_active[i]) {
            filtered.push_back(positions[i]);
        }
    }
    return filtered;
}

// ----- Online -----

Online::Online(std::size_t dofs, double delta_time, std::size_t max_waypoints)
    : planner_(dofs, max_waypoints), delta_time_(delta_time), trajectory_(dofs, max_waypoints + 1) {
    step_.position.resize(dofs);
    step_.velocity.resize(dofs);
    step_.acceleration.resize(dofs);
    step_.jerk.resize(dofs);
}

void Online::set_state(const State& state) {
    state_ = state;
    has_state_ = true;
    has_trajectory_ = false;
}

void Online::reset() {
    has_state_ = false;
    has_trajectory_ = false;
    time_ = 0.0;
    cycle_ = 0;
    section_ = 0;
    consumed_waypoints_ = 0;
}

bool Online::goal_changed(const Problem& problem) const {
    if (problem.goal != current_.goal || problem.limits != current_.limits || problem.options != current_.options) {
        return true;
    }
    // Waypoints already passed were dropped from current_; compare against the remaining tail
    const auto& mine = current_.waypoints;
    const auto& theirs = problem.waypoints;
    const std::size_t consumed = consumed_waypoints_;
    if (theirs.positions.size() < consumed || theirs.positions.size() - consumed != mine.positions.size()) {
        return true;
    }
    if (!std::equal(theirs.positions.begin() + static_cast<std::ptrdiff_t>(consumed), theirs.positions.end(), mine.positions.begin())) {
        return true;
    }
    if (theirs.per_section_minimum_duration.has_value() != mine.per_section_minimum_duration.has_value()
        || theirs.per_section_limits.has_value() != mine.per_section_limits.has_value()) {
        return true;
    }
    if (theirs.per_section_minimum_duration && (theirs.per_section_minimum_duration->size() < consumed
        || !std::equal(theirs.per_section_minimum_duration->begin() + static_cast<std::ptrdiff_t>(consumed), theirs.per_section_minimum_duration->end(), mine.per_section_minimum_duration->begin(), mine.per_section_minimum_duration->end()))) {
        return true;
    }
    if (theirs.per_section_limits && (theirs.per_section_limits->size() < consumed
        || !std::equal(theirs.per_section_limits->begin() + static_cast<std::ptrdiff_t>(consumed), theirs.per_section_limits->end(), mine.per_section_limits->begin(), mine.per_section_limits->end()))) {
        return true;
    }
    return false;
}

Result<Online::Step> Online::update(const Problem& problem) {
    if (delta_time_ <= 0.0) {
        return invalid(std::format("delta_time {} must be > 0", delta_time_));
    }
    if (problem.dofs() != dofs()) {
        return invalid(std::format("problem has {} DoFs but the generator was created for {}", problem.dofs(), dofs()));
    }

    bool need_plan = !has_trajectory_;
    if (!has_state_) {
        state_ = problem.start;
        has_state_ = true;
        need_plan = true;
    }
    if (!need_plan && goal_changed(problem)) {
        need_plan = true;
    }

    step_.replanned = false;
    if (need_plan) {
        const auto t0 = std::chrono::steady_clock::now();
        current_ = problem;
        current_.start = state_;
        consumed_waypoints_ = 0;
        if (auto r = planner_.plan(current_, trajectory_); !r) {
            has_trajectory_ = false;
            return std::unexpected(std::move(r.error()));
        }
        const auto t1 = std::chrono::steady_clock::now();
        calculation_duration_us_ = std::chrono::duration<double, std::micro>(t1 - t0).count();
        has_trajectory_ = true;
        time_ = 0.0;
        cycle_ = 0;
        section_ = 0;
        step_.replanned = true;
    }

    // Advance, clamping to the end: a finished generator holds the exact goal state
    const std::size_t old_section = section_;
    const double duration = trajectory_.duration();
    ++cycle_;
    time_ = static_cast<double>(cycle_) * delta_time_;
    if (time_ >= duration - 1e-12 * std::max(1.0, duration)) {
        time_ = duration;
    }
    trajectory_.at(time_, step_);
    step_.finished = time_ >= trajectory_.duration();
    section_ = step_.section;

    // Track the state along the trajectory; drop waypoints that have been passed
    state_.position = step_.position;
    state_.velocity = step_.velocity;
    state_.acceleration = step_.acceleration;
    current_.start = state_;
    if (section_ > old_section) {
        auto& w = current_.waypoints;
        const std::size_t passed = std::min(section_ - old_section, w.positions.size());
        if (passed > 0) {
            w.positions.erase(w.positions.begin(), w.positions.begin() + static_cast<std::ptrdiff_t>(passed));
            if (w.per_section_minimum_duration) {
                w.per_section_minimum_duration->erase(w.per_section_minimum_duration->begin(), w.per_section_minimum_duration->begin() + static_cast<std::ptrdiff_t>(passed));
            }
            if (w.per_section_limits) {
                w.per_section_limits->erase(w.per_section_limits->begin(), w.per_section_limits->begin() + static_cast<std::ptrdiff_t>(passed));
            }
            consumed_waypoints_ += passed;
        }
    }

    return step_;
}

}  // namespace charqui
