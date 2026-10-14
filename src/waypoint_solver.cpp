#include <charqui/detail/waypoint_solver.hpp>

#include <algorithm>
#include <cmath>
#include <format>

namespace charqui::detail {

namespace {

constexpr int kBisectionSteps = 8;  // resolution 1/256 of the limit, as in Kiemel & Kröger 2024

[[nodiscard]] double sign(double x) noexcept {
    return (x > 0.0) - (x < 0.0);
}

}  // namespace

WaypointSolver::WaypointSolver(std::size_t dofs) : dofs_(dofs), target_(dofs), probe_solver_(1) {
    section_.resize(dofs);
    probe_.resize(1);
    probe_.interface[0] = Interface::Position;
    probe_.sync[0] = Sync::Time;
    probe_.enabled[0] = 1;
}

bool WaypointSolver::feasible(std::size_t dof, const Limits& limits, double p0, double v0, double a0, double pf, double vf, double af) {
    probe_.p0[0] = p0;
    probe_.v0[0] = v0;
    probe_.a0[0] = a0;
    probe_.pf[0] = pf;
    probe_.vf[0] = vf;
    probe_.af[0] = af;
    DofLimits& l = probe_.limits[0];
    l.v_max = limits.max_velocity[dof];
    l.a_max = limits.max_acceleration[dof];
    l.j_max = limits.max_jerk[dof];
    l.v_min = limits.min_velocity ? (*limits.min_velocity)[dof] : -l.v_max;
    l.a_min = limits.min_acceleration ? (*limits.min_acceleration)[dof] : -l.a_max;

    double duration = 0.0;
    if (!probe_solver_.solve(probe_, {&probe_profile_, 1}, {&probe_min_duration_, 1}, duration)) {
        return false;
    }

    // The segment must stay between its end points: no overshoot, no reversal
    const Bound b = probe_profile_.get_position_extrema();
    const double tol = Profile::p_precision + 1e-9 * std::abs(pf - p0);
    return b.min >= std::min(p0, pf) - tol && b.max <= std::max(p0, pf) + tol;
}

void WaypointSolver::choose_states(const Problem& problem) {
    const Waypoints& w = problem.waypoints;
    const std::size_t m = w.positions.size();
    const auto limits_of = [&](std::size_t section) -> const Limits& {
        return w.per_section_limits ? (*w.per_section_limits)[section] : problem.limits;
    };
    const auto p = [&](std::size_t i, std::size_t dof) { return p_[i * dofs_ + dof]; };

    for (std::size_t dof = 0; dof < dofs_; ++dof) {
        for (std::size_t i = 1; i <= m; ++i) {
            const Candidate prev = states_[(i - 1) * dofs_ + dof];
            const double p_prev = p(i - 1, dof), p_cur = p(i, dof), p_next = p(i + 1, dof);
            const Limits& lim_in = limits_of(i - 1);
            const Limits& lim_out = limits_of(i);

            // State assumed at the following waypoint: the goal state itself for the last waypoint, a stop otherwise
            const Candidate next = (i == m) ? states_[(m + 1) * dofs_ + dof] : Candidate{0.0, 0.0};

            const double d_in = sign(p_cur - p_prev);
            const double d_out = sign(p_next - p_cur);
            const bool turning = d_in * d_out <= 0.0;

            Candidate& chosen = states_[i * dofs_ + dof];
            chosen = {0.0, 0.0};

            // The quantity to maximise and its admissible extreme (within both neighbouring sections' limits)
            double extreme = 0.0;
            if (turning) {
                if (d_out > 0) {
                    extreme = std::min(lim_in.max_acceleration[dof], lim_out.max_acceleration[dof]);
                } else if (d_out < 0) {
                    const double in_min = lim_in.min_acceleration ? (*lim_in.min_acceleration)[dof] : -lim_in.max_acceleration[dof];
                    const double out_min = lim_out.min_acceleration ? (*lim_out.min_acceleration)[dof] : -lim_out.max_acceleration[dof];
                    extreme = std::max(in_min, out_min);
                }
            } else {
                if (d_out > 0) {
                    extreme = std::min(lim_in.max_velocity[dof], lim_out.max_velocity[dof]);
                } else {
                    const double in_min = lim_in.min_velocity ? (*lim_in.min_velocity)[dof] : -lim_in.max_velocity[dof];
                    const double out_min = lim_out.min_velocity ? (*lim_out.min_velocity)[dof] : -lim_out.max_velocity[dof];
                    extreme = std::max(in_min, out_min);
                }
            }
            if (extreme == 0.0 || std::isinf(extreme)) {
                continue;
            }

            const auto candidate = [&](double s) { return turning ? Candidate{0.0, s * extreme} : Candidate{s * extreme, 0.0}; };
            const auto is_feasible = [&](const Candidate& c) {
                return feasible(dof, lim_in, p_prev, prev.v, prev.a, p_cur, c.v, c.a)
                    && feasible(dof, lim_out, p_cur, c.v, c.a, p_next, next.v, next.a);
            };

            // Binary search for the largest feasible fraction of the extreme; zero is always acceptable
            if (is_feasible(candidate(1.0))) {
                chosen = candidate(1.0);
                continue;
            }
            double lo = 0.0, hi = 1.0;
            for (int step = 0; step < kBisectionSteps; ++step) {
                const double mid = 0.5 * (lo + hi);
                if (is_feasible(candidate(mid))) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            chosen = candidate(lo);
        }
    }
}

Result<void> WaypointSolver::solve_sections(const Problem& problem, Trajectory& traj, bool stop_at_every_waypoint) {
    const Waypoints& w = problem.waypoints;
    const std::size_t sections = w.sections();
    traj.resize(dofs_, sections);

    double cumulative = 0.0;
    for (std::size_t k = 0; k < sections; ++k) {
        const Limits& limits = w.per_section_limits ? (*w.per_section_limits)[k] : problem.limits;
        section_.set_limits(limits);
        for (std::size_t dof = 0; dof < dofs_; ++dof) {
            const Candidate s0 = (k == 0 || !stop_at_every_waypoint) ? states_[k * dofs_ + dof] : Candidate{0.0, 0.0};
            const Candidate s1 = (k + 1 == sections || !stop_at_every_waypoint) ? states_[(k + 1) * dofs_ + dof] : Candidate{0.0, 0.0};
            section_.p0[dof] = p_[k * dofs_ + dof];
            section_.v0[dof] = s0.v;
            section_.a0[dof] = s0.a;
            section_.pf[dof] = p_[(k + 1) * dofs_ + dof];
            section_.vf[dof] = s1.v;
            section_.af[dof] = s1.a;
        }
        section_.minimum_duration = w.per_section_minimum_duration ? std::optional<double>((*w.per_section_minimum_duration)[k]) : std::nullopt;

        double duration = 0.0;
        std::span<double> min_durations = (k == 0) ? std::span<double>(traj.independent_min_durations_) : std::span<double>(section_durations_);
        if (auto r = target_.solve(section_, traj.section_profiles(k), min_durations, duration); !r) {
            r.error().message = std::format("section {}: {}", k, r.error().message);
            return r;
        }
        cumulative += duration;
        traj.section_end_times_[k] = cumulative;
    }
    traj.duration_ = cumulative;
    return {};
}

Result<void> WaypointSolver::solve(const Problem& problem, Trajectory& traj) {
    const Waypoints& w = problem.waypoints;
    const std::size_t m = w.positions.size();
    const std::size_t rows = m + 2;

    p_.resize(rows * dofs_);
    states_.resize(rows * dofs_);
    section_durations_.resize(dofs_);

    section_.resize(dofs_);
    section_.set_options(problem.options);
    section_.minimum_duration = std::nullopt;
    section_.duration_step = std::nullopt;

    // Positions and the fixed boundary states
    for (std::size_t dof = 0; dof < dofs_; ++dof) {
        const bool enabled = section_.enabled[dof] != 0;
        const double p_start = problem.start.position[dof];
        p_[dof] = p_start;
        for (std::size_t i = 1; i <= m; ++i) {
            p_[i * dofs_ + dof] = enabled ? w.positions[i - 1][dof] : p_start;
        }
        p_[(m + 1) * dofs_ + dof] = enabled ? problem.goal.position[dof] : p_start;

        states_[dof] = {problem.start.velocity.empty() ? 0.0 : problem.start.velocity[dof],
                        problem.start.acceleration.empty() ? 0.0 : problem.start.acceleration[dof]};
        states_[(m + 1) * dofs_ + dof] = {problem.goal.velocity.empty() ? 0.0 : problem.goal.velocity[dof],
                                          problem.goal.acceleration.empty() ? 0.0 : problem.goal.acceleration[dof]};
    }

    choose_states(problem);

    const auto greedy = solve_sections(problem, traj, false);
    const double greedy_duration = traj.duration();

    // Guarantee: never slower than stopping at every waypoint
    Trajectory& baseline = baseline_;
    const auto stop = solve_sections(problem, baseline, true);
    if (!greedy) {
        if (!stop) {
            return greedy;
        }
        traj = baseline;
        return {};
    }
    if (stop && baseline.duration() < greedy_duration) {
        traj = baseline;
    }
    return {};
}

}  // namespace charqui::detail
