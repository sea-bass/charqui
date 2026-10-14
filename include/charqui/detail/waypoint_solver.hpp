#pragma once

#include <cstddef>
#include <vector>

#include <charqui/detail/resolved.hpp>
#include <charqui/detail/target_solver.hpp>
#include <charqui/trajectory.hpp>
#include <charqui/types.hpp>

namespace charqui::detail {

//! Solves a problem with intermediate waypoints: chooses a pass-through state per DoF and waypoint,
//! then solves and synchronizes every section with the TargetSolver.
class WaypointSolver {
public:
    explicit WaypointSolver(std::size_t dofs);

    //! `problem` must be validated and contain at least one waypoint. Writes all sections of `traj`.
    [[nodiscard]] Result<void> solve(const Problem& problem, Trajectory& traj);

private:
    struct Candidate {
        double v, a;
    };

    std::size_t dofs_;
    TargetSolver target_;
    ResolvedProblem section_;
    ResolvedProblem probe_;        // single-DoF scratch problem for feasibility tests
    TargetSolver probe_solver_;
    Profile probe_profile_;
    double probe_min_duration_{0.0};

    std::vector<double> p_;                 // waypoint positions, flat [waypoint * dofs + dof], including start and goal
    std::vector<Candidate> states_;         // chosen state at every waypoint, flat
    std::vector<double> section_durations_;
    Trajectory baseline_;                   // stop-at-every-waypoint fallback

    [[nodiscard]] bool feasible(std::size_t dof, const Limits& limits, double p0, double v0, double a0, double pf, double vf, double af);
    void choose_states(const Problem& problem);
    [[nodiscard]] Result<void> solve_sections(const Problem& problem, Trajectory& traj, bool stop_at_every_waypoint);
};

}  // namespace charqui::detail
