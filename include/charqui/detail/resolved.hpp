#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <charqui/detail/math.hpp>
#include <charqui/types.hpp>

namespace charqui::detail {

//! Per-DoF kinematic limits with all defaults resolved
struct DofLimits {
    double v_max, v_min, a_max, a_min, j_max;

    [[nodiscard]] bool has_zero() const noexcept { return a_max == 0.0 || a_min == 0.0 || j_max == 0.0; }
};

//! A point-to-point problem with every optional field resolved into dense per-DoF storage.
//! Sized once by the owning Planner so that filling it is allocation-free.
struct ResolvedProblem {
    std::size_t dofs{0};

    std::vector<double> p0, v0, a0;
    std::vector<double> pf, vf, af;
    std::vector<DofLimits> limits;
    std::vector<Interface> interface;
    std::vector<Sync> sync;
    std::vector<std::uint8_t> enabled;

    std::optional<double> minimum_duration;
    std::optional<double> duration_step;

    void resize(std::size_t n) {
        dofs = n;
        p0.resize(n);
        v0.resize(n);
        a0.resize(n);
        pf.resize(n);
        vf.resize(n);
        af.resize(n);
        limits.resize(n);
        interface.resize(n);
        sync.resize(n);
        enabled.resize(n);
    }

    //! Fill the limits from a Limits struct (defaults: infinite acceleration/jerk, symmetric minima)
    void set_limits(const Limits& l) {
        for (std::size_t i = 0; i < dofs; ++i) {
            DofLimits& d = limits[i];
            d.v_max = l.max_velocity.empty() ? kInf : l.max_velocity[i];
            d.a_max = l.max_acceleration.empty() ? kInf : l.max_acceleration[i];
            d.j_max = l.max_jerk.empty() ? kInf : l.max_jerk[i];
            d.v_min = l.min_velocity ? (*l.min_velocity)[i] : -d.v_max;
            d.a_min = l.min_acceleration ? (*l.min_acceleration)[i] : -d.a_max;
        }
    }

    void set_options(const Options& o) {
        for (std::size_t i = 0; i < dofs; ++i) {
            interface[i] = o.per_dof_interface ? (*o.per_dof_interface)[i] : o.interface;
            sync[i] = o.per_dof_sync ? (*o.per_dof_sync)[i] : o.sync;
            enabled[i] = o.enabled ? static_cast<std::uint8_t>((*o.enabled)[i]) : std::uint8_t{1};
        }
        minimum_duration = o.minimum_duration;
        duration_step = o.duration_step;
    }

    static void copy_or_zero(std::vector<double>& dst, const std::vector<double>& src, std::size_t n) {
        if (src.empty()) {
            dst.assign(n, 0.0);
        } else {
            dst.assign(src.begin(), src.begin() + static_cast<std::ptrdiff_t>(n));
        }
    }

    void set_start(const State& s) {
        copy_or_zero(p0, s.position, dofs);
        copy_or_zero(v0, s.velocity, dofs);
        copy_or_zero(a0, s.acceleration, dofs);
    }

    void set_goal(const State& g) {
        copy_or_zero(pf, g.position, dofs);
        copy_or_zero(vf, g.velocity, dofs);
        copy_or_zero(af, g.acceleration, dofs);
    }

    //! Resolve a complete (waypoint-free) problem; the problem must already be validated
    void assign(const Problem& problem) {
        resize(problem.dofs());
        set_start(problem.start);
        set_goal(problem.goal);
        set_limits(problem.limits);
        set_options(problem.options);
    }
};

}  // namespace charqui::detail
