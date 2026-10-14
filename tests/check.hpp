#pragma once

// Strict trajectory checker shared by the tests: limits, goal accuracy, phase continuity.

#include <cmath>
#include <doctest/doctest.h>
#include <format>
#include <random>
#include <string>
#include <vector>

#include <charqui/charqui.hpp>
#include <charqui/detail/resolved.hpp>

namespace charqui::test {

constexpr double kLimitTol = 1e-12;
constexpr double kStateTol = 1e-8;
constexpr double kAccelTol = 1e-10;

//! Verify every section profile of a trajectory against the limits and the boundary states it claims to reach
inline void check_trajectory(const Trajectory& traj, const Problem& problem, double limit_tol = kLimitTol) {
    const std::size_t dofs = traj.dofs();
    REQUIRE(traj.sections() >= 1);
    CHECK(traj.duration() >= 0.0);

    detail::ResolvedProblem rp;
    rp.assign(problem);

    for (std::size_t section = 0; section < traj.sections(); ++section) {
        const Limits& limits = problem.waypoints.per_section_limits ? (*problem.waypoints.per_section_limits)[section] : problem.limits;
        rp.set_limits(limits);
        for (std::size_t dof = 0; dof < dofs; ++dof) {
            if (!rp.enabled[dof]) {
                continue;
            }
            const Profile& p = traj.profile(section, dof);
            const detail::DofLimits& l = rp.limits[dof];
            const bool position = rp.interface[dof] == Interface::Position;
            INFO(std::format("section {} dof {} profile {}", section, dof, p.to_string()));

            // Durations are non-negative and sum up
            double sum = 0.0;
            for (std::size_t i = 0; i < 7; ++i) {
                CHECK(p.t[i] >= 0.0);
                sum += p.t[i];
                CHECK(p.t_sum[i] == doctest::Approx(sum).epsilon(1e-12));
            }

            // Integrating the phases reproduces the stored states. In jerk-free profiles a[i] is the acceleration *during*
            // phase i (and v[i] the velocity during phase i for acceleration-free ones), so only the lower derivatives chain.
            const bool jerk_free = std::isinf(l.j_max);
            const bool accel_free = jerk_free && std::isinf(l.a_max);
            for (std::size_t i = 0; i < 7; ++i) {
                const auto [pp, vv, aa] = detail::integrate(p.t[i], p.p[i], p.v[i], p.a[i], p.j[i]);
                CHECK(std::abs(pp - p.p[i + 1]) < 1e-8);
                if (!accel_free) {
                    CHECK(std::abs(vv - p.v[i + 1]) < 1e-8);
                }
                if (!jerk_free) {
                    CHECK(std::abs(aa - p.a[i + 1]) < 1e-10);
                }
            }

            // Jerk limit
            for (std::size_t i = 0; i < 7; ++i) {
                CHECK(std::abs(p.j[i]) <= l.j_max + limit_tol);
            }

            // Acceleration and velocity limits at phase boundaries (after the brake, velocity may still be outside at the very start)
            if (!jerk_free) {
                for (std::size_t i = 1; i < 8; ++i) {
                    CHECK(p.a[i] <= l.a_max + limit_tol);
                    CHECK(p.a[i] >= l.a_min - limit_tol);
                }
            } else if (!accel_free) {
                for (std::size_t i = 0; i < 7; ++i) {
                    CHECK(p.a[i] <= l.a_max + limit_tol);
                    CHECK(p.a[i] >= l.a_min - limit_tol);
                }
            }
            if (position && !accel_free) {
                // A start state outside the limits may still violate them during the first two phases (as in Ruckig)
                for (std::size_t i = 3; i < 8; ++i) {
                    CHECK(p.v[i] <= l.v_max + limit_tol);
                    CHECK(p.v[i] >= l.v_min - limit_tol);
                }
                // Velocity extrema inside a phase (where acceleration crosses zero)
                for (std::size_t i = 2; i < 7; ++i) {
                    if (p.j[i] != 0.0 && p.a[i] * p.a[i + 1] < 0.0) {
                        const double v_ext = p.v[i] - p.a[i] * p.a[i] / (2 * p.j[i]);
                        CHECK(v_ext <= l.v_max + 1e-9);
                        CHECK(v_ext >= l.v_min - 1e-9);
                    }
                }
            }

            // Goal state
            if (position) {
                CHECK(std::abs(p.p.back() - p.pf) < kStateTol);
            }
            CHECK(std::abs(p.v.back() - p.vf) < kStateTol);
            if (!jerk_free) {
                CHECK(std::abs(p.a.back() - p.af) < kAccelTol);
            }
        }
    }

    // Sections are contiguous in time
    const auto ends = traj.section_end_times();
    double prev = 0.0;
    for (double t : ends) {
        CHECK(t >= prev - 1e-12);
        prev = t;
    }
    CHECK(ends.back() == doctest::Approx(traj.duration()));
}

//! Random inputs following Ruckig's test distributions
template <class Dist>
struct Randomizer {
    Dist dist;
    std::default_random_engine gen;
    std::uniform_real_distribution<double> uniform{0.0, 1.0};

    Randomizer(Dist dist, unsigned seed) : dist(dist), gen(seed) {}

    void fill(std::vector<double>& v) {
        for (auto& x : v) {
            x = dist(gen);
        }
    }
    void fill_or_zero(std::vector<double>& v, double p) {
        for (auto& x : v) {
            x = (uniform(gen) < p) ? dist(gen) : 0.0;
        }
    }
    void fill(std::vector<double>& v, const std::vector<double>& offset) {
        for (std::size_t i = 0; i < v.size(); ++i) {
            v[i] = dist(gen) + std::abs(offset[i]);
        }
    }
    void fill_min(std::vector<double>& v, const std::vector<double>& offset) {
        for (std::size_t i = 0; i < v.size(); ++i) {
            v[i] = dist(gen) - std::abs(offset[i]);
        }
    }
};

inline std::string describe(const Problem& pb) {
    const auto join = [](const std::vector<double>& v) {
        std::string s;
        for (double x : v) {
            s += std::format("{:.17g}, ", x);
        }
        return s;
    };
    return std::format("start p={} v={} a={} goal p={} v={} a={} vmax={} amax={} jmax={}", join(pb.start.position), join(pb.start.velocity), join(pb.start.acceleration),
                       join(pb.goal.position), join(pb.goal.velocity), join(pb.goal.acceleration), join(pb.limits.max_velocity), join(pb.limits.max_acceleration), join(pb.limits.max_jerk));
}

}  // namespace charqui::test
