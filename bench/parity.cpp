// Parity test: identical random problems through Ruckig and charqui must give identical durations and states.
// Usage: parity [cases] [seed]

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <iostream>
#include <string>

#include "common.hpp"

using namespace compare;

namespace {

struct Stats {
    std::size_t cases{0}, both_error{0}, mismatches{0};
    double max_duration_diff{0.0}, max_state_diff{0.0};
};

std::string describe(const charqui::Problem& pb) {
    const auto join = [](const std::vector<double>& v) {
        std::string s;
        for (double x : v) s += std::format("{:.17g} ", x);
        return s;
    };
    return std::format("p0=[{}] v0=[{}] a0=[{}] pf=[{}] vf=[{}] af=[{}] vmax=[{}] amax=[{}] jmax=[{}] sync={} step={}",
                       join(pb.start.position), join(pb.start.velocity), join(pb.start.acceleration), join(pb.goal.position), join(pb.goal.velocity),
                       join(pb.goal.acceleration), join(pb.limits.max_velocity), join(pb.limits.max_acceleration), join(pb.limits.max_jerk),
                       static_cast<int>(pb.options.sync), pb.options.duration_step.value_or(0.0));
}

void compare_one(const charqui::Problem& pb, charqui::Planner& planner, ruckig::Ruckig<ruckig::DynamicDOFs>& otg, Stats& st) {
    ++st.cases;
    const auto in = to_ruckig(pb);
    ruckig::Trajectory<ruckig::DynamicDOFs> rt(pb.dofs());
    const auto rr = otg.calculate(in, rt);
    const bool ruckig_ok = (rr == ruckig::Result::Working || rr == ruckig::Result::Finished);

    auto ct = planner.plan(pb);
    if (!ruckig_ok && !ct) {
        ++st.both_error;
        return;
    }
    if (ruckig_ok != ct.has_value()) {
        ++st.mismatches;
        std::cout << std::format("MISMATCH (ruckig {} / charqui {}): {}\n", static_cast<int>(rr), ct ? "ok" : ct.error().message, describe(pb));
        return;
    }

    const double rd = rt.get_duration();
    const double cd = ct->duration();
    const double ddiff = std::abs(rd - cd);
    st.max_duration_diff = std::max(st.max_duration_diff, ddiff);
    if (ddiff > 1e-8 * std::max(1.0, rd)) {
        ++st.mismatches;
        std::cout << std::format("MISMATCH duration ruckig {:.12g} charqui {:.12g}: {}\n", rd, cd, describe(pb));
        return;
    }

    std::vector<double> p(pb.dofs()), v(pb.dofs()), a(pb.dofs());
    double sdiff = 0.0;
    for (int k = 0; k <= 8; ++k) {
        const double t = rd * k / 8.0;
        rt.at_time(t, p, v, a);
        const charqui::Sample s = ct->at(t);
        for (std::size_t dof = 0; dof < pb.dofs(); ++dof) {
            sdiff = std::max({sdiff, std::abs(p[dof] - s.position[dof]), std::abs(v[dof] - s.velocity[dof]), std::abs(a[dof] - s.acceleration[dof])});
        }
    }
    st.max_state_diff = std::max(st.max_state_diff, sdiff);
    if (sdiff > 1e-6 * std::max(1.0, std::abs(rd))) {
        ++st.mismatches;
        std::cout << std::format("MISMATCH state diff {:.3g}: {}\n", sdiff, describe(pb));
    }
}

void report(const char* name, const Stats& st) {
    std::cout << std::format("{:<28} cases {:>7}  both-error {:>5}  mismatches {:>4}  max |dT| {:.2e}  max |dstate| {:.2e}\n",
                             name, st.cases, st.both_error, st.mismatches, st.max_duration_diff, st.max_state_diff);
}

}  // namespace

int main(int argc, char** argv) {
    const std::size_t n = argc > 1 ? std::strtoull(argv[1], nullptr, 10) : 20000;
    const unsigned seed = argc > 2 ? static_cast<unsigned>(std::strtoul(argv[2], nullptr, 10)) : 42;

    std::normal_distribution<double> position_dist{0.0, 4.0};
    std::normal_distribution<double> dynamic_dist{0.0, 0.8};
    std::uniform_real_distribution<double> limit_dist{0.08, 16.0};
    std::uniform_real_distribution<double> limit_dist_high{10.0, 1000000.0};

    std::size_t total_mismatches = 0;

    // Position interface: time / phase sync, some discrete durations, some minimum durations
    {
        Stats st;
        charqui::Planner planner(3);
        ruckig::Ruckig<ruckig::DynamicDOFs> otg(3, 0.005);
        charqui::Problem pb = blank_problem(3);
        Randomizer p{position_dist, seed + 3};
        Randomizer d{dynamic_dist, seed + 4};
        Randomizer l{limit_dist, seed + 5};
        for (std::size_t i = 0; i < n; ++i) {
            pb.options.sync = (i % 2 == 0) ? charqui::Sync::Time : charqui::Sync::Phase;
            pb.options.duration_step = (i % 20 == 0) ? std::optional<double>(0.005) : std::nullopt;
            pb.options.minimum_duration = (i % 25 == 0) ? std::optional<double>(3.0) : std::nullopt;
            p.fill(pb.start.position);
            d.fill_or_zero(pb.start.velocity, 0.9);
            d.fill_or_zero(pb.start.acceleration, 0.8);
            p.fill(pb.goal.position);
            d.fill_or_zero(pb.goal.velocity, 0.7);
            d.fill_or_zero(pb.goal.acceleration, 0.6);
            l.fill(pb.limits.max_velocity, pb.goal.velocity);
            l.fill(pb.limits.max_acceleration, pb.goal.acceleration);
            l.fill(pb.limits.max_jerk);
            if (!charqui::Planner::validate(pb)) {
                --i;
                continue;
            }
            compare_one(pb, planner, otg, st);
        }
        report("position-3", st);
        total_mismatches += st.mismatches;
    }

    // Position interface, 7 DoF, like Ruckig's benchmark
    {
        Stats st;
        charqui::Planner planner(7);
        ruckig::Ruckig<ruckig::DynamicDOFs> otg(7, 0.005);
        charqui::Problem pb = blank_problem(7);
        std::uniform_real_distribution<double> bench_limit{0.1, 12.0};
        Randomizer p{position_dist, 42u};
        Randomizer d{dynamic_dist, 43u};
        Randomizer l{bench_limit, 44u};
        for (std::size_t i = 0; i < n / 4; ++i) {
            p.fill(pb.start.position);
            d.fill_or_zero(pb.start.velocity, 0.9);
            d.fill_or_zero(pb.start.acceleration, 0.8);
            p.fill(pb.goal.position);
            d.fill_or_zero(pb.goal.velocity, 0.7);
            d.fill_or_zero(pb.goal.acceleration, 0.6);
            l.fill(pb.limits.max_velocity, pb.goal.velocity);
            l.fill(pb.limits.max_acceleration, pb.goal.acceleration);
            l.fill(pb.limits.max_jerk);
            if (!charqui::Planner::validate(pb)) {
                --i;
                continue;
            }
            compare_one(pb, planner, otg, st);
        }
        report("position-7", st);
        total_mismatches += st.mismatches;
    }

    // High limits
    {
        Stats st;
        charqui::Planner planner(3);
        ruckig::Ruckig<ruckig::DynamicDOFs> otg(3, 0.005);
        charqui::Problem pb = blank_problem(3);
        Randomizer p{position_dist, seed + 3};
        Randomizer d{dynamic_dist, seed + 4};
        Randomizer l{limit_dist_high, seed + 5};
        for (std::size_t i = 0; i < n / 10; ++i) {
            p.fill(pb.start.position);
            d.fill_or_zero(pb.start.velocity, 0.1);
            d.fill_or_zero(pb.start.acceleration, 0.1);
            p.fill(pb.goal.position);
            d.fill_or_zero(pb.goal.velocity, 0.1);
            d.fill_or_zero(pb.goal.acceleration, 0.1);
            l.fill(pb.limits.max_velocity, pb.goal.velocity);
            l.fill(pb.limits.max_acceleration, pb.goal.acceleration);
            l.fill(pb.limits.max_jerk);
            if (!charqui::Planner::validate(pb)) {
                --i;
                continue;
            }
            compare_one(pb, planner, otg, st);
        }
        report("position-3-high", st);
        total_mismatches += st.mismatches;
    }

    // Velocity interface
    {
        Stats st;
        charqui::Planner planner(3);
        ruckig::Ruckig<ruckig::DynamicDOFs> otg(3, 0.005);
        charqui::Problem pb = blank_problem(3);
        pb.options.interface = charqui::Interface::Velocity;
        Randomizer d{dynamic_dist, seed + 4};
        Randomizer l{limit_dist, seed + 5};
        for (std::size_t i = 0; i < n / 4; ++i) {
            pb.options.duration_step = (i % 10 == 0) ? std::optional<double>(0.005) : std::nullopt;
            d.fill_or_zero(pb.start.velocity, 0.9);
            d.fill_or_zero(pb.start.acceleration, 0.8);
            d.fill_or_zero(pb.goal.velocity, 0.7);
            d.fill_or_zero(pb.goal.acceleration, 0.6);
            l.fill(pb.limits.max_acceleration, pb.goal.acceleration);
            l.fill(pb.limits.max_jerk);
            l.fill(pb.limits.max_velocity);
            if (!charqui::Planner::validate(pb)) {
                --i;
                continue;
            }
            compare_one(pb, planner, otg, st);
        }
        report("velocity-3", st);
        total_mismatches += st.mismatches;
    }

    // Second order (infinite jerk)
    {
        Stats st;
        charqui::Planner planner(3);
        ruckig::Ruckig<ruckig::DynamicDOFs> otg(3, 0.005);
        charqui::Problem pb = blank_problem(3);
        pb.limits.max_jerk.clear();
        Randomizer p{position_dist, seed + 3};
        Randomizer d{dynamic_dist, seed + 4};
        Randomizer l{limit_dist, seed + 5};
        for (std::size_t i = 0; i < n / 4; ++i) {
            pb.options.sync = (i % 2 == 0) ? charqui::Sync::Time : charqui::Sync::Phase;
            p.fill(pb.start.position);
            d.fill_or_zero(pb.start.velocity, 0.9);
            d.fill_or_zero(pb.start.acceleration, 0.8);
            p.fill(pb.goal.position);
            d.fill_or_zero(pb.goal.velocity, 0.7);
            d.fill_or_zero(pb.goal.acceleration, 0.6);
            l.fill(pb.limits.max_velocity, pb.goal.velocity);
            l.fill(pb.limits.max_acceleration, pb.goal.acceleration);
            if (!charqui::Planner::validate(pb)) {
                --i;
                continue;
            }
            compare_one(pb, planner, otg, st);
        }
        report("position-second-order-3", st);
        total_mismatches += st.mismatches;
    }

    std::cout << (total_mismatches == 0 ? "PARITY OK\n" : std::format("PARITY FAILED: {} mismatches\n", total_mismatches));
    return total_mismatches == 0 ? 0 : 1;
}
