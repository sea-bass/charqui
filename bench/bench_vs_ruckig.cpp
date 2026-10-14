// Timing benchmark: charqui vs Ruckig on Ruckig's own examples and on Ruckig's randomized benchmark workload.
// Usage: bench_vs_ruckig [repetitions-per-example] [random-trajectories]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <format>
#include <iostream>
#include <numeric>

#include "common.hpp"

using namespace compare;
using Clock = std::chrono::steady_clock;

namespace {

struct Timing {
    double mean_us{0.0}, worst_us{0.0};
};

template <class F>
Timing time_calls(std::size_t n, F&& f) {
    double total = 0.0, worst = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const auto t0 = Clock::now();
        f();
        const auto t1 = Clock::now();
        const double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        total += us;
        worst = std::max(worst, us);
    }
    return {total / static_cast<double>(n), worst};
}

struct Example {
    const char* name;
    charqui::Problem problem;
};

std::vector<Example> examples() {
    using namespace charqui;
    std::vector<Example> ex;
    ex.push_back({"01_position", Problem{
        .start = {.position = {0.0, 0.0, 0.5}, .velocity = {0.0, -2.2, -0.5}, .acceleration = {0.0, 2.5, -0.5}},
        .goal = {.position = {5.0, -2.0, -3.5}, .velocity = {0.0, -0.5, -2.0}, .acceleration = {0.0, 0.0, 0.5}},
        .limits = {.max_velocity = {3.0, 1.0, 3.0}, .max_acceleration = {3.0, 2.0, 1.0}, .max_jerk = {4.0, 3.0, 2.0}}}});
    ex.push_back({"02_position_offline", Problem{
        .start = {.position = {0.0, 0.0, 0.5}, .velocity = {0.0, -2.2, -0.5}, .acceleration = {0.0, 2.5, -0.5}},
        .goal = {.position = {5.0, -2.0, -3.5}, .velocity = {0.0, -0.5, -2.0}, .acceleration = {0.0, 0.0, 0.5}},
        .limits = {.max_velocity = {3.0, 1.0, 3.0}, .max_acceleration = {3.0, 2.0, 1.0}, .max_jerk = {4.0, 3.0, 2.0},
                   .min_velocity = std::vector<double>{-2.0, -0.5, -3.0}, .min_acceleration = std::vector<double>{-2.0, -2.0, -2.0}}}});
    ex.push_back({"05_velocity", Problem{
        .start = {.position = {0.0, 0.0, 0.5}, .velocity = {3.0, -2.2, -0.5}, .acceleration = {0.0, 2.5, -0.5}},
        .goal = {.velocity = {0.0, -0.5, -1.5}, .acceleration = {0.0, 0.0, 0.5}},
        .limits = {.max_acceleration = {3.0, 2.0, 1.0}, .max_jerk = {6.0, 6.0, 4.0}},
        .options = {.interface = Interface::Velocity}}});
    ex.push_back({"06_stop", Problem{
        .start = {.position = {0.0, 0.0, 0.5}, .velocity = {0.0, -2.2, -0.5}, .acceleration = {0.0, 2.5, -0.5}},
        .goal = {.velocity = {0.0, 0.0, 0.0}, .acceleration = {0.0, 0.0, 0.0}},
        .limits = {.max_velocity = {3.0, 1.0, 3.0}, .max_acceleration = {3.0, 2.0, 1.0}, .max_jerk = {12.0, 10.0, 8.0}},
        .options = {.interface = Interface::Velocity, .sync = Sync::None}}});
    ex.push_back({"07_minimum_duration", Problem{
        .start = {.position = {0.0, 0.0, 0.5}, .velocity = {0.0, -2.2, -0.5}, .acceleration = {0.0, 2.5, -0.5}},
        .goal = {.position = {-5.0, -2.0, -3.5}, .velocity = {0.0, -0.5, -2.0}, .acceleration = {0.0, 0.0, 0.5}},
        .limits = {.max_velocity = {3.0, 1.0, 3.0}, .max_acceleration = {3.0, 2.0, 1.0}, .max_jerk = {4.0, 3.0, 2.0}},
        .options = {.minimum_duration = 5.0}}});
    return ex;
}

charqui::Problem waypoint_example(int which) {
    using namespace charqui;
    if (which == 3) {
        return Problem{
            .start = {.position = {0.2, 0.0, -0.3}, .velocity = {0.0, 0.2, 0.0}, .acceleration = {0.0, 0.6, 0.0}},
            .goal = {.position = {0.5, 1.0, 0.0}, .velocity = {0.2, 0.0, 0.3}, .acceleration = {0.0, 0.1, -0.1}},
            .limits = {.max_velocity = {1.0, 2.0, 1.0}, .max_acceleration = {3.0, 2.0, 2.0}, .max_jerk = {6.0, 10.0, 20.0}},
            .waypoints = {.positions = {{1.4, -1.6, 1.0}, {-0.6, -0.5, 0.4}, {-0.4, -0.35, 0.0}, {0.8, 1.8, -0.1}}}};
    }
    return Problem{
        .start = {.position = {0.8, 0.0, 0.5}},
        .goal = {.position = {0.5, 1.2, 0.0}},
        .limits = {.max_velocity = {3.0, 2.0, 2.0}, .max_acceleration = {6.0, 4.0, 4.0}, .max_jerk = {16.0, 10.0, 20.0}},
        .waypoints = {.positions = {{1.4, -1.6, 1.0}, {-0.6, -0.5, 0.4}, {-0.4, -0.35, 0.0}, {-0.2, 0.35, -0.1}, {0.2, 0.5, -0.1}, {0.8, 1.8, -0.1}},
                      .per_section_minimum_duration = std::vector<double>{0.0, 2.0, 0.0, 1.0, 0.0, 2.0, 0.0}}};
}

}  // namespace

int main(int argc, char** argv) {
    const std::size_t reps = argc > 1 ? std::strtoull(argv[1], nullptr, 10) : 20000;
    const std::size_t random_n = argc > 2 ? std::strtoull(argv[2], nullptr, 10) : 32 * 1024;

    std::cout << "| workload | ruckig mean [µs] | ruckig worst [µs] | charqui mean [µs] | charqui worst [µs] | duration ruckig [s] | duration charqui [s] |\n";
    std::cout << "|---|---|---|---|---|---|---|\n";

    // Fixed examples
    for (const Example& ex : examples()) {
        const std::size_t dofs = ex.problem.dofs();
        ruckig::Ruckig<ruckig::DynamicDOFs> otg(dofs, 0.01);
        const auto in = to_ruckig(ex.problem);
        ruckig::Trajectory<ruckig::DynamicDOFs> rt(dofs);
        charqui::Planner planner(dofs);
        charqui::Trajectory ct(dofs);

        const Timing r = time_calls(reps, [&] { otg.calculate(in, rt); });
        const Timing c = time_calls(reps, [&] { (void)planner.plan(ex.problem, ct); });
        std::cout << std::format("| {} | {:.3f} | {:.2f} | {:.3f} | {:.2f} | {:.9f} | {:.9f} |\n", ex.name, r.mean_us, r.worst_us, c.mean_us, c.worst_us, rt.get_duration(), ct.duration());
    }

    // Ruckig's randomized benchmark: 7 DoF, seeds 42/43/44
    {
        constexpr std::size_t dofs = 7;
        std::normal_distribution<double> position_dist{0.0, 4.0};
        std::normal_distribution<double> dynamic_dist{0.0, 0.8};
        std::uniform_real_distribution<double> limit_dist{0.1, 12.0};
        Randomizer p{position_dist, 42u};
        Randomizer d{dynamic_dist, 43u};
        Randomizer l{limit_dist, 44u};

        std::vector<charqui::Problem> problems;
        problems.reserve(random_n);
        charqui::Problem pb = blank_problem(dofs);
        while (problems.size() < random_n) {
            p.fill(pb.start.position);
            d.fill_or_zero(pb.start.velocity, 0.9);
            d.fill_or_zero(pb.start.acceleration, 0.8);
            p.fill(pb.goal.position);
            d.fill_or_zero(pb.goal.velocity, 0.7);
            d.fill_or_zero(pb.goal.acceleration, 0.6);
            l.fill(pb.limits.max_velocity, pb.goal.velocity);
            l.fill(pb.limits.max_acceleration, pb.goal.acceleration);
            l.fill(pb.limits.max_jerk);
            if (charqui::Planner::validate(pb)) {
                problems.push_back(pb);
            }
        }
        std::vector<ruckig::InputParameter<ruckig::DynamicDOFs>> inputs;
        inputs.reserve(random_n);
        for (const auto& q : problems) {
            inputs.push_back(to_ruckig(q));
        }

        ruckig::Ruckig<ruckig::DynamicDOFs> otg(dofs, 0.005);
        ruckig::Trajectory<ruckig::DynamicDOFs> rt(dofs);
        charqui::Planner planner(dofs);
        charqui::Trajectory ct(dofs);

        std::size_t i = 0;
        const Timing r = time_calls(random_n, [&] { otg.calculate(inputs[i++], rt); });
        i = 0;
        const Timing c = time_calls(random_n, [&] { (void)planner.plan(problems[i++], ct); });

        double sum_r = 0.0, sum_c = 0.0;
        for (std::size_t k = 0; k < random_n; ++k) {
            if (otg.calculate(inputs[k], rt) == ruckig::Result::Working && planner.plan(problems[k], ct)) {
                sum_r += rt.get_duration();
                sum_c += ct.duration();
            }
        }
        std::cout << std::format("| random 7-DoF x{} | {:.3f} | {:.2f} | {:.3f} | {:.2f} | {:.6f} (sum) | {:.6f} (sum) |\n", random_n, r.mean_us, r.worst_us, c.mean_us, c.worst_us, sum_r, sum_c);
    }

    // Waypoint examples (charqui only: community Ruckig needs the cloud API for these)
    std::cout << "\n| waypoint workload | charqui mean [µs] | charqui worst [µs] | duration [s] | sections |\n|---|---|---|---|---|\n";
    for (int which : {3, 8}) {
        const charqui::Problem pb = waypoint_example(which);
        charqui::Planner planner(3, 10);
        charqui::Trajectory ct(3, 10);
        const std::size_t n = std::max<std::size_t>(reps / 20, 1);
        const Timing c = time_calls(n, [&] { (void)planner.plan(pb, ct); });
        std::cout << std::format("| 0{}_waypoints | {:.1f} | {:.1f} | {:.6f} | {} |\n", which, c.mean_us, c.worst_us, ct.duration(), ct.sections());
    }
    return 0;
}
