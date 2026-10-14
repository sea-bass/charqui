// Randomized self-consistency tests following Ruckig's distributions, with a stricter checker than Ruckig's own.

#include <cstdlib>

#include "check.hpp"

using namespace charqui;
using namespace charqui::test;

namespace {

std::size_t case_count(std::size_t base) {
    if (const char* env = std::getenv("CHARQUI_TEST_CASES")) {
        return static_cast<std::size_t>(std::strtoull(env, nullptr, 10));
    }
    return base;
}

constexpr unsigned seed = 42;
std::normal_distribution<double> position_dist{0.0, 4.0};
std::normal_distribution<double> dynamic_dist{0.0, 0.8};
std::uniform_real_distribution<double> limit_dist{0.08, 16.0};
std::uniform_real_distribution<double> limit_dist_high{10.0, 1000000.0};
std::uniform_real_distribution<double> min_limit_dist{-16.0, -0.08};

Problem make_problem(std::size_t dofs) {
    Problem pb;
    pb.start.position.resize(dofs);
    pb.start.velocity.resize(dofs);
    pb.start.acceleration.resize(dofs);
    pb.goal.position.resize(dofs);
    pb.goal.velocity.resize(dofs);
    pb.goal.acceleration.resize(dofs);
    pb.limits.max_velocity.resize(dofs);
    pb.limits.max_acceleration.resize(dofs);
    pb.limits.max_jerk.resize(dofs);
    return pb;
}

void check_plan(Planner& planner, const Problem& pb, std::size_t& failures) {
    auto traj = planner.plan(pb);
    if (!traj) {
        if (traj.error().code == ErrorCode::TrajectoryDuration) {
            return;
        }
        ++failures;
        INFO(describe(pb));
        FAIL_CHECK(traj.error().message);
        return;
    }
    INFO(describe(pb));
    check_trajectory(*traj, pb);
    for (double d : traj->independent_min_durations()) {
        CHECK(traj->duration() >= d - 1e-9);
    }
    const Sample sf = traj->at(traj->duration());
    for (std::size_t dof = 0; dof < 3; ++dof) {
        CHECK_FALSE(std::isnan(sf.position[dof]));
        CHECK_FALSE(std::isnan(sf.velocity[dof]));
    }
}

}  // namespace

TEST_CASE("position-random-3") {
    const std::size_t n = case_count(50000);
    Planner planner(3);
    Problem pb = make_problem(3);
    Randomizer p{position_dist, seed + 3};
    Randomizer d{dynamic_dist, seed + 4};
    Randomizer l{limit_dist, seed + 5};
    std::size_t failures = 0;

    for (std::size_t i = 0; i < n; ++i) {
        pb.options.sync = (i < n / 2) ? Sync::Phase : Sync::Time;
        pb.options.duration_step = (i < n / 20) ? std::optional<double>(0.005) : std::nullopt;

        p.fill(pb.start.position);
        d.fill_or_zero(pb.start.velocity, 0.9);
        d.fill_or_zero(pb.start.acceleration, 0.8);
        p.fill(pb.goal.position);
        d.fill_or_zero(pb.goal.velocity, 0.7);
        d.fill_or_zero(pb.goal.acceleration, 0.6);
        l.fill(pb.limits.max_velocity, pb.goal.velocity);
        l.fill(pb.limits.max_acceleration, pb.goal.acceleration);
        l.fill(pb.limits.max_jerk);
        if (!Planner::validate(pb)) {
            --i;
            continue;
        }
        check_plan(planner, pb, failures);
    }
    CHECK(failures == 0);
}

TEST_CASE("random-direction-3") {
    const std::size_t n = case_count(20000);
    Planner planner(3);
    Problem pb = make_problem(3);
    pb.limits.min_velocity = std::vector<double>(3);
    pb.limits.min_acceleration = std::vector<double>(3);
    Randomizer p{position_dist, seed + 3};
    Randomizer d{dynamic_dist, seed + 4};
    Randomizer l{limit_dist, seed + 5};
    Randomizer min_l{min_limit_dist, seed + 5};
    std::size_t failures = 0;

    for (std::size_t i = 0; i < n; ++i) {
        p.fill(pb.start.position);
        d.fill_or_zero(pb.start.velocity, 0.9);
        d.fill_or_zero(pb.start.acceleration, 0.8);
        p.fill(pb.goal.position);
        d.fill_or_zero(pb.goal.velocity, 0.7);
        d.fill_or_zero(pb.goal.acceleration, 0.6);
        l.fill(pb.limits.max_velocity, pb.goal.velocity);
        l.fill(pb.limits.max_acceleration, pb.goal.acceleration);
        l.fill(pb.limits.max_jerk);
        min_l.fill_min(*pb.limits.min_velocity, pb.goal.velocity);
        min_l.fill_min(*pb.limits.min_acceleration, pb.goal.acceleration);
        if (!Planner::validate(pb)) {
            --i;
            continue;
        }
        check_plan(planner, pb, failures);
    }
    CHECK(failures == 0);
}

TEST_CASE("random-3-high") {
    const std::size_t n = case_count(5000);
    Planner planner(3);
    Problem pb = make_problem(3);
    Randomizer p{position_dist, seed + 3};
    Randomizer d{dynamic_dist, seed + 4};
    Randomizer l{limit_dist_high, seed + 5};
    std::size_t failures = 0;

    for (std::size_t i = 0; i < n; ++i) {
        p.fill(pb.start.position);
        d.fill_or_zero(pb.start.velocity, 0.1);
        d.fill_or_zero(pb.start.acceleration, 0.1);
        p.fill(pb.goal.position);
        d.fill_or_zero(pb.goal.velocity, 0.1);
        d.fill_or_zero(pb.goal.acceleration, 0.1);
        l.fill(pb.limits.max_velocity, pb.goal.velocity);
        l.fill(pb.limits.max_acceleration, pb.goal.acceleration);
        l.fill(pb.limits.max_jerk);
        if (!Planner::validate(pb)) {
            --i;
            continue;
        }
        check_plan(planner, pb, failures);
    }
    CHECK(failures == 0);
}

TEST_CASE("velocity-random-3") {
    const std::size_t n = case_count(20000);
    Planner planner(3);
    Problem pb = make_problem(3);
    pb.options.interface = Interface::Velocity;
    Randomizer d{dynamic_dist, seed + 4};
    Randomizer l{limit_dist, seed + 5};
    std::size_t failures = 0;

    for (std::size_t i = 0; i < n; ++i) {
        pb.options.duration_step = (i < n / 10) ? std::optional<double>(0.005) : std::nullopt;
        d.fill_or_zero(pb.start.velocity, 0.9);
        d.fill_or_zero(pb.start.acceleration, 0.8);
        d.fill_or_zero(pb.goal.velocity, 0.7);
        d.fill_or_zero(pb.goal.acceleration, 0.6);
        l.fill(pb.limits.max_acceleration, pb.goal.acceleration);
        l.fill(pb.limits.max_jerk);
        if (!Planner::validate(pb)) {
            --i;
            continue;
        }
        check_plan(planner, pb, failures);
    }
    CHECK(failures == 0);
}

TEST_CASE("position-second-random-3") {
    const std::size_t n = case_count(10000);
    Planner planner(3);
    Problem pb = make_problem(3);
    pb.limits.max_jerk.clear();  // infinite jerk
    Randomizer p{position_dist, seed + 3};
    Randomizer d{dynamic_dist, seed + 4};
    Randomizer l{limit_dist, seed + 5};
    std::size_t failures = 0;

    for (std::size_t i = 0; i < n; ++i) {
        pb.options.sync = (i < n / 2) ? Sync::Phase : Sync::Time;
        pb.options.duration_step = (i < n / 20) ? std::optional<double>(0.005) : std::nullopt;
        p.fill(pb.start.position);
        d.fill_or_zero(pb.start.velocity, 0.9);
        d.fill_or_zero(pb.start.acceleration, 0.8);
        p.fill(pb.goal.position);
        d.fill_or_zero(pb.goal.velocity, 0.7);
        d.fill_or_zero(pb.goal.acceleration, 0.6);
        l.fill(pb.limits.max_velocity, pb.goal.velocity);
        l.fill(pb.limits.max_acceleration, pb.goal.acceleration);
        if (!Planner::validate(pb)) {
            --i;
            continue;
        }
        check_plan(planner, pb, failures);
    }
    CHECK(failures == 0);
}

TEST_CASE("velocity-second-random-3") {
    const std::size_t n = case_count(10000);
    Planner planner(3);
    Problem pb = make_problem(3);
    pb.options.interface = Interface::Velocity;
    pb.limits.max_jerk.clear();
    Randomizer d{dynamic_dist, seed + 4};
    Randomizer l{limit_dist, seed + 5};
    std::size_t failures = 0;

    for (std::size_t i = 0; i < n; ++i) {
        d.fill_or_zero(pb.start.velocity, 0.9);
        d.fill_or_zero(pb.start.acceleration, 0.8);
        d.fill_or_zero(pb.goal.velocity, 0.7);
        d.fill_or_zero(pb.goal.acceleration, 0.6);
        l.fill(pb.limits.max_acceleration, pb.goal.acceleration);
        if (!Planner::validate(pb)) {
            --i;
            continue;
        }
        check_plan(planner, pb, failures);
    }
    CHECK(failures == 0);
}

TEST_CASE("online-random-stepping") {
    // Step through random trajectories cycle by cycle; the generator must never replan on its own
    const std::size_t n = case_count(200);
    Online otg(3, 0.01);
    Problem pb = make_problem(3);
    Randomizer p{position_dist, seed + 13};
    Randomizer d{dynamic_dist, seed + 14};
    Randomizer l{limit_dist, seed + 15};

    for (std::size_t i = 0; i < n; ++i) {
        p.fill(pb.start.position);
        d.fill_or_zero(pb.start.velocity, 0.9);
        d.fill_or_zero(pb.start.acceleration, 0.8);
        p.fill(pb.goal.position);
        d.fill_or_zero(pb.goal.velocity, 0.7);
        d.fill_or_zero(pb.goal.acceleration, 0.6);
        l.fill(pb.limits.max_velocity, pb.goal.velocity);
        l.fill(pb.limits.max_acceleration, pb.goal.acceleration);
        l.fill(pb.limits.max_jerk);
        if (!Planner::validate(pb)) {
            --i;
            continue;
        }
        otg.reset();
        std::size_t replans = 0;
        std::size_t steps = 0;
        for (const auto& step : otg.run(pb)) {
            replans += step.replanned;
            if (++steps > 2'000'000) {
                break;
            }
        }
        INFO(describe(pb));
        CHECK(replans == 1);
        for (std::size_t dof = 0; dof < 3; ++dof) {
            CHECK(std::abs(otg.state().position[dof] - pb.goal.position[dof]) < 1e-5);
        }
    }
}
