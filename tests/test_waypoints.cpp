#include "check.hpp"

using namespace charqui;
using namespace charqui::test;

namespace {

Problem example_03() {
    return Problem{
        .start = {.position = {0.2, 0.0, -0.3}, .velocity = {0.0, 0.2, 0.0}, .acceleration = {0.0, 0.6, 0.0}},
        .goal = {.position = {0.5, 1.0, 0.0}, .velocity = {0.2, 0.0, 0.3}, .acceleration = {0.0, 0.1, -0.1}},
        .limits = {.max_velocity = {1.0, 2.0, 1.0}, .max_acceleration = {3.0, 2.0, 2.0}, .max_jerk = {6.0, 10.0, 20.0}},
        .waypoints = {.positions = {{1.4, -1.6, 1.0}, {-0.6, -0.5, 0.4}, {-0.4, -0.35, 0.0}, {0.8, 1.8, -0.1}}},
    };
}

Problem example_08() {
    return Problem{
        .start = {.position = {0.8, 0.0, 0.5}},
        .goal = {.position = {0.5, 1.2, 0.0}},
        .limits = {.max_velocity = {3.0, 2.0, 2.0}, .max_acceleration = {6.0, 4.0, 4.0}, .max_jerk = {16.0, 10.0, 20.0}},
        .waypoints = {
            .positions = {{1.4, -1.6, 1.0}, {-0.6, -0.5, 0.4}, {-0.4, -0.35, 0.0}, {-0.2, 0.35, -0.1}, {0.2, 0.5, -0.1}, {0.8, 1.8, -0.1}},
            .per_section_minimum_duration = std::vector<double>{0.0, 2.0, 0.0, 1.0, 0.0, 2.0, 0.0},
        },
    };
}

//! Every waypoint is reached exactly at its section end, states are continuous across sections
void check_waypoints(const Trajectory& traj, const Problem& pb) {
    const auto& w = pb.waypoints;
    REQUIRE(traj.sections() == w.sections());
    const auto ends = traj.section_end_times();
    for (std::size_t i = 0; i < w.positions.size(); ++i) {
        const Sample s = traj.at(ends[i]);
        const Sample before = traj.at(std::nextafter(ends[i], 0.0));
        for (std::size_t dof = 0; dof < traj.dofs(); ++dof) {
            INFO("waypoint " << i << " dof " << dof);
            CHECK(std::abs(s.position[dof] - w.positions[i][dof]) < 1e-7);
            CHECK(std::abs(s.position[dof] - before.position[dof]) < 1e-7);
            CHECK(std::abs(s.velocity[dof] - before.velocity[dof]) < 1e-7);
            CHECK(std::abs(s.acceleration[dof] - before.acceleration[dof]) < 1e-7);
        }
    }
    const Sample sf = traj.at(traj.duration());
    for (std::size_t dof = 0; dof < traj.dofs(); ++dof) {
        CHECK(std::abs(sf.position[dof] - pb.goal.position[dof]) < 1e-7);
    }
}

double stop_at_every_waypoint_duration(Planner& planner, const Problem& pb) {
    // Chain of point-to-point problems with zero velocity/acceleration at every waypoint
    double total = 0.0;
    std::vector<std::vector<double>> points{pb.start.position};
    points.insert(points.end(), pb.waypoints.positions.begin(), pb.waypoints.positions.end());
    points.push_back(pb.goal.position);
    for (std::size_t k = 0; k + 1 < points.size(); ++k) {
        Problem seg{.start = {.position = points[k]}, .goal = {.position = points[k + 1]}, .limits = pb.limits};
        if (k == 0) {
            seg.start = pb.start;
        }
        if (k + 2 == points.size()) {
            seg.goal = pb.goal;
        }
        if (pb.waypoints.per_section_minimum_duration) {
            seg.options.minimum_duration = (*pb.waypoints.per_section_minimum_duration)[k];
        }
        auto t = planner.plan(seg);
        REQUIRE(t);
        total += t->duration();
    }
    return total;
}

}  // namespace

TEST_CASE("waypoints example 03") {
    Planner planner(3, 10);
    const Problem pb = example_03();
    auto traj = planner.plan(pb);
    REQUIRE(traj);
    CHECK(traj->sections() == 5);
    check_trajectory(*traj, pb);
    check_waypoints(*traj, pb);

    const double baseline = stop_at_every_waypoint_duration(planner, pb);
    CHECK(traj->duration() <= baseline + 1e-9);
    MESSAGE("example 03: " << traj->duration() << " s (stop at every waypoint: " << baseline << " s)");
}

TEST_CASE("waypoints example 08 with per-section minimum durations") {
    Planner planner(3, 10);
    const Problem pb = example_08();
    auto traj = planner.plan(pb);
    REQUIRE(traj);
    check_trajectory(*traj, pb);
    check_waypoints(*traj, pb);

    const auto ends = traj->section_end_times();
    double prev = 0.0;
    for (std::size_t k = 0; k < ends.size(); ++k) {
        CHECK(ends[k] - prev >= (*pb.waypoints.per_section_minimum_duration)[k] - 1e-9);
        prev = ends[k];
    }
    MESSAGE("example 08: " << traj->duration() << " s");
}

TEST_CASE("waypoints online loop consumes sections") {
    Online otg(3, 0.01, 10);
    const Problem pb = example_03();
    std::size_t replans = 0;
    std::size_t max_section = 0;
    std::size_t steps = 0;
    for (const auto& step : otg.run(pb)) {
        replans += step.replanned;
        CHECK(step.section >= max_section);
        max_section = std::max(max_section, step.section);
        ++steps;
    }
    CHECK(replans == 1);
    CHECK(max_section == 4);
    CHECK(steps > 0);
    for (std::size_t dof = 0; dof < 3; ++dof) {
        CHECK(std::abs(otg.state().position[dof] - pb.goal.position[dof]) < 1e-5);
    }
}

TEST_CASE("waypoint validation") {
    Problem pb = example_03();
    pb.options.minimum_duration = 1.0;
    CHECK_FALSE(Planner::validate(pb));
    pb.options.minimum_duration = std::nullopt;
    pb.waypoints.positions[1] = {0.0};
    CHECK_FALSE(Planner::validate(pb));
    pb = example_03();
    pb.waypoints.per_section_minimum_duration = std::vector<double>{1.0};
    CHECK_FALSE(Planner::validate(pb));
}

TEST_CASE("filter waypoints") {
    Problem pb{
        .start = {.position = {0.0, 0.0}},
        .goal = {.position = {4.0, 0.0}},
        .limits = {.max_velocity = {1.0, 1.0}, .max_acceleration = {1.0, 1.0}, .max_jerk = {1.0, 1.0}},
        .waypoints = {.positions = {{1.0, 0.01}, {2.0, 0.02}, {3.0, 1.0}}},
    };
    // The first waypoint lies within the corridor of the chord to its successor and is dropped
    const auto filtered = Planner::filter_waypoints(pb, std::vector<double>{0.1, 0.1});
    REQUIRE(filtered.size() == 2);
    CHECK(filtered[0][0] == doctest::Approx(2.0));
    CHECK(filtered[1][0] == doctest::Approx(3.0));
}

TEST_CASE("waypoints random") {
    const std::size_t n = 300;
    Planner planner(3, 8);
    std::default_random_engine gen(7);
    std::normal_distribution<double> position_dist{0.0, 2.0};
    std::uniform_real_distribution<double> limit_dist{0.5, 8.0};
    std::uniform_int_distribution<std::size_t> count_dist{1, 8};

    for (std::size_t i = 0; i < n; ++i) {
        Problem pb;
        pb.start.position = {position_dist(gen), position_dist(gen), position_dist(gen)};
        pb.goal.position = {position_dist(gen), position_dist(gen), position_dist(gen)};
        pb.limits.max_velocity = {limit_dist(gen), limit_dist(gen), limit_dist(gen)};
        pb.limits.max_acceleration = {limit_dist(gen), limit_dist(gen), limit_dist(gen)};
        pb.limits.max_jerk = {limit_dist(gen), limit_dist(gen), limit_dist(gen)};
        const std::size_t count = count_dist(gen);
        for (std::size_t k = 0; k < count; ++k) {
            pb.waypoints.positions.push_back({position_dist(gen), position_dist(gen), position_dist(gen)});
        }
        auto traj = planner.plan(pb);
        INFO(describe(pb) << " waypoints=" << count);
        REQUIRE(traj);
        check_trajectory(*traj, pb);
        check_waypoints(*traj, pb);
        CHECK(traj->duration() <= stop_at_every_waypoint_duration(planner, pb) + 1e-9);
    }
}
