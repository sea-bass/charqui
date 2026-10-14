// Deterministic cases, with the reference values of Ruckig's own test-suite.

#include "check.hpp"

using namespace charqui;
using charqui::test::check_trajectory;

namespace {

bool approx_eq(std::span<const double> a, std::span<const double> b, double eps = 1e-6) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i] - b[i]) > eps) {
            return false;
        }
    }
    return true;
}

}  // namespace

TEST_CASE("trajectory") {
    Planner planner(3);
    Problem pb{
        .start = {.position = {0.0, -2.0, 0.0}},
        .goal = {.position = {1.0, -3.0, 2.0}, .velocity = {0.0, 0.3, 0.0}},
        .limits = {.max_velocity = {1.0, 1.0, 1.0}, .max_acceleration = {1.0, 1.0, 1.0}, .max_jerk = {1.0, 1.0, 1.0}},
    };

    auto traj = planner.plan(pb);
    REQUIRE(traj);
    CHECK(traj->duration() == doctest::Approx(4.0));
    check_trajectory(*traj, pb);

    SUBCASE("at-time") {
        const Sample s0 = traj->at(0.0);
        CHECK(approx_eq(s0.position, pb.start.position));
        CHECK(approx_eq(s0.velocity, std::vector<double>{0.0, 0.0, 0.0}));
        CHECK(approx_eq(s0.acceleration, std::vector<double>{0.0, 0.0, 0.0}));

        const Sample sf = traj->at(traj->duration());
        CHECK(approx_eq(sf.position, pb.goal.position));
        CHECK(approx_eq(sf.velocity, pb.goal.velocity));
        CHECK(approx_eq(sf.acceleration, std::vector<double>{0.0, 0.0, 0.0}));

        const Sample s2 = traj->at(2.0);
        CHECK(approx_eq(s2.position, std::vector<double>{0.5, -2.6871268303, 1.0}));
        CHECK(approx_eq(s2.jerk, std::vector<double>{0.0, 0.0, -1.0}));
        CHECK(s2.section == 0);

        const Sample s5 = traj->at(5.0);
        CHECK(approx_eq(s5.jerk, std::vector<double>{0.0, 0.0, 0.0}));
        CHECK(s5.section == 1);
    }

    SUBCASE("single-dof") {
        Planner planner1(1);
        Problem pb1{
            .start = {.position = {0.0}},
            .goal = {.position = {1.0}},
            .limits = {.max_velocity = {1.0}, .max_acceleration = {1.0}, .max_jerk = {1.0}},
        };
        auto traj1 = planner1.plan(pb1);
        REQUIRE(traj1);
        CHECK(traj1->duration() == doctest::Approx(3.1748));
        CHECK(traj1->at(3.1748 / 2).position[0] == doctest::Approx(0.5));
    }

    SUBCASE("independent-min-durations") {
        const auto d = traj->independent_min_durations();
        CHECK(d[0] == doctest::Approx(3.1748021039));
        CHECK(d[1] == doctest::Approx(3.6860977315));
        CHECK(d[2] == doctest::Approx(traj->duration()));
    }

    SUBCASE("first-time-at-position") {
        CHECK(traj->first_time_at_position(0, 0.0).value() == doctest::Approx(0.0));
        CHECK(traj->first_time_at_position(0, 0.5).value() == doctest::Approx(2.0));
        CHECK(traj->first_time_at_position(0, 1.0).value() == doctest::Approx(4.0));
        CHECK(traj->first_time_at_position(1, -3.0).value() == doctest::Approx(2.6004877902));
        CHECK(traj->first_time_at_position(1, -3.1).value() == doctest::Approx(2.8644154489));
        CHECK(traj->first_time_at_position(2, 0.05).value() == doctest::Approx(0.6694329501));

        CHECK_FALSE(traj->first_time_at_position(0, -1.0));
        CHECK_FALSE(traj->first_time_at_position(1, -3.4));
        CHECK_FALSE(traj->first_time_at_position(6, 0.0));
    }

    SUBCASE("sampling") {
        const auto samples = traj->sample(0.3);
        REQUIRE(samples.size() >= 2);
        CHECK(samples.front().time == 0.0);
        CHECK(samples.back().time == doctest::Approx(traj->duration()));
        for (std::size_t i = 1; i < samples.size(); ++i) {
            CHECK(samples[i].time > samples[i - 1].time);
        }
        const std::vector<double> times{0.0, 1.0, 2.0};
        const auto at_times = traj->sample(times);
        CHECK(approx_eq(at_times[2].position, std::vector<double>{0.5, -2.6871268303, 1.0}));
    }

    SUBCASE("position-extrema") {
        const auto extrema = traj->position_extrema();
        CHECK(extrema[0].min == doctest::Approx(0.0));
        CHECK(extrema[0].max == doctest::Approx(1.0));
        CHECK(extrema[2].max == doctest::Approx(2.0));
    }

    // Goal velocity exceeding the limit is rejected
    pb.goal.velocity = {2.0, 0.3, 0.0};
    auto bad = planner.plan(pb);
    REQUIRE_FALSE(bad);
    CHECK(bad.error().code == ErrorCode::InvalidInput);
    CHECK(bad.error().message.find("exceeds its maximum") != std::string::npos);

    pb.goal.velocity = {0.2, -0.3, 0.8};
    REQUIRE(planner.plan(pb));

    pb.options.minimum_duration = 12.0;
    auto slow = planner.plan(pb);
    REQUIRE(slow);
    CHECK(slow->duration() == doctest::Approx(12.0));
    check_trajectory(*slow, pb);

    // High limits
    Problem high{
        .start = {.position = {1300.0, 0.0, 0.02}, .velocity = {1200.0, 0.0, 0.0}},
        .goal = {.position = {1400.0, 0.0, 0.02}},
        .limits = {.max_velocity = {800.0, 1.0, 1.0}, .max_acceleration = {40000.0, 1.0, 1.0}, .max_jerk = {200000.0, 1.0, 1.0}},
    };
    auto fast = planner.plan(high);
    REQUIRE(fast);
    CHECK(fast->duration() == doctest::Approx(0.167347));
    CHECK(fast->independent_min_durations()[0] == doctest::Approx(fast->duration()));
    CHECK(fast->independent_min_durations()[1] == doctest::Approx(0.0));
    CHECK(fast->independent_min_durations()[2] == doctest::Approx(0.0));
}

TEST_CASE("validation") {
    Problem pb{
        .start = {.position = {0.0, 0.0}},
        .goal = {.position = {1.0, 1.0}},
        .limits = {.max_velocity = {1.0, 1.0}, .max_acceleration = {1.0, 1.0}, .max_jerk = {1.0, 1.0}},
    };
    CHECK(Planner::validate(pb));

    SUBCASE("sizes") {
        pb.goal.position = {1.0};
        CHECK_FALSE(Planner::validate(pb));
    }
    SUBCASE("negative limits") {
        pb.limits.max_jerk = {1.0, -1.0};
        CHECK_FALSE(Planner::validate(pb));
    }
    SUBCASE("nan") {
        pb.start.position = {0.0, std::nan("")};
        CHECK_FALSE(Planner::validate(pb));
    }
    SUBCASE("goal velocity reachability") {
        pb.goal.velocity = {0.0, 1.0};
        pb.goal.acceleration = {0.0, -0.0001};
        auto r = Planner::validate(pb);
        REQUIRE_FALSE(r);
        CHECK(r.error().message.find("will inevitably") != std::string::npos);
    }
    SUBCASE("planner dofs mismatch") {
        Planner planner(3);
        auto r = planner.plan(pb);
        REQUIRE_FALSE(r);
        CHECK(r.error().code == ErrorCode::InvalidInput);
    }
    SUBCASE("empty velocity means zero") {
        Planner planner(2);
        auto a = planner.plan(pb);
        pb.start.velocity = {0.0, 0.0};
        pb.goal.acceleration = {0.0, 0.0};
        auto b = planner.plan(pb);
        REQUIRE(a);
        REQUIRE(b);
        CHECK(a->duration() == doctest::Approx(b->duration()));
    }
}

TEST_CASE("enabled") {
    Planner planner(3);
    Problem pb{
        .start = {.position = {0.0, -2.0, 0.0}, .velocity = {0.0, 0.1, 0.0}, .acceleration = {0.0, 0.0, -0.2}},
        .goal = {.position = {1.0, -3.0, 2.0}},
        .limits = {.max_velocity = {1.0, 1.0, 1.0}, .max_acceleration = {1.0, 1.0, 1.0}, .max_jerk = {1.0, 1.0, 1.0}},
        .options = {.enabled = std::vector<bool>{true, false, false}},
    };

    auto traj = planner.plan(pb);
    REQUIRE(traj);
    CHECK(traj->duration() == doctest::Approx(3.1748021039));

    const Sample s0 = traj->at(0.0);
    CHECK(approx_eq(s0.position, pb.start.position));
    CHECK(approx_eq(s0.velocity, pb.start.velocity));
    CHECK(approx_eq(s0.acceleration, pb.start.acceleration));

    const Sample sf = traj->at(traj->duration());
    CHECK(approx_eq(sf.position, std::vector<double>{1.0, -1.6825197896, -1.0079368399}));

    // Disabled DoFs overwrite prior blocks
    pb.options.enabled = std::vector<bool>{true, true, true};
    pb.start.position = {0.0, 0.0, 0.0};
    pb.goal.position = {100.0, -3000.0, 2000.0};
    pb.goal.velocity = {1.0, 1.0, 1.0};
    REQUIRE(planner.plan(pb));

    pb.options.enabled = std::vector<bool>{false, false, true};
    pb.start = {.position = {0.0, -2.0, 0.0}, .velocity = {0.0, 0.2, 0.0}, .acceleration = {0.0, 0.2, 0.0}};
    pb.goal = {.position = {1.0, -3.0, 2.0}, .velocity = {0.0, 0.0, 0.2}, .acceleration = {0.0, 0.0, -0.1}};
    auto traj2 = planner.plan(pb);
    REQUIRE(traj2);
    CHECK(traj2->duration() == doctest::Approx(3.6578610221));
}

TEST_CASE("phase-synchronization") {
    Planner planner(3);
    Problem pb{
        .start = {.position = {0.0, -2.0, 0.0}},
        .goal = {.position = {1.0, -3.0, 2.0}},
        .limits = {.max_velocity = {1.0, 1.0, 1.0}, .max_acceleration = {1.0, 1.0, 1.0}, .max_jerk = {1.0, 1.0, 1.0}},
        .options = {.sync = Sync::Phase},
    };

    auto traj = planner.plan(pb);
    REQUIRE(traj);
    CHECK(traj->duration() == doctest::Approx(4.0));
    CHECK(approx_eq(traj->profile(0, 0).t, traj->profile(0, 1).t));
    CHECK(approx_eq(traj->profile(0, 0).t, traj->profile(0, 2).t));
    CHECK(approx_eq(traj->at(1.0).position, std::vector<double>{0.0833333333, -2.0833333333, 0.1666666667}));
    check_trajectory(*traj, pb);

    // Non-collinear input falls back to time synchronization
    pb.goal.velocity = {0.3, 0.0, 0.0};
    auto traj2 = planner.plan(pb);
    REQUIRE(traj2);
    check_trajectory(*traj2, pb);

    // All-zero motion
    pb.start = {.position = {0.0, 0.0, 0.0}};
    pb.goal = {.position = {0.0, 0.0, 0.0}};
    REQUIRE(planner.plan(pb));
}

TEST_CASE("zero-limits") {
    Planner planner(3);
    Problem pb{
        .start = {.position = {0.0, -2.0, 0.0}, .velocity = {0.2, 0.0, 0.0}},
        .goal = {.position = {1.0, -3.0, 0.0}, .velocity = {0.2, 0.0, 0.0}},
        .limits = {.max_velocity = {1.0, 1.0, 1.0}, .max_acceleration = {0.0, 1.0, 0.0}, .max_jerk = {0.0, 1.0, 0.0}},
    };
    auto traj = planner.plan(pb);
    REQUIRE(traj);
    CHECK(traj->duration() == doctest::Approx(5.0));

    pb.start = {.position = {0.0, -2.0, 0.0}, .velocity = {-0.2, 0.0, 0.0}, .acceleration = {1.0, 0.0, 0.0}};
    pb.goal = {.position = {0.4, -3.0, 0.0}, .velocity = {0.8, 0.0, 0.0}, .acceleration = {1.0, 0.0, 0.0}};
    pb.limits = {.max_velocity = {1.0, 200.0, 1.0}, .max_acceleration = {1.0, 200.0, 0.0}, .max_jerk = {0.0, 200.0, 0.0}};
    auto r = planner.plan(pb);
    REQUIRE_FALSE(r);
    CHECK(r.error().code == ErrorCode::ZeroLimits);
    CHECK(r.error().message.find("step 1") != std::string::npos);

    pb.goal.position = {0.3, -3.0, 0.0};
    pb.limits = {.max_velocity = {1.0, 2.0, 1.0}, .max_acceleration = {1.0, 2.0, 0.0}, .max_jerk = {0.0, 2.0, 0.0}};
    r = planner.plan(pb);
    REQUIRE_FALSE(r);
    CHECK(r.error().code == ErrorCode::ZeroLimits);
    CHECK(r.error().message.find("other degrees of freedom") != std::string::npos);

    pb.options.interface = Interface::Velocity;
    pb.start = {.position = {0.0, -2.0, 0.0}, .velocity = {-0.2, 0.0, 0.0}, .acceleration = {1.0, 0.0, 0.2}};
    pb.goal = {.position = {0.4, -3.0, 0.0}, .velocity = {0.9, 0.5, 0.4}, .acceleration = {1.0, 0.0, 0.2}};
    pb.limits = {.max_velocity = {1.0, 2.0, 1.0}, .max_acceleration = {1.0, 2.0, 6.0}, .max_jerk = {0.0, 2.0, 0.0}};
    r = planner.plan(pb);
    REQUIRE_FALSE(r);
    CHECK(r.error().code == ErrorCode::ZeroLimits);

    pb.limits.max_jerk = {1.0, 2.0, 0.0};
    traj = planner.plan(pb);
    REQUIRE(traj);
    CHECK(traj->duration() == doctest::Approx(2.0));

    pb.limits.max_jerk = {0.0, 2.0, 20.0};
    traj = planner.plan(pb);
    REQUIRE(traj);
    CHECK(traj->duration() == doctest::Approx(1.1));
}

TEST_CASE("online") {
    Online otg(3, 0.005);
    Problem pb{
        .start = {.position = {0.0, -2.0, 0.0}},
        .goal = {.position = {1.0, -3.0, 2.0}, .velocity = {0.0, 0.3, 0.0}},
        .limits = {.max_velocity = {1.0, 1.0, 1.0}, .max_acceleration = {1.0, 1.0, 1.0}, .max_jerk = {1.0, 1.0, 1.0}},
    };

    std::size_t steps = 0;
    std::size_t replans = 0;
    double last_time = 0.0;
    bool saw_finished = false;
    for (const auto& step : otg.run(pb)) {
        ++steps;
        replans += step.replanned;
        CHECK(step.time > last_time);
        CHECK_FALSE(saw_finished);
        last_time = step.time;
        saw_finished = step.finished;
    }
    CHECK(saw_finished);
    CHECK(replans == 1);
    CHECK(otg.trajectory().duration() == doctest::Approx(4.0));
    CHECK(steps >= 800);
    CHECK(steps <= 801);
    CHECK(otg.finished());
    CHECK(otg.time() == doctest::Approx(4.0));
    CHECK(approx_eq(otg.state().position, pb.goal.position, 1e-9));
    CHECK(approx_eq(otg.state().velocity, pb.goal.velocity, 1e-9));

    // A finished generator holds the goal
    auto held = otg.update(pb);
    REQUIRE(held);
    CHECK(held->finished);
    CHECK(approx_eq(held->position, pb.goal.position, 1e-9));

    // A changed goal triggers exactly one new calculation
    pb.goal.position = {0.5, -3.0, 2.0};
    auto step = otg.update(pb);
    REQUIRE(step);
    CHECK(step->replanned);
    CHECK_FALSE(step->finished);
    step = otg.update(pb);
    REQUIRE(step);
    CHECK_FALSE(step->replanned);

    // Errors are reported, not thrown
    pb.goal.velocity = {5.0, 0.0, 0.0};
    auto bad = otg.update(pb);
    REQUIRE_FALSE(bad);
    CHECK(bad.error().code == ErrorCode::InvalidInput);
}

TEST_CASE("example 01 online") {
    Online otg(3, 0.01);
    Problem pb{
        .start = {.position = {0.0, 0.0, 0.5}, .velocity = {0.0, -2.2, -0.5}, .acceleration = {0.0, 2.5, -0.5}},
        .goal = {.position = {5.0, -2.0, -3.5}, .velocity = {0.0, -0.5, -2.0}, .acceleration = {0.0, 0.0, 0.5}},
        .limits = {.max_velocity = {3.0, 1.0, 3.0}, .max_acceleration = {3.0, 2.0, 1.0}, .max_jerk = {4.0, 3.0, 2.0}},
    };
    for (const auto& step : otg.run(pb)) {
        (void)step;
    }
    check_trajectory(otg.trajectory(), pb);
}

TEST_CASE("velocity interface") {
    Planner planner(3);
    Problem pb{
        .start = {.position = {0.0, 0.0, 0.5}, .velocity = {3.0, -2.2, -0.5}, .acceleration = {0.0, 2.5, -0.5}},
        .goal = {.velocity = {0.0, -0.5, -1.5}, .acceleration = {0.0, 0.0, 0.5}},
        .limits = {.max_acceleration = {3.0, 2.0, 1.0}, .max_jerk = {6.0, 6.0, 4.0}},
        .options = {.interface = Interface::Velocity},
    };
    auto traj = planner.plan(pb);
    REQUIRE(traj);
    check_trajectory(*traj, pb);
    const Sample sf = traj->at(traj->duration());
    CHECK(approx_eq(sf.velocity, pb.goal.velocity));
    CHECK(approx_eq(sf.acceleration, pb.goal.acceleration));
}

TEST_CASE("duration step") {
    Planner planner(3);
    Problem pb{
        .start = {.position = {0.0, -2.0, 0.0}},
        .goal = {.position = {1.0, -3.0, 2.0}, .velocity = {0.0, 0.3, 0.0}},
        .limits = {.max_velocity = {1.0, 1.0, 1.0}, .max_acceleration = {1.0, 1.0, 1.0}, .max_jerk = {1.0, 1.0, 1.0}},
        .options = {.duration_step = 0.07},
    };
    auto traj = planner.plan(pb);
    REQUIRE(traj);
    const double k = traj->duration() / 0.07;
    CHECK(std::abs(k - std::round(k)) < 1e-9);
    CHECK(traj->duration() >= 4.0 - 1e-12);
    check_trajectory(*traj, pb);
}

TEST_CASE("second and first order") {
    Planner planner(2);
    Problem pb{
        .start = {.position = {0.0, 1.0}, .velocity = {0.3, 0.0}},
        .goal = {.position = {2.0, -1.0}},
        .limits = {.max_velocity = {1.0, 2.0}, .max_acceleration = {1.0, 1.5}},
    };
    auto traj = planner.plan(pb);
    REQUIRE(traj);
    check_trajectory(*traj, pb);

    pb.limits.max_acceleration = {};
    pb.start.velocity = {};
    traj = planner.plan(pb);
    REQUIRE(traj);
    CHECK(traj->duration() == doctest::Approx(2.0));
    check_trajectory(*traj, pb);
}
