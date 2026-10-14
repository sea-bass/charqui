// Waypoints with a minimum duration per section.
#include <format>
#include <iostream>

#include <charqui/charqui.hpp>

using namespace charqui;

int main() {
    Planner planner(3, 10);

    Problem pb{
        .start = {.position = {0.8, 0.0, 0.5}},
        .goal = {.position = {0.5, 1.2, 0.0}},
        .limits = {.max_velocity = {3.0, 2.0, 2.0}, .max_acceleration = {6.0, 4.0, 4.0}, .max_jerk = {16.0, 10.0, 20.0}},
        .waypoints = {
            .positions = {{1.4, -1.6, 1.0}, {-0.6, -0.5, 0.4}, {-0.4, -0.35, 0.0}, {-0.2, 0.35, -0.1}, {0.2, 0.5, -0.1}, {0.8, 1.8, -0.1}},
            .per_section_minimum_duration = std::vector<double>{0.0, 2.0, 0.0, 1.0, 0.0, 2.0, 0.0},
        },
    };

    auto traj = planner.plan(pb);
    if (!traj) {
        std::cout << std::format("error: {}\n", traj.error().message);
        return 1;
    }

    std::cout << std::format("Trajectory duration: {:.6f} s\n", traj->duration());
    double previous = 0.0;
    for (std::size_t k = 0; double t : traj->section_end_times()) {
        std::cout << std::format("  section {}: {:.4f} s (minimum {:.1f} s)\n", k, t - previous, (*pb.waypoints.per_section_minimum_duration)[k]);
        previous = t;
        ++k;
    }
}
