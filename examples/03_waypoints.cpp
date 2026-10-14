// Intermediate waypoints: the trajectory passes through every waypoint before reaching the goal.
#include <format>
#include <iostream>

#include <charqui/charqui.hpp>

using namespace charqui;

int main() {
    Online otg(3, 0.01, 10);  // pre-allocate for up to 10 waypoints

    Problem pb{
        .start = {.position = {0.2, 0.0, -0.3}, .velocity = {0.0, 0.2, 0.0}, .acceleration = {0.0, 0.6, 0.0}},
        .goal = {.position = {0.5, 1.0, 0.0}, .velocity = {0.2, 0.0, 0.3}, .acceleration = {0.0, 0.1, -0.1}},
        .limits = {.max_velocity = {1.0, 2.0, 1.0}, .max_acceleration = {3.0, 2.0, 2.0}, .max_jerk = {6.0, 10.0, 20.0}},
        .waypoints = {.positions = {{1.4, -1.6, 1.0}, {-0.6, -0.5, 0.4}, {-0.4, -0.35, 0.0}, {0.8, 1.8, -0.1}}},
    };

    std::cout << std::format("t | section | position\n");
    for (const auto& step : otg.run(pb)) {
        std::cout << std::format("{:.2f} | {} | {:.5f}, {:.5f}, {:.5f}\n", step.time, step.section, step.position[0], step.position[1], step.position[2]);
    }

    std::cout << std::format("Reached the goal in {:.6f} s (calculated in {:.1f} us).\n", otg.trajectory().duration(), otg.calculation_duration_us());
    for (std::size_t k = 0; double t : otg.trajectory().section_end_times()) {
        std::cout << std::format("  section {} ends at {:.4f} s\n", k++, t);
    }
}
