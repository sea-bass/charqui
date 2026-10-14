// Minimum duration: the trajectory is stretched to take at least the given time.
#include <format>
#include <iostream>

#include <charqui/charqui.hpp>

using namespace charqui;

int main() {
    Online otg(3, 0.01);

    Problem pb{
        .start = {.position = {0.0, 0.0, 0.5}, .velocity = {0.0, -2.2, -0.5}, .acceleration = {0.0, 2.5, -0.5}},
        .goal = {.position = {-5.0, -2.0, -3.5}, .velocity = {0.0, -0.5, -2.0}, .acceleration = {0.0, 0.0, 0.5}},
        .limits = {.max_velocity = {3.0, 1.0, 3.0}, .max_acceleration = {3.0, 2.0, 1.0}, .max_jerk = {4.0, 3.0, 2.0}},
        .options = {.minimum_duration = 5.0},
    };

    std::cout << std::format("t | position\n");
    for (const auto& step : otg.run(pb)) {
        std::cout << std::format("{:.3f} | {:.5f}, {:.5f}, {:.5f}\n", step.time, step.position[0], step.position[1], step.position[2]);
    }

    std::cout << std::format("Reached the goal in {:.6f} s.\n", otg.trajectory().duration());
}
