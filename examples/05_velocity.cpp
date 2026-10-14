// Velocity interface: the goal is a velocity and acceleration, the position is free.
#include <format>
#include <iostream>

#include <charqui/charqui.hpp>

using namespace charqui;

int main() {
    Online otg(3, 0.01);

    Problem pb{
        .start = {.position = {0.0, 0.0, 0.5}, .velocity = {3.0, -2.2, -0.5}, .acceleration = {0.0, 2.5, -0.5}},
        .goal = {.velocity = {0.0, -0.5, -1.5}, .acceleration = {0.0, 0.0, 0.5}},
        .limits = {.max_acceleration = {3.0, 2.0, 1.0}, .max_jerk = {6.0, 6.0, 4.0}},
        .options = {.interface = Interface::Velocity},
    };

    std::cout << std::format("t | velocity\n");
    for (const auto& step : otg.run(pb)) {
        std::cout << std::format("{:.3f} | {:.5f}, {:.5f}, {:.5f}\n", step.time, step.velocity[0], step.velocity[1], step.velocity[2]);
    }

    std::cout << std::format("Reached the goal velocity in {:.6f} s.\n", otg.trajectory().duration());
}
