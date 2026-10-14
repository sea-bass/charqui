// Stopping mid-trajectory: switch to the velocity interface with a zero goal, without synchronization.
#include <format>
#include <iostream>

#include <charqui/charqui.hpp>

using namespace charqui;

int main() {
    Online otg(3, 0.01);

    Problem pb{
        .start = {.position = {0.0, 0.0, 0.5}, .velocity = {0.0, -2.2, -0.5}, .acceleration = {0.0, 2.5, -0.5}},
        .goal = {.position = {5.0, -2.0, -3.5}, .velocity = {0.0, -0.5, -2.0}, .acceleration = {0.0, 0.0, 0.5}},
        .limits = {.max_velocity = {3.0, 1.0, 3.0}, .max_acceleration = {3.0, 2.0, 1.0}, .max_jerk = {4.0, 3.0, 2.0}},
    };

    std::cout << std::format("t | position\n");
    bool stopping = false;
    double t = 0.0;  // wall clock; otg.time() restarts when the stop trajectory is computed
    while (!otg.finished()) {
        auto step = otg.update(pb);
        if (!step) {
            std::cout << std::format("error: {}\n", step.error().message);
            return 1;
        }
        t += otg.delta_time();
        std::cout << std::format("{:.3f} | {:.5f}, {:.5f}, {:.5f}\n", t, step->position[0], step->position[1], step->position[2]);

        if (!stopping && t >= 1.0) {
            std::cout << std::format("Stop!\n");
            stopping = true;
            pb.options = {.interface = Interface::Velocity, .sync = Sync::None};
            pb.goal = {.velocity = {0.0, 0.0, 0.0}, .acceleration = {0.0, 0.0, 0.0}};
            pb.limits.max_jerk = {12.0, 10.0, 8.0};
        }
    }

    std::cout << std::format("Stopped after {:.3f} s (stop trajectory: {:.4f} s).\n", t, otg.trajectory().duration());
}
