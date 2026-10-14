// Waypoints in an online loop: the goal changes mid-flight and the remaining waypoints are kept.
#include <format>
#include <iostream>

#include <charqui/charqui.hpp>

using namespace charqui;

int main() {
    Online otg(3, 0.01, 10);

    Problem pb{
        .start = {.position = {0.2, 0.0, -0.3}, .velocity = {0.0, 0.2, 0.0}, .acceleration = {0.0, 0.6, 0.0}},
        .goal = {.position = {0.5, 1.0, 0.0}, .velocity = {0.2, 0.0, 0.3}, .acceleration = {0.0, 0.1, -0.1}},
        .limits = {.max_velocity = {1.0, 2.0, 1.0}, .max_acceleration = {3.0, 2.0, 2.0}, .max_jerk = {6.0, 10.0, 20.0}},
        .waypoints = {.positions = {{1.4, -1.6, 1.0}, {-0.6, -0.5, 0.4}, {-0.4, -0.35, 0.0}, {0.8, 1.8, -0.1}}},
    };

    double t = 0.0;  // wall clock; otg.time() restarts whenever a new trajectory is computed
    while (!otg.finished()) {
        auto step = otg.update(pb);
        if (!step) {
            std::cout << std::format("error: {}\n", step.error().message);
            return 1;
        }
        t += otg.delta_time();
        if (step->replanned) {
            std::cout << std::format("[t = {:.2f}] new trajectory of {:.4f} s through {} remaining waypoints, computed in {:.1f} us\n", t, otg.trajectory().duration(),
                                     otg.trajectory().sections() - 1, otg.calculation_duration_us());
        }

        // After two seconds, move the goal: the generator replans from its current state through the remaining waypoints
        if (t > 2.0 && pb.goal.position[2] == 0.0) {
            pb.goal.position = {0.5, 1.0, 0.5};
        }
    }

    const State& s = otg.state();
    std::cout << std::format("Reached {:.4f}, {:.4f}, {:.4f} at t = {:.2f} s.\n", s.position[0], s.position[1], s.position[2], t);
}
