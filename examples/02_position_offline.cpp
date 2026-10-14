// Offline planning: compute the whole trajectory at once and query it.
#include <format>
#include <iostream>

#include <charqui/charqui.hpp>

using namespace charqui;

int main() {
    Planner planner(3);

    Problem pb{
        .start = {.position = {0.0, 0.0, 0.5}, .velocity = {0.0, -2.2, -0.5}, .acceleration = {0.0, 2.5, -0.5}},
        .goal = {.position = {5.0, -2.0, -3.5}, .velocity = {0.0, -0.5, -2.0}, .acceleration = {0.0, 0.0, 0.5}},
        .limits = {.max_velocity = {3.0, 1.0, 3.0}, .max_acceleration = {3.0, 2.0, 1.0}, .max_jerk = {4.0, 3.0, 2.0},
                   .min_velocity = std::vector<double>{-2.0, -0.5, -3.0}, .min_acceleration = std::vector<double>{-2.0, -2.0, -2.0}},
    };

    auto traj = planner.plan(pb);
    if (!traj) {
        std::cout << std::format("error: {}\n", traj.error().message);
        return 1;
    }

    std::cout << std::format("Trajectory duration: {:.6f} s\n", traj->duration());

    const Sample s = traj->at(1.0);
    std::cout << std::format("At t = 1.0 s: position {:.5f}, {:.5f}, {:.5f}\n", s.position[0], s.position[1], s.position[2]);

    for (std::size_t dof = 0; const Bound& b : traj->position_extrema()) {
        std::cout << std::format("DoF {}: position in [{:.4f}, {:.4f}] (at t = {:.3f} and {:.3f} s)\n", dof++, b.min, b.max, b.t_min, b.t_max);
    }
}
