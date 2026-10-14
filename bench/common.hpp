#pragma once

// Shared helpers for the Ruckig comparison programs: random problem generation following Ruckig's
// benchmark/test distributions, and conversion of a charqui::Problem into a ruckig::InputParameter.

#include <random>
#include <vector>

#include <charqui/charqui.hpp>
#include <ruckig/ruckig.hpp>

namespace compare {

template <class Dist>
struct Randomizer {
    Dist dist;
    std::default_random_engine gen;
    std::uniform_real_distribution<double> uniform{0.0, 1.0};

    Randomizer(Dist dist, unsigned seed) : dist(dist), gen(seed) {}

    void fill(std::vector<double>& v) {
        for (auto& x : v) x = dist(gen);
    }
    void fill_or_zero(std::vector<double>& v, double p) {
        for (auto& x : v) x = (uniform(gen) < p) ? dist(gen) : 0.0;
    }
    void fill(std::vector<double>& v, const std::vector<double>& offset) {
        for (std::size_t i = 0; i < v.size(); ++i) v[i] = dist(gen) + std::abs(offset[i]);
    }
};

inline charqui::Problem blank_problem(std::size_t dofs) {
    charqui::Problem pb;
    for (auto* v : {&pb.start.position, &pb.start.velocity, &pb.start.acceleration, &pb.goal.position, &pb.goal.velocity, &pb.goal.acceleration,
                    &pb.limits.max_velocity, &pb.limits.max_acceleration, &pb.limits.max_jerk}) {
        v->assign(dofs, 0.0);
    }
    return pb;
}

inline ruckig::InputParameter<ruckig::DynamicDOFs> to_ruckig(const charqui::Problem& pb) {
    const std::size_t dofs = pb.dofs();
    ruckig::InputParameter<ruckig::DynamicDOFs> in(dofs);
    const auto or_zero = [dofs](const std::vector<double>& v) { return v.empty() ? std::vector<double>(dofs, 0.0) : v; };
    const auto or_inf = [dofs](const std::vector<double>& v) { return v.empty() ? std::vector<double>(dofs, std::numeric_limits<double>::infinity()) : v; };
    in.current_position = or_zero(pb.start.position);
    in.current_velocity = or_zero(pb.start.velocity);
    in.current_acceleration = or_zero(pb.start.acceleration);
    in.target_position = or_zero(pb.goal.position);
    in.target_velocity = or_zero(pb.goal.velocity);
    in.target_acceleration = or_zero(pb.goal.acceleration);
    in.max_velocity = or_inf(pb.limits.max_velocity);
    in.max_acceleration = or_inf(pb.limits.max_acceleration);
    in.max_jerk = or_inf(pb.limits.max_jerk);
    if (pb.limits.min_velocity) in.min_velocity = *pb.limits.min_velocity;
    if (pb.limits.min_acceleration) in.min_acceleration = *pb.limits.min_acceleration;
    in.control_interface = pb.options.interface == charqui::Interface::Position ? ruckig::ControlInterface::Position : ruckig::ControlInterface::Velocity;
    switch (pb.options.sync) {
        case charqui::Sync::Time: in.synchronization = ruckig::Synchronization::Time; break;
        case charqui::Sync::TimeIfNecessary: in.synchronization = ruckig::Synchronization::TimeIfNecessary; break;
        case charqui::Sync::Phase: in.synchronization = ruckig::Synchronization::Phase; break;
        case charqui::Sync::None: in.synchronization = ruckig::Synchronization::None; break;
    }
    in.duration_discretization = pb.options.duration_step ? ruckig::DurationDiscretization::Discrete : ruckig::DurationDiscretization::Continuous;
    in.minimum_duration = pb.options.minimum_duration;
    if (pb.options.enabled) {
        in.enabled = std::vector<bool>(pb.options.enabled->begin(), pb.options.enabled->end());
    }
    return in;
}

}  // namespace compare
