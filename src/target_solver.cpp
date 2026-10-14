#include <charqui/detail/target_solver.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <format>
#include <limits>
#include <numeric>

#include <charqui/detail/steps.hpp>

namespace charqui::detail {

namespace {

constexpr double eps = std::numeric_limits<double>::epsilon();
constexpr double kMaxDuration = 7.6e3;

std::string describe_dof(const ResolvedProblem& rp, std::size_t dof) {
    const DofLimits& l = rp.limits[dof];
    return std::format("dof {}: p0={} v0={} a0={} pf={} vf={} af={} vmax={} vmin={} amax={} amin={} jmax={}",
                       dof, rp.p0[dof], rp.v0[dof], rp.a0[dof], rp.pf[dof], rp.vf[dof], rp.af[dof],
                       l.v_max, l.v_min, l.a_max, l.a_min, l.j_max);
}

}  // namespace

TargetSolver::TargetSolver(std::size_t dofs)
    : dofs_(dofs), blocks_(dofs), new_phase_control_(dofs), pd_(dofs), possible_t_syncs_(3 * dofs + 1), idx_(3 * dofs + 1) {}

bool TargetSolver::is_input_collinear(const ResolvedProblem& rp, Profile::Direction limiting_direction, std::size_t limiting_dof) {
    // Check that the vectors pd, v0, a0, vf, af are collinear
    for (std::size_t dof = 0; dof < dofs_; ++dof) {
        pd_[dof] = rp.pf[dof] - rp.p0[dof];
    }

    const std::vector<double>* scale_vector = nullptr;
    std::optional<std::size_t> scale_dof;  // The limiting DoF might not be phase synchronized
    for (std::size_t dof = 0; dof < dofs_; ++dof) {
        if (rp.sync[dof] != Sync::Phase) {
            continue;
        }

        if (rp.interface[dof] == Interface::Position && std::abs(pd_[dof]) > eps) {
            scale_vector = &pd_;
            scale_dof = dof;
            break;
        } else if (std::abs(rp.v0[dof]) > eps) {
            scale_vector = &rp.v0;
            scale_dof = dof;
            break;
        } else if (std::abs(rp.a0[dof]) > eps) {
            scale_vector = &rp.a0;
            scale_dof = dof;
            break;
        } else if (std::abs(rp.vf[dof]) > eps) {
            scale_vector = &rp.vf;
            scale_dof = dof;
            break;
        } else if (std::abs(rp.af[dof]) > eps) {
            scale_vector = &rp.af;
            scale_dof = dof;
            break;
        }
    }

    if (!scale_dof) {
        return false;  // All zero is trivially collinear, but better handled elsewhere
    }

    const double scale = (*scale_vector)[*scale_dof];
    const double pd_scale = pd_[*scale_dof] / scale;
    const double v0_scale = rp.v0[*scale_dof] / scale;
    const double vf_scale = rp.vf[*scale_dof] / scale;
    const double a0_scale = rp.a0[*scale_dof] / scale;
    const double af_scale = rp.af[*scale_dof] / scale;

    const double scale_limiting = (*scale_vector)[limiting_dof];
    const DofLimits& ll = rp.limits[limiting_dof];
    double control_limiting = (limiting_direction == Profile::Direction::UP) ? ll.j_max : -ll.j_max;
    if (std::isinf(ll.j_max)) {
        control_limiting = (limiting_direction == Profile::Direction::UP) ? ll.a_max : ll.a_min;
    }

    for (std::size_t dof = 0; dof < dofs_; ++dof) {
        if (rp.sync[dof] != Sync::Phase) {
            continue;
        }

        const double current_scale = (*scale_vector)[dof];
        if ((rp.interface[dof] == Interface::Position && std::abs(pd_[dof] - pd_scale * current_scale) > eps)
            || std::abs(rp.v0[dof] - v0_scale * current_scale) > eps
            || std::abs(rp.a0[dof] - a0_scale * current_scale) > eps
            || std::abs(rp.vf[dof] - vf_scale * current_scale) > eps
            || std::abs(rp.af[dof] - af_scale * current_scale) > eps) {
            return false;
        }

        new_phase_control_[dof] = control_limiting * current_scale / scale_limiting;
    }

    return true;
}

bool TargetSolver::synchronize(const ResolvedProblem& rp, double& t_sync, std::optional<std::size_t>& limiting_dof, std::span<Profile> profiles) {
    const std::optional<double> t_min = rp.minimum_duration;

    // Candidate durations: every block's t_min and the right edges of its blocked intervals, plus the optional minimum
    bool any_interval{false};
    for (std::size_t dof = 0; dof < dofs_; ++dof) {
        if (rp.sync[dof] == Sync::None) {
            possible_t_syncs_[dof] = 0.0;
            possible_t_syncs_[dofs_ + dof] = kInf;
            possible_t_syncs_[2 * dofs_ + dof] = kInf;
            continue;
        }

        const Block& b = blocks_[dof];
        possible_t_syncs_[dof] = b.t_min;
        possible_t_syncs_[dofs_ + dof] = b.a ? b.a->right : kInf;
        possible_t_syncs_[2 * dofs_ + dof] = b.b ? b.b->right : kInf;
        any_interval |= b.a.has_value() || b.b.has_value();
    }
    possible_t_syncs_[3 * dofs_] = t_min.value_or(kInf);
    any_interval |= t_min.has_value();

    if (rp.duration_step) {
        const double step = *rp.duration_step;
        for (auto& possible_t_sync : possible_t_syncs_) {
            if (std::isinf(possible_t_sync)) {
                continue;
            }
            const double remainder = std::fmod(possible_t_sync, step);  // in [0, step)
            if (remainder > eps) {
                possible_t_sync += step - remainder;
            }
        }
    }

    // Test the candidates in sorted order
    const auto idx_end = any_interval ? idx_.end() : idx_.begin() + static_cast<std::ptrdiff_t>(dofs_);
    std::iota(idx_.begin(), idx_end, std::size_t{0});
    std::sort(idx_.begin(), idx_end, [&](std::size_t i, std::size_t j) { return possible_t_syncs_[i] < possible_t_syncs_[j]; });

    // Start at the largest t_min (or worse)
    for (auto i = idx_.begin() + static_cast<std::ptrdiff_t>(dofs_) - 1; i != idx_end; ++i) {
        const double possible_t_sync = possible_t_syncs_[*i];
        bool is_blocked{false};
        for (std::size_t dof = 0; dof < dofs_; ++dof) {
            if (rp.sync[dof] == Sync::None) {
                continue;
            }
            if (blocks_[dof].is_blocked(possible_t_sync)) {
                is_blocked = true;
                break;
            }
        }
        if (is_blocked || possible_t_sync < t_min.value_or(0.0) || std::isinf(possible_t_sync)) {
            continue;
        }

        t_sync = possible_t_sync;
        if (*i == 3 * dofs_) {  // The optional minimum duration
            limiting_dof = std::nullopt;
            return true;
        }

        const std::size_t which = *i / dofs_;
        const std::size_t dof = *i % dofs_;
        limiting_dof = dof;
        switch (which) {
            case 0: profiles[dof] = blocks_[dof].p_min(); break;
            case 1: profiles[dof] = blocks_[dof].profile_of(*blocks_[dof].a); break;
            case 2: profiles[dof] = blocks_[dof].profile_of(*blocks_[dof].b); break;
            default: break;
        }
        return true;
    }

    return false;
}

Result<void> TargetSolver::solve(const ResolvedProblem& rp, std::span<Profile> profiles, std::span<double> independent_min_durations, double& duration) {
    for (std::size_t dof = 0; dof < dofs_; ++dof) {
        Profile& p = profiles[dof];
        const DofLimits& l = rp.limits[dof];

        if (!rp.enabled[dof]) {
            p.p.back() = rp.p0[dof];
            p.v.back() = rp.v0[dof];
            p.a.back() = rp.a0[dof];
            p.t_sum.back() = 0.0;
            p.brake.duration = 0.0;
            blocks_[dof].t_min = 0.0;
            blocks_[dof].a = std::nullopt;
            blocks_[dof].b = std::nullopt;
            independent_min_durations[dof] = 0.0;
            continue;
        }

        const bool third_order = !std::isinf(l.j_max);
        const bool second_order = !third_order && !std::isinf(l.a_max);

        // Brake pre-trajectory if the current state exceeds (or will inevitably exceed) the limits
        switch (rp.interface[dof]) {
            case Interface::Position: {
                if (third_order) {
                    p.brake.get_position_brake_trajectory(rp.v0[dof], rp.a0[dof], l.v_max, l.v_min, l.a_max, l.a_min, l.j_max);
                } else if (second_order) {
                    p.brake.get_second_order_position_brake_trajectory(rp.v0[dof], l.v_max, l.v_min, l.a_max, l.a_min);
                } else {
                    p.brake = BrakeProfile{};
                }
                p.set_boundary(rp.p0[dof], rp.v0[dof], rp.a0[dof], rp.pf[dof], rp.vf[dof], rp.af[dof]);
            } break;
            case Interface::Velocity: {
                if (third_order) {
                    p.brake.get_velocity_brake_trajectory(rp.a0[dof], l.a_max, l.a_min, l.j_max);
                } else {
                    p.brake.get_second_order_velocity_brake_trajectory();
                }
                p.set_boundary_for_velocity(rp.p0[dof], rp.v0[dof], rp.a0[dof], rp.vf[dof], rp.af[dof]);
            } break;
        }

        if (third_order) {
            p.brake.finalize(p.p[0], p.v[0], p.a[0]);
        } else if (second_order) {
            p.brake.finalize_second_order(p.p[0], p.v[0], p.a[0]);
        } else {
            p.brake.duration = 0.0;
        }

        bool found_profile{false};
        switch (rp.interface[dof]) {
            case Interface::Position: {
                if (third_order) {
                    PositionThirdOrderStep1 step1{p.p[0], p.v[0], p.a[0], p.pf, p.vf, p.af, l.v_max, l.v_min, l.a_max, l.a_min, l.j_max};
                    found_profile = step1.get_profile(p, blocks_[dof]);
                } else if (second_order) {
                    PositionSecondOrderStep1 step1{p.p[0], p.v[0], p.pf, p.vf, l.v_max, l.v_min, l.a_max, l.a_min};
                    found_profile = step1.get_profile(p, blocks_[dof]);
                } else {
                    PositionFirstOrderStep1 step1{p.p[0], p.pf, l.v_max, l.v_min};
                    found_profile = step1.get_profile(p, blocks_[dof]);
                }
            } break;
            case Interface::Velocity: {
                if (third_order) {
                    VelocityThirdOrderStep1 step1{p.v[0], p.a[0], p.vf, p.af, l.a_max, l.a_min, l.j_max};
                    found_profile = step1.get_profile(p, blocks_[dof]);
                } else {
                    VelocitySecondOrderStep1 step1{p.v[0], p.vf, l.a_max, l.a_min};
                    found_profile = step1.get_profile(p, blocks_[dof]);
                }
            } break;
        }

        if (!found_profile) {
            if (l.has_zero()) {
                return std::unexpected(Error{ErrorCode::ZeroLimits, std::format("zero limits conflict in step 1, {}", describe_dof(rp, dof))});
            }
            return std::unexpected(Error{ErrorCode::ExecutionTime, std::format("no extremal profile found in step 1, {}", describe_dof(rp, dof))});
        }

        independent_min_durations[dof] = blocks_[dof].t_min;
    }

    const bool discrete_duration = rp.duration_step.has_value();
    if (dofs_ == 1 && !rp.minimum_duration && !discrete_duration) {
        duration = blocks_[0].t_min;
        profiles[0] = blocks_[0].p_min();
        return {};
    }

    std::optional<std::size_t> limiting_dof;  // The DoF that does not need step 2
    if (!synchronize(rp, duration, limiting_dof, profiles)) {
        const bool has_zero_limits = std::any_of(rp.limits.begin(), rp.limits.end(), [](const DofLimits& l) { return l.has_zero(); });
        if (has_zero_limits) {
            return std::unexpected(Error{ErrorCode::ZeroLimits, "zero limits conflict with other degrees of freedom in time synchronization"});
        }
        return std::unexpected(Error{ErrorCode::Synchronization, "no synchronized duration found"});
    }

    // DoFs without synchronization keep their fastest profile
    for (std::size_t dof = 0; dof < dofs_; ++dof) {
        if (rp.enabled[dof] && rp.sync[dof] == Sync::None) {
            profiles[dof] = blocks_[dof].p_min();
            if (blocks_[dof].t_min > duration) {
                duration = blocks_[dof].t_min;
                limiting_dof = dof;
            }
        }
    }

    if (duration > kMaxDuration) {
        return std::unexpected(Error{ErrorCode::TrajectoryDuration, std::format("trajectory duration {} exceeds the supported maximum of {}", duration, kMaxDuration)});
    }

    if (duration == 0.0) {
        for (std::size_t dof = 0; dof < dofs_; ++dof) {
            profiles[dof] = blocks_[dof].p_min();
        }
        return {};
    }

    const bool all_none = std::all_of(rp.sync.begin(), rp.sync.end(), [](Sync s) { return s == Sync::None; });
    if (!discrete_duration && all_none) {
        return {};
    }

    // Phase synchronization
    const bool any_phase = std::any_of(rp.sync.begin(), rp.sync.end(), [](Sync s) { return s == Sync::Phase; });
    if (limiting_dof && any_phase) {
        const Profile& p_limiting = profiles[*limiting_dof];
        if (is_input_collinear(rp, p_limiting.direction, *limiting_dof)) {
            bool found_time_synchronization{true};
            for (std::size_t dof = 0; dof < dofs_; ++dof) {
                if (!rp.enabled[dof] || dof == *limiting_dof || rp.sync[dof] != Sync::Phase) {
                    continue;
                }

                Profile& p = profiles[dof];
                const DofLimits& l = rp.limits[dof];
                const double t_profile = duration - p.brake.duration;
                const double control = new_phase_control_[dof];
                const bool third_order = !std::isinf(l.j_max);
                const bool second_order = !third_order && !std::isinf(l.a_max);

                p.t = p_limiting.t;  // Copy the timing of the limiting DoF
                p.control_signs = p_limiting.control_signs;

                using CS = Profile::ControlSigns;
                using RL = Profile::ReachedLimits;
                bool ok = false;
                switch (rp.interface[dof]) {
                    case Interface::Position: {
                        if (p.control_signs == CS::UDDU) {
                            if (third_order) {
                                ok = p.check_with_timing<CS::UDDU, RL::NONE>(t_profile, control, l.v_max, l.v_min, l.a_max, l.a_min, l.j_max);
                            } else if (second_order) {
                                ok = p.check_for_second_order_with_timing<CS::UDDU, RL::NONE>(t_profile, control, -control, l.v_max, l.v_min, l.a_max, l.a_min);
                            } else {
                                ok = p.check_for_first_order_with_timing<CS::UDDU, RL::NONE>(t_profile, control, l.v_max, l.v_min);
                            }
                        } else {
                            if (third_order) {
                                ok = p.check_with_timing<CS::UDUD, RL::NONE>(t_profile, control, l.v_max, l.v_min, l.a_max, l.a_min, l.j_max);
                            } else {
                                ok = p.check_for_second_order_with_timing<CS::UDUD, RL::NONE>(t_profile, control, -control, l.v_max, l.v_min, l.a_max, l.a_min);
                            }
                        }
                    } break;
                    case Interface::Velocity: {
                        if (p.control_signs == CS::UDDU) {
                            if (third_order) {
                                ok = p.check_for_velocity_with_timing<CS::UDDU, RL::NONE>(t_profile, control, l.a_max, l.a_min, l.j_max);
                            } else {
                                ok = p.check_for_second_order_velocity_with_timing<CS::UDDU, RL::NONE>(t_profile, control, l.a_max, l.a_min);
                            }
                        } else {
                            if (third_order) {
                                ok = p.check_for_velocity_with_timing<CS::UDUD, RL::NONE>(t_profile, control, l.a_max, l.a_min, l.j_max);
                            } else {
                                ok = p.check_for_second_order_velocity_with_timing<CS::UDUD, RL::NONE>(t_profile, control, l.a_max, l.a_min);
                            }
                        }
                    } break;
                }
                found_time_synchronization &= ok;

                p.limits = p_limiting.limits;  // After the check, which overwrites it
            }

            const bool all_phase_or_none = std::all_of(rp.sync.begin(), rp.sync.end(), [](Sync s) { return s == Sync::Phase || s == Sync::None; });
            if (found_time_synchronization && all_phase_or_none) {
                return {};
            }
        }
    }

    // Time synchronization (step 2)
    for (std::size_t dof = 0; dof < dofs_; ++dof) {
        const bool skip = (dof == limiting_dof || rp.sync[dof] == Sync::None) && !discrete_duration;
        if (!rp.enabled[dof] || skip) {
            continue;
        }

        Profile& p = profiles[dof];
        const DofLimits& l = rp.limits[dof];
        const double t_profile = duration - p.brake.duration;

        if (rp.sync[dof] == Sync::TimeIfNecessary && std::abs(rp.vf[dof]) < eps && std::abs(rp.af[dof]) < eps) {
            p = blocks_[dof].p_min();
            continue;
        }

        // Reuse an extremal profile from step 1 if the duration matches (2*eps for discrete durations)
        if (std::abs(t_profile - blocks_[dof].t_min) < 2 * eps) {
            p = blocks_[dof].p_min();
            continue;
        } else if (blocks_[dof].a && std::abs(t_profile - blocks_[dof].a->right) < 2 * eps) {
            p = blocks_[dof].profile_of(*blocks_[dof].a);
            continue;
        } else if (blocks_[dof].b && std::abs(t_profile - blocks_[dof].b->right) < 2 * eps) {
            p = blocks_[dof].profile_of(*blocks_[dof].b);
            continue;
        }

        const bool third_order = !std::isinf(l.j_max);
        const bool second_order = !third_order && !std::isinf(l.a_max);

        bool found{false};
        switch (rp.interface[dof]) {
            case Interface::Position: {
                if (third_order) {
                    PositionThirdOrderStep2 step2{t_profile, p.p[0], p.v[0], p.a[0], p.pf, p.vf, p.af, l.v_max, l.v_min, l.a_max, l.a_min, l.j_max};
                    found = step2.get_profile(p);
                } else if (second_order) {
                    PositionSecondOrderStep2 step2{t_profile, p.p[0], p.v[0], p.pf, p.vf, l.v_max, l.v_min, l.a_max, l.a_min};
                    found = step2.get_profile(p);
                } else {
                    PositionFirstOrderStep2 step2{t_profile, p.p[0], p.pf, l.v_max, l.v_min};
                    found = step2.get_profile(p);
                }
            } break;
            case Interface::Velocity: {
                if (third_order) {
                    VelocityThirdOrderStep2 step2{t_profile, p.v[0], p.a[0], p.vf, p.af, l.a_max, l.a_min, l.j_max};
                    found = step2.get_profile(p);
                } else {
                    VelocitySecondOrderStep2 step2{t_profile, p.v[0], p.vf, l.a_max, l.a_min};
                    found = step2.get_profile(p);
                }
            } break;
        }
        if (!found) {
            return std::unexpected(Error{ErrorCode::Synchronization, std::format("no profile of duration {} found in step 2, {}", duration, describe_dof(rp, dof))});
        }
    }

    return {};
}

}  // namespace charqui::detail
