#pragma once

#include <array>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

#include <charqui/detail/math.hpp>
#include <charqui/detail/roots.hpp>

namespace charqui {

//! Position extremum of a single degree of freedom and the times it is reached
struct Bound {
    double min{detail::kInf};
    double max{-detail::kInf};
    double t_min{0.0};
    double t_max{0.0};
};

//! Pre-trajectory bringing an out-of-bounds initial state back within the limits (at most two phases)
class BrakeProfile {
    static constexpr double eps{2.2e-14};

    void acceleration_brake(double v0, double a0, double vMax, double vMin, double aMax, double aMin, double jMax);
    void velocity_brake(double v0, double a0, double vMax, double vMin, double aMax, double aMin, double jMax);

public:
    double duration{0.0};
    std::array<double, 2> t, j, a, v, p;  // left indeterminate on purpose: step 1 probes thousands of these

    //! Third-order position interface
    void get_position_brake_trajectory(double v0, double a0, double vMax, double vMin, double aMax, double aMin, double jMax);
    //! Second-order position interface
    void get_second_order_position_brake_trajectory(double v0, double vMax, double vMin, double aMax, double aMin);
    //! Third-order velocity interface
    void get_velocity_brake_trajectory(double a0, double aMax, double aMin, double jMax);
    //! Second-order velocity interface
    void get_second_order_velocity_brake_trajectory();

    //! Finalize third-order braking by integrating the kinematic state in place
    void finalize(double& ps, double& vs, double& as) {
        if (t[0] <= 0.0 && t[1] <= 0.0) {
            duration = 0.0;
            return;
        }

        duration = t[0];
        p[0] = ps;
        v[0] = vs;
        a[0] = as;
        std::tie(ps, vs, as) = detail::integrate(t[0], ps, vs, as, j[0]);

        if (t[1] > 0.0) {
            duration += t[1];
            p[1] = ps;
            v[1] = vs;
            a[1] = as;
            std::tie(ps, vs, as) = detail::integrate(t[1], ps, vs, as, j[1]);
        }
    }

    //! Finalize second-order braking by integrating the kinematic state in place
    void finalize_second_order(double& ps, double& vs, double& as) {
        if (t[0] <= 0.0) {
            duration = 0.0;
            return;
        }

        duration = t[0];
        p[0] = ps;
        v[0] = vs;
        std::tie(ps, vs, as) = detail::integrate(t[0], ps, vs, a[0], 0.0);
    }

    bool operator==(const BrakeProfile&) const = default;
};

//! A single-DoF kinematic profile: up to seven constant-jerk phases, optionally preceded by a brake pre-trajectory.
//! Index 0 of a, v, p is the state at the start of phase 0; index 7 is the end state.
class Profile {
public:
    static constexpr double v_eps{1e-12};
    static constexpr double a_eps{1e-12};
    static constexpr double j_eps{1e-12};

    static constexpr double p_precision{1e-8};
    static constexpr double v_precision{1e-8};
    static constexpr double a_precision{1e-10};

    static constexpr double t_max{1e12};

    // Left indeterminate on purpose: step 1 probes many candidate profiles and zeroing them costs more than
    // the solver itself. Value-initialization (Profile{}) still zeroes everything.
    std::array<double, 7> t, t_sum, j;
    std::array<double, 8> a, v, p;

    BrakeProfile brake;

    //! Target (final) kinematic state
    double pf{}, vf{}, af{};

    enum class ReachedLimits { ACC0_ACC1_VEL, VEL, ACC0, ACC1, ACC0_ACC1, ACC0_VEL, ACC1_VEL, NONE };
    enum class Direction { UP, DOWN };
    enum class ControlSigns { UDDU, UDUD };

    ReachedLimits limits{ReachedLimits::NONE};
    Direction direction{Direction::UP};
    ControlSigns control_signs{ControlSigns::UDDU};

    //! Total duration including the brake pre-trajectory
    [[nodiscard]] double total_duration() const noexcept { return t_sum.back() + brake.duration; }

    // ----- third-order velocity interface -----
    template <ControlSigns control_signs_, ReachedLimits limits_>
    bool check_for_velocity(double jf, double aMax, double aMin) {
        if (t[0] < 0) {
            return false;
        }

        t_sum[0] = t[0];
        for (std::size_t i = 0; i < 6; ++i) {
            if (t[i + 1] < 0) {
                return false;
            }
            t_sum[i + 1] = t_sum[i] + t[i + 1];
        }

        if constexpr (limits_ == ReachedLimits::ACC0) {
            if (t[1] < std::numeric_limits<double>::epsilon()) {
                return false;
            }
        }

        if (t_sum.back() > t_max) {
            return false;
        }

        if constexpr (control_signs_ == ControlSigns::UDDU) {
            j = {(t[0] > 0 ? jf : 0), 0, (t[2] > 0 ? -jf : 0), 0, (t[4] > 0 ? -jf : 0), 0, (t[6] > 0 ? jf : 0)};
        } else {
            j = {(t[0] > 0 ? jf : 0), 0, (t[2] > 0 ? -jf : 0), 0, (t[4] > 0 ? jf : 0), 0, (t[6] > 0 ? -jf : 0)};
        }

        for (std::size_t i = 0; i < 7; ++i) {
            a[i + 1] = a[i] + t[i] * j[i];
            v[i + 1] = v[i] + t[i] * (a[i] + t[i] * j[i] / 2);
            p[i + 1] = p[i] + t[i] * (v[i] + t[i] * (a[i] / 2 + t[i] * j[i] / 6));
        }

        this->control_signs = control_signs_;
        this->limits = limits_;

        direction = (aMax > 0) ? Direction::UP : Direction::DOWN;
        const double aUppLim = (direction == Direction::UP ? aMax : aMin) + a_eps;
        const double aLowLim = (direction == Direction::UP ? aMin : aMax) - a_eps;

        return std::abs(v.back() - vf) < v_precision && std::abs(a.back() - af) < a_precision
            && a[1] >= aLowLim && a[3] >= aLowLim && a[5] >= aLowLim
            && a[1] <= aUppLim && a[3] <= aUppLim && a[5] <= aUppLim;
    }

    template <ControlSigns control_signs_, ReachedLimits limits_>
    bool check_for_velocity_with_timing(double, double jf, double aMax, double aMin) {
        // The duration does not need to be checked: every profile has a tf - ... equation
        return check_for_velocity<control_signs_, limits_>(jf, aMax, aMin);
    }

    template <ControlSigns control_signs_, ReachedLimits limits_>
    bool check_for_velocity_with_timing(double tf, double jf, double aMax, double aMin, double jMax) {
        return (std::abs(jf) < std::abs(jMax) + j_eps) && check_for_velocity_with_timing<control_signs_, limits_>(tf, jf, aMax, aMin);
    }

    void set_boundary_for_velocity(double p0_new, double v0_new, double a0_new, double vf_new, double af_new) {
        a[0] = a0_new;
        v[0] = v0_new;
        p[0] = p0_new;
        af = af_new;
        vf = vf_new;
    }

    // ----- second-order velocity interface -----
    template <ControlSigns control_signs_, ReachedLimits limits_>
    bool check_for_second_order_velocity(double aUp) {
        // ReachedLimits::ACC0
        if (t[1] < 0.0) {
            return false;
        }

        t_sum = {0, t[1], t[1], t[1], t[1], t[1], t[1]};
        if (t_sum.back() > t_max) {
            return false;
        }

        j = {0, 0, 0, 0, 0, 0, 0};
        a = {0, (t[1] > 0) ? aUp : 0, 0, 0, 0, 0, 0, af};
        for (std::size_t i = 0; i < 7; ++i) {
            v[i + 1] = v[i] + t[i] * a[i];
            p[i + 1] = p[i] + t[i] * (v[i] + t[i] * a[i] / 2);
        }

        this->control_signs = control_signs_;
        this->limits = limits_;

        direction = (aUp > 0) ? Direction::UP : Direction::DOWN;

        return std::abs(v.back() - vf) < v_precision;
    }

    template <ControlSigns control_signs_, ReachedLimits limits_>
    bool check_for_second_order_velocity_with_timing(double, double aUp) {
        return check_for_second_order_velocity<control_signs_, limits_>(aUp);
    }

    template <ControlSigns control_signs_, ReachedLimits limits_>
    bool check_for_second_order_velocity_with_timing(double tf, double aUp, double aMax, double aMin) {
        return (aMin - a_eps < aUp) && (aUp < aMax + a_eps) && check_for_second_order_velocity_with_timing<control_signs_, limits_>(tf, aUp);
    }

    // ----- third-order position interface -----
    template <ControlSigns control_signs_, ReachedLimits limits_, bool set_limits = false>
    bool check(double jf, double vMax, double vMin, double aMax, double aMin) {
        if (t[0] < 0) {
            return false;
        }

        t_sum[0] = t[0];
        for (std::size_t i = 0; i < 6; ++i) {
            if (t[i + 1] < 0) {
                return false;
            }
            t_sum[i + 1] = t_sum[i] + t[i + 1];
        }

        if constexpr (limits_ == ReachedLimits::ACC0_ACC1_VEL || limits_ == ReachedLimits::ACC0_VEL || limits_ == ReachedLimits::ACC1_VEL || limits_ == ReachedLimits::VEL) {
            if (t[3] < std::numeric_limits<double>::epsilon()) {
                return false;
            }
        }

        if constexpr (limits_ == ReachedLimits::ACC0 || limits_ == ReachedLimits::ACC0_ACC1) {
            if (t[1] < std::numeric_limits<double>::epsilon()) {
                return false;
            }
        }

        if constexpr (limits_ == ReachedLimits::ACC1 || limits_ == ReachedLimits::ACC0_ACC1) {
            if (t[5] < std::numeric_limits<double>::epsilon()) {
                return false;
            }
        }

        if (t_sum.back() > t_max) {
            return false;
        }

        if constexpr (control_signs_ == ControlSigns::UDDU) {
            j = {(t[0] > 0 ? jf : 0), 0, (t[2] > 0 ? -jf : 0), 0, (t[4] > 0 ? -jf : 0), 0, (t[6] > 0 ? jf : 0)};
        } else {
            j = {(t[0] > 0 ? jf : 0), 0, (t[2] > 0 ? -jf : 0), 0, (t[4] > 0 ? jf : 0), 0, (t[6] > 0 ? -jf : 0)};
        }

        direction = (vMax > 0) ? Direction::UP : Direction::DOWN;
        const double vUppLim = (direction == Direction::UP ? vMax : vMin) + v_eps;
        const double vLowLim = (direction == Direction::UP ? vMin : vMax) - v_eps;

        for (std::size_t i = 0; i < 7; ++i) {
            a[i + 1] = a[i] + t[i] * j[i];
            v[i + 1] = v[i] + t[i] * (a[i] + t[i] * j[i] / 2);
            p[i + 1] = p[i] + t[i] * (v[i] + t[i] * (a[i] / 2 + t[i] * j[i] / 6));

            if constexpr (limits_ == ReachedLimits::ACC0_ACC1_VEL || limits_ == ReachedLimits::ACC0_ACC1 || limits_ == ReachedLimits::ACC0_VEL || limits_ == ReachedLimits::ACC1_VEL || limits_ == ReachedLimits::VEL) {
                if (i == 2) {
                    a[3] = 0.0;
                }
            }

            if constexpr (set_limits) {
                if constexpr (limits_ == ReachedLimits::ACC1) {
                    if (i == 2) {
                        a[3] = aMin;
                    }
                }

                if constexpr (limits_ == ReachedLimits::ACC0_ACC1) {
                    if (i == 0) {
                        a[1] = aMax;
                    }
                    if (i == 4) {
                        a[5] = aMin;
                    }
                }
            }

            if (i > 1 && a[i + 1] * a[i] < -std::numeric_limits<double>::epsilon()) {
                const double v_a_zero = v[i] - (a[i] * a[i]) / (2 * j[i]);
                if (v_a_zero > vUppLim || v_a_zero < vLowLim) {
                    return false;
                }
            }
        }

        this->control_signs = control_signs_;
        this->limits = limits_;

        const double aUppLim = (direction == Direction::UP ? aMax : aMin) + a_eps;
        const double aLowLim = (direction == Direction::UP ? aMin : aMax) - a_eps;

        return std::abs(p.back() - pf) < p_precision && std::abs(v.back() - vf) < v_precision && std::abs(a.back() - af) < a_precision
            && a[1] >= aLowLim && a[3] >= aLowLim && a[5] >= aLowLim
            && a[1] <= aUppLim && a[3] <= aUppLim && a[5] <= aUppLim
            && v[3] <= vUppLim && v[4] <= vUppLim && v[5] <= vUppLim && v[6] <= vUppLim
            && v[3] >= vLowLim && v[4] >= vLowLim && v[5] >= vLowLim && v[6] >= vLowLim;
    }

    template <ControlSigns control_signs_, ReachedLimits limits_>
    bool check_with_timing(double, double jf, double vMax, double vMin, double aMax, double aMin) {
        return check<control_signs_, limits_>(jf, vMax, vMin, aMax, aMin);
    }

    template <ControlSigns control_signs_, ReachedLimits limits_>
    bool check_with_timing(double tf, double jf, double vMax, double vMin, double aMax, double aMin, double jMax) {
        return (std::abs(jf) < std::abs(jMax) + j_eps) && check_with_timing<control_signs_, limits_>(tf, jf, vMax, vMin, aMax, aMin);
    }

    void set_boundary(const Profile& profile) {
        a[0] = profile.a[0];
        v[0] = profile.v[0];
        p[0] = profile.p[0];
        af = profile.af;
        vf = profile.vf;
        pf = profile.pf;
        brake = profile.brake;
    }

    void set_boundary(double p0_new, double v0_new, double a0_new, double pf_new, double vf_new, double af_new) {
        a[0] = a0_new;
        v[0] = v0_new;
        p[0] = p0_new;
        af = af_new;
        vf = vf_new;
        pf = pf_new;
    }

    // ----- second-order position interface -----
    template <ControlSigns control_signs_, ReachedLimits limits_>
    bool check_for_second_order(double aUp, double aDown, double vMax, double vMin) {
        if (t[0] < 0) {
            return false;
        }

        t_sum[0] = t[0];
        for (std::size_t i = 0; i < 6; ++i) {
            if (t[i + 1] < 0) {
                return false;
            }
            t_sum[i + 1] = t_sum[i] + t[i + 1];
        }

        if (t_sum.back() > t_max) {
            return false;
        }

        j = {0, 0, 0, 0, 0, 0, 0};
        if constexpr (control_signs_ == ControlSigns::UDDU) {
            a = {(t[0] > 0 ? aUp : 0), 0, (t[2] > 0 ? aDown : 0), 0, (t[4] > 0 ? aDown : 0), 0, (t[6] > 0 ? aUp : 0), af};
        } else {
            a = {(t[0] > 0 ? aUp : 0), 0, (t[2] > 0 ? aDown : 0), 0, (t[4] > 0 ? aUp : 0), 0, (t[6] > 0 ? aDown : 0), af};
        }

        direction = (vMax > 0) ? Direction::UP : Direction::DOWN;
        const double vUppLim = (direction == Direction::UP ? vMax : vMin) + v_eps;
        const double vLowLim = (direction == Direction::UP ? vMin : vMax) - v_eps;

        for (std::size_t i = 0; i < 7; ++i) {
            v[i + 1] = v[i] + t[i] * a[i];
            p[i + 1] = p[i] + t[i] * (v[i] + t[i] * a[i] / 2);
        }

        this->control_signs = control_signs_;
        this->limits = limits_;

        return std::abs(p.back() - pf) < p_precision && std::abs(v.back() - vf) < v_precision
            && v[2] <= vUppLim && v[3] <= vUppLim && v[4] <= vUppLim && v[5] <= vUppLim && v[6] <= vUppLim
            && v[2] >= vLowLim && v[3] >= vLowLim && v[4] >= vLowLim && v[5] >= vLowLim && v[6] >= vLowLim;
    }

    template <ControlSigns control_signs_, ReachedLimits limits_>
    bool check_for_second_order_with_timing(double, double aUp, double aDown, double vMax, double vMin) {
        return check_for_second_order<control_signs_, limits_>(aUp, aDown, vMax, vMin);
    }

    template <ControlSigns control_signs_, ReachedLimits limits_>
    bool check_for_second_order_with_timing(double tf, double aUp, double aDown, double vMax, double vMin, double aMax, double aMin) {
        return (aMin - a_eps < aUp) && (aUp < aMax + a_eps) && (aMin - a_eps < aDown) && (aDown < aMax + a_eps)
            && check_for_second_order_with_timing<control_signs_, limits_>(tf, aUp, aDown, vMax, vMin);
    }

    // ----- first-order position interface -----
    template <ControlSigns control_signs_, ReachedLimits limits_>
    bool check_for_first_order(double vUp) {
        // ReachedLimits::VEL
        if (t[3] < 0.0) {
            return false;
        }

        t_sum = {0, 0, 0, t[3], t[3], t[3], t[3]};
        if (t_sum.back() > t_max) {
            return false;
        }

        j = {0, 0, 0, 0, 0, 0, 0};
        a = {0, 0, 0, 0, 0, 0, 0, af};
        v = {0, 0, 0, t[3] > 0 ? vUp : 0, 0, 0, 0, vf};
        for (std::size_t i = 0; i < 7; ++i) {
            p[i + 1] = p[i] + t[i] * (v[i] + t[i] * a[i] / 2);
        }

        this->control_signs = control_signs_;
        this->limits = limits_;

        direction = (vUp > 0) ? Direction::UP : Direction::DOWN;

        return std::abs(p.back() - pf) < p_precision;
    }

    template <ControlSigns control_signs_, ReachedLimits limits_>
    bool check_for_first_order_with_timing(double, double vUp) {
        return check_for_first_order<control_signs_, limits_>(vUp);
    }

    template <ControlSigns control_signs_, ReachedLimits limits_>
    bool check_for_first_order_with_timing(double tf, double vUp, double vMax, double vMin) {
        return (vMin - v_eps < vUp) && (vUp < vMax + v_eps) && check_for_first_order_with_timing<control_signs_, limits_>(tf, vUp);
    }

    // ----- secondary features -----
    [[nodiscard]] Bound get_position_extrema() const;
    [[nodiscard]] bool get_first_state_at_position(double pt, double& time, double time_after = 0.0) const;
    [[nodiscard]] std::string to_string() const;

private:
    static void check_position_extremum(double t_ext, double t_sum, double t, double p, double v, double a, double j, Bound& ext);
    static void check_step_for_position_extremum(double t_sum, double t, double p, double v, double a, double j, Bound& ext);
};

}  // namespace charqui
