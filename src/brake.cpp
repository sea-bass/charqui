#include <charqui/profile.hpp>

#include <algorithm>
#include <cmath>

namespace charqui {

using detail::v_at_a_zero;
using detail::v_at_t;

void BrakeProfile::acceleration_brake(double v0, double a0, double vMax, double vMin, double aMax, double aMin, double jMax) {
    j[0] = -jMax;

    const double t_to_a_max = (a0 - aMax) / jMax;
    const double t_to_a_zero = a0 / jMax;

    const double v_at_a_max = v_at_t(v0, a0, -jMax, t_to_a_max);
    const double v_at_a_zero_ = v_at_t(v0, a0, -jMax, t_to_a_zero);

    if ((v_at_a_zero_ > vMax && jMax > 0) || (v_at_a_zero_ < vMax && jMax < 0)) {
        velocity_brake(v0, a0, vMax, vMin, aMax, aMin, jMax);

    } else if ((v_at_a_max < vMin && jMax > 0) || (v_at_a_max > vMin && jMax < 0)) {
        const double t_to_v_min = -(v_at_a_max - vMin) / aMax;
        const double t_to_v_max = -aMax / (2 * jMax) - (v_at_a_max - vMax) / aMax;

        t[0] = t_to_a_max + eps;
        t[1] = std::max(std::min(t_to_v_min, t_to_v_max - eps), 0.0);

    } else {
        t[0] = t_to_a_max + eps;
    }
}

void BrakeProfile::velocity_brake(double v0, double a0, double vMax, double vMin, double, double aMin, double jMax) {
    j[0] = -jMax;
    const double t_to_a_min = (a0 - aMin) / jMax;
    const double t_to_v_max = a0 / jMax + std::sqrt(a0 * a0 + 2 * jMax * (v0 - vMax)) / std::abs(jMax);
    const double t_to_v_min = a0 / jMax + std::sqrt(a0 * a0 / 2 + jMax * (v0 - vMin)) / std::abs(jMax);
    const double t_min_to_v_max = std::min(t_to_v_max, t_to_v_min);

    if (t_to_a_min < t_min_to_v_max) {
        const double v_at_a_min = v_at_t(v0, a0, -jMax, t_to_a_min);
        const double t_to_v_max_with_constant = -(v_at_a_min - vMax) / aMin;
        const double t_to_v_min_with_constant = aMin / (2 * jMax) - (v_at_a_min - vMin) / aMin;

        t[0] = std::max(t_to_a_min - eps, 0.0);
        t[1] = std::max(std::min(t_to_v_max_with_constant, t_to_v_min_with_constant), 0.0);

    } else {
        t[0] = std::max(t_min_to_v_max - eps, 0.0);
    }
}

void BrakeProfile::get_position_brake_trajectory(double v0, double a0, double vMax, double vMin, double aMax, double aMin, double jMax) {
    t[0] = 0.0;
    t[1] = 0.0;
    j[0] = 0.0;
    j[1] = 0.0;

    if (jMax == 0.0 || aMax == 0.0 || aMin == 0.0) {
        return;  // Ignore braking for zero limits
    }

    if (a0 > aMax) {
        acceleration_brake(v0, a0, vMax, vMin, aMax, aMin, jMax);

    } else if (a0 < aMin) {
        acceleration_brake(v0, a0, vMin, vMax, aMin, aMax, -jMax);

    } else if ((v0 > vMax && v_at_a_zero(v0, a0, -jMax) > vMin) || (a0 > 0 && v_at_a_zero(v0, a0, jMax) > vMax)) {
        velocity_brake(v0, a0, vMax, vMin, aMax, aMin, jMax);

    } else if ((v0 < vMin && v_at_a_zero(v0, a0, jMax) < vMax) || (a0 < 0 && v_at_a_zero(v0, a0, -jMax) < vMin)) {
        velocity_brake(v0, a0, vMin, vMax, aMin, aMax, -jMax);
    }
}

void BrakeProfile::get_second_order_position_brake_trajectory(double v0, double vMax, double vMin, double aMax, double aMin) {
    t[0] = 0.0;
    t[1] = 0.0;
    j[0] = 0.0;
    j[1] = 0.0;
    a[0] = 0.0;
    a[1] = 0.0;

    if (aMax == 0.0 || aMin == 0.0) {
        return;  // Ignore braking for zero limits
    }

    if (v0 > vMax) {
        a[0] = aMin;
        t[0] = (vMax - v0) / aMin + eps;

    } else if (v0 < vMin) {
        a[0] = aMax;
        t[0] = (vMin - v0) / aMax + eps;
    }
}

void BrakeProfile::get_velocity_brake_trajectory(double a0, double aMax, double aMin, double jMax) {
    t[0] = 0.0;
    t[1] = 0.0;
    j[0] = 0.0;
    j[1] = 0.0;

    if (jMax == 0.0) {
        return;  // Ignore braking for zero limits
    }

    if (a0 > aMax) {
        j[0] = -jMax;
        t[0] = (a0 - aMax) / jMax + eps;

    } else if (a0 < aMin) {
        j[0] = jMax;
        t[0] = -(a0 - aMin) / jMax + eps;
    }
}

void BrakeProfile::get_second_order_velocity_brake_trajectory() {
    t[0] = 0.0;
    t[1] = 0.0;
    j[0] = 0.0;
    j[1] = 0.0;
}

// ----- Profile secondary features -----

void Profile::check_position_extremum(double t_ext, double t_sum, double t, double p, double v, double a, double j, Bound& ext) {
    if (0 < t_ext && t_ext < t) {
        const auto [p_ext, v_ext, a_ext] = detail::integrate(t_ext, p, v, a, j);
        (void)v_ext;
        if (a_ext > 0 && p_ext < ext.min) {
            ext.min = p_ext;
            ext.t_min = t_sum + t_ext;
        } else if (a_ext < 0 && p_ext > ext.max) {
            ext.max = p_ext;
            ext.t_max = t_sum + t_ext;
        }
    }
}

void Profile::check_step_for_position_extremum(double t_sum, double t, double p, double v, double a, double j, Bound& ext) {
    if (p < ext.min) {
        ext.min = p;
        ext.t_min = t_sum;
    }
    if (p > ext.max) {
        ext.max = p;
        ext.t_max = t_sum;
    }

    if (j != 0) {
        const double D = a * a - 2 * j * v;
        if (std::abs(D) < std::numeric_limits<double>::epsilon()) {
            check_position_extremum(-a / j, t_sum, t, p, v, a, j, ext);

        } else if (D > 0.0) {
            const double D_sqrt = std::sqrt(D);
            check_position_extremum((-a - D_sqrt) / j, t_sum, t, p, v, a, j, ext);
            check_position_extremum((-a + D_sqrt) / j, t_sum, t, p, v, a, j, ext);
        }
    }
}

Bound Profile::get_position_extrema() const {
    Bound extrema;

    if (brake.duration > 0.0) {
        if (brake.t[0] > 0.0) {
            check_step_for_position_extremum(0.0, brake.t[0], brake.p[0], brake.v[0], brake.a[0], brake.j[0], extrema);

            if (brake.t[1] > 0.0) {
                check_step_for_position_extremum(brake.t[0], brake.t[1], brake.p[1], brake.v[1], brake.a[1], brake.j[1], extrema);
            }
        }
    }

    double t_current_sum{0.0};
    for (std::size_t i = 0; i < 7; ++i) {
        if (i > 0) {
            t_current_sum = t_sum[i - 1];
        }
        check_step_for_position_extremum(t_current_sum + brake.duration, t[i], p[i], v[i], a[i], j[i], extrema);
    }

    if (pf < extrema.min) {
        extrema.min = pf;
        extrema.t_min = t_sum.back() + brake.duration;
    }
    if (pf > extrema.max) {
        extrema.max = pf;
        extrema.t_max = t_sum.back() + brake.duration;
    }

    return extrema;
}

bool Profile::get_first_state_at_position(double pt, double& time, double time_after) const {
    double t_cum = 0.0;

    for (std::size_t i = 0; i < 7; ++i) {
        if (t[i] == 0.0) {
            continue;
        }

        if (std::abs(p[i] - pt) < DBL_EPSILON && t_cum >= time_after) {
            time = t_cum;
            return true;
        }

        for (const double _t : detail::roots::solve_cubic(j[i] / 6, a[i] / 2, v[i], p[i] - pt)) {
            if (0 < _t && time_after - t_cum <= _t && _t <= t[i]) {
                time = _t + t_cum;
                return true;
            }
        }

        t_cum += t[i];
    }

    if ((t[6] > 0.0 || t_sum.back() == 0.0) && std::abs(pf - pt) < 1e-9 && t_sum.back() >= time_after) {
        time = t_sum.back();
        return true;
    }

    return false;
}

std::string Profile::to_string() const {
    std::string result;
    switch (direction) {
        case Direction::UP: result += "UP_"; break;
        case Direction::DOWN: result += "DOWN_"; break;
    }
    switch (limits) {
        case ReachedLimits::ACC0_ACC1_VEL: result += "ACC0_ACC1_VEL"; break;
        case ReachedLimits::VEL: result += "VEL"; break;
        case ReachedLimits::ACC0: result += "ACC0"; break;
        case ReachedLimits::ACC1: result += "ACC1"; break;
        case ReachedLimits::ACC0_ACC1: result += "ACC0_ACC1"; break;
        case ReachedLimits::ACC0_VEL: result += "ACC0_VEL"; break;
        case ReachedLimits::ACC1_VEL: result += "ACC1_VEL"; break;
        case ReachedLimits::NONE: result += "NONE"; break;
    }
    switch (control_signs) {
        case ControlSigns::UDDU: result += "_UDDU"; break;
        case ControlSigns::UDUD: result += "_UDUD"; break;
    }
    return result;
}

}  // namespace charqui
