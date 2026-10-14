#pragma once

#include <cmath>
#include <limits>
#include <tuple>

namespace charqui::detail {

inline constexpr double kEps = std::numeric_limits<double>::epsilon();
inline constexpr double kInf = std::numeric_limits<double>::infinity();

template <class T>
[[nodiscard]] constexpr T pow2(T v) noexcept {
    return v * v;
}

//! Integrate with constant jerk for duration t. Returns the new position, velocity and acceleration.
[[nodiscard]] constexpr std::tuple<double, double, double> integrate(double t, double p0, double v0, double a0, double j) noexcept {
    return {
        p0 + t * (v0 + t * (a0 / 2 + t * j / 6)),
        v0 + t * (a0 + t * j / 2),
        a0 + t * j,
    };
}

[[nodiscard]] constexpr double v_at_t(double v0, double a0, double j, double t) noexcept {
    return v0 + t * (a0 + j * t / 2);
}

[[nodiscard]] constexpr double v_at_a_zero(double v0, double a0, double j) noexcept {
    return v0 + (a0 * a0) / (2 * j);
}

}  // namespace charqui::detail
