#pragma once

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <numbers>

namespace charqui::detail::roots {

//! Fixed-capacity set living on the stack (real-time safe). Elements are sorted lazily on iteration.
template <class T, std::size_t N>
class Set {
protected:
    std::array<T, N> data_{};
    std::size_t size_{0};

public:
    using iterator = typename std::array<T, N>::iterator;

    iterator begin() {
        // Insertion sort: N is tiny, and it keeps GCC's std::sort range diagnostics quiet
        for (std::size_t i = 1; i < size_; ++i) {
            T value = data_[i];
            std::size_t j = i;
            while (j > 0 && value < data_[j - 1]) {
                data_[j] = data_[j - 1];
                --j;
            }
            data_[j] = value;
        }
        return data_.begin();
    }
    iterator end() { return data_.begin() + size_; }

    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    [[nodiscard]] bool empty() const noexcept { return size_ == 0; }

    void insert(T value) {
        data_[size_] = value;
        ++size_;
    }
};

//! Set that only keeps non-negative values.
template <class T, std::size_t N>
class PositiveSet : public Set<T, N> {
public:
    void insert(T value) {
        if (value >= 0) {
            Set<T, N>::insert(value);
        }
    }
};

//! All real, non-negative roots of a*x^3 + b*x^2 + c*x + d = 0
inline PositiveSet<double, 3> solve_cubic(double a, double b, double c, double d) {
    PositiveSet<double, 3> roots;

    if (std::abs(d) < DBL_EPSILON) {
        // First solution is x = 0, reduce to a quadratic
        roots.insert(0.0);
        d = c;
        c = b;
        b = a;
        a = 0.0;
    }

    if (std::abs(a) < DBL_EPSILON) {
        if (std::abs(b) < DBL_EPSILON) {
            // Linear
            if (std::abs(c) > DBL_EPSILON) {
                roots.insert(-d / c);
            }
        } else {
            // Quadratic
            const double discriminant = c * c - 4 * b * d;
            if (discriminant >= 0) {
                const double inv2b = 1.0 / (2 * b);
                const double y = std::sqrt(discriminant);
                roots.insert((-c + y) * inv2b);
                roots.insert((-c - y) * inv2b);
            }
        }
    } else {
        // Cubic
        const double inva = 1.0 / a;
        const double invaa = inva * inva;
        const double bb = b * b;
        const double bover3a = b * inva / 3;
        const double p = (a * c - bb / 3) * invaa;
        const double halfq = (2 * bb * b - 9 * a * b * c + 27 * a * a * d) / 54 * invaa * inva;
        const double yy = p * p * p / 27 + halfq * halfq;

        constexpr double cos120 = -0.50;
        constexpr double sin120 = 0.866025403784438646764;

        if (yy > DBL_EPSILON) {
            // One real solution
            const double y = std::sqrt(yy);
            const double uuu = -halfq + y;
            const double vvv = -halfq - y;
            const double www = std::abs(uuu) > std::abs(vvv) ? uuu : vvv;
            const double w = std::cbrt(www);
            roots.insert(w - p / (3 * w) - bover3a);
        } else if (yy < -DBL_EPSILON) {
            // Three real solutions
            const double x = -halfq;
            const double y = std::sqrt(-yy);
            double theta;
            double r;

            if (std::abs(x) > DBL_EPSILON) {
                theta = (x > 0.0) ? std::atan(y / x) : (std::atan(y / x) + std::numbers::pi);
                r = std::sqrt(x * x - yy);
            } else {
                theta = std::numbers::pi / 2;
                r = y;
            }
            theta /= 3;
            r = 2 * std::cbrt(r);
            const double ux = std::cos(theta) * r;
            const double uyi = std::sin(theta) * r;

            roots.insert(ux - bover3a);
            roots.insert(ux * cos120 - uyi * sin120 - bover3a);
            roots.insert(ux * cos120 + uyi * sin120 - bover3a);
        } else {
            // Two real solutions
            const double www = -halfq;
            const double w = 2 * std::cbrt(www);

            roots.insert(w - bover3a);
            roots.insert(w * cos120 - bover3a);
        }
    }
    return roots;
}

//! Solve the resolvent cubic of a quartic (Ferrari). Returns the number of real roots written to x.
inline int solve_resolvent(std::array<double, 3>& x, double a, double b, double c) {
    constexpr double cos120 = -0.50;
    constexpr double sin120 = 0.866025403784438646764;

    a /= 3;
    const double a2 = a * a;
    double q = a2 - b / 3;
    const double r = (a * (2 * a2 - b) + c) / 2;
    const double r2 = r * r;
    const double q3 = q * q * q;

    if (r2 < q3) {
        const double qsqrt = std::sqrt(q);
        const double t = std::clamp(r / (q * qsqrt), -1.0, 1.0);
        q = -2 * qsqrt;

        const double theta = std::acos(t) / 3;
        const double ux = std::cos(theta) * q;
        const double uyi = std::sin(theta) * q;
        x[0] = ux - a;
        x[1] = ux * cos120 - uyi * sin120 - a;
        x[2] = ux * cos120 + uyi * sin120 - a;
        return 3;
    }

    double A = -std::cbrt(std::abs(r) + std::sqrt(r2 - q3));
    if (r < 0.0) {
        A = -A;
    }
    const double B = (0.0 == A ? 0.0 : q / A);

    x[0] = (A + B) - a;
    x[1] = -(A + B) / 2 - a;
    x[2] = std::sqrt(3.0) * (A - B) / 2;
    if (std::abs(x[2]) < DBL_EPSILON) {
        x[2] = x[1];
        return 2;
    }
    return 1;
}

//! All real, non-negative roots of the monic quartic x^4 + a*x^3 + b*x^2 + c*x + d = 0
inline PositiveSet<double, 4> solve_quart_monic(double a, double b, double c, double d) {
    PositiveSet<double, 4> roots;

    if (std::abs(d) < DBL_EPSILON) {
        if (std::abs(c) < DBL_EPSILON) {
            roots.insert(0.0);

            const double D = a * a - 4 * b;
            if (std::abs(D) < DBL_EPSILON) {
                roots.insert(-a / 2);
            } else if (D > 0.0) {
                const double sqrtD = std::sqrt(D);
                roots.insert((-a - sqrtD) / 2);
                roots.insert((-a + sqrtD) / 2);
            }
            return roots;
        }

        if (std::abs(a) < DBL_EPSILON && std::abs(b) < DBL_EPSILON) {
            roots.insert(0.0);
            roots.insert(-std::cbrt(c));
            return roots;
        }
    }

    const double a3 = -b;
    const double b3 = a * c - 4 * d;
    const double c3 = -a * a * d - c * c + 4 * b * d;

    std::array<double, 3> x3;
    const int number_zeroes = solve_resolvent(x3, a3, b3, c3);

    double y = x3[0];
    if (number_zeroes != 1) {
        if (std::abs(x3[1]) > std::abs(y)) {
            y = x3[1];
        }
        if (std::abs(x3[2]) > std::abs(y)) {
            y = x3[2];
        }
    }

    double q1, q2, p1, p2;

    double D = y * y - 4 * d;
    if (std::abs(D) < DBL_EPSILON) {
        q1 = q2 = y / 2;
        D = a * a - 4 * (b - y);
        if (std::abs(D) < DBL_EPSILON) {
            p1 = p2 = a / 2;
        } else {
            const double sqrtD = std::sqrt(D);
            p1 = (a + sqrtD) / 2;
            p2 = (a - sqrtD) / 2;
        }
    } else {
        const double sqrtD = std::sqrt(D);
        q1 = (y + sqrtD) / 2;
        q2 = (y - sqrtD) / 2;
        p1 = (a * q1 - c) / (q1 - q2);
        p2 = (c - a * q2) / (q1 - q2);
    }

    constexpr double eps{16 * DBL_EPSILON};

    D = p1 * p1 - 4 * q1;
    if (std::abs(D) < eps) {
        roots.insert(-p1 / 2);
    } else if (D > 0.0) {
        const double sqrtD = std::sqrt(D);
        roots.insert((-p1 - sqrtD) / 2);
        roots.insert((-p1 + sqrtD) / 2);
    }

    D = p2 * p2 - 4 * q2;
    if (std::abs(D) < eps) {
        roots.insert(-p2 / 2);
    } else if (D > 0.0) {
        const double sqrtD = std::sqrt(D);
        roots.insert((-p2 - sqrtD) / 2);
        roots.insert((-p2 + sqrtD) / 2);
    }

    return roots;
}

inline PositiveSet<double, 4> solve_quart_monic(const std::array<double, 4>& polynom) {
    return solve_quart_monic(polynom[0], polynom[1], polynom[2], polynom[3]);
}

//! Relative margin for quart_monic_no_root_in: the polynomial must stay this far (relative to its own
//! magnitude on the window) from zero before a quartic solve is skipped
inline constexpr double kNoRootMargin{1e-6};

//! Conservative proof that the monic quartic x^4 + c[0] x^3 + c[1] x^2 + c[2] x + c[3] has no real root in [lo, hi].
//! The polynomial is rewritten in the Bernstein basis over the window; if every Bernstein coefficient lies on one
//! side of zero by `margin` (relative), the convex-hull property guarantees no zero crossing. A `false` answer
//! only means "could not prove it", in which case the caller solves the quartic as usual. This skips the
//! majority of quartic solves in step 1 without changing any result.
[[nodiscard]] inline bool quart_monic_no_root_in(const std::array<double, 4>& c, double lo, double hi, double margin = kNoRootMargin) noexcept {
    if (hi < lo) {
        return true;  // Empty window: every root would be rejected anyway
    }

    // Taylor coefficients of p(lo + x)
    const double s = lo;
    const double q0 = (((s + c[0]) * s + c[1]) * s + c[2]) * s + c[3];
    const double q1 = ((4 * s + 3 * c[0]) * s + 2 * c[1]) * s + c[2];
    const double q2 = (6 * s + 3 * c[0]) * s + c[1];
    const double q3 = 4 * s + c[0];

    // Rescale to u in [0, 1] and convert to the Bernstein basis of degree 4
    const double h = hi - lo;
    const double h2 = h * h;
    const double a1 = q1 * h;
    const double a2 = q2 * h2;
    const double a3 = q3 * h2 * h;
    const double a4 = h2 * h2;
    const double b0 = q0;
    const double b1 = q0 + a1 / 4;
    const double b2 = q0 + a1 / 2 + a2 / 6;
    const double b3 = q0 + 3 * a1 / 4 + a2 / 2 + a3 / 4;
    const double b4 = q0 + a1 + a2 + a3 + a4;
    if (!std::isfinite(b0 + b1 + b2 + b3 + b4)) {
        return false;
    }

    const double b_min = std::min({b0, b1, b2, b3, b4});
    const double b_max = std::max({b0, b1, b2, b3, b4});
    const double tolerance = margin * std::max(std::abs(b_min), std::abs(b_max));
    return b_min > tolerance || b_max < -tolerance;
}

//! Evaluate a polynomial with descending coefficients (p[0] x^(N-1) + ... + p[N-1]) at x.
//! Power-form summation rather than Horner: the running power and the running sum are independent
//! dependency chains, which is measurably faster than Horner's serial multiply-add on a degree-6 polynomial.
template <std::size_t N>
[[nodiscard]] constexpr double poly_eval(const std::array<double, N>& p, double x) noexcept {
    if constexpr (N == 0) {
        return 0.0;
    } else {
        if (std::abs(x) < DBL_EPSILON) {
            return p[N - 1];
        }
        if (x == 1.0) {
            double sum = 0.0;
            for (std::size_t i = N; i-- > 0;) {
                sum += p[i];
            }
            return sum;
        }
        double result = 0.0;
        double xn = 1.0;
        for (std::size_t i = N; i-- > 0;) {
            result += p[i] * xn;
            xn *= x;
        }
        return result;
    }
}

//! Coefficients of the derivative of a polynomial with descending coefficients
template <std::size_t N>
[[nodiscard]] constexpr std::array<double, N - 1> poly_derivative(const std::array<double, N>& coeffs) noexcept {
    std::array<double, N - 1> deriv{};
    for (std::size_t i = 0; i < N - 1; ++i) {
        deriv[i] = static_cast<double>(N - 1 - i) * coeffs[i];
    }
    return deriv;
}

//! Derivative of a monic polynomial, normalised to stay monic
template <std::size_t N>
[[nodiscard]] constexpr std::array<double, N - 1> poly_monic_derivative(const std::array<double, N>& monic_coeffs) noexcept {
    std::array<double, N - 1> deriv{};
    deriv[0] = 1.0;
    for (std::size_t i = 1; i < N - 1; ++i) {
        deriv[i] = static_cast<double>(N - 1 - i) * monic_coeffs[i] / static_cast<double>(N - 1);
    }
    return deriv;
}

inline constexpr double kTolerance{1e-14};

//! Safeguarded Newton-bisection: a single root of p inside [l, h], requiring p(l) * p(h) < 0.
template <std::size_t N, std::size_t MaxIterations = 128>
[[nodiscard]] double shrink_interval(const std::array<double, N>& p, double l, double h) {
    const double fl = poly_eval(p, l);
    const double fh = poly_eval(p, h);
    if (fl == 0.0) {
        return l;
    }
    if (fh == 0.0) {
        return h;
    }
    if (fl > 0.0) {
        std::swap(l, h);
    }

    double rts = (l + h) / 2;
    double dxold = std::abs(h - l);
    double dx = dxold;
    const auto deriv = poly_derivative(p);
    double f = poly_eval(p, rts);
    double df = poly_eval(deriv, rts);
    for (std::size_t j = 0; j < MaxIterations; ++j) {
        if ((((rts - h) * df - f) * ((rts - l) * df - f) > 0.0) || (std::abs(2 * f) > std::abs(dxold * df))) {
            dxold = dx;
            dx = (h - l) / 2;
            rts = l + dx;
            if (l == rts) {
                break;
            }
        } else {
            dxold = dx;
            dx = f / df;
            const double temp = rts;
            rts -= dx;
            if (temp == rts) {
                break;
            }
        }

        if (std::abs(dx) < kTolerance) {
            break;
        }

        f = poly_eval(p, rts);
        df = poly_eval(deriv, rts);
        if (f < 0.0) {
            l = rts;
        } else {
            h = rts;
        }
    }

    return rts;
}

}  // namespace charqui::detail::roots
