#include <doctest/doctest.h>

#include <cmath>
#include <random>

#include <charqui/detail/roots.hpp>

using namespace charqui::detail::roots;

TEST_CASE("cubic roots") {
    // (x - 1)(x - 2)(x - 3) = x^3 - 6x^2 + 11x - 6
    auto r = solve_cubic(1.0, -6.0, 11.0, -6.0);
    REQUIRE(r.size() == 3);
    auto it = r.begin();
    CHECK(*it++ == doctest::Approx(1.0));
    CHECK(*it++ == doctest::Approx(2.0));
    CHECK(*it++ == doctest::Approx(3.0));

    // Negative roots are dropped: (x + 1)(x - 2) = x^2 - x - 2
    auto q = solve_cubic(0.0, 1.0, -1.0, -2.0);
    REQUIRE(q.size() == 1);
    CHECK(*q.begin() == doctest::Approx(2.0));

    // Linear
    auto l = solve_cubic(0.0, 0.0, 2.0, -4.0);
    REQUIRE(l.size() == 1);
    CHECK(*l.begin() == doctest::Approx(2.0));
}

TEST_CASE("monic quartic roots") {
    // (x - 1)(x - 2)(x - 3)(x - 4) = x^4 - 10x^3 + 35x^2 - 50x + 24
    auto r = solve_quart_monic(-10.0, 35.0, -50.0, 24.0);
    REQUIRE(r.size() == 4);
    auto it = r.begin();
    for (double expected : {1.0, 2.0, 3.0, 4.0}) {
        CHECK(*it++ == doctest::Approx(expected).epsilon(1e-9));
    }

    // Random quartics with known positive roots
    std::mt19937 gen(7);
    std::uniform_real_distribution<double> dist(0.1, 5.0);
    for (int n = 0; n < 2000; ++n) {
        const double x1 = dist(gen), x2 = dist(gen), x3 = dist(gen), x4 = dist(gen);
        const double a = -(x1 + x2 + x3 + x4);
        const double b = x1 * x2 + x1 * x3 + x1 * x4 + x2 * x3 + x2 * x4 + x3 * x4;
        const double c = -(x1 * x2 * x3 + x1 * x2 * x4 + x1 * x3 * x4 + x2 * x3 * x4);
        const double d = x1 * x2 * x3 * x4;
        auto roots = solve_quart_monic(a, b, c, d);
        for (double x : roots) {
            const double residual = std::abs(poly_eval(std::array<double, 5>{1.0, a, b, c, d}, x));
            CHECK(residual < 1e-6);
        }
    }
}

TEST_CASE("polynomial evaluation and derivative") {
    const std::array<double, 4> p{2.0, -3.0, 0.5, 1.0};  // 2x^3 - 3x^2 + 0.5x + 1
    CHECK(poly_eval(p, 2.0) == doctest::Approx(16.0 - 12.0 + 1.0 + 1.0));
    const auto d = poly_derivative(p);  // 6x^2 - 6x + 0.5
    CHECK(d[0] == doctest::Approx(6.0));
    CHECK(d[1] == doctest::Approx(-6.0));
    CHECK(d[2] == doctest::Approx(0.5));
    CHECK(poly_eval(d, 1.0) == doctest::Approx(0.5));
}

TEST_CASE("safeguarded newton bracket") {
    const std::array<double, 3> p{1.0, 0.0, -2.0};  // x^2 - 2
    const double root = shrink_interval(p, 0.0, 2.0);
    CHECK(root == doctest::Approx(std::sqrt(2.0)).epsilon(1e-12));
}
