#include "unicycle_ugv_controller/common/heading_recovery.h"

#include <stdexcept>
#define CHECK(expr)                          \
    do {                                     \
        if (!(expr))                         \
            throw std::runtime_error(#expr); \
    } while (false)
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
using namespace unicycle_ugv_controller;
int main() try {
    HeadingRecoveryConfig cfg;
    cfg.gain = 1.0;
    auto lateral = headingRecoveryCentre(0, 0.3, 0, 0, 0, 0.15, cfg);
    CHECK(lateral.valid && lateral.centre > 0);
    auto right = headingRecoveryCentre(0, -0.3, 0, 0, 0, 0.15, cfg);
    CHECK(right.valid && std::abs(right.centre + lateral.centre) < 1e-15);
    auto rest = headingRecoveryCentre(0, 0, 0, 0, 0, 0.15, cfg);
    CHECK(rest.valid && rest.centre == 0);
    auto reverse = headingRecoveryCentre(-0.3, 0, 0, 0, 0, 0.15, cfg);
    CHECK(reverse.valid && reverse.centre == 0);
    // A small perturbation about the reverse axis restores that axis.
    CHECK(headingRecoveryCentre(-0.3, 0, 0.01, 0, 0, 0.15, cfg).centre < 0);
    CHECK(headingRecoveryCentre(0.3, 0, 0.01, 0, 0, 0.15, cfg).centre < 0);
    CHECK(headingRecoveryCentre(0, 0, 0, 0.2, 0, 0.15, cfg).centre < 0);
    double result = 0;
    CHECK(centredDampedYawRate(0, 10, 0.15, lateral.centre, result));
    CHECK(result == lateral.centre);
    std::mt19937 gen(93);
    std::uniform_real_distribution<double> d(-2, 2);
    for (int i = 0; i < 10000; ++i) {
        double x = d(gen), y = d(gen), yaw = d(gen), speed = d(gen), a = d(gen), b = d(gen);
        auto r = headingRecoveryCentre(x, y, yaw, 0, 0, 0.15, cfg);
        const double h = 1e-6;
        auto plus = headingRecoveryCentre(x, y, yaw + h, 0, 0, 0.15, cfg);
        auto minus = headingRecoveryCentre(x, y, yaw - h, 0, 0, 0.15, cfg);
        CHECK(std::abs((plus.potential - minus.potential) / (2 * h) + r.descent) < 1e-8);
        CHECK(centredDampedYawRate(speed, a, 0.15, b, result));
        CHECK(std::abs((speed * speed + 0.15 * 0.15) * result - speed * a - 0.15 * 0.15 * b) <
              1e-13);
        double angle = d(gen), c = std::cos(angle), s = std::sin(angle);
        auto rotated =
            headingRecoveryCentre(c * x - s * y, s * x + c * y, yaw + angle, 0, 0, 0.15, cfg);
        CHECK(std::abs(rotated.centre - r.centre) < 1e-14);
    }
    const double max = std::numeric_limits<double>::max();
    CHECK(headingRecoveryCentre(max, max, 0, 0, 0, 0.15, cfg).valid);
    CHECK(centredDampedYawRate(max, 1, 0.15, 0, result) && std::isfinite(result));
    CHECK(!centredDampedYawRate(0, 1, 0, 0, result));
    CHECK(!headingRecoveryCentre(std::nan(""), 0, 0, 0, 0, 0.15, cfg).valid);
    double rate;
    CHECK(regularizedReferenceRate(0, 0, 1, 1, 0.15, rate) && rate == 0);
    CHECK(!regularizedReferenceRate(0, 0, std::nan(""), 1, 0.15, rate));
    bool threw = false;
    cfg.axis_bias = 1;
    try {
        cfg.validate();
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
    double lo, hi;
    cfg.axis_bias = 0.5;
    CHECK(centredDampedYawRate(-1e-9, 0.2, 0.15, 0.4, lo));
    CHECK(centredDampedYawRate(1e-9, 0.2, 0.15, 0.4, hi));
    CHECK(std::abs(hi - lo) < 2e-8);
    std::cout << "PASS: 10000 gradient/KKT/rotation cases, rest/lateral/reverse, damping, finite "
                 "extremes, rejection, zero-speed continuity\n";
    return 0;
} catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
}
