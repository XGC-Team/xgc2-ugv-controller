// Differential test: mecanum_ugv_controller::Time/Duration against
// ros::Time/ros::Duration on the operations the controller core uses. Every
// result must match to the bit, including which inputs throw.
//
// Usage: time_equivalence [iterations]   (exit 0 = identical)

#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>

#include <ros/time.h>

#include "mecanum_ugv_controller/common/time.h"

namespace pmc = mecanum_ugv_controller;

namespace {

uint64_t bits(double v) {
    uint64_t b;
    std::memcpy(&b, &v, sizeof b);
    return b;
}

long failures = 0;
long checks = 0;

void expect(bool same, const char* what, double input) {
    ++checks;
    if (!same && failures++ < 20) std::fprintf(stderr, "MISMATCH %s input=%.17g\n", what, input);
}

template <typename F>
std::string outcome(F&& f) {
    try {
        return f();
    } catch (const std::exception& e) {
        return std::string("throw:") + e.what();
    }
}

std::string str(const ros::Time& t) { return std::to_string(t.sec) + "." + std::to_string(t.nsec); }
std::string str(const pmc::Time& t) { return std::to_string(t.sec) + "." + std::to_string(t.nsec); }
std::string str(const ros::Duration& d) { return std::to_string(d.sec) + "." + std::to_string(d.nsec); }
std::string str(const pmc::Duration& d) { return std::to_string(d.sec) + "." + std::to_string(d.nsec); }

}  // namespace

int main(int argc, char** argv) {
    const long iterations = argc > 1 ? std::stol(argv[1]) : 2000000;
    std::mt19937_64 rng(0x1cfb627);
    std::uniform_real_distribution<double> epoch(1.7e9, 1.8e9);       // Session/wall seconds
    std::uniform_real_distribution<double> small(-20.0, 20.0);        // stage offsets, dt
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_int_distribution<int> k(0, 60);

    for (long i = 0; i < iterations; ++i) {
        // Mix typical values with edge cases: exact integers, half-nanosecond
        // boundaries, near-rollover fractions, zero, negatives.
        double t = epoch(rng);
        switch (i % 7) {
            case 1: t = std::floor(t); break;
            case 2: t = std::floor(t) + 0.9999999995; break;
            case 3: t = std::floor(t) + (std::floor(unit(rng) * 1e9) + 0.5) * 1e-9; break;
            case 4: t = unit(rng) * 10.0; break;
            case 5: t = 0.0; break;
            case 6: t = -unit(rng); break;
            default: break;
        }
        const double d = small(rng) * (i % 5 == 0 ? 1e-6 : 1.0) + (i % 11 == 0 ? std::floor(small(rng)) : 0.0);
        const double t2 = epoch(rng);

        expect(outcome([&] { return str(ros::Time(t)); }) == outcome([&] { return str(pmc::Time(t)); }), "Time(double)", t);
        expect(outcome([&] { return str(ros::Duration(d)); }) == outcome([&] { return str(pmc::Duration(d)); }), "Duration(double)", d);
        if (t < 0) continue;

        const ros::Time rt(t), rt2(t2);
        const pmc::Time pt(t), pt2(t2);
        const ros::Duration rd(d);
        const pmc::Duration pd(d);
        expect(bits(rt.toSec()) == bits(pt.toSec()), "Time::toSec", t);
        expect(rt.toNSec() == pt.toNSec(), "Time::toNSec", t);
        expect(rt.isZero() == pt.isZero(), "Time::isZero", t);
        expect(str(rt2 - rt) == str(pt2 - pt), "Time-Time", t);
        expect(bits((rt2 - rt).toSec()) == bits((pt2 - pt).toSec()), "(Time-Time).toSec", t);
        expect(outcome([&] { return str(rt + rd); }) == outcome([&] { return str(pt + pd); }), "Time+Duration", d);
        expect(outcome([&] { return str(rt - rd); }) == outcome([&] { return str(pt - pd); }), "Time-Duration", d);
        expect(bits(rd.toSec()) == bits(pd.toSec()), "Duration::toSec", d);
        expect(str(-rd) == str(-pd), "-Duration", d);
        const double scale = unit(rng);
        expect(outcome([&] { return str(rd * scale); }) == outcome([&] { return str(pd * scale); }), "Duration*", d);
        expect((rt < rt2) == (pt < pt2) && (rt >= rt2) == (pt >= pt2) && (rt == rt) == (pt == pt), "Time compare", t);
        // The core's NMPC reference timing: (now + Duration(i*dt) - planning).toSec()
        const double stage = 0.01 * (1 + i % 5);
        const int n = k(rng);
        expect(outcome([&] { return std::to_string(bits((rt2 + ros::Duration(n * stage) - rt).toSec())); }) ==
                   outcome([&] { return std::to_string(bits((pt2 + pmc::Duration(n * stage) - pt).toSec())); }),
               "stage timing", stage);
    }
    std::printf("%ld checks, %ld mismatches\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
