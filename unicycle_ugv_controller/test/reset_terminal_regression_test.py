#!/usr/bin/env python3
"""Compile production Reset math and type blocks without ROS/acados.

The narrow extraction keeps the tested equations/defaults in production source.
A kinematic unicycle integrates the returned commands; this is not Gazebo/vehicle
or full state-machine validation. No controller formula is reimplemented here.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]


def block(text, marker, suffix=""):
    start = text.index(marker)
    opening = text.index("{", start)
    depth = 0
    for end in range(opening, len(text)):
        if text[end] == "{":
            depth += 1
        elif text[end] == "}":
            depth -= 1
            if depth == 0:
                return text[start:end + 1] + suffix + "\n"
    raise ValueError("unclosed production block: " + marker)


PRELUDE = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
namespace ros { struct Time {}; }
namespace xgc2_math {
// Boundary double for the external angle utility; no Reset equation lives here.
inline double normalizeAngle(double value) { return std::atan2(std::sin(value), std::cos(value)); }
}
namespace unicycle_ugv_controller {
constexpr double kOmegaDenomEps = 1.0e-8;
'''
MAIN = r'''
}
using namespace unicycle_ugv_controller;
UnicycleBezierPlan straight(bool reverse = false, double length = 1.0) {
    UnicycleBezierPlan plan;
    plan.p1x = length / 3;
    plan.p2x = 2 * length / 3;
    plan.p3x = length;
    plan.T = length / 1.05;
    plan.reverse = reverse;
    plan.valid = true;
    return plan;
}
void integrate(const char* name, const UnicycleBezierPlan& plan, double lateral, double turn,
               double step, bool terminal = false) {
    ControllerConfig config;
    UgvState state;
    UnicycleResetSample initial;
    assert(sampleUnicycleReset(plan, 0, initial));
    state.x = terminal ? plan.p3x + .2 : initial.x;
    state.y = terminal ? plan.p3y + .1 : initial.y;
    state.yaw = initial.yaw;
    bool disturbed = false;
    double t = terminal ? plan.T : 0;
    for (int i = 0; t < config.reset_timeout; ++i) {
        if (!disturbed && t >= .2) {
            state.y += lateral;
            state.yaw += turn;
            disturbed = true;
        }
        const auto command = trackUnicycleReset(state, plan, t, config);
        assert(std::isfinite(command.linear_speed) && std::isfinite(command.angular_speed));
        assert(std::abs(command.linear_speed) <= config.chassis_max_linear_speed + 1e-12);
        assert(std::abs(command.angular_speed) <= config.chassis_max_yaw_rate + 1e-12);
        if (command.position_ok) {
            assert(std::hypot(state.x - plan.p3x, state.y - plan.p3y) <= .05);
            assert(command.linear_speed == 0 && command.angular_speed == 0);
            std::cout << name << " arrived at model t=" << t << '\n';
            return;
        }
        // Deliberately nonuniform integration intervals, not wall-clock measurements.
        const double dt = step * (i % 2 == 0 ? .8 : 1.2);
        state.x += command.linear_speed * std::cos(state.yaw) * dt;
        state.y += command.linear_speed * std::sin(state.yaw) * dt;
        state.yaw = wrapAngle(state.yaw + command.angular_speed * dt);
        t += dt;
    }
    std::cerr << name << " failed; residual=" << std::hypot(state.x-plan.p3x,state.y-plan.p3y) << '\n';
    assert(false && "Reset did not reach unchanged XY tolerance");
}
int main() {
    ControllerConfig config;
    auto plan = straight();
    for (double dt : {.002, .02, .035}) {
        integrate("nominal", plan, 0, 0, dt);
        integrate("lateral-positive", plan, .1, 0, dt);
        integrate("lateral-negative", plan, -.1, 0, dt);
        integrate("yaw-disturbance", plan, .1, .3, dt);
        integrate("reverse", straight(true), -.1, -.3, dt);
        integrate("overshoot", plan, 0, 0, dt, true);
    }
    auto curve = plan;
    curve.p1x = .5;
    curve.p2x = .8;
    curve.p2y = .8;
    curve.p3y = 1;
    curve.T = 2.5;
    integrate("curve", curve, .1, .2, .02);
    integrate("short", straight(false, .12), 0, 0, .02);
    UnicycleResetSample sample;
    for (double t : {plan.T, plan.T + 10.0}) {
        assert(sampleUnicycleReset(plan, t, sample));
        assert(sample.linear_speed == 0 && sample.angular_speed == 0);
        assert(sample.x == plan.p3x && sample.y == plan.p3y);
    }
    UgvState state;
    state.x = plan.p3x;
    state.yaw = M_PI; // XY-only arrival, not a new final-yaw gate.
    auto command = trackUnicycleReset(state, plan, plan.T, config);
    assert(command.position_ok && command.linear_speed == 0 && command.angular_speed == 0);
    state.x = 0;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    command = trackUnicycleReset(state, plan, nan, config);
    assert(!command.position_ok && command.linear_speed == 0 && command.angular_speed == 0);
    assert(!sampleUnicycleReset(plan, nan, sample));
    plan.T = 0;
    command = trackUnicycleReset(state, plan, 1, config);
    assert(command.linear_speed == 0 && command.angular_speed == 0);
}
'''


class ResetTerminalRegression(unittest.TestCase):
    def test_production_reset_math(self):
        source = (PACKAGE / "src/common_types.cpp").read_text()
        header = (PACKAGE / "include/unicycle_ugv_controller/common/types.h").read_text()
        text = PRELUDE
        for name in ("StateSource", "TrackingStrategy"):
            text += block(header, "enum class " + name + " {", ";")
        for name in ("NmpcCostWeights", "ControllerConfig", "UgvState", "UnicycleBezierPlan",
                     "UnicycleResetSample", "UnicycleResetOutput"):
            text += block(header, "struct " + name + " {", ";")
        text += block(source, "struct Vec2 {", ";")
        for marker in (
            "double wrapAngle(", "double hypot2(", "bool finitePose(", "double clamp(",
            "void boxSaturateUnicycle(", "Vec2 bezierPoint(", "Vec2 bezierDeriv(",
            "Vec2 bezierSecondDeriv(", "bool kinematicsAt(", "bool sampleUnicycleReset(",
            "UnicycleResetOutput trackUnicycleReset(",
        ):
            text += block(source, marker)
        text += MAIN
        compiler = shutil.which(os.environ.get("CXX", "g++"))
        self.assertIsNotNone(compiler, "C++17 compiler is required")
        with tempfile.TemporaryDirectory(prefix="reset-terminal-test-") as directory:
            root = Path(directory)
            cpp, binary = root / "test.cpp", root / "test"
            cpp.write_text(text)
            subprocess.run([compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
                            str(cpp), "-o", str(binary)], check=True, timeout=30)
            subprocess.run([str(binary)], check=True, timeout=10)


if __name__ == "__main__":
    unittest.main()
