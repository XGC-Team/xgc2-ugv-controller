// Deterministic replay of the mecanum UGV controller core, for refactoring it
// safely.
//
// Closed-loop runs of MecanumUgvController against a kinematic holonomic
// plant, driven through the controller's public API at its 500 Hz control
// rate. The harness serves the output events as the ROS node does: it applies
// the command for PUBLISH_CMD_VEL (outside Reset) and zero for
// PUBLISH_ZERO_CMD_VEL. Runs:
//   A  world velocity reference on a curve, heading hold
//   B  a stop and a restart, a pose dropout and a pose timeout
//   C  a fence exit
//   D  a reset request without a coordinator grant, until reset_timeout
// Every step writes the controller states, the output events, the command and
// the plant state, with doubles as hex bits, so two builds of the core are
// compared with `cmp`. The harness sets every stamp; no wall-clock value
// reaches the output.
//
// Usage: mecanum_replay OUT.txt

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <optional>
#include <type_traits>
#include <cstring>
#include <exception>
#include <utility>

#include "mecanum_ugv_controller/common/types.h"
#include "mecanum_ugv_controller/mecanum_ugv_controller.h"

namespace mecanum_ugv_controller {
namespace {

constexpr double kDt = 0.002;  // 500 Hz control rate

FILE* out = nullptr;

uint64_t bits(double v) {
    uint64_t b;
    std::memcpy(&b, &v, sizeof b);
    return b;
}

void d(double v) {
    std::fprintf(out, " %016" PRIx64, bits(v));
}

// The core's stamp, from harness seconds.
Time stampAt(double t) {
    return Time(t);
}

struct Plant {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};

    // FLU body command.
    void step(double vx, double vy, double wz, double dt) {
        const double c = std::cos(yaw), s = std::sin(yaw);
        x += (c * vx - s * vy) * dt;
        y += (s * vx + c * vy) * dt;
        yaw = wrapAngle(yaw + wz * dt);
    }
};

class Run {
   public:
    Run(const char* name, const ControllerConfig& config, double t0) : controller_(state_), t_(t0) {
        std::fprintf(out, "run %s\n", name);
        controller_.setConfig(config);
    }

    MecanumUgvController& controller() {
        return controller_;
    }
    Plant& plant() {
        return plant_;
    }
    double now() const {
        return t_;
    }

    void post(OperatorCommand command, double t) {
        pending_command_ = command;  // test script: deliver at the same original owner turn
        const unsigned id = command == OperatorCommand::kCustom1 ? 1U :
                            command == OperatorCommand::kStop ? 2U : 3U;
        std::fprintf(out, "post %u at", id);
        d(t);
        std::fprintf(out, " ok 1\n");
    }

    void reference(double vx, double vy) {
        WorldVelocityReference ref;
        ref.stamp = stampAt(t_);
        ref.vx = vx;
        ref.vy = vy;
        ref.valid = true;
        controller_.setWorldReference(ref);
    }

    void step(bool pose_available = true) {
        if (pose_available) {
            state_.received = true;
            state_.stamp = stampAt(t_);
            state_.x = plant_.x;
            state_.y = plant_.y;
            state_.yaw = plant_.yaw;
        }
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        if (pending_command_) {
            controller_.command(*pending_command_, t_, wall, "replay");
            pending_command_.reset();
        } else controller_.update(t_, wall);
        const auto events = controller_.outputs();
        std::fprintf(out, "%.3f h %u c %u out", t_,
                     100U,
                     static_cast<uint32_t>(controller_.mode()));
        for (const auto& event : events) {
            std::fprintf(out, " %u", (std::holds_alternative<PublishCommand>(event) ? 10001U : 10002U));
        }
        const ControlCommand command = controller_.command();
        std::fprintf(out, " cmd %d", command.valid ? 1 : 0);
        d(command.stamp.toSec());
        d(command.linear_x);
        d(command.linear_y);
        d(command.angular_z);
        std::fprintf(out, " plant");
        d(plant_.x);
        d(plant_.y);
        d(plant_.yaw);
        std::fputc('\n', out);
        for (const auto& event : events) {
            if (std::holds_alternative<PublishCommand>(event)) {
                const bool reset = static_cast<uint32_t>(controller_.mode()) ==
                                   static_cast<uint32_t>(ControlMode::Reset);
                vx_ = command.valid && !reset ? command.linear_x : 0.0;
                vy_ = command.valid && !reset ? command.linear_y : 0.0;
                wz_ = command.valid && !reset ? command.angular_z : 0.0;
            }
            if (std::holds_alternative<PublishZero>(event)) {
                vx_ = vy_ = wz_ = 0.0;
            }
        }
        plant_.step(vx_, vy_, wz_, kDt);
        t_ += kDt;
    }

    void steps(int n, bool pose_available = true) {
        for (int i = 0; i < n; ++i) {
            step(pose_available);
        }
    }

   private:
    std::optional<OperatorCommand> pending_command_;
    UgvState state_;
    MecanumUgvController controller_;
    Plant plant_;
    double vx_{0.0};
    double vy_{0.0};
    double wz_{0.0};
    double t_;
};

void runCurve() {
    ControllerConfig cfg;
    cfg.heading_target_yaw = 0.4;
    Run run("A world velocity curve, heading hold", cfg, 10.0);
    run.plant().yaw = -0.3;
    run.steps(3);
    run.post(OperatorCommand::kCustom1, run.now());
    for (int k = 0; k < 2000; ++k) {
        const double s = run.now() - 10.0;
        run.reference(0.4 * std::cos(0.3 * s), 0.3 * std::sin(0.5 * s));
        run.step();
    }
}

void runStopDropout() {
    ControllerConfig cfg;
    Run run("B stop, restart, pose dropout, pose timeout", cfg, 20.0);
    run.steps(3);
    run.post(OperatorCommand::kCustom1, run.now());
    for (int k = 0; k < 600; ++k) {
        run.reference(0.3, -0.1);
        run.step();
    }
    run.post(OperatorCommand::kStop, run.now());
    run.steps(100);
    run.post(OperatorCommand::kCustom1, run.now());
    for (int k = 0; k < 400; ++k) {
        run.reference(-0.2, 0.25);
        run.step();
    }
    for (int k = 0; k < 60; ++k) {
        run.reference(-0.2, 0.25);
        run.step(false);  // below state_timeout
    }
    for (int k = 0; k < 300; ++k) {
        run.reference(-0.2, 0.25);
        run.step(false);  // above state_timeout
    }
    run.steps(300);
}

void runFence() {
    ControllerConfig cfg;
    cfg.fence_x_max = 0.8;
    Run run("C fence exit", cfg, 30.0);
    run.steps(3);
    run.post(OperatorCommand::kCustom1, run.now());
    for (int k = 0; k < 1500; ++k) {
        run.reference(0.5, 0.05);
        run.step();
    }
}

void runResetTimeout() {
    ControllerConfig cfg;
    cfg.reset_timeout = 0.3;
    Run run("D reset request, no grant, timeout", cfg, 40.0);
    run.steps(3);
    ResetTarget goal;
    goal.x = 1.5;
    goal.y = -0.5;
    goal.yaw = 0.2;
    goal.valid = true;
    run.controller().setResetTarget(goal);
    run.post(OperatorCommand::kReset, run.now());
    run.steps(250);
}

}  // namespace
}  // namespace mecanum_ugv_controller

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: mecanum_replay OUT.txt\n");
        return 2;
    }
    using namespace mecanum_ugv_controller;
    out = std::fopen(argv[1], "w");
    if (!out) {
        return 2;
    }
    try {
        runCurve();
        runStopDropout();
        runFence();
        runResetTimeout();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "replay failed: %s\n", error.what());
        std::fclose(out);
        return 1;
    } catch (...) {
        std::fprintf(stderr, "replay failed: unknown exception\n");
        std::fclose(out);
        return 1;
    }
    std::fclose(out);
    return 0;
}
