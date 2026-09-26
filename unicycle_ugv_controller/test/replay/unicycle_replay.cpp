// Deterministic replay of the unicycle UGV controller core, for refactoring
// it safely.
//
// Closed-loop runs of UnicycleUgvController against a kinematic unicycle
// plant, driven through the controller's public API at its 500 Hz control
// rate. Where the ROS node hands work to its output consumers, the harness
// does the same work synchronously:
//   - REQUEST_NMPC_SOLVE: sample the reference horizon, unwrap its yaw against
//     the state, solve with NmpcTrackingBackend and post the result event,
//     as NmpcOutputConsumer does on its worker;
//   - PUBLISH_CMD_VEL / PUBLISH_ZERO_CMD_VEL: saturate the command as
//     CmdVelOutputConsumer does and apply it to the plant.
// Runs:
//   A  NMPC, analytic circle, state-estimator source
//   B  NMPC, analytic figure eight, then a stop and a restart
//   C  NMPC, sampled reference (explicit planar kinematics)
//   D  NMPC, polynomial reference, platform-pose source (the controller's
//      own pose velocity filter), a pose dropout and a pose timeout
//   E  flatness, world PVA reference on a line, then a fence exit
//   F  a reset request without a coordinator grant, until reset_timeout
// Every step writes the controller states, the output events, the command and
// the plant state; every solve writes its status, command and predicted
// states. Doubles are written as hex bits, so two builds of the core are
// compared with `cmp`. Time is explicit: the harness sets every stamp; no
// wall-clock value reaches the output.
//
// Usage: unicycle_replay OUT.txt

#include <ros/time.h>
#include <unicycle_reference_trajectory_msgs/ActivePolynomialReference.h>
#include <unicycle_reference_trajectory_msgs/AnalyticReference.h>
#include <unicycle_reference_trajectory_msgs/SampledReference.h>

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/nmpc/nmpc_tracking_backend.h"
#include "unicycle_ugv_controller/nmpc/unicycle_nmpc_solver.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {
namespace {

namespace refmsg = unicycle_reference_trajectory_msgs;

constexpr double kDt = 0.002;             // 500 Hz control rate
constexpr uint8_t kEstimatorRunning = 3;  // RigidStateEstimate::STATE_RUNNING

FILE* out = nullptr;

uint64_t bits(double v) {
    uint64_t b;
    std::memcpy(&b, &v, sizeof b);
    return b;
}

void d(double v) {
    std::fprintf(out, " %016" PRIx64, bits(v));
}

// The stamp type of the core, from harness seconds.
ros::Time stampAt(double t) {
    return ros::Time(t);
}

struct Plant {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double v{0.0};
    double omega{0.0};

    void step(double linear, double angular, double dt) {
        v = linear;
        omega = angular;
        x += v * std::cos(yaw) * dt;
        y += v * std::sin(yaw) * dt;
        yaw = wrapAngle(yaw + omega * dt);
    }
};

// NmpcOutputConsumer's yaw unwrapping of the sampled horizon.
void unwrapReferenceYaw(std::vector<Se2Reference>& refs, double anchor_yaw) {
    if (!std::isfinite(anchor_yaw))
        return;
    double previous_yaw = anchor_yaw;
    for (auto& ref : refs) {
        if (!std::isfinite(ref.state.yaw))
            continue;
        ref.state.yaw = previous_yaw + wrapAngle(ref.state.yaw - previous_yaw);
        previous_yaw = ref.state.yaw;
    }
}

class Run {
   public:
    Run(const char* name, const ControllerConfig& config, double t0) : controller_(state_), t_(t0) {
        std::fprintf(out, "run %s\n", name);
        controller_.setConfig(config);
        backend_.configure(config);
    }

    UnicycleUgvController& controller() {
        return controller_;
    }
    UgvState& state() {
        return state_;
    }
    Plant& plant() {
        return plant_;
    }
    double now() const {
        return t_;
    }

    void post(uint32_t id, double t) {
        ::state_machine::Event event(id, ::state_machine::EventTimestamp{t});
        event.source = "replay";
        event.category = ::state_machine::EventCategory::kInput;
        const bool ok = controller_.postEvent(std::move(event)).ok();
        std::fprintf(out, "post %u at", id);
        d(t);
        std::fprintf(out, " ok %d\n", ok ? 1 : 0);
    }

    // One control period: measure the plant, update the controller, serve its
    // output events, move the plant.
    void step(bool pose_available = true) {
        if (pose_available)
            measure();
        controller_.update(t_);
        const auto events = controller_.stateMachine().currentOutputEvents();
        std::fprintf(out, "%.3f h %u c %u out", t_,
                     controller_.stateMachine().currentState(region_type::HEALTH),
                     controller_.stateMachine().currentState(region_type::CONTROL));
        for (const auto& event : events)
            std::fprintf(out, " %u", event.id);
        const ControlCommand command = controller_.command();
        std::fprintf(out, " cmd %d", command.valid ? 1 : 0);
        d(command.stamp.toSec());
        d(command.linear_speed);
        d(command.angular_speed);
        std::fprintf(out, " plant");
        d(plant_.x);
        d(plant_.y);
        d(plant_.yaw);
        std::fputc('\n', out);
        for (const auto& event : events) {
            if (event.id == output_event_type::REQUEST_NMPC_SOLVE)
                solve(event);
            if (event.id == output_event_type::PUBLISH_CMD_VEL)
                apply(command);
            if (event.id == output_event_type::PUBLISH_ZERO_CMD_VEL) {
                linear_ = 0.0;
                angular_ = 0.0;
            }
        }
        plant_.step(linear_, angular_, kDt);
        t_ += kDt;
    }

    void steps(int n, bool pose_available = true) {
        for (int i = 0; i < n; ++i)
            step(pose_available);
    }

   private:
    void measure() {
        const auto cfg = controller_.config();
        state_.received = true;
        state_.stamp = stampAt(t_);
        state_.x = plant_.x;
        state_.y = plant_.y;
        state_.yaw = plant_.yaw;
        if (cfg.state_source == StateSource::STATE_ESTIMATOR) {
            state_.vx = plant_.v * std::cos(plant_.yaw);
            state_.vy = plant_.v * std::sin(plant_.yaw);
            state_.speed = plant_.v;
            state_.yaw_rate = plant_.omega;
            state_.velocity_valid = true;
            state_.estimator_state = kEstimatorRunning;
            state_.estimator_flags = 0U;
        }
    }

    void solve(const ::state_machine::Event& request) {
        const ControllerConfig config = controller_.config();
        const double t = request.timestamp > 0.0 ? request.timestamp : t_;
        const ros::Time now = stampAt(t);
        const UgvState state = controller_.state();
        const double stage_dt =
            config.prediction_horizon / static_cast<double>(UnicycleNmpcSolver::horizonSteps());
        std::vector<Se2Reference> refs;
        bool ok = controller_.referenceCache().sampleHorizon(
            now, stage_dt, UnicycleNmpcSolver::horizonSteps(), refs);
        ControlCommand command;
        if (ok) {
            unwrapReferenceYaw(refs, state.yaw);
            backend_.configure(config);
            if (!entered_)
                entered_ = backend_.enter();
            ok = entered_ && backend_.compute(state, refs, now, command);
        }
        std::fprintf(out, "  solve %" PRIu64 " ok %d status %d cmd %d", request.correlation_id,
                     ok ? 1 : 0, backend_.status(), command.valid ? 1 : 0);
        d(command.stamp.toSec());
        d(command.linear_speed);
        d(command.angular_speed);
        std::fprintf(out, " refs %zu", refs.size());
        for (const auto& ref : refs) {
            d(ref.state.position.x());
            d(ref.state.position.y());
            d(ref.state.yaw);
        }
        if (ok) {
            std::fprintf(out, " pred %zu", backend_.predictedStateCount());
            for (size_t i = 0; i < backend_.predictedStateCount(); ++i)
                for (int k = 0; k < backend_.predictedStates()[i].size(); ++k)
                    d(backend_.predictedStates()[i](k));
        }
        std::fputc('\n', out);
        const bool success = ok && command.valid;
        ::state_machine::Event result(
            success ? event_type::INPUT_NMPC_SOLVE_SUCCEEDED : event_type::INPUT_NMPC_SOLVE_FAILED,
            ::state_machine::EventTimestamp{t});
        result.source = "nmpc_output_consumer";
        result.category = ::state_machine::EventCategory::kInput;
        result.correlation_id = request.correlation_id;
        if (success) {
            result.payload["command_stamp"] = command.stamp.toSec();
            result.payload["linear_speed"] = command.linear_speed;
            result.payload["angular_speed"] = command.angular_speed;
        }
        (void)controller_.postEvent(std::move(result));
    }

    // CmdVelOutputConsumer::makeTwist outside Reset.
    void apply(const ControlCommand& command) {
        const auto cfg = controller_.config();
        linear_ = 0.0;
        angular_ = 0.0;
        if (!command.valid || !std::isfinite(command.linear_speed) ||
            !std::isfinite(command.angular_speed))
            return;
        const auto control = controller_.stateMachine().currentState(region_type::CONTROL);
        if (control == state_type::Reset)
            return;
        if (control == state_type::Custom1 && cfg.tracking_strategy == TrackingStrategy::NMPC) {
            linear_ = clamp(command.linear_speed, cfg.min_linear_speed, cfg.max_linear_speed);
            angular_ = clamp(command.angular_speed, -cfg.max_angular_speed, cfg.max_angular_speed);
        } else {
            linear_ = clamp(command.linear_speed, -cfg.chassis_max_linear_speed,
                            cfg.chassis_max_linear_speed);
            angular_ =
                clamp(command.angular_speed, -cfg.chassis_max_yaw_rate, cfg.chassis_max_yaw_rate);
        }
    }

    UgvState state_;
    UnicycleUgvController controller_;
    NmpcTrackingBackend backend_;
    bool entered_{false};
    Plant plant_;
    double linear_{0.0};
    double angular_{0.0};
    double t_;
};

refmsg::AnalyticReference analytic(uint16_t type, double start, double duration,
                                   std::vector<double> params) {
    refmsg::AnalyticReference msg;
    msg.trajectory_id = 7U;
    msg.revision = 1U;
    msg.analytic_type = type;
    msg.start_time = stampAt(start);
    msg.duration = duration;
    msg.origin.orientation.w = 1.0;
    msg.params = std::move(params);
    return msg;
}

void runAnalyticCircle() {
    ControllerConfig cfg;
    Run run("A nmpc analytic circle estimator", cfg, 10.0);
    run.steps(3);
    // radius, line speed, entry duration, center x, center y
    const bool accepted = run.controller().referenceCache().updateAnalytic(analytic(
        refmsg::AnalyticReference::ANALYTIC_CIRCLE, 10.004, 30.0, {1.5, 0.6, 3.0, 0.0, 1.5}));
    std::fprintf(out, "reference %d\n", accepted ? 1 : 0);
    run.post(event_type::CUSTOM1_REQUESTED, run.now());
    run.steps(2500);
}

void runFigureEightStopRestart() {
    ControllerConfig cfg;
    Run run("B nmpc analytic figure eight, stop, restart", cfg, 20.0);
    run.plant().yaw = 0.3;
    run.steps(3);
    const bool accepted = run.controller().referenceCache().updateAnalytic(
        analytic(refmsg::AnalyticReference::ANALYTIC_FIGURE_EIGHT, 20.004, 40.0,
                 {1.2, 0.5, 3.0, 0.2, -0.1}));
    std::fprintf(out, "reference %d\n", accepted ? 1 : 0);
    run.post(event_type::CUSTOM1_REQUESTED, run.now());
    run.steps(1200);
    run.post(event_type::STOP_REQUESTED, run.now());
    run.steps(150);
    run.post(event_type::CUSTOM1_REQUESTED, run.now());
    run.steps(900);
}

void runSampled() {
    ControllerConfig cfg;
    Run run("C nmpc sampled reference", cfg, 30.0);
    run.steps(3);
    refmsg::SampledReference msg;
    msg.trajectory_id = 9U;
    msg.revision = 2U;
    msg.flags = refmsg::SampledReference::FLAG_EXPLICIT_PLANAR_KINEMATICS;
    msg.start_time = stampAt(30.004);
    msg.sample_dt = 0.05;
    // A constant-curvature arc: speed 0.5 m/s, yaw rate 0.25 rad/s.
    const double v = 0.5, w = 0.25;
    for (int k = 0; k <= 200; ++k) {
        const double s = k * msg.sample_dt;
        refmsg::PlanarReferencePoint p;
        p.t_from_start = s;
        p.yaw = w * s;
        p.x = v / w * std::sin(p.yaw);
        p.y = v / w * (1.0 - std::cos(p.yaw));
        p.speed = v;
        p.yaw_rate = w;
        p.curvature = w / v;
        p.vx = v * std::cos(p.yaw);
        p.vy = v * std::sin(p.yaw);
        p.ax = -v * w * std::sin(p.yaw);
        p.ay = v * w * std::cos(p.yaw);
        p.jx = -v * w * w * std::cos(p.yaw);
        p.jy = -v * w * w * std::sin(p.yaw);
        msg.points.push_back(p);
    }
    const bool accepted = run.controller().referenceCache().updateSampled(msg);
    std::fprintf(out, "reference %d\n", accepted ? 1 : 0);
    run.post(event_type::CUSTOM1_REQUESTED, run.now());
    run.steps(2000);
}

void runPolynomialPose() {
    ControllerConfig cfg;
    cfg.state_source = StateSource::PLATFORM_POSE;
    Run run("D nmpc polynomial reference, pose source, dropout, timeout", cfg, 40.0);
    run.steps(3);
    refmsg::ActivePolynomialReference msg;
    msg.trajectory_id = 11U;
    msg.revision = 1U;
    msg.start_time = stampAt(40.004);
    msg.order = 3U;
    msg.segment_durations = {4.0, 4.0};
    msg.duration = 8.0;
    // Two cubic segments per axis, coefficients c0..c3 in time from segment start.
    msg.coeff_x = {0.0, 0.3, 0.02, 0.0, 1.52, 0.46, -0.04, 0.0};
    msg.coeff_y = {0.0, 0.0, 0.03, -0.002, 0.352, 0.144, -0.03, 0.002};
    msg.coeff_yaw = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const bool accepted = run.controller().referenceCache().updatePolynomial(msg);
    std::fprintf(out, "reference %d\n", accepted ? 1 : 0);
    run.post(event_type::CUSTOM1_REQUESTED, run.now());
    run.steps(1000);
    run.steps(60, false);  // a brief pose dropout (below state_timeout)
    run.steps(500);
    run.steps(300, false);  // a pose timeout (above state_timeout)
    run.steps(400);
}

void runFlatnessFence() {
    ControllerConfig cfg;
    cfg.tracking_strategy = TrackingStrategy::FLATNESS;
    cfg.state_source = StateSource::PLATFORM_POSE;
    cfg.fence_x_max = 1.2;
    Run run("E flatness world PVA, fence exit", cfg, 50.0);
    run.steps(3);
    run.post(event_type::CUSTOM1_REQUESTED, run.now());
    for (int k = 0; k < 2500; ++k) {
        const double s = run.now() - 50.0;
        WorldPvaReference ref;
        ref.stamp = stampAt(run.now());
        ref.x = 0.4 * s;
        ref.y = 0.1 * std::sin(0.5 * s);
        ref.yaw = std::atan2(0.05 * std::cos(0.5 * s), 0.4);
        ref.vx = 0.4;
        ref.vy = 0.05 * std::cos(0.5 * s);
        ref.ax = 0.0;
        ref.ay = -0.025 * std::sin(0.5 * s);
        ref.valid = true;
        run.controller().setWorldPva(ref);
        run.step();
    }
}

void runResetTimeout() {
    ControllerConfig cfg;
    cfg.state_source = StateSource::PLATFORM_POSE;
    cfg.reset_timeout = 0.3;
    Run run("F reset request, no grant, timeout", cfg, 60.0);
    run.steps(3);
    ResetTarget goal;
    goal.x = 2.0;
    goal.y = 0.5;
    goal.valid = true;
    run.controller().setResetTarget(goal);
    run.post(event_type::RESET_REQUESTED, run.now());
    run.steps(250);
}

}  // namespace
}  // namespace unicycle_ugv_controller

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: unicycle_replay OUT.txt\n");
        return 2;
    }
    using namespace unicycle_ugv_controller;
    out = std::fopen(argv[1], "w");
    if (!out)
        return 2;
    ros::Time::init();  // simulated time: the harness sets every stamp
    runAnalyticCircle();
    runFigureEightStopRestart();
    runSampled();
    runPolynomialPose();
    runFlatnessFence();
    runResetTimeout();
    std::fclose(out);
    return 0;
}
