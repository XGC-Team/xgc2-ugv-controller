// ugv_unicycle_controller: the unicycle UGV controller as a module.
//
// The controller is unicycle_ugv_controller_core, the code of the ROS node
// unicycle_ugv_controller_node (health, Reset, Custom1 tracking with NMPC or flatness): the vehicle
// state, the operator commands, the Reset target and clearance, and the active references arrive on
// input ports instead of topics, and the twist for the chassis, the status and the Reset session
// leave on output ports. The NMPC solve runs on the worker of NmpcExecution; when it has posted its
// result the worker wakes the instance, so the result is taken at once instead of at the next
// period.
//
//   in  state            state  xgc2.ugv.planar_state.v1                      required
//   in  command          event  xgc2.ugv.command.v1
//   in  reset_target     state  xgc2.ugv.reset_target.v1
//   in  reset_clearance  state  xgc2.ugv.reset_clearance.v1
//   in  active_analytic  state  xgc2.ugv.unicycle_reference.analytic.v1       (NMPC)
//   in  active_polynomial state xgc2.ugv.unicycle_reference.polynomial.v1     (NMPC)
//   in  active_sampled   state  xgc2.ugv.unicycle_reference.sampled.v1        (NMPC)
//   in  pva              state  xgc2.ugv.planar_pva.v1                        (flatness)
//   out cmd_vel          state  xgc2.ugv.cmd_vel.v1
//   out status           state  xgc2.ugv.controller_status.v1
//   out reset_session    state  xgc2.ugv.reset_session.v1
//
// The configuration is a JSON object with the keys of config/unicycle_ugv_controller.yaml
// (state_source, tracking_strategy, limits/*, nmpc/*, flatness/*, fence/*, reset/*, chassis/*,
// filter/*, reset_initial_x/y/yaw, ...) and status_publish_rate_hz. A key that is absent keeps the
// controller's default; a key the module does not know is an error. control_rate_hz does not apply:
// the instance's period (period_ms of the manifest) is the control rate. A live configure replaces
// the whole configuration.
//
// The controller's state machine belongs to the thread that built it, and the host steps an
// instance on whichever worker is free; the controller therefore lives on a thread of the instance
// (OwnerThread) and only the reading of the inputs and the writing of the outputs happen on the
// host's thread.
//
// What the state, the references and the Reset target carry is in the host clock; the controller's
// time is the now_ns of the step. Two cmd_vel decisions of one step are one commit: the chassis is
// given the later one, as it would be by the two messages the node published.

#include <xgc2/module.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "module_support.h"
#include "unicycle_ugv_controller/common/core_log.h"
#include "unicycle_ugv_controller/common/reference_types.h"
#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/config_loader.h"
#include "unicycle_ugv_controller/nmpc/nmpc_execution.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

#ifndef UGV_MODULES_VERSION
#define UGV_MODULES_VERSION "0.0.0"
#endif

namespace {

using namespace ugv_modules;
namespace ugv = unicycle_ugv_controller;
namespace sm = ::state_machine;

enum Port : uint32_t {
    kState = 0,
    kCommand,
    kResetTarget,
    kResetClearance,
    kActiveAnalytic,
    kActivePolynomial,
    kActiveSampled,
    kPva,
    kCmdVel,
    kStatus,
    kResetSession,
    kPortCount
};
constexpr uint32_t kInputCount = kCmdVel;

const xgc2_port_desc kPorts[kPortCount] = {
    port<xgc2_ugv_planar_state>("state", XGC2_PORT_IN, XGC2_PORT_STATE,
                                XGC2_UGV_SCHEMA_PLANAR_STATE, 0U, XGC2_PORT_REQUIRED),
    port<xgc2_ugv_command>("command", XGC2_PORT_IN, XGC2_PORT_EVENT, XGC2_UGV_SCHEMA_COMMAND, 8U),
    port<xgc2_ugv_reset_target>("reset_target", XGC2_PORT_IN, XGC2_PORT_STATE,
                                XGC2_UGV_SCHEMA_RESET_TARGET),
    port<xgc2_ugv_reset_clearance>("reset_clearance", XGC2_PORT_IN, XGC2_PORT_STATE,
                                   XGC2_UGV_SCHEMA_RESET_CLEARANCE),
    port<xgc2_ugv_analytic_reference>("active_analytic", XGC2_PORT_IN, XGC2_PORT_STATE,
                                      XGC2_UGV_SCHEMA_ANALYTIC_REFERENCE),
    port<xgc2_ugv_polynomial_reference>("active_polynomial", XGC2_PORT_IN, XGC2_PORT_STATE,
                                        XGC2_UGV_SCHEMA_POLYNOMIAL_REFERENCE),
    port<xgc2_ugv_sampled_reference>("active_sampled", XGC2_PORT_IN, XGC2_PORT_STATE,
                                     XGC2_UGV_SCHEMA_SAMPLED_REFERENCE),
    port<xgc2_ugv_planar_pva>("pva", XGC2_PORT_IN, XGC2_PORT_STATE, XGC2_UGV_SCHEMA_PLANAR_PVA),
    port<xgc2_ugv_cmd_vel>("cmd_vel", XGC2_PORT_OUT, XGC2_PORT_STATE, XGC2_UGV_SCHEMA_CMD_VEL),
    port<xgc2_ugv_controller_status>("status", XGC2_PORT_OUT, XGC2_PORT_STATE,
                                     XGC2_UGV_SCHEMA_CONTROLLER_STATUS),
    port<xgc2_ugv_reset_session>("reset_session", XGC2_PORT_OUT, XGC2_PORT_STATE,
                                 XGC2_UGV_SCHEMA_RESET_SESSION),
};

// The payload constants are the core's.
static_assert(XGC2_UGV_CONTROL_SELF_CHECK == ugv::state_type::SelfCheck, "");
static_assert(XGC2_UGV_CONTROL_READY == ugv::state_type::Ready, "");
static_assert(XGC2_UGV_CONTROL_CUSTOM1 == ugv::state_type::Custom1, "");
static_assert(XGC2_UGV_CONTROL_RESET == ugv::state_type::Reset, "");
static_assert(XGC2_UGV_RESET_RUNNING == ugv::ResetClearance::RUNNING, "");
static_assert(XGC2_UGV_RESET_ARRIVED == ugv::ResetClearance::ARRIVED, "");
static_assert(XGC2_UGV_RESET_REJECTED == ugv::ResetClearance::REJECTED, "");
static_assert(XGC2_UGV_ANALYTIC_HOLD == ugv::reference::AnalyticReference::ANALYTIC_HOLD, "");
static_assert(XGC2_UGV_ANALYTIC_CIRCLE == ugv::reference::AnalyticReference::ANALYTIC_CIRCLE, "");
static_assert(XGC2_UGV_ANALYTIC_CIRCLE_ENTRY ==
                  ugv::reference::AnalyticReference::ANALYTIC_CIRCLE_ENTRY,
              "");
static_assert(XGC2_UGV_ANALYTIC_FIGURE_EIGHT ==
                  ugv::reference::AnalyticReference::ANALYTIC_FIGURE_EIGHT,
              "");
static_assert(XGC2_UGV_SAMPLED_FLAG_EXPLICIT_PLANAR_KINEMATICS ==
                  ugv::reference::SampledReference::FLAG_EXPLICIT_PLANAR_KINEMATICS,
              "");

// ---- payloads -> the controller's plain references ---------------------------------------------

bool toCore(const xgc2_ugv_analytic_reference& p, ugv::reference::AnalyticReference& m) {
    if (p.param_count > XGC2_UGV_MAX_ANALYTIC_PARAMS) {
        return false;
    }
    m = ugv::reference::AnalyticReference{};
    m.request_id = p.request_id;
    m.trajectory_id = p.trajectory_id;
    m.revision = p.revision;
    m.analytic_type = p.analytic_type;
    m.flags = p.flags;
    m.start_time = timeOf<ugv::Time>(p.start_time_ns);
    m.duration = p.duration;
    m.origin.position = {p.origin_x, p.origin_y, p.origin_z};
    m.origin.orientation = {p.origin_qx, p.origin_qy, p.origin_qz, p.origin_qw};
    m.params.assign(p.params, p.params + p.param_count);
    return true;
}

bool toCore(const xgc2_ugv_sampled_reference& p, ugv::reference::SampledReference& m) {
    if (p.point_count > XGC2_UGV_MAX_SAMPLED_POINTS) {
        return false;
    }
    m = ugv::reference::SampledReference{};
    m.trajectory_id = p.trajectory_id;
    m.revision = p.revision;
    m.flags = p.flags;
    m.start_time = timeOf<ugv::Time>(p.start_time_ns);
    m.sample_dt = p.sample_dt;
    m.points.reserve(p.point_count);
    for (uint32_t i = 0; i < p.point_count; ++i) {
        const xgc2_ugv_planar_point& q = p.points[i];
        m.points.push_back({q.t_from_start, q.x, q.y, q.yaw, q.speed, q.linear_acceleration,
                            q.yaw_rate, q.yaw_acceleration, q.curvature, q.vx, q.vy, q.ax, q.ay,
                            q.jx, q.jy});
    }
    return true;
}

bool toCore(const xgc2_ugv_polynomial_reference& p, ugv::reference::ActivePolynomialReference& m) {
    constexpr uint32_t kCoefficients =
        XGC2_UGV_MAX_POLYNOMIAL_SEGMENTS * XGC2_UGV_POLYNOMIAL_COEFFS;
    if (p.segment_count > XGC2_UGV_MAX_POLYNOMIAL_SEGMENTS || p.coeff_x_count > kCoefficients ||
        p.coeff_y_count > kCoefficients || p.coeff_yaw_count > kCoefficients) {
        return false;
    }
    m = ugv::reference::ActivePolynomialReference{};
    m.trajectory_id = p.trajectory_id;
    m.revision = p.revision;
    m.flags = p.flags;
    m.start_time = timeOf<ugv::Time>(p.start_time_ns);
    m.duration = p.duration;
    m.order = static_cast<uint8_t>(p.order);
    m.segment_durations.assign(p.segment_durations, p.segment_durations + p.segment_count);
    m.coeff_x.assign(p.coeff_x, p.coeff_x + p.coeff_x_count);
    m.coeff_y.assign(p.coeff_y, p.coeff_y + p.coeff_y_count);
    m.coeff_yaw.assign(p.coeff_yaw, p.coeff_yaw + p.coeff_yaw_count);
    return true;
}

// ---- the core's log goes to the host ------------------------------------------------------------

std::atomic<const Host*> g_log_host{nullptr};

void forwardCoreLog(ugv::LogLevel level, const char* message) {
    const Host* host = g_log_host.load();
    if (host == nullptr) {
        return;
    }
    host->log(level == ugv::LogLevel::kError ? kLogError
                                             : level == ugv::LogLevel::kWarn ? kLogWarn : kLogInfo,
              message);
}

// ---- the instance
// ---------------------------------------------------------------------------------

// The samples of the input ports in one step, borrowed from the host until the step returns. The
// step reads them on the host's thread; the controller works on them on its own, while the host's
// waits.
struct Inputs {
    struct Reference {
        uint32_t port;
        int64_t stamp_ns;
        const void* payload;
    };
    const xgc2_ugv_planar_state* state{nullptr};
    const xgc2_ugv_reset_target* reset_target{nullptr};
    const xgc2_ugv_reset_clearance* clearance{nullptr};
    const xgc2_ugv_planar_pva* pva{nullptr};
    std::vector<Reference> references;  // the new ones, oldest first
    std::vector<const xgc2_ugv_command*> commands;
};

// What the controller decided in one step, for the host's thread to write.
struct Outputs {
    bool cmd_vel{false};
    uint32_t cmd_kind{XGC2_UGV_CMD_VEL_ZERO};
    double linear_x{0.0};
    double angular_z{0.0};
    xgc2_ugv_reset_session session{};  // the Reset session as it stands now
    bool status_due{false};
    xgc2_ugv_controller_status status{};
};

class Instance {
   public:
    explicit Instance(const Host& host) : host_(host) {}

    ~Instance() {
        shutDown();
        // The controller is torn down where it was built.
        owner_.run([this] {
            execution_.reset();
            controller_.reset();
        });
    }

    const Host& host() const {
        return host_;
    }

    void configure(const xgc2_config* config) {
        JsonSource source(config);
        ugv::ControllerConfig loaded;
        ugv::loadControllerConfig(source, loaded);
        double status_rate = status_publish_rate_hz_;
        source.get("status_publish_rate_hz", status_rate);
        source.rejectUnused();
        if (source.has("control_rate_hz")) {
            throw std::invalid_argument(
                "control_rate_hz does not apply to the module: the period of the instance is the "
                "control rate");
        }
        owner_.run([&] { apply(loaded, status_rate); });
    }

    void start() {
        // The core logs through one process-wide sink.
        g_log_host.store(&host_);
        ugv::setLogSink(&forwardCoreLog);
        if (execution_) {
            execution_->start();
        }
        started_ = true;
    }

    void stop() {
        shutDown();
    }

    void step(const xgc2_step_ctx& ctx) {
        const double now = secondsOf<ugv::Time>(ctx.now_ns);
        const Inputs inputs = read();
        Outputs outputs;
        owner_.run([&] { control(inputs, ctx.now_ns, now, outputs); });
        // Commit last: the consumers run right after this step.
        write(outputs, ctx.now_ns);
    }

   private:
    // ---- the owner thread's part -------------------------------------------------------------

    void apply(const ugv::ControllerConfig& loaded, double status_rate) {
        if (!controller_) {
            controller_ = std::make_unique<ugv::UnicycleUgvController>(state_);
        }
        config_ = loaded;
        status_publish_rate_hz_ =
            std::isfinite(status_rate) && status_rate > 0.0 ? status_rate : 5.0;
        controller_->setConfig(config_);
        const ugv::ResetTarget seed = ugv::initialResetTarget(config_);
        if (seed.valid && !reset_target_from_port_) {
            controller_->setResetTarget(seed);
        }
        if (config_.tracking_strategy == ugv::TrackingStrategy::NMPC && !execution_) {
            execution_ = std::make_unique<ugv::NmpcExecution>(
                *controller_,
                [this](sm::Event event) { return controller_->postEvent(std::move(event)); },
                [this] { return secondsOf<ugv::Time>(host_.nowNs()); }, [this] { host_.wake(); });
            if (started_) {
                execution_->start();
            }
        }
    }

    // One control step: the inputs go to the controller, the controller updates, its output events
    // are served, and what is to be written is collected.
    void control(const Inputs& in, int64_t now_ns, double now, Outputs& out) {
        if (in.state != nullptr) {
            applyState(*in.state, now);
        }
        if (in.reset_target != nullptr) {
            applyResetTarget(*in.reset_target, now);
        }
        for (const Inputs::Reference& reference : in.references) {
            applyReference(reference, now);
        }
        if (in.pva != nullptr) {
            applyPva(*in.pva, now);
        }
        if (in.clearance != nullptr) {
            applyClearance(*in.clearance);
        }
        for (const xgc2_ugv_command* command : in.commands) {
            applyCommand(*command, now_ns, now);
        }

        controller_->update(now);
        if (!controller_->lastResetAdmissionMiss().empty() &&
            controller_->lastResetAdmissionMiss() != last_reset_miss_) {
            last_reset_miss_ = controller_->lastResetAdmissionMiss();
            host_.log(kLogError, last_reset_miss_);
        }
        logStateChanges();
        dispatchOutputs(controller_->stateMachine().currentOutputEvents(), now, out);
        describeSession(out);
        describeStatus(now_ns, out);
    }

    void post(uint32_t id, double stamp_sec, const char* source) {
        sm::Event event(id, sm::EventTimestamp{stamp_sec});
        event.source = source;
        event.category = sm::EventCategory::kInput;
        const auto status = controller_->postEvent(std::move(event));
        if (!status.ok()) {
            host_.log(kLogWarn,
                      format("Failed to post the %s event: %s", source, status.message.c_str()));
        }
    }

    void warn(uint32_t kind, double now, const std::string& message) {
        if (throttle_[kind].due(now)) {
            host_.log(kLogWarn, message);
        }
    }

    void applyState(const xgc2_ugv_planar_state& p, double now) {
        const bool estimator = config_.state_source == ugv::StateSource::STATE_ESTIMATOR;
        const uint8_t expected =
            estimator ? XGC2_UGV_STATE_SOURCE_ESTIMATE : XGC2_UGV_STATE_SOURCE_POSE;
        if (p.source != expected) {
            warn(kState, now,
                 estimator
                     ? "The vehicle state is a pose but the controller takes a state estimate"
                     : "The vehicle state is a state estimate but the controller takes a pose");
            return;
        }
        state_.stamp = timeOf<ugv::Time>(p.stamp_ns);
        state_.x = p.x;
        state_.y = p.y;
        state_.yaw = p.yaw;
        if (estimator) {
            state_.vx = p.vx;
            state_.vy = p.vy;
            state_.speed = p.speed;
            state_.yaw_rate = p.yaw_rate;
            state_.velocity_valid = p.velocity_valid != 0U;
            state_.estimator_state = static_cast<uint8_t>(p.estimator_state);
            state_.estimator_flags = p.estimator_flags;
        } else {
            state_.estimator_state = 0U;
            state_.estimator_flags = 0U;
        }
        state_.received = true;
        post(ugv::event_type::INPUT_STATE_UPDATED, state_.stamp.toSec(),
             estimator ? "state_estimate" : "platform_pose");
    }

    void applyResetTarget(const xgc2_ugv_reset_target& p, double now) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw)) {
            host_.log(kLogWarn, "Ignoring non-finite reset target");
            return;
        }
        ugv::ResetTarget target;
        target.x = p.x;
        target.y = p.y;
        target.yaw = ugv::wrapAngle(p.yaw);
        target.valid = true;
        controller_->setResetTarget(target);
        reset_target_from_port_ = true;
        host_.log(kLogInfo,
                  format("Reset target x=%.3f y=%.3f yaw=%.3f", target.x, target.y, target.yaw));
        post(ugv::event_type::INPUT_RESET_TARGET_UPDATED, now, "reset_pose");
    }

    void applyReference(const Inputs::Reference& reference, double now) {
        bool accepted = false;
        const char* name = "";
        if (reference.port == kActiveAnalytic) {
            ugv::reference::AnalyticReference plain;
            accepted = toCore(*static_cast<const xgc2_ugv_analytic_reference*>(reference.payload),
                              plain) &&
                       controller_->referenceCache().updateAnalytic(plain);
            name = "active_analytic";
        } else if (reference.port == kActivePolynomial) {
            ugv::reference::ActivePolynomialReference plain;
            accepted = toCore(*static_cast<const xgc2_ugv_polynomial_reference*>(reference.payload),
                              plain) &&
                       controller_->referenceCache().updatePolynomial(plain);
            name = "active_polynomial";
        } else {
            ugv::reference::SampledReference plain;
            accepted =
                toCore(*static_cast<const xgc2_ugv_sampled_reference*>(reference.payload), plain) &&
                controller_->referenceCache().updateSampled(plain);
            name = "active_sampled";
        }
        if (!accepted) {
            warn(reference.port, now, std::string("Rejected ") + name + " reference");
            return;
        }
        post(ugv::event_type::INPUT_REFERENCE_UPDATED, now, name);
    }

    void applyPva(const xgc2_ugv_planar_pva& p, double now) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.vx) ||
            !std::isfinite(p.vy) || !std::isfinite(p.ax) || !std::isfinite(p.ay)) {
            warn(kPva, now, "Rejecting non-finite PVA");
            return;
        }
        ugv::WorldPvaReference reference;
        reference.stamp = timeOf<ugv::Time>(p.stamp_ns);
        reference.x = p.x;
        reference.y = p.y;
        reference.yaw = std::isfinite(p.yaw) ? ugv::wrapAngle(p.yaw) : 0.0;
        reference.vx = p.vx;
        reference.vy = p.vy;
        reference.ax = p.ax;
        reference.ay = p.ay;
        reference.valid = true;
        controller_->setWorldPva(reference);
        post(ugv::event_type::INPUT_REFERENCE_UPDATED, reference.stamp.toSec(), "reference_pva");
    }

    void applyClearance(const xgc2_ugv_reset_clearance& p) {
        if (p.status > XGC2_UGV_RESET_REJECTED || p.stamp_ns <= 0) {
            host_.log(kLogWarn, "Ignoring a malformed reset clearance");
            return;
        }
        ugv::ResetClearance clearance;
        clearance.generation = p.generation;
        clearance.stamp_ns = static_cast<uint64_t>(p.stamp_ns);
        clearance.issue_wall = p.issue_wall;
        clearance.status = static_cast<ugv::ResetClearance::Status>(p.status);
        clearance.linear_x = p.linear_x;
        clearance.linear_y = p.linear_y;
        clearance.yaw_rate = p.yaw_rate;
        clearance.lease_seconds = p.lease_seconds;
        controller_->setResetClearance(clearance);
    }

    void applyCommand(const xgc2_ugv_command& command, int64_t now_ns, double now) {
        uint32_t id = 0;
        switch (command.kind) {
            case XGC2_UGV_COMMAND_CUSTOM1:
                id = ugv::event_type::CUSTOM1_REQUESTED;
                break;
            case XGC2_UGV_COMMAND_STOP:
                id = ugv::event_type::STOP_REQUESTED;
                break;
            case XGC2_UGV_COMMAND_RESET:
                id = ugv::event_type::RESET_REQUESTED;
                break;
            default:
                warn(kCommand, now, format("Unknown command kind %u", command.kind));
                return;
        }
        post(id, secondsOf<ugv::Time>(command.stamp_ns > 0 ? command.stamp_ns : now_ns),
             command.source == XGC2_UGV_COMMAND_SOURCE_PUBLIC ? "/command" : "command");
    }

    void dispatchOutputs(const std::vector<sm::Event>& events, double now, Outputs& out) {
        for (const sm::Event& event : events) {
            if (event.id == ugv::output_event_type::REQUEST_NMPC_SOLVE) {
                if (!execution_ || !execution_->handle(event)) {
                    warn(kCmdVel, now, "An NMPC solve was requested but no solver runs");
                }
            } else if (event.id == ugv::output_event_type::PUBLISH_CMD_VEL) {
                const ugv::CmdVel twist = controller_->cmdVel();
                out.cmd_vel = true;
                out.cmd_kind = XGC2_UGV_CMD_VEL_COMMAND;
                out.linear_x = twist.linear_x;
                out.angular_z = twist.angular_z;
            } else if (event.id == ugv::output_event_type::PUBLISH_ZERO_CMD_VEL) {
                out.cmd_vel = true;
                out.cmd_kind = XGC2_UGV_CMD_VEL_ZERO;
                out.linear_x = 0.0;
                out.angular_z = 0.0;
            } else {
                warn(kStatus, now,
                     format("Unhandled output event id: %u", static_cast<unsigned>(event.id)));
            }
        }
    }

    // The Reset session and the health the vehicle's requests are issued under, as they stand.
    void describeSession(Outputs& out) const {
        const ugv::ResetSession session = controller_->resetSession();
        out.session.target_x = session.target.x;
        out.session.target_y = session.target.y;
        out.session.target_yaw = session.target.yaw;
        out.session.generation = session.generation;
        out.session.flags = (session.active ? XGC2_UGV_RESET_SESSION_ACTIVE : 0U) |
                            (controller_->healthReady() ? XGC2_UGV_RESET_SESSION_HEALTHY : 0U);
    }

    void describeStatus(int64_t now_ns, Outputs& out) const {
        const auto period_ns = static_cast<int64_t>(1.0e9 / status_publish_rate_hz_);
        out.status_due =
            now_ns > 0 && (last_status_ns_ == 0 || now_ns - last_status_ns_ >= period_ns);
        if (!out.status_due) {
            return;
        }
        std::string name = controller_->stateMachine().currentStateName(ugv::region_type::CONTROL);
        if (name.empty()) {
            name = "Unknown";
        }
        out.status.stamp_ns = now_ns;
        out.status.control_state =
            controller_->stateMachine().currentState(ugv::region_type::CONTROL);
        out.status.health_state =
            controller_->stateMachine().currentState(ugv::region_type::HEALTH);
        std::strncpy(out.status.control_state_name, name.c_str(),
                     sizeof(out.status.control_state_name) - 1U);
    }

    void logStateChanges() {
        const auto control = controller_->stateMachine().currentState(ugv::region_type::CONTROL);
        const auto health = controller_->stateMachine().currentState(ugv::region_type::HEALTH);
        if (control != last_control_state_) {
            host_.log(kLogInfo, "CONTROL state -> " + controller_->stateMachine().currentStateName(
                                                          ugv::region_type::CONTROL));
            last_control_state_ = control;
        }
        if (health != last_health_state_) {
            host_.log(kLogInfo, format("HEALTH state -> %u", static_cast<unsigned>(health)));
            last_health_state_ = health;
        }
    }

    // ---- the host thread's part ---------------------------------------------------------------

    // The new samples of the input ports, in the order the node's callbacks would have served them.
    Inputs read() {
        Inputs in;
        const auto state = host_.latest<xgc2_ugv_planar_state>(kState);
        if (seen_.fresh(kState, state)) {
            in.state = state.data;
        }
        const auto target = host_.latest<xgc2_ugv_reset_target>(kResetTarget);
        if (seen_.fresh(kResetTarget, target)) {
            in.reset_target = target.data;
        }
        if (config_.tracking_strategy == ugv::TrackingStrategy::FLATNESS) {
            const auto pva = host_.latest<xgc2_ugv_planar_pva>(kPva);
            if (seen_.fresh(kPva, pva)) {
                in.pva = pva.data;
            }
        } else {
            // The three references of the NMPC strategy, the one that was committed last, last.
            const auto analytic = host_.latest<xgc2_ugv_analytic_reference>(kActiveAnalytic);
            const auto polynomial = host_.latest<xgc2_ugv_polynomial_reference>(kActivePolynomial);
            const auto sampled = host_.latest<xgc2_ugv_sampled_reference>(kActiveSampled);
            if (seen_.fresh(kActiveAnalytic, analytic)) {
                in.references.push_back({kActiveAnalytic, analytic.stamp_ns, analytic.data});
            }
            if (seen_.fresh(kActivePolynomial, polynomial)) {
                in.references.push_back({kActivePolynomial, polynomial.stamp_ns, polynomial.data});
            }
            if (seen_.fresh(kActiveSampled, sampled)) {
                in.references.push_back({kActiveSampled, sampled.stamp_ns, sampled.data});
            }
            std::stable_sort(in.references.begin(), in.references.end(),
                             [](const Inputs::Reference& a, const Inputs::Reference& b) {
                                 return a.stamp_ns < b.stamp_ns;
                             });
        }
        const auto clearance = host_.latest<xgc2_ugv_reset_clearance>(kResetClearance);
        if (seen_.fresh(kResetClearance, clearance)) {
            in.clearance = clearance.data;
        }
        for (;;) {
            const auto command = host_.next<xgc2_ugv_command>(kCommand);
            if (!command) {
                break;
            }
            in.commands.push_back(command.data);
        }
        return in;
    }

    void write(const Outputs& out, int64_t now_ns) {
        writeResetSession(out.session, now_ns);
        if (out.status_due) {
            auto slot = host_.write<xgc2_ugv_controller_status>(kStatus);
            if (slot) {
                *slot = out.status;
                if (slot.commit(now_ns)) {
                    last_status_ns_ = now_ns;
                }
            }
        }
        if (out.cmd_vel) {
            auto slot = host_.write<xgc2_ugv_cmd_vel>(kCmdVel);
            if (slot) {
                slot->stamp_ns = now_ns;
                slot->linear_x = out.linear_x;
                slot->linear_y = 0.0;
                slot->angular_z = out.angular_z;
                slot->kind = out.cmd_kind;
                slot.commit(now_ns);
            }
        }
    }

    // The session is written when it changes, and the edge keeps requesting for as long as it
    // holds.
    void writeResetSession(const xgc2_ugv_reset_session& session, int64_t now_ns) {
        const bool same = session_published_ && session.flags == published_session_.flags &&
                          session.generation == published_session_.generation &&
                          session.target_x == published_session_.target_x &&
                          session.target_y == published_session_.target_y &&
                          session.target_yaw == published_session_.target_yaw;
        if (same) {
            return;
        }
        auto slot = host_.write<xgc2_ugv_reset_session>(kResetSession);
        if (!slot) {
            return;
        }
        *slot = session;
        if (slot.commit(now_ns)) {
            session_published_ = true;
            published_session_ = session;
        }
    }

    void shutDown() {
        if (execution_) {
            execution_->stop();
        }
        started_ = false;
        const Host* self = &host_;
        g_log_host.compare_exchange_strong(self, nullptr);
    }

    Host host_;
    OwnerThread owner_;
    // The controller and everything that refers to it are built and used on owner_.
    ugv::UgvState state_;
    std::unique_ptr<ugv::UnicycleUgvController> controller_;
    std::unique_ptr<ugv::NmpcExecution> execution_;
    ugv::ControllerConfig config_;
    double status_publish_rate_hz_{5.0};
    bool reset_target_from_port_{false};
    LogThrottle throttle_[kPortCount];
    std::string last_reset_miss_;
    sm::StateId last_control_state_{0U};
    sm::StateId last_health_state_{0U};
    // The host's thread.
    bool started_{false};
    SeenSamples seen_;
    int64_t last_status_ns_{0};
    bool session_published_{false};
    xgc2_ugv_reset_session published_session_{};
};

const xgc2_module_desc kDescriptor = {XGC2_MODULE_ABI_MAJOR,
                                      XGC2_MODULE_ABI_MINOR,
                                      "ugv_unicycle_controller",
                                      UGV_MODULES_VERSION,
                                      kPorts,
                                      kPortCount,
                                      Lifecycle<Instance>::create,
                                      Lifecycle<Instance>::configure,
                                      Lifecycle<Instance>::start,
                                      Lifecycle<Instance>::step,
                                      Lifecycle<Instance>::stop,
                                      Lifecycle<Instance>::destroy};

}  // namespace

UGV_MODULE_EXPORT const xgc2_module_desc* xgc2_module_entry(void) {
    return &kDescriptor;
}
