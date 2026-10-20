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
//   in  reset_grant      event  xgc2.ugv.reset_grant.v1, 16     (the coordinator's messages, in order)
//   in  active_analytic  state  xgc2.ugv.unicycle_reference.analytic.v1       (NMPC)
//   in  active_polynomial state xgc2.ugv.unicycle_reference.polynomial.v1     (NMPC)
//   in  active_sampled   state  xgc2.ugv.unicycle_reference.sampled.v1        (NMPC)
//   in  pva              state  xgc2.ugv.planar_pva.v1                        (flatness)
//   out cmd_vel          state  xgc2.ugv.cmd_vel.v1
//   out status           state  xgc2.ugv.controller_status.v1
//   out reset_session    state  xgc2.ugv.reset_session.v2      (the executor decides when it is due)
//   out reset_result     event  xgc2.ugv.reset_result.v1, 4    (the one outcome of a session)
//
// The configuration is a JSON object with the keys of config/unicycle_ugv_controller.yaml
// (state_source, tracking_strategy, limits/*, nmpc/*, flatness/*, fence/*, reset/*, chassis/*,
// filter/*, reset_initial_x/y/yaw, ...) and status_publish_rate_hz. A key that is absent keeps the
// controller's default; a key the module does not know is an error. control_rate_hz does not apply:
// the instance's period (period_ms of the manifest) is the control rate. A live configure replaces
// the whole configuration.
//
// The controller's state machine belongs to the thread that built it, so the instance has to be
// called on one thread for its life: the host's thread affinity (manifest `affinity = "sticky"`). A
// step on another thread fails the instance with a message that says so.
//
// What the state, the references and the Reset target carry is in the host clock; the controller's
// time is the now_ns of the step. Two cmd_vel decisions of one step are one commit: the chassis is
// given the later one, as it would be by the two messages the node published.

#include <xgc2/module.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
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
namespace rc = ugv_reset_client;
namespace sm = ::state_machine;

enum Port : uint32_t {
    kState = 0,
    kCommand,
    kResetTarget,
    kResetGrant,
    kActiveAnalytic,
    kActivePolynomial,
    kActiveSampled,
    kPva,
    kCmdVel,
    kStatus,
    kResetSession,
    kResetResult,
    kPortCount
};
constexpr uint32_t kInputCount = kCmdVel;

const xgc2_port_desc kPorts[kPortCount] = {
    port<xgc2_ugv_planar_state>("state", XGC2_PORT_IN, XGC2_PORT_STATE,
                                XGC2_UGV_SCHEMA_PLANAR_STATE, 0U, XGC2_PORT_REQUIRED),
    port<xgc2_ugv_command>("command", XGC2_PORT_IN, XGC2_PORT_EVENT, XGC2_UGV_SCHEMA_COMMAND, 8U),
    port<xgc2_ugv_reset_target>("reset_target", XGC2_PORT_IN, XGC2_PORT_STATE,
                                XGC2_UGV_SCHEMA_RESET_TARGET),
    port<xgc2_ugv_reset_grant>("reset_grant", XGC2_PORT_IN, XGC2_PORT_EVENT,
                               XGC2_UGV_SCHEMA_RESET_GRANT, 16U),
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
    port<xgc2_ugv_reset_result>("reset_result", XGC2_PORT_OUT, XGC2_PORT_EVENT,
                                XGC2_UGV_SCHEMA_RESET_RESULT, 4U),
};

// The payload constants are the core's.
static_assert(XGC2_UGV_CONTROL_SELF_CHECK == ugv::state_type::SelfCheck, "");
static_assert(XGC2_UGV_CONTROL_READY == ugv::state_type::Ready, "");
static_assert(XGC2_UGV_CONTROL_CUSTOM1 == ugv::state_type::Custom1, "");
static_assert(XGC2_UGV_CONTROL_RESET == ugv::state_type::Reset, "");
static_assert(XGC2_UGV_RESET_PHASE_IDLE == static_cast<uint32_t>(rc::Phase::Idle), "");
static_assert(XGC2_UGV_RESET_PHASE_REQUESTING == static_cast<uint32_t>(rc::Phase::Requesting), "");
static_assert(XGC2_UGV_RESET_PHASE_WAITING == static_cast<uint32_t>(rc::Phase::Waiting), "");
static_assert(XGC2_UGV_RESET_PHASE_MOVING == static_cast<uint32_t>(rc::Phase::Moving), "");
static_assert(XGC2_UGV_RESET_PHASE_SETTLING == static_cast<uint32_t>(rc::Phase::Settling), "");
static_assert(XGC2_UGV_RESET_PHASE_DONE == static_cast<uint32_t>(rc::Phase::Done), "");
static_assert(XGC2_UGV_RESET_MODEL_UNICYCLE == static_cast<uint32_t>(rc::Model::Unicycle), "");
static_assert(XGC2_UGV_RESET_MODEL_OMNI == static_cast<uint32_t>(rc::Model::Omni), "");
static_assert(XGC2_UGV_RESET_GRANT_HOLD == static_cast<uint32_t>(rc::GrantState::Hold), "");
static_assert(XGC2_UGV_RESET_GRANT_GO == static_cast<uint32_t>(rc::GrantState::Go), "");
static_assert(XGC2_UGV_RESET_GRANT_BRAKE == static_cast<uint32_t>(rc::GrantState::Brake), "");
static_assert(XGC2_UGV_RESET_GRANT_REJECT == static_cast<uint32_t>(rc::GrantState::Reject), "");
static_assert(XGC2_UGV_MAX_RESET_VIA == rc::kMaxVia, "");
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

class Instance {
   public:
    explicit Instance(const Host& host) : host_(host), controller_(state_) {
        thread_.bind();  // the controller's state machine was built by the member controller_
    }

    ~Instance() {
        shutDown();
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

        config_ = loaded;
        status_publish_rate_hz_ =
            std::isfinite(status_rate) && status_rate > 0.0 ? status_rate : 5.0;
        controller_.setConfig(config_);
        const ugv::ResetTarget seed = ugv::initialResetTarget(config_);
        if (seed.valid && !reset_target_from_port_) {
            controller_.setResetTarget(seed);
        }
        if (config_.tracking_strategy == ugv::TrackingStrategy::NMPC && !execution_) {
            execution_ = std::make_unique<ugv::NmpcExecution>(
                controller_,
                [this](sm::Event event) { return controller_.postEvent(std::move(event)); },
                [this] { return secondsOf<ugv::Time>(host_.nowNs()); }, [this] { host_.wake(); });
            if (started_) {
                execution_->start();
            }
        }
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
        thread_.check("step");
        const double now = secondsOf<ugv::Time>(ctx.now_ns);
        readState(now);
        readResetTarget(now);
        readReferences(now);
        readGrants();
        readCommands(ctx.now_ns, now);

        controller_.update(now);
        if (!controller_.lastResetAdmissionMiss().empty() &&
            controller_.lastResetAdmissionMiss() != last_reset_miss_) {
            last_reset_miss_ = controller_.lastResetAdmissionMiss();
            host_.log(kLogError, last_reset_miss_);
        }
        logStateChanges();

        // Commit last: the consumers run right after this step.
        dispatchOutputs(controller_.stateMachine().currentOutputEvents(), now);
        publishResetSession(ctx.now_ns);
        publishResetResult(ctx.now_ns);
        publishStatusIfDue(ctx.now_ns);
        publishCmdVel(ctx.now_ns);
    }

   private:
    void shutDown() {
        if (execution_) {
            execution_->stop();
        }
        started_ = false;
        const Host* self = &host_;
        g_log_host.compare_exchange_strong(self, nullptr);
    }

    // ---- inputs ---------------------------------------------------------------------------------

    void post(uint32_t id, double stamp_sec, const char* source) {
        sm::Event event(id, sm::EventTimestamp{stamp_sec});
        event.source = source;
        event.category = sm::EventCategory::kInput;
        const auto status = controller_.postEvent(std::move(event));
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

    void readState(double now) {
        const auto sample = host_.latest<xgc2_ugv_planar_state>(kState);
        if (!seen_.fresh(kState, sample)) {
            return;
        }
        const xgc2_ugv_planar_state& p = *sample;
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

    void readResetTarget(double now) {
        const auto sample = host_.latest<xgc2_ugv_reset_target>(kResetTarget);
        if (!seen_.fresh(kResetTarget, sample)) {
            return;
        }
        if (!std::isfinite(sample->x) || !std::isfinite(sample->y) || !std::isfinite(sample->yaw)) {
            host_.log(kLogWarn, "Ignoring non-finite reset target");
            return;
        }
        ugv::ResetTarget target;
        target.x = sample->x;
        target.y = sample->y;
        target.yaw = ugv::wrapAngle(sample->yaw);
        target.valid = true;
        controller_.setResetTarget(target);
        reset_target_from_port_ = true;
        host_.log(kLogInfo,
                  format("Reset target x=%.3f y=%.3f yaw=%.3f", target.x, target.y, target.yaw));
        post(ugv::event_type::INPUT_RESET_TARGET_UPDATED, now, "reset_pose");
    }

    // The three references of the NMPC strategy, the one that was committed last last.
    void readReferences(double now) {
        if (config_.tracking_strategy == ugv::TrackingStrategy::FLATNESS) {
            readPva(now);
            return;
        }
        const auto analytic = host_.latest<xgc2_ugv_analytic_reference>(kActiveAnalytic);
        const auto polynomial = host_.latest<xgc2_ugv_polynomial_reference>(kActivePolynomial);
        const auto sampled = host_.latest<xgc2_ugv_sampled_reference>(kActiveSampled);
        struct Fresh {
            int64_t stamp_ns;
            uint32_t port;
        };
        std::vector<Fresh> fresh;
        if (seen_.fresh(kActiveAnalytic, analytic)) {
            fresh.push_back({analytic.stamp_ns, kActiveAnalytic});
        }
        if (seen_.fresh(kActivePolynomial, polynomial)) {
            fresh.push_back({polynomial.stamp_ns, kActivePolynomial});
        }
        if (seen_.fresh(kActiveSampled, sampled)) {
            fresh.push_back({sampled.stamp_ns, kActiveSampled});
        }
        std::stable_sort(fresh.begin(), fresh.end(),
                         [](const Fresh& a, const Fresh& b) { return a.stamp_ns < b.stamp_ns; });
        for (const Fresh& item : fresh) {
            bool accepted = false;
            const char* name = "";
            if (item.port == kActiveAnalytic) {
                ugv::reference::AnalyticReference plain;
                accepted =
                    toCore(*analytic, plain) && controller_.referenceCache().updateAnalytic(plain);
                name = "active_analytic";
            } else if (item.port == kActivePolynomial) {
                ugv::reference::ActivePolynomialReference plain;
                accepted = toCore(*polynomial, plain) &&
                           controller_.referenceCache().updatePolynomial(plain);
                name = "active_polynomial";
            } else {
                ugv::reference::SampledReference plain;
                accepted =
                    toCore(*sampled, plain) && controller_.referenceCache().updateSampled(plain);
                name = "active_sampled";
            }
            if (!accepted) {
                warn(item.port, now, std::string("Rejected ") + name + " reference");
                continue;
            }
            post(ugv::event_type::INPUT_REFERENCE_UPDATED, now, name);
        }
    }

    void readPva(double now) {
        const auto sample = host_.latest<xgc2_ugv_planar_pva>(kPva);
        if (!seen_.fresh(kPva, sample)) {
            return;
        }
        const xgc2_ugv_planar_pva& p = *sample;
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
        controller_.setWorldPva(reference);
        post(ugv::event_type::INPUT_REFERENCE_UPDATED, reference.stamp.toSec(), "reference_pva");
    }

    // The coordinator's messages for the vehicle's session, in order (a brake and a new route can
    // arrive in the same step). The controller's executor drops what answers another session.
    void readGrants() {
        for (;;) {
            const auto sample = host_.next<xgc2_ugv_reset_grant>(kResetGrant);
            if (!sample) {
                return;
            }
            const xgc2_ugv_reset_grant& p = *sample;
            if (p.state > XGC2_UGV_RESET_GRANT_REJECT || p.via_count > XGC2_UGV_MAX_RESET_VIA) {
                host_.log(kLogWarn, "Ignoring a malformed reset grant");
                continue;
            }
            rc::Grant grant;
            grant.session = p.session;
            grant.state = static_cast<rc::GrantState>(p.state);
            grant.grant_id = p.grant_id;
            grant.start = {p.start_x, p.start_y, p.start_yaw};
            grant.goal = {p.goal_x, p.goal_y, p.goal_yaw};
            grant.final_move = p.final_move != 0U;
            grant.ell_plan = p.ell_plan;
            grant.s_limit = p.s_limit;
            grant.ttl = p.ttl;
            grant.reason = static_cast<rc::Reason>(p.reason);
            grant.detail.assign(p.detail, strnlen(p.detail, sizeof(p.detail)));
            for (uint32_t i = 0; i < p.via_count; ++i) {
                grant.via.push_back({p.via[i][0], p.via[i][1]});
            }
            controller_.onResetGrant(grant);
        }
    }

    void readCommands(int64_t now_ns, double now) {
        for (;;) {
            const auto sample = host_.next<xgc2_ugv_command>(kCommand);
            if (!sample) {
                return;
            }
            uint32_t id = 0;
            switch (sample->kind) {
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
                    warn(kCommand, now, format("Unknown command kind %u", sample->kind));
                    continue;
            }
            post(id, secondsOf<ugv::Time>(sample->stamp_ns > 0 ? sample->stamp_ns : now_ns),
                 sample->source == XGC2_UGV_COMMAND_SOURCE_PUBLIC ? "/command" : "command");
        }
    }

    // ---- outputs --------------------------------------------------------------------------------

    void dispatchOutputs(const std::vector<sm::Event>& events, double now) {
        for (const sm::Event& event : events) {
            if (event.id == ugv::output_event_type::REQUEST_NMPC_SOLVE) {
                if (!execution_ || !execution_->handle(event)) {
                    warn(kCmdVel, now, "An NMPC solve was requested but no solver runs");
                }
            } else if (event.id == ugv::output_event_type::PUBLISH_CMD_VEL) {
                const ugv::CmdVel twist = controller_.cmdVel();
                pending_ = {true, XGC2_UGV_CMD_VEL_COMMAND, twist.linear_x, twist.angular_z};
            } else if (event.id == ugv::output_event_type::PUBLISH_ZERO_CMD_VEL) {
                pending_ = {true, XGC2_UGV_CMD_VEL_ZERO, 0.0, 0.0};
            } else {
                warn(kStatus, now,
                     format("Unhandled output event id: %u", static_cast<unsigned>(event.id)));
            }
        }
    }

    void publishCmdVel(int64_t now_ns) {
        if (!pending_.valid) {
            return;
        }
        const Pending twist = pending_;
        pending_ = {};
        auto slot = host_.write<xgc2_ugv_cmd_vel>(kCmdVel);
        if (!slot) {
            return;
        }
        slot->stamp_ns = now_ns;
        slot->linear_x = twist.linear_x;
        slot->linear_y = 0.0;
        slot->angular_z = twist.angular_z;
        slot->kind = twist.kind;
        slot.commit(now_ns);
    }

    // The vehicle's Session for the coordinator: the executor says when one is due (every 0.2 s and
    // on every change of phase or session).
    void publishResetSession(int64_t now_ns) {
        rc::Session session;
        if (!controller_.takeResetSession(&session)) {
            return;
        }
        auto slot = host_.write<xgc2_ugv_reset_session>(kResetSession);
        if (!slot) {
            return;
        }
        slot->stamp_ns = now_ns;
        slot->pose_stamp_ns = static_cast<int64_t>(std::llround(session.pose_stamp * 1.0e9));
        slot->pose_x = session.pose.x;
        slot->pose_y = session.pose.y;
        slot->pose_yaw = session.pose.yaw;
        slot->target_x = session.target.x;
        slot->target_y = session.target.y;
        slot->target_yaw = session.target.yaw;
        slot->progress = session.progress;
        slot->cross_track = session.cross_track;
        slot->half_length = session.spec.footprint.half_length;
        slot->half_width = session.spec.footprint.half_width;
        slot->offset_x = session.spec.footprint.offset_x;
        slot->offset_y = session.spec.footprint.offset_y;
        slot->max_v = session.spec.max_v;
        slot->max_vy = session.spec.max_vy;
        slot->max_w = session.spec.max_w;
        slot->acc_v = session.spec.acc_v;
        slot->acc_w = session.spec.acc_w;
        slot->ell_hat = session.ell_hat;
        slot->ell_spread = session.ell_spread;
        slot->ell_bound = session.spec.ell_bound;
        slot->session = session.session;
        slot->phase = static_cast<uint32_t>(session.phase);
        slot->model = static_cast<uint32_t>(session.spec.model);
        slot->healthy = session.healthy ? 1U : 0U;
        slot.commit(now_ns);
    }

    void publishResetResult(int64_t now_ns) {
        rc::Result result;
        if (!controller_.resetExecutor().resultPending()) {
            return;
        }
        auto slot = host_.write<xgc2_ugv_reset_result>(kResetResult);
        if (!slot) {
            return;  // the queue is full: the result stays pending for the next step
        }
        controller_.takeResetResult(&result);
        slot->stamp_ns = now_ns;
        slot->final_planar_error = result.final_planar_error;
        slot->final_yaw_error = result.final_yaw_error;
        slot->duration = result.duration;
        slot->session = result.session;
        slot->outcome = static_cast<uint32_t>(result.outcome);
        slot->reason = static_cast<uint32_t>(result.reason);
        std::strncpy(slot->detail, result.detail.c_str(), sizeof(slot->detail) - 1U);
        slot.commit(now_ns);
    }

    void publishStatusIfDue(int64_t now_ns) {
        if (now_ns <= 0) {
            return;
        }
        const auto period_ns = static_cast<int64_t>(1.0e9 / status_publish_rate_hz_);
        if (last_status_ns_ != 0 && now_ns - last_status_ns_ < period_ns) {
            return;
        }
        auto slot = host_.write<xgc2_ugv_controller_status>(kStatus);
        if (!slot) {
            return;
        }
        last_status_ns_ = now_ns;
        std::string name = controller_.stateMachine().currentStateName(ugv::region_type::CONTROL);
        if (name.empty()) {
            name = "Unknown";
        }
        slot->stamp_ns = now_ns;
        slot->control_state = controller_.stateMachine().currentState(ugv::region_type::CONTROL);
        slot->health_state = controller_.stateMachine().currentState(ugv::region_type::HEALTH);
        std::strncpy(slot->control_state_name, name.c_str(), sizeof(slot->control_state_name) - 1U);
        slot.commit(now_ns);
    }

    void logStateChanges() {
        const auto control = controller_.stateMachine().currentState(ugv::region_type::CONTROL);
        const auto health = controller_.stateMachine().currentState(ugv::region_type::HEALTH);
        if (control != last_control_state_) {
            host_.log(kLogInfo, "CONTROL state -> " + controller_.stateMachine().currentStateName(
                                                          ugv::region_type::CONTROL));
            last_control_state_ = control;
        }
        if (health != last_health_state_) {
            host_.log(kLogInfo, format("HEALTH state -> %u", static_cast<unsigned>(health)));
            last_health_state_ = health;
        }
    }

    struct Pending {
        bool valid{false};
        uint32_t kind{XGC2_UGV_CMD_VEL_ZERO};
        double linear_x{0.0};
        double angular_z{0.0};
    };

    Host host_;
    ThreadGuard thread_;
    ugv::UgvState state_;
    ugv::UnicycleUgvController controller_;
    std::unique_ptr<ugv::NmpcExecution> execution_;
    ugv::ControllerConfig config_;
    double status_publish_rate_hz_{5.0};
    bool started_{false};
    bool reset_target_from_port_{false};
    SeenSamples seen_;
    LogThrottle throttle_[kPortCount];
    Pending pending_;
    std::string last_reset_miss_;
    sm::StateId last_control_state_{0U};
    sm::StateId last_health_state_{0U};
    int64_t last_status_ns_{0};
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
