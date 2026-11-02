#include "unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "unicycle_reference_trajectory/state_machine/active_state.h"
#include "unicycle_reference_trajectory/state_machine/planning_state.h"
#include "unicycle_reference_trajectory/state_machine/ready_state.h"
#include "unicycle_reference_trajectory/state_machine/self_check_state.h"

namespace unicycle_reference_trajectory {
namespace {

namespace sm = ::state_machine;

void requireOk(const sm::Status& status, const char* operation) {
    if (!status.ok()) {
        throw std::runtime_error(std::string(operation) + ": " + status.message);
    }
}

double finiteOr(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
}

Eigen::Vector2d pointToVector(const reference::Point& point) {
    return Eigen::Vector2d(point.x, point.y);
}

Eigen::Vector2d vectorToEigen(const reference::Vector3& value) {
    return Eigen::Vector2d(value.x, value.y);
}

double yawFromQuaternion(const reference::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return finiteOr(std::atan2(siny_cosp, cosy_cosp), 0.0);
}

double unwrapYaw(double yaw, double previous) {
    return previous + std::atan2(std::sin(yaw - previous), std::cos(yaw - previous));
}

double adjustedStartTime(double requested, double now, double min_lead_time) {
    const double minimum = now + std::max(0.0, min_lead_time);
    if (!std::isfinite(requested) || requested <= 0.0) {
        return minimum;
    }
    return std::max(requested, minimum);
}

double paramAt(const reference::AnalyticReference& msg, size_t index, double fallback) {
    return msg.params.size() > index && std::isfinite(msg.params[index]) ? msg.params[index]
                                                                         : fallback;
}

void appendCoefficients(const std::vector<double>& input, std::vector<double>& output) {
    output.insert(output.end(), input.begin(), input.end());
}

}  // namespace

ReferenceTrajectoryRuntime::ReferenceTrajectoryRuntime() {
    reset();
}

void ReferenceTrajectoryRuntime::setConfig(const ReferenceTrajectoryConfig& config) {
    config_ = config;
    if (!std::isfinite(config_.status_rate_hz) || config_.status_rate_hz <= 0.0) {
        config_.status_rate_hz = 10.0;
    }
    if (!std::isfinite(config_.active_publish_rate_hz) || config_.active_publish_rate_hz <= 0.0) {
        config_.active_publish_rate_hz = 10.0;
    }
    if (!std::isfinite(config_.validation_sample_dt) || config_.validation_sample_dt <= 0.0) {
        config_.validation_sample_dt = 0.02;
    }
    if (!std::isfinite(config_.trajectory_timeout) || config_.trajectory_timeout < 0.0) {
        config_.trajectory_timeout = 0.5;
    }
    if (!std::isfinite(config_.min_lead_time) || config_.min_lead_time < 0.0) {
        config_.min_lead_time = 0.2;
    }
    reset();
}

void ReferenceTrajectoryRuntime::reset() {
    state_ = reference::ReferenceStatus::STATE_SELF_CHECK;
    current_time_sec_ = 0.0;
    flags_ = 0U;
    pending_spec_ = std::monostate{};
    active_spec_ = std::monostate{};
    setupMachine();
}

sm::Status ReferenceTrajectoryRuntime::postEvent(sm::Event event) {
    event.category = sm::EventCategory::kInput;
    return machine_->postEvent(std::move(event));
}

void ReferenceTrajectoryRuntime::update(double now_sec) {
    current_time_sec_ = now_sec;
    sm::Result<sm::UpdateResult> tick_result;
    if (machine_->snapshot().inbox_size > 0) {
        const auto transition_result = machine_->update({64, 64, false});
        tick_result = transition_result.status.ok() ? machine_->update({64, 64, true})
                                                    : transition_result;
    } else {
        tick_result = machine_->update({64, 64, true});
    }
    if (!tick_result.status.ok()) {
        flags_ |= trajectory::kFlagInvalidInput;
        state_ = reference::ReferenceStatus::STATE_SELF_CHECK;
    }
}

bool ReferenceTrajectoryRuntime::acceptAnalytic(const reference::AnalyticReference& msg) {
    uint32_t flags = 0U;
    if (!buildAnalyticEvaluator(msg, flags)) {
        flags_ |= flags;
        return false;
    }
    reference::AnalyticReference adjusted = msg;
    adjusted.start_time =
        Time(adjustedStartTime(msg.start_time.toSec(), current_time_sec_, config_.min_lead_time));
    pending_spec_ = std::move(adjusted);
    return true;
}

bool ReferenceTrajectoryRuntime::acceptSampled(const reference::SampledReference& msg) {
    trajectory::SampledEvaluator2 evaluator;
    uint32_t flags = 0U;
    if (!buildSampledEvaluator(msg, evaluator, flags)) {
        flags_ |= flags;
        return false;
    }
    reference::SampledReference adjusted = msg;
    adjusted.start_time =
        Time(adjustedStartTime(msg.start_time.toSec(), current_time_sec_, config_.min_lead_time));
    pending_spec_ = std::move(adjusted);
    return true;
}

bool ReferenceTrajectoryRuntime::acceptWaypoint(const reference::WaypointReferenceRequest& msg) {
    trajectory::WaypointProblem2 problem;
    uint32_t flags = 0U;
    if (!buildWaypointProblem(msg, problem, flags)) {
        flags_ |= flags;
        return false;
    }
    pending_spec_ = msg;
    return true;
}

bool ReferenceTrajectoryRuntime::activatePending() {
    return std::visit(
        sm::adt::Overloaded{
            [this](std::monostate) -> bool { return evaluator() != nullptr; },
            [this](const reference::AnalyticReference& pending_analytic) -> bool {
                uint32_t flags = 0U;
                auto eval = buildAnalyticEvaluator(pending_analytic, flags);
                if (!eval) {
                    flags_ |= flags;
                    pending_spec_ = std::monostate{};
                    return false;
                }
                const auto msg_copy = pending_analytic;
                pending_spec_ = std::monostate{};
                setActiveAnalytic(msg_copy, std::move(eval), flags);
                return true;
            },
            [this](const reference::SampledReference& pending_sampled) -> bool {
                auto eval = std::make_unique<trajectory::SampledEvaluator2>();
                uint32_t flags = 0U;
                if (!buildSampledEvaluator(pending_sampled, *eval, flags)) {
                    flags_ |= flags;
                    pending_spec_ = std::monostate{};
                    return false;
                }
                const auto msg_copy = pending_sampled;
                pending_spec_ = std::monostate{};
                setActiveSampled(msg_copy, std::move(eval), flags);
                return true;
            },
            [](const reference::WaypointReferenceRequest&) -> bool { return false; }},
        pending_spec_);
}

bool ReferenceTrajectoryRuntime::planPendingWaypoint() {
    const auto* pending_waypoint =
        std::get_if<reference::WaypointReferenceRequest>(&pending_spec_);
    if (pending_waypoint == nullptr) {
        flags_ |= trajectory::kFlagInvalidInput;
        return false;
    }
    const reference::WaypointReferenceRequest waypoint_copy = *pending_waypoint;
    trajectory::WaypointProblem2 problem;
    uint32_t flags = 0U;
    if (!buildWaypointProblem(waypoint_copy, problem, flags)) {
        flags_ |= flags;
        pending_spec_ = std::monostate{};
        return false;
    }

    auto evaluator = std::make_unique<trajectory::PiecewisePolynomialEvaluator2>();
    trajectory::SepticWaypointInterpolator2 solver;
    if (!solver.solve(problem, *evaluator, &flags)) {
        flags_ |= flags;
        pending_spec_ = std::monostate{};
        return false;
    }

    reference::ActivePolynomialReference msg;
    msg.header = waypoint_copy.header;
    msg.header.stamp = Time(current_time_sec_);
    msg.trajectory_id = waypoint_copy.trajectory_id;
    msg.revision = waypoint_copy.revision;
    if (msg.revision == 0U) {
        msg.revision = activeRevision() + 1U;
    }
    msg.flags = flags | waypoint_copy.flags;
    msg.start_time = Time(adjustedStartTime(waypoint_copy.header.stamp.toSec(), current_time_sec_,
                                            config_.min_lead_time));
    msg.duration = evaluator->duration();
    msg.order = evaluator->order();
    for (const auto& segment : evaluator->segments()) {
        msg.segment_durations.push_back(segment.duration);
        appendCoefficients(segment.x, msg.coeff_x);
        appendCoefficients(segment.y, msg.coeff_y);
        appendCoefficients(segment.yaw, msg.coeff_yaw);
    }
    pending_spec_ = std::monostate{};
    setActivePolynomial(std::move(msg), std::move(evaluator), flags);
    return true;
}

bool ReferenceTrajectoryRuntime::activeExpired(double now_sec) const {
    return std::visit(
        sm::adt::Overloaded{
            [](std::monostate) -> bool { return true; },
            [this, now_sec](const auto& spec) -> bool {
                if (!spec.evaluator) {
                    return true;
                }
                if (spec.duration <= 0.0) {
                    return false;
                }
                return now_sec > spec.start_sec + spec.duration + config_.trajectory_timeout;
            }},
        active_spec_);
}

trajectory::TrajectoryModelType ReferenceTrajectoryRuntime::activeType() const {
    return std::visit(
        sm::adt::Overloaded{
            [](std::monostate) { return trajectory::TrajectoryModelType::kNone; },
            [](const ActiveAnalyticSpec&) { return trajectory::TrajectoryModelType::kAnalytic; },
            [](const ActiveSampledSpec&) { return trajectory::TrajectoryModelType::kSampled; },
            [](const ActivePolynomialSpec&) {
                return trajectory::TrajectoryModelType::kPolynomial;
            }},
        active_spec_);
}

uint32_t ReferenceTrajectoryRuntime::activeTrajectoryId() const {
    return std::visit(
        sm::adt::Overloaded{
            [](std::monostate) -> uint32_t { return 0U; },
            [](const auto& spec) -> uint32_t { return spec.message.trajectory_id; }},
        active_spec_);
}

uint32_t ReferenceTrajectoryRuntime::activeRevision() const {
    return std::visit(
        sm::adt::Overloaded{
            [](std::monostate) -> uint32_t { return 0U; },
            [](const auto& spec) -> uint32_t { return spec.message.revision; }},
        active_spec_);
}

const reference::AnalyticReference& ReferenceTrajectoryRuntime::activeAnalyticMessage() const {
    static const reference::AnalyticReference kEmpty{};
    if (const auto* spec = std::get_if<ActiveAnalyticSpec>(&active_spec_)) {
        return spec->message;
    }
    return kEmpty;
}

const reference::SampledReference& ReferenceTrajectoryRuntime::activeSampledMessage() const {
    static const reference::SampledReference kEmpty{};
    if (const auto* spec = std::get_if<ActiveSampledSpec>(&active_spec_)) {
        return spec->message;
    }
    return kEmpty;
}

const reference::ActivePolynomialReference& ReferenceTrajectoryRuntime::activePolynomialMessage()
    const {
    static const reference::ActivePolynomialReference kEmpty{};
    if (const auto* spec = std::get_if<ActivePolynomialSpec>(&active_spec_)) {
        return spec->message;
    }
    return kEmpty;
}

const trajectory::TrajectoryEvaluator2* ReferenceTrajectoryRuntime::evaluator() const {
    return std::visit(
        sm::adt::Overloaded{
            [](std::monostate) -> const trajectory::TrajectoryEvaluator2* { return nullptr; },
            [](const auto& spec) -> const trajectory::TrajectoryEvaluator2* {
                return spec.evaluator.get();
            }},
        active_spec_);
}

sm::adt::TickPolicy ReferenceTrajectoryRuntime::tickPolicy() const {
    if (state_ == reference::ReferenceStatus::STATE_ACTIVE) {
        const double rate = std::max(config_.status_rate_hz, config_.active_publish_rate_hz);
        return sm::adt::TickPeriodic{1.0 / std::max(1.0, rate)};
    }
    return sm::adt::TickPeriodic{1.0 / std::max(1.0, config_.status_rate_hz)};
}

void ReferenceTrajectoryRuntime::enterState(uint8_t state) {
    state_ = state;
}

reference::ReferenceStatus ReferenceTrajectoryRuntime::makeStatus(double stamp_sec) const {
    reference::ReferenceStatus status;
    status.header.stamp = Time(stamp_sec);
    status.state = state_;
    status.flags = flags_;
    status.active_trajectory_id = activeTrajectoryId();
    status.active_revision = activeRevision();
    status.active_type = reference::ReferenceStatus::TYPE_NONE;
    const auto type = activeType();
    if (type == trajectory::TrajectoryModelType::kAnalytic) {
        status.active_type = reference::ReferenceStatus::TYPE_ANALYTIC;
    } else if (type == trajectory::TrajectoryModelType::kPolynomial) {
        status.active_type = reference::ReferenceStatus::TYPE_POLYNOMIAL;
    } else if (type == trajectory::TrajectoryModelType::kSampled) {
        status.active_type = reference::ReferenceStatus::TYPE_SAMPLED;
    }
    return status;
}

void ReferenceTrajectoryRuntime::setupMachine() {
    auto builder = sm::StateMachine::builder("ReferenceTrajectoryStateMachine");
    builder.region(region_type::REFERENCE)
        .name("reference")
        .order(0)
        .initial(state_type::SelfCheck)
        .state(state_type::SelfCheck)
        .name("SelfCheck")
        .impl(std::make_unique<SelfCheckState>(*this))
        .state(state_type::Ready)
        .name("Ready")
        .impl(std::make_unique<ReadyState>(*this))
        .state(state_type::Planning)
        .name("Planning")
        .impl(std::make_unique<PlanningState>(*this))
        .state(state_type::Active)
        .name("Active")
        .impl(std::make_unique<ActiveState>(*this))
        .endRegion();

    builder.transition()
        .from(state_type::SelfCheck)
        .to(state_type::Ready)
        .on(event_type::CONFIG_READY)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Ready)
        .to(state_type::Active)
        .on(event_type::ANALYTIC_RECEIVED)
        .priority(transition_priority::REQUEST);
    builder.transition()
        .from(state_type::Ready)
        .to(state_type::Active)
        .on(event_type::SAMPLED_RECEIVED)
        .priority(transition_priority::REQUEST);
    builder.transition()
        .from(state_type::Ready)
        .to(state_type::Planning)
        .on(event_type::WAYPOINT_RECEIVED)
        .priority(transition_priority::REQUEST);
    builder.transition()
        .from(state_type::Active)
        .to(state_type::Active)
        .on(event_type::ANALYTIC_RECEIVED)
        .priority(transition_priority::REQUEST);
    builder.transition()
        .from(state_type::Active)
        .to(state_type::Active)
        .on(event_type::SAMPLED_RECEIVED)
        .priority(transition_priority::REQUEST);
    builder.transition()
        .from(state_type::Active)
        .to(state_type::Planning)
        .on(event_type::WAYPOINT_RECEIVED)
        .priority(transition_priority::REQUEST);
    builder.transition()
        .from(state_type::Planning)
        .to(state_type::Active)
        .on(event_type::PLAN_SUCCEEDED)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Planning)
        .to(state_type::SelfCheck)
        .on(event_type::PLAN_FAILED)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Active)
        .to(state_type::SelfCheck)
        .on(event_type::PLAN_FAILED)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Active)
        .to(state_type::Ready)
        .on(event_type::TRAJECTORY_EXPIRED)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Ready)
        .to(state_type::SelfCheck)
        .on(event_type::RESET_REQUESTED)
        .priority(transition_priority::REQUEST);
    builder.transition()
        .from(state_type::Active)
        .to(state_type::SelfCheck)
        .on(event_type::RESET_REQUESTED)
        .priority(transition_priority::REQUEST);
    auto machine_result = builder.build();
    requireOk(machine_result.status, "build reference trajectory state machine");
    machine_ = std::move(machine_result.value);
    requireOk(machine_->start(), "start reference trajectory state machine");
}

std::unique_ptr<trajectory::TrajectoryEvaluator2>
ReferenceTrajectoryRuntime::buildAnalyticEvaluator(const reference::AnalyticReference& msg,
                                                   uint32_t& flags) const {
    flags = msg.flags;
    const double duration = msg.duration > 0.0 ? msg.duration : 60.0;
    const Eigen::Vector2d origin = pointToVector(msg.origin.position);
    const double origin_yaw = yawFromQuaternion(msg.origin.orientation);
    const double radius = paramAt(msg, 0U, 3.0);
    const double line_speed = paramAt(msg, 1U, 1.0);
    const double entry_duration = paramAt(msg, 2U, 3.0);
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    center.x() = paramAt(msg, 3U, center.x());
    center.y() = paramAt(msg, 4U, center.y());

    std::unique_ptr<trajectory::TrajectoryEvaluator2> evaluator;
    switch (msg.analytic_type) {
        case reference::AnalyticReference::ANALYTIC_HOLD: {
            trajectory::HoldCurveParameters2 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.position = origin;
            params.yaw = origin_yaw;
            evaluator = std::make_unique<trajectory::HoldCurveEvaluator2>(params);
            break;
        }
        case reference::AnalyticReference::ANALYTIC_CIRCLE: {
            trajectory::CircleCurveParameters2 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.center = center;
            params.radius = radius;
            params.line_speed = line_speed;
            evaluator = std::make_unique<trajectory::CircleCurveEvaluator2>(params);
            break;
        }
        case reference::AnalyticReference::ANALYTIC_FIGURE_EIGHT: {
            trajectory::FigureEightCurveParameters2 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.center = center;
            params.radius = radius;
            params.line_speed = line_speed;
            evaluator = std::make_unique<trajectory::FigureEightCurveEvaluator2>(params);
            break;
        }
        case reference::AnalyticReference::ANALYTIC_CIRCLE_ENTRY:
        default: {
            trajectory::CircleEntryCurveParameters2 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.origin = origin;
            params.origin_yaw = origin_yaw;
            params.entry_duration = entry_duration;
            params.circle.flags = msg.flags;
            params.circle.duration = std::max(0.0, duration - std::max(0.0, entry_duration));
            params.circle.center = center;
            params.circle.radius = radius;
            params.circle.line_speed = line_speed;
            evaluator = std::make_unique<trajectory::CircleEntryCurveEvaluator2>(params);
            break;
        }
    }

    if (!evaluator) {
        flags |= trajectory::kFlagInvalidInput;
        return nullptr;
    }
    flags |= trajectory::TrajectoryValidator2::validate(*evaluator, config_.limits,
                                                        config_.validation_sample_dt);
    if ((flags & (trajectory::kFlagInvalidInput | trajectory::kFlagNonFinite)) != 0U) {
        return nullptr;
    }
    return evaluator;
}

bool ReferenceTrajectoryRuntime::buildSampledEvaluator(const reference::SampledReference& msg,
                                                       trajectory::SampledEvaluator2& evaluator,
                                                       uint32_t& flags) const {
    flags = msg.flags;
    std::vector<trajectory::SampledPoint2> samples;
    samples.reserve(msg.points.size());
    for (const auto& point : msg.points) {
        trajectory::SampledPoint2 sample;
        sample.t = point.t_from_start;
        sample.reference.position = Eigen::Vector2d(point.x, point.y);
        sample.reference.velocity = Eigen::Vector2d(point.vx, point.vy);
        sample.reference.acceleration = Eigen::Vector2d(point.ax, point.ay);
        sample.reference.jerk = Eigen::Vector2d(point.jx, point.jy);
        sample.reference.yaw = point.yaw;
        sample.reference.speed = point.speed;
        sample.reference.linear_acceleration = point.linear_acceleration;
        sample.reference.yaw_rate = point.yaw_rate;
        sample.reference.yaw_acceleration = point.yaw_acceleration;
        sample.reference.curvature = point.curvature;
        samples.push_back(sample);
    }
    const bool preserve_explicit_planar_kinematics =
        (msg.flags & reference::SampledReference::FLAG_EXPLICIT_PLANAR_KINEMATICS) != 0U;
    if (!evaluator.setSamples(std::move(samples), preserve_explicit_planar_kinematics)) {
        flags |= trajectory::kFlagInvalidInput;
        return false;
    }
    flags |= trajectory::TrajectoryValidator2::validate(evaluator, config_.limits,
                                                        config_.validation_sample_dt);
    return (flags & (trajectory::kFlagInvalidInput | trajectory::kFlagNonFinite)) == 0U;
}

bool ReferenceTrajectoryRuntime::buildWaypointProblem(
    const reference::WaypointReferenceRequest& msg, trajectory::WaypointProblem2& problem,
    uint32_t& flags) const {
    flags = msg.flags;
    problem.flags = msg.flags;
    problem.segment_times = msg.segment_times;
    problem.start_velocity = vectorToEigen(msg.start_velocity);
    problem.start_acceleration = vectorToEigen(msg.start_acceleration);
    problem.end_velocity = vectorToEigen(msg.end_velocity);
    problem.end_acceleration = vectorToEigen(msg.end_acceleration);
    problem.desired_speed = msg.desired_speed > 0.0 ? msg.desired_speed : 1.0;
    problem.limits.max_velocity = msg.max_velocity;
    problem.limits.max_acceleration =
        msg.max_linear_acceleration > 0.0 ? msg.max_linear_acceleration : msg.max_acceleration;
    problem.limits.max_yaw_rate = msg.max_yaw_rate;
    problem.validation_sample_dt = config_.validation_sample_dt;
    problem.constraints.reserve(msg.waypoints.size());
    for (size_t i = 0; i < msg.waypoints.size(); ++i) {
        trajectory::WaypointConstraint2 constraint;
        constraint.position = pointToVector(msg.waypoints[i].position);
        constraint.yaw = yawFromQuaternion(msg.waypoints[i].orientation);
        if (!problem.constraints.empty()) {
            constraint.yaw = unwrapYaw(constraint.yaw, problem.constraints.back().yaw);
        }
        problem.constraints.push_back(std::move(constraint));
    }
    if (problem.constraints.size() < 2U ||
        (!problem.segment_times.empty() &&
         problem.segment_times.size() + 1U != problem.constraints.size())) {
        flags |= trajectory::kFlagInvalidInput;
        return false;
    }
    return true;
}

void ReferenceTrajectoryRuntime::setActiveAnalytic(
    const reference::AnalyticReference& msg,
    std::unique_ptr<trajectory::TrajectoryEvaluator2> evaluator, uint32_t flags) {
    active_spec_ =
        ActiveAnalyticSpec{msg, msg.start_time.toSec(), msg.duration, std::move(evaluator)};
    flags_ = flags;
}

void ReferenceTrajectoryRuntime::setActiveSampled(
    const reference::SampledReference& msg,
    std::unique_ptr<trajectory::TrajectoryEvaluator2> evaluator, uint32_t flags) {
    const double duration = evaluator ? evaluator->duration() : 0.0;
    active_spec_ =
        ActiveSampledSpec{msg, msg.start_time.toSec(), duration, std::move(evaluator)};
    flags_ = flags;
}

void ReferenceTrajectoryRuntime::setActivePolynomial(
    reference::ActivePolynomialReference msg,
    std::unique_ptr<trajectory::TrajectoryEvaluator2> evaluator, uint32_t flags) {
    const double start_sec = msg.start_time.toSec();
    const double duration = msg.duration;
    active_spec_ =
        ActivePolynomialSpec{std::move(msg), start_sec, duration, std::move(evaluator)};
    flags_ = flags;
}

}  // namespace unicycle_reference_trajectory
