#include "unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>

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

Eigen::Vector2d pointToVector(const geometry_msgs::Point& point) {
    return Eigen::Vector2d(point.x, point.y);
}

Eigen::Vector2d vectorToEigen(const geometry_msgs::Vector3& value) {
    return Eigen::Vector2d(value.x, value.y);
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
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

double paramAt(const unicycle_reference_trajectory_msgs::AnalyticReference& msg, size_t index,
               double fallback) {
    return msg.params.size() > index && std::isfinite(msg.params[index]) ? msg.params[index]
                                                                         : fallback;
}

trajectory::WaypointConstraintType2 constraintType(uint8_t value) {
    switch (value) {
        case unicycle_reference_trajectory_msgs::WaypointReferenceRequest::CONSTRAINT_SPHERE:
            return trajectory::WaypointConstraintType2::kSphere;
        case unicycle_reference_trajectory_msgs::WaypointReferenceRequest::CONSTRAINT_BOX:
            return trajectory::WaypointConstraintType2::kBox;
        case unicycle_reference_trajectory_msgs::WaypointReferenceRequest::CONSTRAINT_GATE:
            return trajectory::WaypointConstraintType2::kGate;
        case unicycle_reference_trajectory_msgs::WaypointReferenceRequest::CONSTRAINT_POINT:
        default:
            return trajectory::WaypointConstraintType2::kPoint;
    }
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
    state_ = unicycle_reference_trajectory_msgs::ReferenceStatus::STATE_SELF_CHECK;
    current_time_sec_ = 0.0;
    flags_ = 0U;
    pending_kind_ = PendingKind::kNone;
    active_type_ = trajectory::TrajectoryModelType::kNone;
    active_trajectory_id_ = 0U;
    active_revision_ = 0U;
    active_start_sec_ = 0.0;
    active_duration_ = 0.0;
    active_evaluator_.reset();
    active_analytic_ = unicycle_reference_trajectory_msgs::AnalyticReference{};
    active_sampled_ = unicycle_reference_trajectory_msgs::SampledReference{};
    active_polynomial_ = unicycle_reference_trajectory_msgs::ActivePolynomialReference{};
    setupMachine();
}

sm::Status ReferenceTrajectoryRuntime::postEvent(sm::Event event) {
    event.category = sm::EventCategory::kInput;
    return machine_->postEvent(std::move(event));
}

void ReferenceTrajectoryRuntime::update(double now_sec) {
    current_time_sec_ = now_sec;
    const auto transition_result = machine_->update({64, 64, false});
    const auto tick_result =
        transition_result.status.ok() ? machine_->update({64, 64, true}) : transition_result;
    if (!tick_result.status.ok()) {
        flags_ |= trajectory::kFlagInvalidInput;
        state_ = unicycle_reference_trajectory_msgs::ReferenceStatus::STATE_SELF_CHECK;
    }
}

bool ReferenceTrajectoryRuntime::acceptAnalytic(
    const unicycle_reference_trajectory_msgs::AnalyticReference& msg) {
    uint32_t flags = 0U;
    if (!buildAnalyticEvaluator(msg, flags)) {
        flags_ |= flags;
        return false;
    }
    pending_analytic_ = msg;
    pending_analytic_.start_time = ros::Time(
        adjustedStartTime(msg.start_time.toSec(), current_time_sec_, config_.min_lead_time));
    pending_kind_ = PendingKind::kAnalytic;
    return true;
}

bool ReferenceTrajectoryRuntime::acceptSampled(
    const unicycle_reference_trajectory_msgs::SampledReference& msg) {
    trajectory::SampledEvaluator2 evaluator;
    uint32_t flags = 0U;
    if (!buildSampledEvaluator(msg, evaluator, flags)) {
        flags_ |= flags;
        return false;
    }
    pending_sampled_ = msg;
    pending_sampled_.start_time = ros::Time(
        adjustedStartTime(msg.start_time.toSec(), current_time_sec_, config_.min_lead_time));
    pending_kind_ = PendingKind::kSampled;
    return true;
}

bool ReferenceTrajectoryRuntime::acceptWaypoint(
    const unicycle_reference_trajectory_msgs::WaypointReferenceRequest& msg) {
    trajectory::WaypointProblem2 problem;
    uint32_t flags = 0U;
    if (!buildWaypointProblem(msg, problem, flags)) {
        flags_ |= flags;
        return false;
    }
    pending_waypoint_ = msg;
    pending_kind_ = PendingKind::kWaypoint;
    return true;
}

bool ReferenceTrajectoryRuntime::activatePending() {
    if (pending_kind_ == PendingKind::kNone) {
        return active_evaluator_ != nullptr;
    }
    if (pending_kind_ == PendingKind::kAnalytic) {
        uint32_t flags = 0U;
        auto evaluator = buildAnalyticEvaluator(pending_analytic_, flags);
        if (!evaluator) {
            flags_ |= flags;
            pending_kind_ = PendingKind::kNone;
            return false;
        }
        setActiveAnalytic(pending_analytic_, std::move(evaluator), flags);
        pending_kind_ = PendingKind::kNone;
        return true;
    }
    if (pending_kind_ == PendingKind::kSampled) {
        auto evaluator = std::make_unique<trajectory::SampledEvaluator2>();
        uint32_t flags = 0U;
        if (!buildSampledEvaluator(pending_sampled_, *evaluator, flags)) {
            flags_ |= flags;
            pending_kind_ = PendingKind::kNone;
            return false;
        }
        setActiveSampled(pending_sampled_, std::move(evaluator), flags);
        pending_kind_ = PendingKind::kNone;
        return true;
    }
    return false;
}

bool ReferenceTrajectoryRuntime::planPendingWaypoint() {
    if (pending_kind_ != PendingKind::kWaypoint) {
        flags_ |= trajectory::kFlagInvalidInput;
        return false;
    }
    trajectory::WaypointProblem2 problem;
    uint32_t flags = 0U;
    if (!buildWaypointProblem(pending_waypoint_, problem, flags)) {
        flags_ |= flags;
        pending_kind_ = PendingKind::kNone;
        return false;
    }

    auto evaluator = std::make_unique<trajectory::PiecewisePolynomialEvaluator2>();
    trajectory::MincoWaypointSolver2 solver;
    if (!solver.solve(problem, *evaluator, &flags)) {
        flags_ |= flags;
        pending_kind_ = PendingKind::kNone;
        return false;
    }

    unicycle_reference_trajectory_msgs::ActivePolynomialReference msg;
    msg.header = pending_waypoint_.header;
    msg.header.stamp = ros::Time(current_time_sec_);
    msg.trajectory_id = pending_waypoint_.trajectory_id;
    msg.revision = pending_waypoint_.revision;
    if (msg.revision == 0U) {
        msg.revision = active_revision_ + 1U;
    }
    msg.flags = flags | pending_waypoint_.flags;
    msg.start_time = ros::Time(adjustedStartTime(pending_waypoint_.header.stamp.toSec(),
                                                 current_time_sec_, config_.min_lead_time));
    msg.duration = evaluator->duration();
    msg.order = evaluator->order();
    for (const auto& segment : evaluator->segments()) {
        msg.segment_durations.push_back(segment.duration);
        appendCoefficients(segment.x, msg.coeff_x);
        appendCoefficients(segment.y, msg.coeff_y);
        appendCoefficients(segment.yaw, msg.coeff_yaw);
    }
    setActivePolynomial(std::move(msg), std::move(evaluator), flags);
    pending_kind_ = PendingKind::kNone;
    return true;
}

bool ReferenceTrajectoryRuntime::activeExpired(double now_sec) const {
    if (!active_evaluator_) {
        return true;
    }
    if (active_duration_ <= 0.0) {
        return false;
    }
    return now_sec > active_start_sec_ + active_duration_ + config_.trajectory_timeout;
}

void ReferenceTrajectoryRuntime::enterState(uint8_t state) {
    state_ = state;
}

unicycle_reference_trajectory_msgs::ReferenceStatus ReferenceTrajectoryRuntime::makeStatus(
    double stamp_sec) const {
    unicycle_reference_trajectory_msgs::ReferenceStatus status;
    status.header.stamp = ros::Time(stamp_sec);
    status.state = state_;
    status.flags = flags_;
    status.active_trajectory_id = active_trajectory_id_;
    status.active_revision = active_revision_;
    status.active_type = unicycle_reference_trajectory_msgs::ReferenceStatus::TYPE_NONE;
    if (active_type_ == trajectory::TrajectoryModelType::kAnalytic) {
        status.active_type = unicycle_reference_trajectory_msgs::ReferenceStatus::TYPE_ANALYTIC;
    } else if (active_type_ == trajectory::TrajectoryModelType::kPolynomial) {
        status.active_type = unicycle_reference_trajectory_msgs::ReferenceStatus::TYPE_POLYNOMIAL;
    } else if (active_type_ == trajectory::TrajectoryModelType::kSampled) {
        status.active_type = unicycle_reference_trajectory_msgs::ReferenceStatus::TYPE_SAMPLED;
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
ReferenceTrajectoryRuntime::buildAnalyticEvaluator(
    const unicycle_reference_trajectory_msgs::AnalyticReference& msg, uint32_t& flags) const {
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
        case unicycle_reference_trajectory_msgs::AnalyticReference::ANALYTIC_HOLD: {
            trajectory::HoldCurveParameters2 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.position = origin;
            params.yaw = origin_yaw;
            evaluator = std::make_unique<trajectory::HoldCurveEvaluator2>(params);
            break;
        }
        case unicycle_reference_trajectory_msgs::AnalyticReference::ANALYTIC_CIRCLE: {
            trajectory::CircleCurveParameters2 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.center = center;
            params.radius = radius;
            params.line_speed = line_speed;
            evaluator = std::make_unique<trajectory::CircleCurveEvaluator2>(params);
            break;
        }
        case unicycle_reference_trajectory_msgs::AnalyticReference::ANALYTIC_FIGURE_EIGHT: {
            trajectory::FigureEightCurveParameters2 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.center = center;
            params.radius = radius;
            params.line_speed = line_speed;
            evaluator = std::make_unique<trajectory::FigureEightCurveEvaluator2>(params);
            break;
        }
        case unicycle_reference_trajectory_msgs::AnalyticReference::ANALYTIC_CIRCLE_ENTRY:
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

bool ReferenceTrajectoryRuntime::buildSampledEvaluator(
    const unicycle_reference_trajectory_msgs::SampledReference& msg,
    trajectory::SampledEvaluator2& evaluator, uint32_t& flags) const {
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
        (msg.flags &
         unicycle_reference_trajectory_msgs::SampledReference::FLAG_EXPLICIT_PLANAR_KINEMATICS) !=
        0U;
    if (!evaluator.setSamples(std::move(samples), preserve_explicit_planar_kinematics)) {
        flags |= trajectory::kFlagInvalidInput;
        return false;
    }
    flags |= trajectory::TrajectoryValidator2::validate(evaluator, config_.limits,
                                                        config_.validation_sample_dt);
    return (flags & (trajectory::kFlagInvalidInput | trajectory::kFlagNonFinite)) == 0U;
}

bool ReferenceTrajectoryRuntime::buildWaypointProblem(
    const unicycle_reference_trajectory_msgs::WaypointReferenceRequest& msg,
    trajectory::WaypointProblem2& problem, uint32_t& flags) const {
    flags = msg.flags;
    problem.flags = msg.flags;
    problem.segment_times = msg.segment_times;
    problem.start_velocity = vectorToEigen(msg.start_velocity);
    problem.start_acceleration = vectorToEigen(msg.start_acceleration);
    problem.end_velocity = vectorToEigen(msg.end_velocity);
    problem.end_acceleration = vectorToEigen(msg.end_acceleration);
    problem.desired_speed = msg.desired_speed > 0.0 ? msg.desired_speed : 1.0;
    problem.time_weight = msg.time_weight > 0.0 ? msg.time_weight : 1.0;
    problem.max_iterations = msg.max_iterations > 0U ? static_cast<int>(msg.max_iterations) : 80;
    problem.rel_cost_tol = msg.rel_cost_tol > 0.0 ? msg.rel_cost_tol : 1.0e-5;
    problem.dynamic_penalty_weight = 1000.0;
    problem.limits.max_velocity = msg.max_velocity;
    problem.limits.max_acceleration =
        msg.max_linear_acceleration > 0.0 ? msg.max_linear_acceleration : msg.max_acceleration;
    problem.limits.max_yaw_rate = msg.max_yaw_rate;
    problem.validation_sample_dt = config_.validation_sample_dt;
    if ((!msg.constraint_types.empty() && msg.constraint_types.size() != msg.waypoints.size()) ||
        (!msg.region_size.empty() && msg.region_size.size() != msg.waypoints.size())) {
        flags |= trajectory::kFlagInvalidInput;
        return false;
    }
    problem.constraints.reserve(msg.waypoints.size());
    for (size_t i = 0; i < msg.waypoints.size(); ++i) {
        trajectory::WaypointConstraint2 constraint;
        constraint.position = pointToVector(msg.waypoints[i].position);
        constraint.yaw = yawFromQuaternion(msg.waypoints[i].orientation);
        if (!problem.constraints.empty()) {
            constraint.yaw = unwrapYaw(constraint.yaw, problem.constraints.back().yaw);
        }
        constraint.type = msg.constraint_types.empty() ? trajectory::WaypointConstraintType2::kPoint
                                                       : constraintType(msg.constraint_types[i]);
        if (!msg.region_size.empty()) {
            constraint.size = vectorToEigen(msg.region_size[i]);
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
    const unicycle_reference_trajectory_msgs::AnalyticReference& msg,
    std::unique_ptr<trajectory::TrajectoryEvaluator2> evaluator, uint32_t flags) {
    active_type_ = trajectory::TrajectoryModelType::kAnalytic;
    active_trajectory_id_ = msg.trajectory_id;
    active_revision_ = msg.revision;
    active_start_sec_ = msg.start_time.toSec();
    active_duration_ = msg.duration;
    active_evaluator_ = std::move(evaluator);
    active_analytic_ = msg;
    flags_ = flags;
}

void ReferenceTrajectoryRuntime::setActiveSampled(
    const unicycle_reference_trajectory_msgs::SampledReference& msg,
    std::unique_ptr<trajectory::TrajectoryEvaluator2> evaluator, uint32_t flags) {
    active_type_ = trajectory::TrajectoryModelType::kSampled;
    active_trajectory_id_ = msg.trajectory_id;
    active_revision_ = msg.revision;
    active_start_sec_ = msg.start_time.toSec();
    active_duration_ = evaluator ? evaluator->duration() : 0.0;
    active_evaluator_ = std::move(evaluator);
    active_sampled_ = msg;
    flags_ = flags;
}

void ReferenceTrajectoryRuntime::setActivePolynomial(
    unicycle_reference_trajectory_msgs::ActivePolynomialReference msg,
    std::unique_ptr<trajectory::TrajectoryEvaluator2> evaluator, uint32_t flags) {
    active_type_ = trajectory::TrajectoryModelType::kPolynomial;
    active_trajectory_id_ = msg.trajectory_id;
    active_revision_ = msg.revision;
    active_start_sec_ = msg.start_time.toSec();
    active_duration_ = msg.duration;
    active_evaluator_ = std::move(evaluator);
    active_polynomial_ = std::move(msg);
    flags_ = flags;
}

}  // namespace unicycle_reference_trajectory
