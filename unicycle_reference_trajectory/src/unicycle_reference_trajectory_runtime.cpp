#include "unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>

namespace unicycle_reference_trajectory {
namespace {
namespace adt = ::state_machine::adt;
constexpr double never = std::numeric_limits<double>::infinity();
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

ReferenceTrajectoryRuntime::ReferenceTrajectoryRuntime() = default;

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
    machine_.apply(adt::Enter{SelfCheck{}}, 0.0);
    current_time_sec_ = 0.0;
    flags_ = 0U;
    request_.emplace<std::monostate>();
    retained_.emplace<std::monostate>();
    outputs_.clear();
}

void ReferenceTrajectoryRuntime::update(double now_sec) {
    current_time_sec_ = now_sec;
    outputs_.clear();
    machine_.apply(machine_.dispatch(Advance{}, *this), now_sec);
    // Planning is a synchronous, bounded trajectory calculation on this owner.
    // Its result is consumed here; there is no queued PLAN event or second tick.
    if (std::holds_alternative<Planning>(machine_.state())) {
        machine_.apply(machine_.dispatch(Advance{}, *this), now_sec);
    }
}

bool ReferenceTrajectoryRuntime::acceptAnalytic(const reference::AnalyticReference& msg,
                                                double receipt_time) {
    current_time_sec_ = receipt_time;
    uint32_t flags = 0U;
    auto evaluator = buildAnalyticEvaluator(msg, flags);
    if (!evaluator) {
        flags_ |= flags;
        return false;
    }
    Analytic value{msg, std::move(evaluator), flags};
    value.message.start_time =
        Time(adjustedStartTime(msg.start_time.toSec(), current_time_sec_, config_.min_lead_time));
    request_.emplace<Analytic>(std::move(value));
    return true;
}

bool ReferenceTrajectoryRuntime::acceptSampled(const reference::SampledReference& msg,
                                               double receipt_time) {
    current_time_sec_ = receipt_time;
    uint32_t flags = 0U;
    auto evaluator = std::make_unique<trajectory::SampledEvaluator2>();
    if (!buildSampledEvaluator(msg, *evaluator, flags)) {
        flags_ |= flags;
        return false;
    }
    Sampled value{msg, std::move(evaluator), flags};
    value.message.start_time =
        Time(adjustedStartTime(msg.start_time.toSec(), current_time_sec_, config_.min_lead_time));
    request_.emplace<Sampled>(std::move(value));
    return true;
}

bool ReferenceTrajectoryRuntime::acceptWaypoint(const reference::WaypointReferenceRequest& msg,
                                                double receipt_time) {
    current_time_sec_ = receipt_time;
    trajectory::WaypointProblem2 problem;
    uint32_t flags = 0U;
    if (!buildWaypointProblem(msg, problem, flags)) {
        flags_ |= flags;
        return false;
    }
    request_.emplace<Waypoint>(Waypoint{msg, std::move(problem), flags});
    return true;
}

bool ReferenceTrajectoryRuntime::activateRequest() {
    if (auto* value = std::get_if<Analytic>(&request_)) {
        flags_ = value->flags;
        retained_.emplace<Analytic>(std::move(*value));
        request_.emplace<std::monostate>();
    } else if (auto* value = std::get_if<Sampled>(&request_)) {
        flags_ = value->flags;
        retained_.emplace<Sampled>(std::move(*value));
        request_.emplace<std::monostate>();
    }
    return evaluator() != nullptr;
}

bool ReferenceTrajectoryRuntime::planWaypoint(Waypoint& request) {
    auto evaluator = std::make_unique<trajectory::PiecewisePolynomialEvaluator2>();
    trajectory::SepticWaypointInterpolator2 solver;
    uint32_t flags = request.flags;
    if (!solver.solve(request.problem, *evaluator, &flags)) {
        flags_ |= flags;
        return false;
    }
    reference::ActivePolynomialReference msg;
    msg.header = request.message.header;
    msg.header.stamp = Time(current_time_sec_);
    msg.trajectory_id = request.message.trajectory_id;
    msg.revision = request.message.revision;
    if (msg.revision == 0U) {
        msg.revision = activeRevision() + 1U;
    }
    msg.flags = flags | request.message.flags;
    msg.start_time = Time(adjustedStartTime(request.message.header.stamp.toSec(), current_time_sec_,
                                            config_.min_lead_time));
    msg.duration = evaluator->duration();
    msg.order = evaluator->order();
    for (const auto& segment : evaluator->segments()) {
        msg.segment_durations.push_back(segment.duration);
        appendCoefficients(segment.x, msg.coeff_x);
        appendCoefficients(segment.y, msg.coeff_y);
        appendCoefficients(segment.yaw, msg.coeff_yaw);
    }
    flags_ = flags;
    retained_.emplace<Polynomial>(Polynomial{std::move(msg), std::move(evaluator), flags});
    return true;
}

void ReferenceTrajectoryRuntime::publish(PeriodicGate& gate, double rate, ReferenceOutput kind) {
    if (gate.due(current_time_sec_, 1.0 / rate)) {
        outputs_.push_back({kind, current_time_sec_});
    }
}
void ReferenceTrajectoryRuntime::publishActive(Active& state) {
    publish(state.status, config_.status_rate_hz, ReferenceOutput::Status);
    const auto kind = std::visit(
        [](const auto& value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, Analytic>)
                return ReferenceOutput::Analytic;
            else if constexpr (std::is_same_v<T, Sampled>)
                return ReferenceOutput::Sampled;
            else
                return ReferenceOutput::Polynomial;
        },
        retained_);
    publish(state.publication, config_.active_publish_rate_hz, kind);
}

ReferenceTrajectoryRuntime::Spec::Change ReferenceTrajectoryRuntime::Spec::handle(
    SelfCheck&, Advance, ReferenceTrajectoryRuntime& rt) {
    Ready ready;
    rt.publish(ready.status, rt.config_.status_rate_hz, ReferenceOutput::Status);
    return adt::Enter{std::move(ready)};
}
ReferenceTrajectoryRuntime::Spec::Change ReferenceTrajectoryRuntime::Spec::handle(
    Ready& state, Advance, ReferenceTrajectoryRuntime& rt) {
    if (auto* request = std::get_if<Waypoint>(&rt.request_)) {
        Planning planning{std::move(*request)};
        rt.request_.emplace<std::monostate>();
        return adt::Enter{std::move(planning)};
    }
    if (!std::holds_alternative<std::monostate>(rt.request_)) {
        rt.activateRequest();
        Active active;
        // Original entry establishes phase; its transient publication was not returned.
        active.status.due(rt.current_time_sec_, 1.0 / rt.config_.status_rate_hz);
        active.publication.due(rt.current_time_sec_, 1.0 / rt.config_.active_publish_rate_hz);
        return adt::Enter{std::move(active)};
    }
    rt.publish(state.status, rt.config_.status_rate_hz, ReferenceOutput::Status);
    return adt::Stay{};
}
ReferenceTrajectoryRuntime::Spec::Change ReferenceTrajectoryRuntime::Spec::handle(
    Planning& state, Advance, ReferenceTrajectoryRuntime& rt) {
    if (!rt.planWaypoint(state.request))
        return adt::Enter{SelfCheck{}};
    Active active;
    rt.publishActive(active);
    return adt::Enter{std::move(active)};
}
ReferenceTrajectoryRuntime::Spec::Change ReferenceTrajectoryRuntime::Spec::handle(
    Active& state, Advance, ReferenceTrajectoryRuntime& rt) {
    if (auto* request = std::get_if<Waypoint>(&rt.request_)) {
        Planning planning{std::move(*request)};
        rt.request_.emplace<std::monostate>();
        return adt::Enter{std::move(planning)};
    }
    if (!rt.activateRequest())
        return adt::Enter{SelfCheck{}};
    if (rt.activeExpired(rt.current_time_sec_)) {
        Ready ready;
        rt.publish(ready.status, rt.config_.status_rate_hz, ReferenceOutput::Status);
        return adt::Enter{std::move(ready)};
    }
    rt.publishActive(state);
    return adt::Stay{};
}

uint8_t ReferenceTrajectoryRuntime::currentState() const {
    return std::visit(
        [](const auto& state) -> uint8_t {
            using T = std::decay_t<decltype(state)>;
            if constexpr (std::is_same_v<T, SelfCheck>)
                return reference::ReferenceStatus::STATE_SELF_CHECK;
            else if constexpr (std::is_same_v<T, Ready>)
                return reference::ReferenceStatus::STATE_READY;
            else if constexpr (std::is_same_v<T, Planning>)
                return reference::ReferenceStatus::STATE_PLANNING;
            else
                return reference::ReferenceStatus::STATE_ACTIVE;
        },
        machine_.state());
}
trajectory::TrajectoryModelType ReferenceTrajectoryRuntime::activeType() const {
    return std::visit(
        [](const auto& value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, Analytic>)
                return trajectory::TrajectoryModelType::kAnalytic;
            else if constexpr (std::is_same_v<T, Sampled>)
                return trajectory::TrajectoryModelType::kSampled;
            else if constexpr (std::is_same_v<T, Polynomial>)
                return trajectory::TrajectoryModelType::kPolynomial;
            else
                return trajectory::TrajectoryModelType::kNone;
        },
        retained_);
}
uint32_t ReferenceTrajectoryRuntime::activeTrajectoryId() const {
    return std::visit(
        [](const auto& value) -> uint32_t {
            if constexpr (std::is_same_v<std::decay_t<decltype(value)>, std::monostate>)
                return 0U;
            else
                return value.message.trajectory_id;
        },
        retained_);
}
uint32_t ReferenceTrajectoryRuntime::activeRevision() const {
    return std::visit(
        [](const auto& value) -> uint32_t {
            if constexpr (std::is_same_v<std::decay_t<decltype(value)>, std::monostate>)
                return 0U;
            else
                return value.message.revision;
        },
        retained_);
}
const trajectory::TrajectoryEvaluator2* ReferenceTrajectoryRuntime::evaluator() const {
    return std::visit(
        [](const auto& value) -> const trajectory::TrajectoryEvaluator2* {
            if constexpr (std::is_same_v<std::decay_t<decltype(value)>, std::monostate>)
                return nullptr;
            else
                return value.evaluator.get();
        },
        retained_);
}
const reference::AnalyticReference& ReferenceTrajectoryRuntime::activeAnalyticMessage() const {
    return std::get<Analytic>(retained_).message;
}
const reference::SampledReference& ReferenceTrajectoryRuntime::activeSampledMessage() const {
    return std::get<Sampled>(retained_).message;
}
const reference::ActivePolynomialReference& ReferenceTrajectoryRuntime::activePolynomialMessage()
    const {
    return std::get<Polynomial>(retained_).message;
}
double ReferenceTrajectoryRuntime::activeEnd() const {
    return std::visit(
        [this](const auto& value) -> double {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, std::monostate>)
                return -never;
            else {
                double duration;
                if constexpr (std::is_same_v<T, Sampled>)
                    duration = value.evaluator->duration();
                else
                    duration = value.message.duration;
                return duration <= 0.0 ? never
                                       : value.message.start_time.toSec() + duration +
                                             config_.trajectory_timeout;
            }
        },
        retained_);
}
bool ReferenceTrajectoryRuntime::activeExpired(double now) const {
    return now > activeEnd();
}
double ReferenceTrajectoryRuntime::nextDeadline() const {
    if (!std::holds_alternative<std::monostate>(request_))
        return current_time_sec_;
    return std::visit(
        [this](const auto& state) -> double {
            using T = std::decay_t<decltype(state)>;
            if constexpr (std::is_same_v<T, Ready>)
                return state.status.next(current_time_sec_, 1.0 / config_.status_rate_hz);
            else if constexpr (std::is_same_v<T, Active>)
                return std::min({state.status.next(current_time_sec_, 1.0 / config_.status_rate_hz),
                                 state.publication.next(current_time_sec_,
                                                        1.0 / config_.active_publish_rate_hz),
                                 std::nextafter(activeEnd(), never)});
            else
                return current_time_sec_;
        },
        machine_.state());
}
reference::ReferenceStatus ReferenceTrajectoryRuntime::makeStatus(double stamp_sec) const {
    reference::ReferenceStatus status;
    status.header.stamp = Time(stamp_sec);
    status.state = currentState();
    status.flags = flags_;
    status.active_trajectory_id = activeTrajectoryId();
    status.active_revision = activeRevision();
    switch (activeType()) {
        case trajectory::TrajectoryModelType::kAnalytic:
            status.active_type = reference::ReferenceStatus::TYPE_ANALYTIC;
            break;
        case trajectory::TrajectoryModelType::kPolynomial:
            status.active_type = reference::ReferenceStatus::TYPE_POLYNOMIAL;
            break;
        case trajectory::TrajectoryModelType::kSampled:
            status.active_type = reference::ReferenceStatus::TYPE_SAMPLED;
            break;
        default:
            status.active_type = reference::ReferenceStatus::TYPE_NONE;
            break;
    }
    return status;
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

}  // namespace unicycle_reference_trajectory
