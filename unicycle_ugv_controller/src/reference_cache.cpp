#include "unicycle_ugv_controller/common/reference_cache.h"

#include <geometry_msgs/Quaternion.h>

#include <algorithm>
#include <cmath>

#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {
namespace {

namespace control = xgc2_math::control;
namespace trajectory = xgc2_math::trajectory;

double paramAt(const unicycle_reference_trajectory::AnalyticReference& msg, size_t index,
               double fallback) {
    return msg.params.size() > index && std::isfinite(msg.params[index]) ? msg.params[index]
                                                                         : fallback;
}

std::unique_ptr<trajectory::TrajectoryEvaluator2> buildAnalytic(
    const unicycle_reference_trajectory::AnalyticReference& msg, uint32_t& flags) {
    flags = msg.flags;
    const double duration = msg.duration > 0.0 ? msg.duration : 60.0;
    const Eigen::Vector2d origin(msg.origin.position.x, msg.origin.position.y);
    const double origin_yaw = yawFromQuaternion(msg.origin.orientation.x, msg.origin.orientation.y,
                                                msg.origin.orientation.z, msg.origin.orientation.w);
    const double radius = paramAt(msg, 0U, 3.0);
    const double line_speed = paramAt(msg, 1U, 1.0);
    const double entry_duration = paramAt(msg, 2U, 3.0);
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    center.x() = paramAt(msg, 3U, center.x());
    center.y() = paramAt(msg, 4U, center.y());

    std::unique_ptr<trajectory::TrajectoryEvaluator2> evaluator;
    switch (msg.analytic_type) {
        case unicycle_reference_trajectory::AnalyticReference::ANALYTIC_HOLD: {
            trajectory::HoldCurveParameters2 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.position = origin;
            params.yaw = origin_yaw;
            evaluator = std::make_unique<trajectory::HoldCurveEvaluator2>(params);
            break;
        }
        case unicycle_reference_trajectory::AnalyticReference::ANALYTIC_CIRCLE: {
            trajectory::CircleCurveParameters2 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.center = center;
            params.radius = radius;
            params.line_speed = line_speed;
            evaluator = std::make_unique<trajectory::CircleCurveEvaluator2>(params);
            break;
        }
        case unicycle_reference_trajectory::AnalyticReference::ANALYTIC_FIGURE_EIGHT: {
            trajectory::FigureEightCurveParameters2 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.center = center;
            params.radius = radius;
            params.line_speed = line_speed;
            evaluator = std::make_unique<trajectory::FigureEightCurveEvaluator2>(params);
            break;
        }
        case unicycle_reference_trajectory::AnalyticReference::ANALYTIC_CIRCLE_ENTRY:
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
    flags |= trajectory::TrajectoryValidator2::validate(*evaluator, trajectory::TrajectoryLimits2{},
                                                        0.02);
    return (flags & (trajectory::kFlagInvalidInput | trajectory::kFlagNonFinite)) == 0U
               ? std::move(evaluator)
               : nullptr;
}

bool fillPolynomial(const unicycle_reference_trajectory::ActivePolynomialReference& msg,
                    trajectory::PiecewisePolynomialEvaluator2& evaluator, uint32_t& flags) {
    flags = msg.flags;
    const size_t coeff_count = static_cast<size_t>(msg.order) + 1U;
    if (coeff_count == 0U || msg.segment_durations.empty() ||
        msg.coeff_x.size() != coeff_count * msg.segment_durations.size() ||
        msg.coeff_y.size() != coeff_count * msg.segment_durations.size() ||
        (!msg.coeff_yaw.empty() &&
         msg.coeff_yaw.size() != coeff_count * msg.segment_durations.size())) {
        flags |= trajectory::kFlagInvalidInput;
        return false;
    }
    std::vector<trajectory::PolynomialSegment2> segments;
    segments.reserve(msg.segment_durations.size());
    for (size_t i = 0; i < msg.segment_durations.size(); ++i) {
        trajectory::PolynomialSegment2 segment;
        segment.duration = msg.segment_durations[i];
        const size_t offset = i * coeff_count;
        segment.x.assign(msg.coeff_x.begin() + static_cast<long>(offset),
                         msg.coeff_x.begin() + static_cast<long>(offset + coeff_count));
        segment.y.assign(msg.coeff_y.begin() + static_cast<long>(offset),
                         msg.coeff_y.begin() + static_cast<long>(offset + coeff_count));
        if (!msg.coeff_yaw.empty()) {
            segment.yaw.assign(msg.coeff_yaw.begin() + static_cast<long>(offset),
                               msg.coeff_yaw.begin() + static_cast<long>(offset + coeff_count));
        }
        segments.push_back(std::move(segment));
    }
    if (!evaluator.setSegments(std::move(segments), msg.order)) {
        flags |= trajectory::kFlagInvalidInput;
        return false;
    }
    flags |= trajectory::TrajectoryValidator2::validate(evaluator, trajectory::TrajectoryLimits2{},
                                                        0.02);
    return (flags & (trajectory::kFlagInvalidInput | trajectory::kFlagNonFinite)) == 0U;
}

bool fillSampled(const unicycle_reference_trajectory::SampledReference& msg,
                 trajectory::SampledEvaluator2& evaluator, uint32_t& flags) {
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
    if (!evaluator.setSamples(std::move(samples))) {
        flags |= trajectory::kFlagInvalidInput;
        return false;
    }
    flags |= trajectory::TrajectoryValidator2::validate(evaluator, trajectory::TrajectoryLimits2{},
                                                        0.02);
    return (flags & (trajectory::kFlagInvalidInput | trajectory::kFlagNonFinite)) == 0U;
}

control::Se2Reference toSample(const trajectory::PlanarReference2& ref) {
    control::Se2Reference sample;
    sample.state.position = ref.position;
    sample.state.yaw = ref.yaw;
    sample.state.linear_speed = ref.speed;
    sample.control.linear_acceleration = ref.linear_acceleration;
    sample.control.yaw_rate = ref.yaw_rate;
    sample.flags = ref.flags;
    return sample;
}

}  // namespace

bool ReferenceCache::updateAnalytic(const unicycle_reference_trajectory::AnalyticReference& msg,
                                    const ros::Time& received_time) {
    uint32_t flags = 0U;
    auto evaluator = buildAnalytic(msg, flags);
    if (!evaluator) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    evaluator_ = std::shared_ptr<const trajectory::TrajectoryEvaluator2>(std::move(evaluator));
    start_time_ = msg.start_time;
    received_time_ = received_time;
    trajectory_id_ = msg.trajectory_id;
    revision_ = msg.revision;
    flags_ = flags;
    return true;
}

bool ReferenceCache::updatePolynomial(
    const unicycle_reference_trajectory::ActivePolynomialReference& msg,
    const ros::Time& received_time) {
    auto evaluator = std::make_unique<trajectory::PiecewisePolynomialEvaluator2>();
    uint32_t flags = 0U;
    if (!fillPolynomial(msg, *evaluator, flags)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    evaluator_ = std::shared_ptr<const trajectory::TrajectoryEvaluator2>(std::move(evaluator));
    start_time_ = msg.start_time;
    received_time_ = received_time;
    trajectory_id_ = msg.trajectory_id;
    revision_ = msg.revision;
    flags_ = flags;
    return true;
}

bool ReferenceCache::updateSampled(const unicycle_reference_trajectory::SampledReference& msg,
                                   const ros::Time& received_time) {
    auto evaluator = std::make_unique<trajectory::SampledEvaluator2>();
    uint32_t flags = 0U;
    if (!fillSampled(msg, *evaluator, flags)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    evaluator_ = std::shared_ptr<const trajectory::TrajectoryEvaluator2>(std::move(evaluator));
    start_time_ = msg.start_time;
    received_time_ = received_time;
    trajectory_id_ = msg.trajectory_id;
    revision_ = msg.revision;
    flags_ = flags;
    return true;
}

void ReferenceCache::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    evaluator_.reset();
    start_time_ = ros::Time{};
    received_time_ = ros::Time{};
    trajectory_id_ = 0U;
    revision_ = 0U;
    flags_ = 0U;
}

bool ReferenceCache::activeLocked(const ros::Time& now, double timeout) const {
    if (!evaluator_ || timeout <= 0.0 || (now - received_time_).toSec() > timeout) {
        return false;
    }
    constexpr uint32_t fatal = trajectory::kFlagInvalidInput | trajectory::kFlagNonFinite;
    return (flags_ & fatal) == 0U;
}

bool ReferenceCache::valid(const ros::Time& now, double timeout) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return activeLocked(now, timeout);
}

bool ReferenceCache::sampleHorizon(const ros::Time& now, double stage_dt, int horizon_steps,
                                   double timeout, std::vector<control::Se2Reference>& refs) const {
    if (horizon_steps <= 0 || !std::isfinite(stage_dt) || stage_dt <= 0.0) {
        return false;
    }
    std::shared_ptr<const trajectory::TrajectoryEvaluator2> evaluator;
    ros::Time start_time;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!activeLocked(now, timeout)) {
            return false;
        }
        evaluator = evaluator_;
        start_time = start_time_;
    }
    if (!evaluator) {
        return false;
    }
    refs.clear();
    refs.reserve(static_cast<size_t>(horizon_steps) + 1U);
    const double base_t = std::max(0.0, (now - start_time).toSec());
    for (int i = 0; i <= horizon_steps; ++i) {
        trajectory::PlanarReference2 ref;
        if (!evaluator->evaluate(base_t + static_cast<double>(i) * stage_dt, ref)) {
            return false;
        }
        refs.push_back(toSample(ref));
    }
    return true;
}

}  // namespace unicycle_ugv_controller
