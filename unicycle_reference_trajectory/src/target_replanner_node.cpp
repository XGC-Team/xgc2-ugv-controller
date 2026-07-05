#include "unicycle_reference_trajectory/target_replanner_node.h"

#include <XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <random>

namespace unicycle_reference_trajectory {
namespace {

constexpr double kPi = 3.14159265358979323846;

double xmlDouble(const XmlRpc::XmlRpcValue& value, const char* key, double fallback) {
    if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct || !value.hasMember(key)) {
        return fallback;
    }
    const auto& item = value[key];
    if (item.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        return static_cast<double>(item);
    }
    if (item.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        return static_cast<int>(item);
    }
    return fallback;
}

}  // namespace

TargetReplannerNode::TargetReplannerNode(ros::NodeHandle& nh) : nh_(nh), private_nh_("~") {
    loadParams();
    if (random_seed_ == 0U) {
        std::random_device rd;
        rng_.seed(rd());
    } else {
        rng_.seed(random_seed_);
    }
    state_sub_ =
        nh_.subscribe(state_topic_, queue_size_, &TargetReplannerNode::stateCallback, this);
    if (enable_target_topic_) {
        target_sub_ =
            nh_.subscribe(target_topic_, queue_size_, &TargetReplannerNode::targetCallback, this);
    }
    sampled_pub_ = nh_.advertise<SampledReference>(sampled_topic_, queue_size_, true);
    replan_timer_ = nh_.createTimer(ros::Duration(replan_period_),
                                    &TargetReplannerNode::timerCallback, this, false, true);
    ROS_INFO(
        "[TargetReplannerNode] Initialized: state=%s target=%s sampled=%s period=%.2fs "
        "random=%s targets=%zu",
        state_topic_.c_str(), target_topic_.c_str(), sampled_topic_.c_str(), replan_period_,
        random_targets_ ? "true" : "false", target_sequence_.size());
}

void TargetReplannerNode::loadParams() {
    int queue_size = static_cast<int>(queue_size_);
    private_nh_.param("queue_size", queue_size, queue_size);
    queue_size_ = static_cast<uint32_t>(std::max(1, queue_size));
    private_nh_.param("state_topic", state_topic_, state_topic_);
    private_nh_.param("target_topic", target_topic_, target_topic_);
    private_nh_.param("sampled_topic", sampled_topic_, sampled_topic_);
    private_nh_.param("frame_id", frame_id_, frame_id_);
    private_nh_.param("replan_period", replan_period_, replan_period_);
    private_nh_.param("state_timeout", state_timeout_, state_timeout_);
    private_nh_.param("start_delay", start_delay_, start_delay_);
    private_nh_.param("enable_target_topic", enable_target_topic_, enable_target_topic_);
    private_nh_.param("random_targets", random_targets_, random_targets_);
    private_nh_.param("random_min_x", random_min_x_, random_min_x_);
    private_nh_.param("random_max_x", random_max_x_, random_max_x_);
    private_nh_.param("random_min_y", random_min_y_, random_min_y_);
    private_nh_.param("random_max_y", random_max_y_, random_max_y_);
    private_nh_.param("random_yaw_mode", random_yaw_mode_, random_yaw_mode_);
    private_nh_.param("random_fixed_yaw", random_fixed_yaw_, random_fixed_yaw_);
    int random_seed = static_cast<int>(random_seed_);
    private_nh_.param("random_seed", random_seed, random_seed);
    random_seed_ = static_cast<uint32_t>(std::max(0, random_seed));
    replan_period_ = std::isfinite(replan_period_) && replan_period_ > 0.0 ? replan_period_ : 5.0;
    state_timeout_ = std::isfinite(state_timeout_) && state_timeout_ > 0.0 ? state_timeout_ : 0.5;
    start_delay_ = std::isfinite(start_delay_) && start_delay_ >= 0.0 ? start_delay_ : 0.2;
    if (!std::isfinite(random_min_x_) || !std::isfinite(random_max_x_) ||
        random_min_x_ >= random_max_x_) {
        random_min_x_ = -5.0;
        random_max_x_ = 5.0;
    }
    if (!std::isfinite(random_min_y_) || !std::isfinite(random_max_y_) ||
        random_min_y_ >= random_max_y_) {
        random_min_y_ = -5.0;
        random_max_y_ = 5.0;
    }
    random_fixed_yaw_ = std::isfinite(random_fixed_yaw_) ? random_fixed_yaw_ : 0.0;

    private_nh_.param("piece_count", planner_options_.piece_count, planner_options_.piece_count);
    private_nh_.param("sample_dt", planner_options_.sample_dt, planner_options_.sample_dt);
    private_nh_.param("desired_speed", planner_options_.desired_speed,
                      planner_options_.desired_speed);
    private_nh_.param("min_boundary_speed", planner_options_.min_boundary_speed,
                      planner_options_.min_boundary_speed);
    private_nh_.param("approach_speed", planner_options_.approach_speed,
                      planner_options_.approach_speed);
    private_nh_.param("hold_duration", planner_options_.hold_duration,
                      planner_options_.hold_duration);
    private_nh_.param("max_duration", planner_options_.max_duration, planner_options_.max_duration);
    private_nh_.param("min_segment_time", planner_options_.min_segment_time,
                      planner_options_.min_segment_time);
    private_nh_.param("max_segment_time", planner_options_.max_segment_time,
                      planner_options_.max_segment_time);
    private_nh_.param("time_weight", planner_options_.time_weight, planner_options_.time_weight);
    private_nh_.param("dynamic_penalty_weight", planner_options_.dynamic_penalty_weight,
                      planner_options_.dynamic_penalty_weight);
    private_nh_.param("max_iterations", planner_options_.max_iterations,
                      planner_options_.max_iterations);
    private_nh_.param("rel_cost_tol", planner_options_.rel_cost_tol, planner_options_.rel_cost_tol);
    private_nh_.param("validation_sample_dt", planner_options_.validation_sample_dt,
                      planner_options_.validation_sample_dt);
    private_nh_.param("position_tolerance", planner_options_.position_tolerance,
                      planner_options_.position_tolerance);
    private_nh_.param("max_velocity", planner_options_.max_velocity, planner_options_.max_velocity);
    private_nh_.param("max_acceleration", planner_options_.max_acceleration,
                      planner_options_.max_acceleration);
    private_nh_.param("max_yaw_rate", planner_options_.max_yaw_rate, planner_options_.max_yaw_rate);

    loadTargetSequence();
}

bool TargetReplannerNode::loadTargetSequence() {
    target_sequence_.clear();
    XmlRpc::XmlRpcValue targets;
    if (!private_nh_.getParam("target_sequence", targets)) {
        return false;
    }
    if (targets.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_WARN("[TargetReplannerNode] target_sequence is not a YAML array");
        return false;
    }
    for (int i = 0; i < targets.size(); ++i) {
        const auto& value = targets[i];
        TargetPose target;
        target.x = xmlDouble(value, "x", 0.0);
        target.y = xmlDouble(value, "y", 0.0);
        target.yaw = xmlDouble(value, "yaw", 0.0);
        if (std::isfinite(target.x) && std::isfinite(target.y) && std::isfinite(target.yaw)) {
            target_sequence_.push_back(target);
        }
    }
    return !target_sequence_.empty();
}

void TargetReplannerNode::stateCallback(
    const rigid_state_estimator_msgs::PlanarStateEstimate::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    const double yaw = yawFromQuaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z,
                                         msg->orientation.w);
    state_.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    state_.x = msg->position.x;
    state_.y = msg->position.y;
    state_.yaw = yaw;
    state_.speed = std::cos(yaw) * msg->velocity.x + std::sin(yaw) * msg->velocity.y;
    state_.received = std::isfinite(state_.x) && std::isfinite(state_.y) &&
                      std::isfinite(state_.yaw) && std::isfinite(state_.speed);
}

void TargetReplannerNode::targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    topic_target_.x = msg->pose.position.x;
    topic_target_.y = msg->pose.position.y;
    topic_target_.yaw = yawFromQuaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                                          msg->pose.orientation.z, msg->pose.orientation.w);
    has_topic_target_ = std::isfinite(topic_target_.x) && std::isfinite(topic_target_.y) &&
                        std::isfinite(topic_target_.yaw);
}

void TargetReplannerNode::timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (!state_.received || (now - state_.stamp).toSec() > state_timeout_) {
        ROS_WARN_THROTTLE(2.0, "[TargetReplannerNode] Waiting for fresh state estimate");
        return;
    }

    xgc2_math::trajectory::Se2TargetState2 start;
    start.position = Eigen::Vector2d(state_.x, state_.y);
    start.yaw = state_.yaw;
    start.speed = state_.speed;

    if (random_targets_ && !has_topic_target_) {
        TargetPose target;
        if (!chooseRandomTarget(target)) {
            ROS_WARN("[TargetReplannerNode] Failed to choose random target");
            return;
        }

        xgc2_math::trajectory::Se2TargetState2 goal;
        goal.position = Eigen::Vector2d(target.x, target.y);
        goal.yaw = target.yaw;
        goal.speed = 0.0;
        (void)publishPlan(start, goal);
        return;
    }

    TargetPose target;
    if (!chooseTarget(target)) {
        ROS_WARN_THROTTLE(2.0, "[TargetReplannerNode] No target pose configured or received");
        return;
    }

    xgc2_math::trajectory::Se2TargetState2 goal;
    goal.position = Eigen::Vector2d(target.x, target.y);
    goal.yaw = target.yaw;
    goal.speed = 0.0;
    (void)publishPlan(start, goal);
}

bool TargetReplannerNode::chooseTarget(TargetPose& target) {
    if (has_topic_target_) {
        target = topic_target_;
        return true;
    }
    if (random_targets_) {
        return chooseRandomTarget(target);
    }
    if (target_sequence_.empty()) {
        return false;
    }
    target = target_sequence_[next_target_index_ % target_sequence_.size()];
    ++next_target_index_;
    return true;
}

bool TargetReplannerNode::chooseRandomTarget(TargetPose& target) {
    std::uniform_real_distribution<double> dist_x(random_min_x_, random_max_x_);
    std::uniform_real_distribution<double> dist_y(random_min_y_, random_max_y_);
    std::uniform_real_distribution<double> dist_yaw(-kPi, kPi);

    target.x = dist_x(rng_);
    target.y = dist_y(rng_);
    const double heading = std::atan2(target.y - state_.y, target.x - state_.x);
    if (random_yaw_mode_ == "fixed") {
        target.yaw = random_fixed_yaw_;
    } else if (random_yaw_mode_ == "random") {
        target.yaw = dist_yaw(rng_);
    } else {
        target.yaw = heading;
    }
    target.yaw = wrapAngle(target.yaw);
    return std::isfinite(target.x) && std::isfinite(target.y) && std::isfinite(target.yaw);
}

bool TargetReplannerNode::publishPlan(const xgc2_math::trajectory::Se2TargetState2& start,
                                      const xgc2_math::trajectory::Se2TargetState2& target) {
    xgc2_math::trajectory::Se2TargetTrajectoryResult2 result;
    const bool ok = xgc2_math::trajectory::Se2MincoTargetPlanner2().plan(start, target,
                                                                         planner_options_, result);
    if (!ok || result.fallback_hold || result.samples.empty()) {
        ROS_WARN(
            "[TargetReplannerNode] Target plan failed; keeping current active reference "
            "target=(%.2f, %.2f, %.2f) flags=0x%08x fallback=%s samples=%zu",
            target.position.x(), target.position.y(), target.yaw, result.flags,
            result.fallback_hold ? "true" : "false", result.samples.size());
        return false;
    }

    constexpr uint32_t kHardPlanFlags =
        xgc2_math::trajectory::kFlagInvalidInput | xgc2_math::trajectory::kFlagNonFinite;
    if ((result.flags & kHardPlanFlags) != 0U) {
        ROS_WARN(
            "[TargetReplannerNode] Target plan has invalid samples; keeping current active "
            "reference target=(%.2f, %.2f, %.2f) flags=0x%08x",
            target.position.x(), target.position.y(), target.yaw, result.flags);
        return false;
    }

    SampledReference msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    msg.trajectory_id = trajectory_id_++;
    msg.revision = revision_++;
    msg.flags = result.flags;
    msg.start_time = msg.header.stamp + ros::Duration(start_delay_);
    msg.sample_dt = planner_options_.sample_dt;
    msg.points.reserve(result.samples.size());
    for (const auto& sample : result.samples) {
        PlanarReferencePoint point;
        point.t_from_start = sample.t;
        point.x = sample.reference.position.x();
        point.y = sample.reference.position.y();
        point.yaw = sample.reference.yaw;
        point.speed = sample.reference.speed;
        point.linear_acceleration = sample.reference.linear_acceleration;
        point.yaw_rate = sample.reference.yaw_rate;
        point.yaw_acceleration = sample.reference.yaw_acceleration;
        point.curvature = sample.reference.curvature;
        point.vx = sample.reference.velocity.x();
        point.vy = sample.reference.velocity.y();
        point.ax = sample.reference.acceleration.x();
        point.ay = sample.reference.acceleration.y();
        point.jx = sample.reference.jerk.x();
        point.jy = sample.reference.jerk.y();
        msg.points.push_back(point);
    }
    sampled_pub_.publish(msg);
    ROS_INFO(
        "[TargetReplannerNode] Published %s target plan id=%u samples=%zu duration=%.2fs "
        "target=(%.2f, %.2f, %.2f) flags=0x%08x",
        "optimized", msg.trajectory_id, msg.points.size(), result.duration, target.position.x(),
        target.position.y(), target.yaw, msg.flags);
    return true;
}

double TargetReplannerNode::yawFromQuaternion(double x, double y, double z, double w) {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return wrapAngle(std::atan2(siny_cosp, cosy_cosp));
}

double TargetReplannerNode::wrapAngle(double value) {
    return std::atan2(std::sin(value), std::cos(value));
}

}  // namespace unicycle_reference_trajectory
