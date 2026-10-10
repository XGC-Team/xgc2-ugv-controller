#include "unicycle_ugv_controller/output/nmpc_output_consumer.h"

#include <cmath>
#include <utility>

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/ros_time_conversion.h"

namespace unicycle_ugv_controller {
namespace {

geometry_msgs::Pose poseFromState(const NmpcStateVector& state) {
    geometry_msgs::Pose pose;
    pose.position.x = state(0);
    pose.position.y = state(1);
    pose.position.z = 0.05;
    const double yaw = state(2);
    if (std::isfinite(yaw)) {
        pose.orientation.z = std::sin(0.5 * yaw);
        pose.orientation.w = std::cos(0.5 * yaw);
    } else {
        pose.orientation.w = 1.0;
    }
    return pose;
}

}  // namespace

NmpcOutputConsumer::NmpcOutputConsumer(ros::NodeHandle& nh, UnicycleUgvController& controller,
                                       EventSink event_sink, uint32_t queue_size)
    : execution_(
          controller, std::move(event_sink), [] { return ros::Time::now().toSec(); }, {},
          [this](const Time& stamp, const NmpcTrackingBackend& backend) {
              publishPrediction(stamp, backend);
          }) {
    predicted_path_pub_ = nh.advertise<nav_msgs::Path>("alg/nmpc/predicted_path", queue_size);
    predicted_poses_pub_ =
        nh.advertise<geometry_msgs::PoseArray>("alg/nmpc/predicted_poses", queue_size);
    execution_.start();
}

void NmpcOutputConsumer::publishPrediction(const Time& time, const NmpcTrackingBackend& backend) {
    const ros::Time stamp = toRosTime(time);
    nav_msgs::Path path;
    geometry_msgs::PoseArray poses;
    path.header.stamp = stamp;
    path.header.frame_id = "world";
    poses.header = path.header;

    const auto& predicted_states = backend.predictedStates();
    const size_t count = backend.predictedStateCount();
    path.poses.reserve(count);
    poses.poses.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const geometry_msgs::Pose pose = poseFromState(predicted_states[i]);
        geometry_msgs::PoseStamped stamped_pose;
        stamped_pose.header = path.header;
        stamped_pose.pose = pose;
        path.poses.push_back(stamped_pose);
        poses.poses.push_back(pose);
    }

    predicted_path_pub_.publish(path);
    predicted_poses_pub_.publish(poses);
}

}  // namespace unicycle_ugv_controller
