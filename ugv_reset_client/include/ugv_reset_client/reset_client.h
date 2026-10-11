#pragma once

#include <ros/ros.h>
#include <ugv_reset_msgs/ResetRequest.h>
#include <ugv_reset_msgs/ResetResponse.h>

#include <functional>
#include <string>
#include <utility>

#include "ugv_reset_client/reset_lease.h"

namespace ugv_reset_client {

// The ROS edge of the lease: publishes the requests on `reset/request` and takes
// the coordinator's responses from `reset/response` (both relative to the
// vehicle's namespace unless the topics are given). The vehicle's controller owns the Reset
// session; the owner of this client reads the session from the controller after each control
// update, hands it to update(), and gives every accepted response back to the
// controller through the sink.
class ResetClient {
   public:
    using ClearanceSink = std::function<void(const ResetLease::Clearance&)>;

    ResetClient(ros::NodeHandle& nh, ClearanceSink sink,
                const std::string& request_topic = "reset/request",
                const std::string& response_topic = "reset/response")
        : sink_(std::move(sink)) {
        request_pub_ = nh.advertise<ugv_reset_msgs::ResetRequest>(request_topic, 1);
        response_sub_ = nh.subscribe(response_topic, 1, &ResetClient::receive, this);
    }

    // Follows the controller's session (`active`, `generation`, `target`) and, while it
    // is active and the vehicle is healthy, sends a request for the current pose.
    void update(bool active, uint32_t generation, ResetLease::Pose target, ResetLease::Pose pose,
                ros::Time pose_stamp, bool healthy) {
        if (!active) {
            lease_.cancel();
            return;
        }
        if (!lease_.active() || lease_.generation() != generation) {
            lease_.begin(generation, target);
        }
        if (!healthy) {
            return;
        }
        const auto request = lease_.issue(pose, ros::Time::now().toNSec(), monotonicSeconds());
        if (!request.valid) {
            return;
        }
        ugv_reset_msgs::ResetRequest message;
        message.generation = request.generation;
        message.header.stamp.fromNSec(request.stamp);
        message.header.frame_id = "world";
        message.pose_stamp = pose_stamp;
        message.applied_stamp.fromNSec(request.applied_stamp);
        message.applied_command.linear.x = request.applied_command.x;
        message.applied_command.linear.y = request.applied_command.y;
        message.applied_command.angular.z = request.applied_command.yaw;
        message.pose.x = request.pose.x;
        message.pose.y = request.pose.y;
        message.pose.theta = request.pose.yaw;
        message.target.x = request.target.x;
        message.target.y = request.target.y;
        message.target.theta = request.target.yaw;
        request_pub_.publish(message);
    }

    // The command the vehicle really sent to its chassis, with the time it was sent.
    void noteApplied(ResetLease::Command command) {
        lease_.noteApplied(command, ros::Time::now().toNSec());
    }

   private:
    void receive(const ugv_reset_msgs::ResetResponse::ConstPtr& message) {
        if (!message || message->header.frame_id != "world" || message->command.linear.z != 0 ||
            message->command.angular.x != 0 || message->command.angular.y != 0) {
            return;
        }
        ResetLease::Clearance clearance;
        if (lease_.accept(
                message->generation, message->header.stamp.toNSec(), message->status,
                {message->command.linear.x, message->command.linear.y, message->command.angular.z},
                ros::Time::now().toNSec(), monotonicSeconds(), clearance)) {
            sink_(clearance);
        }
    }

    ClearanceSink sink_;
    ResetLease lease_;
    ros::Publisher request_pub_;
    ros::Subscriber response_sub_;
};

}  // namespace ugv_reset_client
