#include "unicycle_ugv_controller/output/cmd_vel_output_consumer.h"

#include <cmath>
#include <memory>
#include <utility>
#include <variant>
#include <std_msgs/String.h>

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/common/flatness_audit.h"

namespace unicycle_ugv_controller {
CmdVelOutputConsumer::CmdVelOutputConsumer(
    ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
    UnicycleUgvController& controller, const std::string& cmd_vel_topic, uint32_t queue_size)
    : controller_(controller) {
    (void)executor;
    (void)queue_size;
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
    flatness_audit_pub_ = nh.advertise<std_msgs::String>(cmd_vel_topic + "/flatness_audit", 1);
}

bool CmdVelOutputConsumer::handle(const ::state_machine::Event& event) {
    if (event.id == output_event_type::PUBLISH_CMD_VEL) {
        const auto held_command = controller_.command();
        const auto command = makeTwist(held_command);
        cmd_vel_pub_.publish(command);
        controller_.resetSession().noteApplied(
            {command.linear.x, command.linear.y, command.angular.z}, ros::Time::now().toNSec());
        const auto sample_it = event.payload.find("flatness_audit_json");
        const auto stamp_it = event.payload.find("flatness_audit_stamp");
        if (sample_it != event.payload.end() && stamp_it != event.payload.end()) {
            const auto* sample = std::get_if<std::string>(&sample_it->second);
            const auto* stamp = std::get_if<double>(&stamp_it->second);
            if (sample && stamp) {
                std_msgs::String audit;
                audit.data = flatnessAuditPublication(*sample, ros::Time::now().toSec(),
                    command.linear.x, command.angular.z, held_command.stamp.toSec(), *stamp);
                if (!audit.data.empty()) flatness_audit_pub_.publish(audit);
            }
        }
        return true;
    }
    if (event.id == output_event_type::PUBLISH_ZERO_CMD_VEL) {
        cmd_vel_pub_.publish(geometry_msgs::Twist{});
        controller_.resetSession().noteApplied({}, ros::Time::now().toNSec());
        return true;
    }
    return false;
}

geometry_msgs::Twist CmdVelOutputConsumer::makeTwist(const ControlCommand& command) const {
    const auto cfg = controller_.config();
    geometry_msgs::Twist msg;
    if (!command.valid || !std::isfinite(command.linear_speed) ||
        !std::isfinite(command.angular_speed)) {
        return msg;
    }
    const auto control = controller_.stateMachine().currentState(region_type::CONTROL);
    if (control == state_type::Reset) {
        const auto feedback = controller_.resetSession().feedback(
            ros::Time::now().toNSec(), ugv_reset_safety::monotonicSeconds());
        if (!feedback.valid || feedback.status != ugv_reset_safety::ResetSession::RUNNING ||
            std::abs(feedback.command.x) > cfg.chassis_max_linear_speed ||
            feedback.command.y != 0.0 ||
            std::abs(feedback.command.yaw) > cfg.chassis_max_yaw_rate) {
            return msg;
        }
        msg.linear.x = feedback.command.x;
        msg.angular.z = feedback.command.yaw;
        return msg;
    }
    if (control == state_type::Custom1 && cfg.tracking_strategy == TrackingStrategy::NMPC) {
        msg.linear.x = clamp(command.linear_speed, cfg.min_linear_speed, cfg.max_linear_speed);
        msg.angular.z = clamp(command.angular_speed, -cfg.max_angular_speed, cfg.max_angular_speed);
    } else {
        msg.linear.x = clamp(command.linear_speed, -cfg.chassis_max_linear_speed,
                             cfg.chassis_max_linear_speed);
        msg.angular.z =
            clamp(command.angular_speed, -cfg.chassis_max_yaw_rate, cfg.chassis_max_yaw_rate);
    }
    return msg;
}

}  // namespace unicycle_ugv_controller
