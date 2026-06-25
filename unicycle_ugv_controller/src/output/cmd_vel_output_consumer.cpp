#include "unicycle_ugv_controller/output/cmd_vel_output_consumer.h"

#include <cmath>
#include <memory>
#include <utility>

namespace unicycle_ugv_controller {
namespace {

template <typename Message>
std::unique_ptr<::state_machine::runtime::Task<ros::NodeHandle>> makePublishTask(
    std::string name, const ros::Publisher& pub, Message msg) {
    return std::make_unique<::state_machine::runtime::LambdaTask<ros::NodeHandle>>(
        std::move(name),
        [pub, msg = std::move(msg)](ros::NodeHandle&) mutable { pub.publish(msg); });
}

}  // namespace

CmdVelOutputConsumer::CmdVelOutputConsumer(
    ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
    UnicycleUgvController& controller, const std::string& cmd_vel_topic, uint32_t queue_size)
    : executor_(executor), controller_(controller) {
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, queue_size);
}

bool CmdVelOutputConsumer::handle(const ::state_machine::Event& event) {
    if (event.id == output_event_type::PUBLISH_CMD_VEL) {
        executor_.pushTask(
            makePublishTask("PublishCmdVel", cmd_vel_pub_, makeTwist(controller_.command())));
        return true;
    }
    if (event.id == output_event_type::PUBLISH_ZERO_CMD_VEL) {
        executor_.pushTask(
            makePublishTask("PublishZeroCmdVel", cmd_vel_pub_, geometry_msgs::Twist{}));
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
    msg.linear.x = clamp(command.linear_speed, cfg.min_linear_speed, cfg.max_linear_speed);
    msg.angular.z = clamp(command.angular_speed, -cfg.max_angular_speed, cfg.max_angular_speed);
    return msg;
}

}  // namespace unicycle_ugv_controller
