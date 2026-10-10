#include "unicycle_ugv_controller/output/cmd_vel_output_consumer.h"

#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {
CmdVelOutputConsumer::CmdVelOutputConsumer(
    ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
    UnicycleUgvController& controller, ugv_reset_client::ResetClient& reset_client,
    const std::string& cmd_vel_topic, uint32_t queue_size)
    : controller_(controller), reset_client_(reset_client) {
    (void)executor;
    (void)queue_size;
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
}

bool CmdVelOutputConsumer::handle(const ::state_machine::Event& event) {
    geometry_msgs::Twist twist;
    if (event.id == output_event_type::PUBLISH_CMD_VEL) {
        const CmdVel command = controller_.cmdVel();
        twist.linear.x = command.linear_x;
        twist.angular.z = command.angular_z;
    } else if (event.id != output_event_type::PUBLISH_ZERO_CMD_VEL) {
        return false;
    }
    cmd_vel_pub_.publish(twist);
    reset_client_.noteApplied({twist.linear.x, twist.linear.y, twist.angular.z});
    return true;
}

}  // namespace unicycle_ugv_controller
