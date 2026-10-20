#include "unicycle_ugv_controller/output/cmd_vel_output_consumer.h"

#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {
CmdVelOutputConsumer::CmdVelOutputConsumer(
    ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
    UnicycleUgvController& controller, const std::string& cmd_vel_topic, uint32_t queue_size)
    : controller_(controller) {
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
    return true;
}

}  // namespace unicycle_ugv_controller
