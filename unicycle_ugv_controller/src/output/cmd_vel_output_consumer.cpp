#include "unicycle_ugv_controller/output/cmd_vel_output_consumer.h"
namespace unicycle_ugv_controller {
CmdVelOutputConsumer::CmdVelOutputConsumer(ros::NodeHandle& nh, UnicycleUgvController& controller,
                                           const std::string& topic, uint32_t, std::function<double()> clock)
    : controller_(controller), clock_(std::move(clock)) { cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(topic, 1); }
void CmdVelOutputConsumer::handle(const PublishCommand&) {
    geometry_msgs::Twist twist;
    const auto command = controller_.cmdVel();
    twist.linear.x = command.linear_x;
    twist.angular.z = command.angular_z;
    publish(twist);
}
void CmdVelOutputConsumer::handle(const PublishZero&) { publish(geometry_msgs::Twist{}); }
void CmdVelOutputConsumer::publish(const geometry_msgs::Twist& twist) {
    cmd_vel_pub_.publish(twist);
    if (twist.linear.x == 0.0 && twist.linear.y == 0.0 && twist.linear.z == 0.0 &&
        twist.angular.x == 0.0 && twist.angular.y == 0.0 && twist.angular.z == 0.0)
        controller_.notifyResetZeroPublished(clock_());
}
}  // namespace unicycle_ugv_controller
