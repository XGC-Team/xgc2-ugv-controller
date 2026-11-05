#pragma once

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <functional>

#include <string>

#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

class CmdVelOutputConsumer {
   public:
    CmdVelOutputConsumer(ros::NodeHandle& nh,
                         UnicycleUgvController& controller, const std::string& cmd_vel_topic,
                         uint32_t queue_size, std::function<double()> clock);

    void handle(const PublishCommand&);
    void handle(const PublishZero&);

   private:
    void publish(const geometry_msgs::Twist& twist);
    UnicycleUgvController& controller_;
    std::function<double()> clock_;
    ros::Publisher cmd_vel_pub_;
};

}  // namespace unicycle_ugv_controller
