#pragma once

#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <string>

#include "unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h"

namespace unicycle_reference_trajectory {

class ReferenceOutputConsumer {
   public:
    ReferenceOutputConsumer(ros::NodeHandle& nh,
                            ReferenceTrajectoryRuntime& runtime, const std::string& status_topic,
                            const std::string& active_analytic_topic,
                            const std::string& active_polynomial_topic,
                            const std::string& active_sampled_topic,
                            const std::string& reference_path_topic,
                            double reference_path_sample_dt, double reference_path_preview_duration,
                            uint32_t queue_size);

    void handle(const ReferencePublication& publication);

   private:
    nav_msgs::Path makeReferencePath(double stamp_sec) const;
    void publishReferencePath(double stamp_sec);

    ReferenceTrajectoryRuntime& runtime_;
    ros::Publisher status_pub_;
    ros::Publisher active_analytic_pub_;
    ros::Publisher active_polynomial_pub_;
    ros::Publisher active_sampled_pub_;
    ros::Publisher reference_path_pub_;
    double reference_path_sample_dt_{0.5};
    double reference_path_preview_duration_{20.0};
};

}  // namespace unicycle_reference_trajectory
