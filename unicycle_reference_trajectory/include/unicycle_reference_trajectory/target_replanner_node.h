#pragma once

#include <estimator_vrpn_ugv_state/PlanarStateEstimate.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <cstdint>
#include <random>
#include <string>
#include <vector>
#include <xgc2_math/trajectory.hpp>

#include "unicycle_reference_trajectory/SampledReference.h"

namespace unicycle_reference_trajectory {

class TargetReplannerNode {
   public:
    explicit TargetReplannerNode(ros::NodeHandle& nh);

   private:
    struct TargetPose {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
    };

    struct PlanarState {
        ros::Time stamp;
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
        double speed{0.0};
        bool received{false};
    };

    void loadParams();
    void stateCallback(const estimator_vrpn_ugv_state::PlanarStateEstimate::ConstPtr& msg);
    void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent& event);
    bool chooseTarget(TargetPose& target);
    bool chooseRandomTarget(TargetPose& target);
    bool loadTargetSequence();
    bool publishPlan(const xgc2_math::trajectory::Se2TargetState2& start,
                     const xgc2_math::trajectory::Se2TargetState2& target);

    static double yawFromQuaternion(double x, double y, double z, double w);
    static double wrapAngle(double value);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber target_sub_;
    ros::Publisher sampled_pub_;
    ros::Timer replan_timer_;

    std::string state_topic_{"alg/state_estimator/state"};
    std::string target_topic_{"alg/unicycle_reference_trajectory/target_pose"};
    std::string sampled_topic_{"alg/unicycle_reference_trajectory/request/sampled"};
    std::string frame_id_{"world"};
    uint32_t queue_size_{10U};
    double replan_period_{5.0};
    double state_timeout_{0.5};
    double start_delay_{0.2};
    bool enable_target_topic_{true};
    bool random_targets_{true};
    double random_min_x_{-5.0};
    double random_max_x_{5.0};
    double random_min_y_{-5.0};
    double random_max_y_{5.0};
    double random_fixed_yaw_{0.0};
    std::string random_yaw_mode_{"heading"};
    uint32_t random_seed_{0U};

    xgc2_math::trajectory::Se2TargetTrajectoryOptions2 planner_options_{};
    std::vector<TargetPose> target_sequence_;
    size_t next_target_index_{0U};
    uint32_t trajectory_id_{1U};
    uint32_t revision_{1U};
    std::mt19937 rng_;

    PlanarState state_;
    TargetPose topic_target_;
    bool has_topic_target_{false};
};

}  // namespace unicycle_reference_trajectory
