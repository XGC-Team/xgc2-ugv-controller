#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <ros/ros.h>
#include <unicycle_reference_trajectory_msgs/SampledReference.h>

#include <cstdint>
#include <random>
#include <string>
#include <vector>
#include <xgc2_math/trajectory.hpp>

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
    void stateCallback(const rigid_state_estimator_msgs::RigidStateEstimate::ConstPtr& msg);
    void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent& event);
    bool chooseTarget(TargetPose& target);
    bool chooseRandomTarget(TargetPose& target);
    bool loadTargetSequence();
    void reloadLiveParams();
    bool handleShuttle(const ros::Time& now);
    bool publishShuttleLeg(double y_goal);
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
    bool shuttle_mode_{false};
    double shuttle_x_{0.0};
    double shuttle_y_min_{-2.0};
    double shuttle_y_max_{2.0};
    double shuttle_speed_{0.5};
    double shuttle_accel_{1.0};
    double shuttle_arrive_tol_{0.35};
    double shuttle_yaw_tol_{0.7};
    bool have_shuttle_goal_{false};
    bool shuttle_approaching_{false};
    double shuttle_goal_y_{0.0};
    ros::Time shuttle_plan_until_;
    double last_plan_duration_{0.0};

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
