#pragma once

#include <geometry_msgs/Twist.h>
#include <ros/time.h>

#include <cstdint>
#include <state_machine/state_machine.hpp>

namespace mecanum_ugv_controller {

struct ControllerConfig {
    double control_rate_hz{50.0};
    double state_timeout{0.2};
    double command_publish_rate_hz{50.0};
    bool placement_idle_silent{true};
    bool auto_start_tracking{false};
    double reference_timeout{0.5};
    double heading_target_yaw{0.0};
    double track_kp_yaw{1.2};
    double track_max_speed{0.8};
    double track_max_yaw_rate{0.6};
    double reset_timeout{45.0};
    double reset_arrive_position{0.40};
    double reset_arrive_yaw{0.60};
    double reset_settle_speed{0.08};
    double reset_settle_yaw_rate{0.12};
    double reset_kp_xy{0.8};
    double reset_kp_yaw{1.2};
    double reset_max_speed{0.5};
    double reset_max_yaw_rate{0.6};
    int reset_settle_frames{8};
};

struct UgvState {
    ros::Time stamp;
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double vx{0.0};
    double vy{0.0};
    double yaw_rate{0.0};
    bool received{false};
};

struct ControlCommand {
    ros::Time stamp;
    double linear_x{0.0};
    double linear_y{0.0};
    double angular_z{0.0};
    bool valid{false};
};

struct ResetTarget {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    bool valid{false};
};

struct WorldVelocityReference {
    ros::Time stamp;
    double vx{0.0};
    double vy{0.0};
    bool valid{false};
};

struct HolonomicResetOutput {
    double linear_x{0.0};
    double linear_y{0.0};
    double angular_z{0.0};
    bool position_ok{false};
    bool yaw_ok{false};
    bool settled{false};
};

struct HolonomicTrackOutput {
    double linear_x{0.0};
    double linear_y{0.0};
    double angular_z{0.0};
};

namespace state_type {
constexpr uint32_t HealthMonitor = 100;
constexpr uint32_t SelfCheck = 1;
constexpr uint32_t Ready = 2;
constexpr uint32_t Tracking = 3;
constexpr uint32_t Hold = 4;
constexpr uint32_t Reset = 5;
}  // namespace state_type

namespace region_type {
constexpr uint32_t HEALTH = 1;
constexpr uint32_t CONTROL = 2;
}  // namespace region_type

namespace event_type {
constexpr uint32_t TRACKING_REQUESTED = 1;
constexpr uint32_t HOLD_REQUESTED = 2;
constexpr uint32_t RESET_REQUESTED = 3;
constexpr uint32_t RESET_ARRIVED = 4;
constexpr uint32_t RESET_TIMEOUT = 5;
constexpr uint32_t INPUT_STATE_UPDATED = 20;
constexpr uint32_t INPUT_REFERENCE_UPDATED = 21;
constexpr uint32_t INPUT_REFERENCE_LOST = 22;
constexpr uint32_t INPUT_RESET_TARGET_UPDATED = 25;
constexpr uint32_t HEALTH_READY = 40;
constexpr uint32_t HEALTH_UNHEALTHY = 41;
}  // namespace event_type

namespace output_event_type {
constexpr uint32_t PUBLISH_CMD_VEL = 10001;
constexpr uint32_t PUBLISH_ZERO_CMD_VEL = 10002;
}  // namespace output_event_type

namespace transition_priority {
constexpr int COMMAND = 50;
constexpr int AUTOMATIC = 20;
}  // namespace transition_priority

double wrapAngle(double value);
double yawFromQuaternion(double x, double y, double z, double w);
bool tryYawFromQuaternion(double x, double y, double z, double w, double& yaw);
bool finiteState(const UgvState& state);
bool stateFresh(const UgvState& state, const ros::Time& now, double timeout);
double clamp(double value, double min_value, double max_value);
void worldVelocityToBody(double yaw, double v_wx, double v_wy, double& v_bx, double& v_by);
double headingRateToTarget(double yaw, double target_yaw, double kp_yaw, double max_yaw_rate);
HolonomicResetOutput computeHolonomicResetCommand(const UgvState& state, const ResetTarget& goal,
                                                  const ControllerConfig& config);
HolonomicTrackOutput computeHolonomicTrackCommand(const UgvState& state,
                                                  const WorldVelocityReference& reference,
                                                  const ControllerConfig& config);

}  // namespace mecanum_ugv_controller
