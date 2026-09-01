#pragma once

#include <geometry_msgs/Twist.h>
#include <ros/time.h>

#include <Eigen/Dense>
#include <cstdint>
#include <state_machine/state_machine.hpp>

namespace unicycle_ugv_controller {

enum class StateSource {
    STATE_ESTIMATOR = 0,
    VRPN_DIRECT = 1,
    PLATFORM_POSE = 2,
};

struct NmpcCostWeights {
    double position_x{20.0};
    double position_y{20.0};
    double yaw{8.0};
    double speed{4.0};
    double accel{0.4};
    double omega{10.0};
    double angular_accel{1.0};
    double terminal_position_x{60.0};
    double terminal_position_y{60.0};
    double terminal_yaw{20.0};
    double terminal_speed{10.0};
};

inline bool operator==(const NmpcCostWeights& lhs, const NmpcCostWeights& rhs) {
    return lhs.position_x == rhs.position_x && lhs.position_y == rhs.position_y &&
           lhs.yaw == rhs.yaw && lhs.speed == rhs.speed && lhs.accel == rhs.accel &&
           lhs.omega == rhs.omega && lhs.angular_accel == rhs.angular_accel &&
           lhs.terminal_position_x == rhs.terminal_position_x &&
           lhs.terminal_position_y == rhs.terminal_position_y &&
           lhs.terminal_yaw == rhs.terminal_yaw && lhs.terminal_speed == rhs.terminal_speed;
}

inline bool operator!=(const NmpcCostWeights& lhs, const NmpcCostWeights& rhs) {
    return !(lhs == rhs);
}

struct ControllerConfig {
    double control_rate_hz{100.0};
    double control_period{0.1};
    double prediction_horizon{1.0};
    double state_timeout{0.2};
    double reference_timeout{0.5};
    double solve_timeout{0.05};
    double result_timeout{0.1};
    double command_publish_rate_hz{100.0};
    double nmpc_request_rate_hz{100.0};
    bool auto_start_tracking{false};
    bool placement_idle_silent{false};
    StateSource state_source{StateSource::STATE_ESTIMATOR};
    double reset_timeout{45.0};
    double reset_arrive_position{0.40};
    double reset_arrive_yaw{0.60};
    double reset_settle_speed{0.08};
    double reset_settle_yaw_rate{0.12};
    double reset_kp_along{0.8};
    double reset_kp_heading{1.2};
    double reset_max_speed{0.6};
    double reset_max_yaw_rate{0.8};
    int reset_settle_frames{8};
    double max_linear_speed{3.0};
    double min_linear_speed{-1.5};
    double max_angular_speed{2.5};
    double max_linear_acceleration{2.0};
    double max_angular_acceleration{3.0};
    NmpcCostWeights nmpc_weights{};
};

struct UgvState {
    ros::Time stamp;
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double speed{0.0};
    double yaw_rate{0.0};
    uint8_t estimator_state{0U};
    uint32_t estimator_flags{0U};
    bool received{false};
};

struct ControlCommand {
    ros::Time stamp;
    double linear_speed{0.0};
    double angular_speed{0.0};
    bool valid{false};
};

struct ResetTarget {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    bool valid{false};
};

struct UnicycleResetOutput {
    double linear_speed{0.0};
    double angular_speed{0.0};
    bool position_ok{false};
    bool yaw_ok{false};
    bool settled{false};
};

UnicycleResetOutput computeUnicycleResetCommand(const UgvState& state, const ResetTarget& goal,
                                                const ControllerConfig& config);

struct NmpcSolveResult {
    uint64_t sequence{0U};
    ros::Time stamp;
    bool success{false};
    int solver_status{-1};
    double solve_time_ms{0.0};
    ControlCommand command;
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
constexpr uint32_t INPUT_RESET_TARGET_UPDATED = 25;
constexpr uint32_t INPUT_REFERENCE_UPDATED = 21;
constexpr uint32_t INPUT_REFERENCE_LOST = 22;
constexpr uint32_t INPUT_NMPC_SOLVE_SUCCEEDED = 23;
constexpr uint32_t INPUT_NMPC_SOLVE_FAILED = 24;
constexpr uint32_t HEALTH_READY = 40;
constexpr uint32_t HEALTH_UNHEALTHY = 41;
}  // namespace event_type

namespace output_event_type {
constexpr uint32_t REQUEST_NMPC_SOLVE = 10000;
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

}  // namespace unicycle_ugv_controller
