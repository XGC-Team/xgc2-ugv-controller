#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {

namespace config_detail {

inline double finitePositiveOr(double value, double fallback) {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}

}  // namespace config_detail

// Reads the controller's parameters (config/unicycle_ugv_controller.yaml) from
// any configuration source: the ROS parameter server in the node, a JSON object
// in the aggregator module. Keys are '/'-separated paths of nested mappings. A
// key that is absent keeps the value already in `config`, so its defaults are
// the defaults of both transports. `Source` provides
//   bool get(const std::string& key, double& value) const;       number (integer or real)
//   bool get(const std::string& key, bool& value) const;
//   bool get(const std::string& key, std::string& value) const;
//   bool has(const std::string& key) const;                       anything is present at key
//   bool isMapping(const std::string& key) const;                 a mapping is present at key
//   std::vector<std::string> keys(const std::string& key) const;  the names in that mapping
// where get() returns true and sets `value` only when the key holds that type.
// Throws std::invalid_argument for a value the controller cannot run with.
template <typename Source>
void loadControllerConfig(const Source& source, ControllerConfig& config) {
    using config_detail::finitePositiveOr;

    std::string state_source{config.state_source == StateSource::PLATFORM_POSE ? "platform_pose"
                                                                               : "state_estimator"};
    source.get("state_source", state_source);
    if (state_source == "platform_pose") {
        config.state_source = StateSource::PLATFORM_POSE;
    } else if (state_source == "state_estimator") {
        config.state_source = StateSource::STATE_ESTIMATOR;
    } else {
        throw std::invalid_argument("Unknown state_source: " + state_source);
    }
    std::string strategy{config.tracking_strategy == TrackingStrategy::FLATNESS ? "flatness"
                                                                                : "nmpc"};
    source.get("tracking_strategy", strategy);
    if (strategy == "flatness") {
        config.tracking_strategy = TrackingStrategy::FLATNESS;
    } else if (strategy == "nmpc") {
        config.tracking_strategy = TrackingStrategy::NMPC;
    } else {
        throw std::invalid_argument("Unknown tracking_strategy: " + strategy);
    }

    const std::string recovery = "flatness/heading_recovery";
    if (source.has(recovery)) {
        if (!source.isMapping(recovery)) {
            throw std::invalid_argument(recovery + " must be a mapping");
        }
        for (const std::string& name : source.keys(recovery)) {
            double* field = nullptr;
            if (name == "gain") {
                field = &config.heading_recovery.gain;
            } else if (name == "axis_bias") {
                field = &config.heading_recovery.axis_bias;
            } else if (name == "rate_damping") {
                field = &config.heading_recovery.rate_damping;
            }
            if (field == nullptr) {
                throw std::invalid_argument("unknown " + recovery + "/" + name);
            }
            if (!source.get(recovery + "/" + name, *field)) {
                throw std::invalid_argument("heading recovery parameter must be numeric: " + name);
            }
        }
    }
    config.heading_recovery.validate();
    if (config.heading_recovery.gain > 0.0 &&
        config.tracking_strategy != TrackingStrategy::FLATNESS) {
        throw std::invalid_argument("heading_recovery is available only for flatness tracking");
    }

    source.get("control_rate_hz", config.control_rate_hz);
    source.get("nmpc/control_period", config.control_period);
    source.get("nmpc/prediction_horizon", config.prediction_horizon);
    source.get("state_timeout", config.state_timeout);
    source.get("nmpc/solve_timeout", config.solve_timeout);
    source.get("nmpc/result_timeout", config.result_timeout);
    source.get("command_publish_rate_hz", config.command_publish_rate_hz);
    source.get("idle_cmd_rate_hz", config.idle_cmd_rate_hz);
    source.get("auto_start_tracking", config.auto_start_tracking);
    source.get("reset/timeout", config.reset_timeout);
    source.get("nmpc/request_rate_hz", config.nmpc_request_rate_hz);
    source.get("limits/max_linear_speed", config.max_linear_speed);
    source.get("limits/min_linear_speed", config.min_linear_speed);
    source.get("limits/max_angular_speed", config.max_angular_speed);
    source.get("limits/max_linear_acceleration", config.max_linear_acceleration);
    source.get("limits/max_angular_acceleration", config.max_angular_acceleration);
    source.get("chassis/max_linear_speed", config.chassis_max_linear_speed);
    source.get("chassis/max_yaw_rate", config.chassis_max_yaw_rate);
    source.get("flatness/kp", config.flatness_kp);
    source.get("flatness/kv", config.flatness_kv);
    source.get("flatness/lateral_response_length", config.flatness_lateral_response_length);
    source.get("flatness/lateral_damping", config.flatness_lateral_damping);
    source.get("flatness/v_eps", config.flatness_v_eps);
    source.get("filter/zeta", config.filter_zeta);
    source.get("filter/wn", config.filter_wn);
    source.get("filter/dt_min", config.velocity_dt_min);
    source.get("filter/dt_max", config.velocity_dt_max);
    source.get("fence/x_min", config.fence_x_min);
    source.get("fence/x_max", config.fence_x_max);
    source.get("fence/y_min", config.fence_y_min);
    source.get("fence/y_max", config.fence_y_max);
    source.get("reset_initial_x", config.reset_initial_x);
    source.get("reset_initial_y", config.reset_initial_y);
    source.get("reset_initial_yaw", config.reset_initial_yaw);
    source.get("nmpc/weights/position_x", config.nmpc_weights.position_x);
    source.get("nmpc/weights/position_y", config.nmpc_weights.position_y);
    source.get("nmpc/weights/yaw", config.nmpc_weights.yaw);
    source.get("nmpc/weights/speed", config.nmpc_weights.speed);
    source.get("nmpc/weights/accel", config.nmpc_weights.accel);
    source.get("nmpc/weights/omega", config.nmpc_weights.omega);
    source.get("nmpc/weights/angular_accel", config.nmpc_weights.angular_accel);
    source.get("nmpc/weights/terminal_position_x", config.nmpc_weights.terminal_position_x);
    source.get("nmpc/weights/terminal_position_y", config.nmpc_weights.terminal_position_y);
    source.get("nmpc/weights/terminal_yaw", config.nmpc_weights.terminal_yaw);
    source.get("nmpc/weights/terminal_speed", config.nmpc_weights.terminal_speed);

    config.control_rate_hz = finitePositiveOr(config.control_rate_hz, 500.0);
    config.control_period = finitePositiveOr(config.control_period, 0.1);
    config.prediction_horizon = finitePositiveOr(config.prediction_horizon, 1.0);
    config.state_timeout = finitePositiveOr(config.state_timeout, 0.5);
    config.solve_timeout = finitePositiveOr(config.solve_timeout, 0.05);
    config.result_timeout = finitePositiveOr(config.result_timeout, 0.1);
    config.command_publish_rate_hz = finitePositiveOr(config.command_publish_rate_hz, 30.0);
    config.idle_cmd_rate_hz = finitePositiveOr(config.idle_cmd_rate_hz, 5.0);
    config.nmpc_request_rate_hz = finitePositiveOr(config.nmpc_request_rate_hz, 100.0);
    config.max_linear_speed = finitePositiveOr(config.max_linear_speed, 3.0);
    if (!std::isfinite(config.min_linear_speed) ||
        config.min_linear_speed >= config.max_linear_speed) {
        config.min_linear_speed = -1.5;
    }
    config.max_angular_speed = finitePositiveOr(config.max_angular_speed, 2.5);
    config.max_linear_acceleration = finitePositiveOr(config.max_linear_acceleration, 2.0);
    config.max_angular_acceleration = finitePositiveOr(config.max_angular_acceleration, 3.0);
    config.chassis_max_linear_speed = finitePositiveOr(config.chassis_max_linear_speed, 1.05);
    config.chassis_max_yaw_rate = finitePositiveOr(config.chassis_max_yaw_rate, 1.05);
    if (!std::isfinite(config.flatness_lateral_response_length) ||
        config.flatness_lateral_response_length <= 0.0 ||
        !std::isfinite(config.flatness_lateral_damping) || config.flatness_lateral_damping <= 0.0) {
        throw std::invalid_argument(
            "flatness lateral response length and damping must be positive and finite");
    }
    config.flatness_kp = finitePositiveOr(config.flatness_kp, 6.0);
    config.flatness_kv = finitePositiveOr(config.flatness_kv, 4.0);
    config.flatness_v_eps = finitePositiveOr(config.flatness_v_eps, 0.15);
    config.filter_zeta = finitePositiveOr(config.filter_zeta, 0.7071067811865476);
    config.filter_wn = finitePositiveOr(config.filter_wn, 31.41592653589793);
    config.velocity_dt_min = finitePositiveOr(config.velocity_dt_min, 1.0e-4);
    config.velocity_dt_max = finitePositiveOr(config.velocity_dt_max, 0.2);
    config.nmpc_weights.position_x = finitePositiveOr(config.nmpc_weights.position_x, 20.0);
    config.nmpc_weights.position_y = finitePositiveOr(config.nmpc_weights.position_y, 20.0);
    config.nmpc_weights.yaw = finitePositiveOr(config.nmpc_weights.yaw, 8.0);
    config.nmpc_weights.speed = finitePositiveOr(config.nmpc_weights.speed, 4.0);
    config.nmpc_weights.accel = finitePositiveOr(config.nmpc_weights.accel, 0.4);
    config.nmpc_weights.omega = finitePositiveOr(config.nmpc_weights.omega, 10.0);
    config.nmpc_weights.angular_accel = finitePositiveOr(config.nmpc_weights.angular_accel, 1.0);
    config.nmpc_weights.terminal_position_x =
        finitePositiveOr(config.nmpc_weights.terminal_position_x, 60.0);
    config.nmpc_weights.terminal_position_y =
        finitePositiveOr(config.nmpc_weights.terminal_position_y, 60.0);
    config.nmpc_weights.terminal_yaw = finitePositiveOr(config.nmpc_weights.terminal_yaw, 20.0);
    config.nmpc_weights.terminal_speed = finitePositiveOr(config.nmpc_weights.terminal_speed, 10.0);
    config.reset_timeout = finitePositiveOr(config.reset_timeout, 90.0);
}

// The Reset target a configuration seeds (reset_initial_x/y/yaw), valid when all three are finite.
inline ResetTarget initialResetTarget(const ControllerConfig& config) {
    ResetTarget target;
    target.x = config.reset_initial_x;
    target.y = config.reset_initial_y;
    target.yaw = wrapAngle(config.reset_initial_yaw);
    target.valid = std::isfinite(target.x) && std::isfinite(target.y) && std::isfinite(target.yaw);
    return target;
}

}  // namespace unicycle_ugv_controller
