#pragma once

#include <algorithm>
#include <cmath>

namespace unicycle_ugv_controller {
namespace flatness {

// Optional diagnostics from the SAME evaluation as the actuator command.
// World x/y are metres; world vx/vy and signed_speed are m/s; yaw is rad.
// Command integrator is an independent m/s state, never a measured velocity.
struct Trace {
    double signed_speed = 0.0;
    double transverse_bandwidth = 0.0;  // 1/s
    double lateral_position_error = 0.0;  // m
    double lateral_velocity_error = 0.0;  // m/s
    double lateral_accel = 0.0;  // m/s^2
    double requested_linear_speed = 0.0;  // m/s, before saturation
    double requested_angular_speed = 0.0;  // rad/s, before saturation
    bool valid = false;
};

// Duck-typed boundary keeps this exact runtime kernel usable without ROS,
// messages, an estimator implementation, or a substitute tracking law.
// Validation and operation ordering are preserved from common_types.cpp at
// 479e1f01ef6a2e5f264ff8abed5502f7a9d6f7a4.
template <typename Output, typename State, typename Reference, typename Config>
Output evaluate(const State& state, const Reference& reference,
                double command_speed, double dt, const Config& config,
                Trace* trace = nullptr) {
    if (trace) *trace = Trace{};
    Output output;
    const bool finite_pose = std::isfinite(state.x) && std::isfinite(state.y) &&
                             std::isfinite(state.yaw);
    const bool ready = reference.valid && std::isfinite(reference.x) &&
                       std::isfinite(reference.y) && std::isfinite(reference.vx) &&
                       std::isfinite(reference.vy) && std::isfinite(reference.ax) &&
                       std::isfinite(reference.ay);
    if (!finite_pose || !ready || !std::isfinite(command_speed) ||
        !std::isfinite(state.vx) || !std::isfinite(state.vy) || !std::isfinite(dt) ||
        dt <= config.velocity_dt_min || dt > config.velocity_dt_max || !state.velocity_valid) {
        return output;
    }
    const double estimated_speed = std::cos(state.yaw) * state.vx + std::sin(state.yaw) * state.vy;
    if (!std::isfinite(estimated_speed) || !std::isfinite(config.flatness_kp) ||
        !std::isfinite(config.flatness_kv) || !std::isfinite(config.flatness_v_eps) ||
        config.flatness_v_eps <= 0.0 || !std::isfinite(config.chassis_max_linear_speed) ||
        config.chassis_max_linear_speed <= 0.0 || !std::isfinite(config.chassis_max_yaw_rate) ||
        config.chassis_max_yaw_rate <= 0.0) {
        return output;
    }
    const double ux = reference.ax + config.flatness_kv * (reference.vx - state.vx) +
                      config.flatness_kp * (reference.x - state.x);
    const double uy = reference.ay + config.flatness_kv * (reference.vy - state.vy) +
                      config.flatness_kp * (reference.y - state.y);
    const double c = std::cos(state.yaw);
    const double s = std::sin(state.yaw);
    output.accel = c * ux + s * uy;
    const double v_eps = std::max(config.flatness_v_eps, 1.0e-6);
    const double length = config.flatness_lateral_response_length;
    const double damping = config.flatness_lateral_damping;
    if (!std::isfinite(length) || length <= 0.0 || !std::isfinite(damping) || damping <= 0.0) {
        return output;
    }
    const double bandwidth = std::fabs(estimated_speed) / length;
    const double lateral_position_error =
        -s * (reference.x - state.x) + c * (reference.y - state.y);
    const double lateral_velocity_error =
        -s * (reference.vx - state.vx) + c * (reference.vy - state.vy);
    const double lateral_accel = -s * reference.ax + c * reference.ay +
                                 2.0 * damping * bandwidth * lateral_velocity_error +
                                 bandwidth * bandwidth * lateral_position_error;
    // One continuous low-speed law, unchanged. At rest the inverse is zero.
    // Do not substitute command_speed or add a recovery mode here.
    output.angular_speed =
        estimated_speed * lateral_accel / (estimated_speed * estimated_speed + v_eps * v_eps);
    output.linear_speed = command_speed + output.accel * dt;
    if (!std::isfinite(output.accel) || !std::isfinite(output.linear_speed) ||
        !std::isfinite(output.angular_speed)) {
        return Output{};
    }
    if (trace) {
        trace->signed_speed = estimated_speed;
        trace->transverse_bandwidth = bandwidth;
        trace->lateral_position_error = lateral_position_error;
        trace->lateral_velocity_error = lateral_velocity_error;
        trace->lateral_accel = lateral_accel;
        trace->requested_linear_speed = output.linear_speed;
        trace->requested_angular_speed = output.angular_speed;
    }
    output.linear_speed = std::max(-config.chassis_max_linear_speed,
                                   std::min(config.chassis_max_linear_speed, output.linear_speed));
    output.angular_speed = std::max(-config.chassis_max_yaw_rate,
                                    std::min(config.chassis_max_yaw_rate, output.angular_speed));
    output.valid = std::isfinite(output.linear_speed) && std::isfinite(output.angular_speed);
    if (trace) trace->valid = output.valid;
    return output;
}

}  // namespace flatness
}  // namespace unicycle_ugv_controller
