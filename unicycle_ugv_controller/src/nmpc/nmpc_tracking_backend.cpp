#include "unicycle_ugv_controller/nmpc/nmpc_tracking_backend.h"

#include <cmath>

namespace unicycle_ugv_controller {

void NmpcTrackingBackend::configure(const ControllerConfig& config) {
    config_ = config;
}

bool NmpcTrackingBackend::enter() {
    entered_ = solver_.initialize();
    if (entered_) {
        solver_.resetWarmStart();
    }
    return entered_;
}

void NmpcTrackingBackend::exit() {
    entered_ = false;
    solver_.resetWarmStart();
}

bool NmpcTrackingBackend::compute(const UgvState& state, const std::vector<Se2Reference>& refs,
                                  const ros::Time& now, ControlCommand& command) {
    if (!entered_ && !enter()) {
        status_ = -100;
        return false;
    }
    control::Se2State feedback;
    feedback.position << state.x, state.y;
    feedback.yaw = state.yaw;
    feedback.linear_speed = state.speed;
    const Se2StateVector x0 = control::packState(feedback);
    const bool ok = solver_.solve(x0, refs);
    status_ = solver_.status();
    solve_time_ms_ = solver_.solveTimeMs();
    if (!ok) {
        return false;
    }
    const auto u0 = solver_.optimalControl();
    const double speed = solver_.predictedSpeed();
    if (!std::isfinite(speed) || !std::isfinite(u0(1))) {
        status_ = -101;
        return false;
    }
    command.stamp = now;
    command.linear_speed = clamp(speed, config_.min_linear_speed, config_.max_linear_speed);
    command.angular_speed = clamp(u0(1), -config_.max_angular_speed, config_.max_angular_speed);
    command.valid = true;
    return true;
}

}  // namespace unicycle_ugv_controller
