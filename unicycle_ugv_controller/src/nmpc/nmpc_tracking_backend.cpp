#include "unicycle_ugv_controller/nmpc/nmpc_tracking_backend.h"

#include <ros/console.h>

#include <cmath>
#include <limits>

#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {
namespace {

bool finiteStateVector(const Se2StateVector& state) {
    return state.array().isFinite().all();
}

bool initialConstraintSatisfied(const std::array<Se2StateVector, UNICYCLE_NMPC_N + 1>& states,
                                size_t count, const Se2StateVector& x0, double& position_error,
                                double& yaw_error, double& speed_error) {
    if (count == 0U || !finiteStateVector(x0) || !finiteStateVector(states[0])) {
        position_error = std::numeric_limits<double>::quiet_NaN();
        yaw_error = std::numeric_limits<double>::quiet_NaN();
        speed_error = std::numeric_limits<double>::quiet_NaN();
        return false;
    }

    position_error = (states[0].head<2>() - x0.head<2>()).norm();
    yaw_error = std::abs(wrapAngle(states[0](2) - x0(2)));
    speed_error = std::abs(states[0](3) - x0(3));
    constexpr double kInitialConstraintTolerance = 1.0e-5;
    return position_error <= kInitialConstraintTolerance &&
           yaw_error <= kInitialConstraintTolerance && speed_error <= kInitialConstraintTolerance;
}

}  // namespace

void NmpcTrackingBackend::configure(const ControllerConfig& config) {
    config_ = config;
    bounds_configured_ = solver_.configureBounds(
        config_.min_linear_speed, config_.max_linear_speed,
        config_.max_linear_acceleration, config_.max_angular_speed);
    if (!bounds_configured_) {
        status_ = -103;
    }
}

bool NmpcTrackingBackend::enter() {
    if (!bounds_configured_) {
        status_ = -103;
        return false;
    }
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
    double initial_position_error = 0.0;
    double initial_yaw_error = 0.0;
    double initial_speed_error = 0.0;
    if (!initialConstraintSatisfied(solver_.predictedStates(), solver_.predictedStateCount(), x0,
                                    initial_position_error, initial_yaw_error,
                                    initial_speed_error)) {
        status_ = -102;
        solver_.resetWarmStart();
        ROS_ERROR_THROTTLE(
            1.0,
            "[UgvNmpcBackend] Reject NMPC solution because stage-0 equality constraint "
            "is not satisfied: pos=%.9f yaw=%.9f speed=%.9f",
            initial_position_error, initial_yaw_error, initial_speed_error);
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
