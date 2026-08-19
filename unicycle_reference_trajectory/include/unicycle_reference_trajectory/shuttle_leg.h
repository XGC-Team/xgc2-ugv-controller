#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace unicycle_reference_trajectory {

// On-rail reverse shuttle only. Off-rail: feasible SE2 plan to a rail entry pose.
struct ShuttleSample {
    double t{0.0};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double vx{0.0};
    double vy{0.0};
    double ax{0.0};
    double ay{0.0};
    double speed{0.0};
    double linear_acceleration{0.0};
    double yaw_rate{0.0};
};

struct ShuttleLegRequest {
    double start_y{0.0};
    double x_fixed{0.0};
    double y_goal{0.0};
    double desired_speed{0.5};
    double max_acceleration{1.0};
    double sample_dt{0.02};
    double hold_duration{0.3};
};

inline double shuttleYawAlongPlusY() {
    return 1.5707963267948966;
}

inline bool buildShuttleLeg(const ShuttleLegRequest& request, std::vector<ShuttleSample>& samples) {
    samples.clear();
    if (!std::isfinite(request.start_y) || !std::isfinite(request.x_fixed) ||
        !std::isfinite(request.y_goal)) {
        return false;
    }
    const double dt =
        std::isfinite(request.sample_dt) && request.sample_dt > 1.0e-4 ? request.sample_dt : 0.02;
    const double accel =
        std::isfinite(request.max_acceleration) && request.max_acceleration > 1.0e-3
            ? request.max_acceleration
            : 1.0;
    const double v_des = std::isfinite(request.desired_speed) && request.desired_speed > 1.0e-3
                             ? request.desired_speed
                             : 0.5;
    const double hold = std::isfinite(request.hold_duration) && request.hold_duration >= 0.0
                            ? request.hold_duration
                            : 0.0;
    const double yaw = shuttleYawAlongPlusY();
    const double dist = request.y_goal - request.start_y;
    const double dir = dist >= 0.0 ? 1.0 : -1.0;
    const double abs_dist = std::fabs(dist);

    double t_acc = v_des / accel;
    double v_peak = v_des;
    double t_cruise = 0.0;
    const double d_acc = 0.5 * accel * t_acc * t_acc;
    if (2.0 * d_acc >= abs_dist) {
        t_acc = std::sqrt(std::max(abs_dist / accel, 0.0));
        v_peak = accel * t_acc;
        t_cruise = 0.0;
    } else {
        t_cruise = (abs_dist - 2.0 * d_acc) / v_des;
    }
    const double t_move = 2.0 * t_acc + t_cruise;
    const double t_total = t_move + hold;
    const std::size_t count = static_cast<std::size_t>(std::ceil(t_total / dt)) + 1U;
    samples.reserve(count + 1U);

    for (std::size_t i = 0; i < count; ++i) {
        const double t = std::min(static_cast<double>(i) * dt, t_total);
        double y = request.y_goal;
        double vy = 0.0;
        double ay = 0.0;
        if (t <= t_move && abs_dist > 1.0e-6) {
            if (t < t_acc) {
                vy = dir * accel * t;
                y = request.start_y + dir * 0.5 * accel * t * t;
                ay = dir * accel;
            } else if (t < t_acc + t_cruise) {
                const double tau = t - t_acc;
                vy = dir * v_peak;
                y = request.start_y + dir * (0.5 * accel * t_acc * t_acc + v_peak * tau);
                ay = 0.0;
            } else {
                const double tau = t - t_acc - t_cruise;
                vy = dir * std::max(0.0, v_peak - accel * tau);
                const double remaining = t_acc - tau;
                y = request.y_goal - dir * 0.5 * accel * remaining * remaining;
                ay = -dir * accel;
            }
        }
        ShuttleSample sample;
        sample.t = t;
        sample.x = request.x_fixed;
        sample.y = y;
        sample.yaw = yaw;
        sample.vx = 0.0;
        sample.vy = vy;
        sample.ax = 0.0;
        sample.ay = ay;
        sample.speed = vy;
        sample.linear_acceleration = ay;
        sample.yaw_rate = 0.0;
        samples.push_back(sample);
    }
    if (samples.empty()) {
        return false;
    }
    ShuttleSample end = samples.back();
    end.t = t_total;
    end.x = request.x_fixed;
    end.y = request.y_goal;
    end.vy = 0.0;
    end.ay = 0.0;
    end.speed = 0.0;
    end.linear_acceleration = 0.0;
    samples.back() = end;
    return true;
}

inline bool shuttleOnRailX(double x, double rail_x, double pos_tol) {
    const double px = std::isfinite(pos_tol) && pos_tol > 0.0 ? pos_tol : 0.35;
    return std::isfinite(x) && std::isfinite(rail_x) && std::fabs(x - rail_x) <= px;
}

inline bool shuttleYawAlongRail(double yaw, double yaw_tol) {
    const double rail_yaw = shuttleYawAlongPlusY();
    const double dyaw = std::atan2(std::sin(yaw - rail_yaw), std::cos(yaw - rail_yaw));
    const double py = std::isfinite(yaw_tol) && yaw_tol > 0.0 ? yaw_tol : 0.7;
    return std::isfinite(yaw) && std::fabs(dyaw) <= py;
}

inline bool shuttleOnRail(double x, double yaw, double rail_x, double pos_tol, double yaw_tol) {
    return shuttleOnRailX(x, rail_x, pos_tol) && shuttleYawAlongRail(yaw, yaw_tol);
}

inline bool buildDirectApproach(double start_x, double start_y, double start_yaw, double goal_x,
                                double goal_y, double goal_yaw, double desired_speed,
                                double max_acceleration, double sample_dt, double hold_duration,
                                std::vector<ShuttleSample>& samples) {
    samples.clear();
    if (!std::isfinite(start_x) || !std::isfinite(start_y) || !std::isfinite(goal_x) ||
        !std::isfinite(goal_y)) {
        return false;
    }
    const double dt = std::isfinite(sample_dt) && sample_dt > 1.0e-4 ? sample_dt : 0.02;
    const double accel =
        std::isfinite(max_acceleration) && max_acceleration > 1.0e-3 ? max_acceleration : 1.0;
    const double v_des =
        std::isfinite(desired_speed) && desired_speed > 1.0e-3 ? desired_speed : 0.5;
    const double hold = std::isfinite(hold_duration) && hold_duration >= 0.0 ? hold_duration : 0.0;
    const double dx = goal_x - start_x;
    const double dy = goal_y - start_y;
    const double abs_dist = std::hypot(dx, dy);
    const double path_yaw = abs_dist > 1.0e-4 ? std::atan2(dy, dx) : goal_yaw;
    const double ux = abs_dist > 1.0e-4 ? dx / abs_dist : 0.0;
    const double uy = abs_dist > 1.0e-4 ? dy / abs_dist : 0.0;
    double t_acc = v_des / accel;
    double v_peak = v_des;
    double t_cruise = 0.0;
    const double d_acc = 0.5 * accel * t_acc * t_acc;
    if (2.0 * d_acc >= abs_dist) {
        t_acc = std::sqrt(std::max(abs_dist / accel, 0.0));
        v_peak = accel * t_acc;
        t_cruise = 0.0;
    } else {
        t_cruise = (abs_dist - 2.0 * d_acc) / v_des;
    }
    const double t_move = 2.0 * t_acc + t_cruise;
    const double t_total = t_move + hold;
    const std::size_t count = static_cast<std::size_t>(std::ceil(t_total / dt)) + 1U;
    samples.reserve(count + 1U);
    for (std::size_t i = 0; i < count; ++i) {
        const double t = std::min(static_cast<double>(i) * dt, t_total);
        double s = abs_dist;
        double v = 0.0;
        double a = 0.0;
        if (t <= t_move && abs_dist > 1.0e-6) {
            if (t < t_acc) {
                v = accel * t;
                s = 0.5 * accel * t * t;
                a = accel;
            } else if (t < t_acc + t_cruise) {
                v = v_peak;
                s = d_acc + v_peak * (t - t_acc);
                a = 0.0;
            } else {
                const double tau = t - t_acc - t_cruise;
                v = std::max(0.0, v_peak - accel * tau);
                const double remaining = t_acc - tau;
                s = abs_dist - 0.5 * accel * remaining * remaining;
                a = -accel;
            }
        }
        ShuttleSample sample;
        sample.t = t;
        sample.x = start_x + ux * s;
        sample.y = start_y + uy * s;
        sample.yaw = t + 1.0e-6 >= t_move ? goal_yaw : path_yaw;
        sample.vx = ux * v;
        sample.vy = uy * v;
        sample.ax = ux * a;
        sample.ay = uy * a;
        sample.speed = v;
        sample.linear_acceleration = a;
        sample.yaw_rate = 0.0;
        samples.push_back(sample);
    }
    if (samples.empty()) {
        return false;
    }
    ShuttleSample end = samples.back();
    end.t = t_total;
    end.x = goal_x;
    end.y = goal_y;
    end.yaw = goal_yaw;
    end.vx = 0.0;
    end.vy = 0.0;
    end.ax = 0.0;
    end.ay = 0.0;
    end.speed = 0.0;
    end.linear_acceleration = 0.0;
    samples.back() = end;
    return true;
}

inline double shuttleHeadingTowardRail(double x, double rail_x) {
    const double dx = rail_x - x;
    if (std::fabs(dx) < 1.0e-3) {
        return shuttleYawAlongPlusY();
    }
    return dx > 0.0 ? 0.0 : 3.141592653589793;
}

inline void shuttleEntryPose(double /*start_x*/, double start_y, double rail_x, double y_min,
                             double y_max, double& entry_x, double& entry_y) {
    const double lo = std::min(y_min, y_max);
    const double hi = std::max(y_min, y_max);
    entry_x = rail_x;
    if (start_y < lo) {
        entry_y = lo;
    } else if (start_y > hi) {
        entry_y = hi;
    } else {
        entry_y = start_y;
    }
}

inline double shuttleDefaultCaptureRadius() {
    return 30.0;
}

// Planar distance to the finite rail segment x=rail_x, y in [y_min, y_max].
inline double shuttleDistanceToRail(double x, double y, double rail_x, double y_min, double y_max) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(rail_x) || !std::isfinite(y_min) ||
        !std::isfinite(y_max)) {
        return std::numeric_limits<double>::infinity();
    }
    const double lo = std::min(y_min, y_max);
    const double hi = std::max(y_min, y_max);
    if (y < lo) {
        return std::hypot(x - rail_x, y - lo);
    }
    if (y > hi) {
        return std::hypot(x - rail_x, y - hi);
    }
    return std::fabs(x - rail_x);
}

inline bool shuttleWithinCapture(double x, double y, double rail_x, double y_min, double y_max,
                                 double radius) {
    const double limit =
        std::isfinite(radius) && radius > 0.0 ? radius : shuttleDefaultCaptureRadius();
    return shuttleDistanceToRail(x, y, rail_x, y_min, y_max) <= limit;
}

// Geometric SE2 entry: always succeeds for finite poses inside the capture disk.
inline bool planShuttleEntry(double start_x, double start_y, double start_yaw, double rail_x,
                             double y_min, double y_max, double desired_speed,
                             double max_acceleration, double sample_dt, double hold_duration,
                             double capture_radius, std::vector<ShuttleSample>& samples,
                             double& entry_x, double& entry_y) {
    samples.clear();
    if (!shuttleWithinCapture(start_x, start_y, rail_x, y_min, y_max, capture_radius)) {
        return false;
    }
    shuttleEntryPose(start_x, start_y, rail_x, y_min, y_max, entry_x, entry_y);
    return buildDirectApproach(start_x, start_y, start_yaw, entry_x, entry_y,
                               shuttleYawAlongPlusY(), desired_speed, max_acceleration, sample_dt,
                               hold_duration, samples);
}

// Arrival is planar hypot(Δx, Δy), not |Δy| alone. Entry target is a point on the rail.
inline bool shuttleArrived(double x, double y, double rail_x, double goal_y, double tol) {
    const double limit = std::isfinite(tol) && tol > 0.0 ? tol : 0.35;
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(rail_x) && std::isfinite(goal_y) &&
           std::hypot(x - rail_x, y - goal_y) <= limit;
}

// Off-rail → feasible SE2 pose-to-entry. On-rail +Y → reverse shuttle.
enum class ShuttleTrackMode { FeasibleApproach, ReverseRail };

inline ShuttleTrackMode shuttleTrackMode(double x, double yaw, double rail_x, double pos_tol,
                                         double yaw_tol) {
    return shuttleOnRail(x, yaw, rail_x, pos_tol, yaw_tol) ? ShuttleTrackMode::ReverseRail
                                                           : ShuttleTrackMode::FeasibleApproach;
}

inline double nextShuttleGoalY(double current_y, double y_min, double y_max, bool have_goal,
                               double current_goal_y) {
    const double lo = std::min(y_min, y_max);
    const double hi = std::max(y_min, y_max);
    if (have_goal) {
        const double mid = 0.5 * (lo + hi);
        return current_goal_y <= mid ? hi : lo;
    }
    const double mid = 0.5 * (lo + hi);
    return current_y <= mid ? hi : lo;
}

// Keep the same end while on-rail. Flip after planar arrival. Off-rail timeout
// replans the entry; reverse motion is only for an on-rail pose.
inline double shuttleGoalYForReplan(bool have_goal, bool arrived, double current_y, double y_min,
                                    double y_max, double current_goal_y) {
    if (have_goal && !arrived) {
        return current_goal_y;
    }
    return nextShuttleGoalY(current_y, y_min, y_max, have_goal, current_goal_y);
}

// Replan on arrival. Approach timeout starts rail motion. A live plan is not interrupted.
enum class ShuttleReplanReason { Keep, FirstPlan, Arrived, TimedOut };

inline bool shuttleBeginReverseMotion(bool on_rail, bool /*have_goal*/, bool /*approaching*/,
                                      ShuttleReplanReason /*reason*/) {
    return on_rail;
}

inline ShuttleReplanReason shuttleReplanReason(bool have_goal, bool arrived, bool plan_alive) {
    if (!have_goal) {
        return ShuttleReplanReason::FirstPlan;
    }
    if (arrived) {
        return ShuttleReplanReason::Arrived;
    }
    if (!plan_alive) {
        return ShuttleReplanReason::TimedOut;
    }
    return ShuttleReplanReason::Keep;
}

}  // namespace unicycle_reference_trajectory
