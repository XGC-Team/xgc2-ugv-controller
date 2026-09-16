#pragma once

#include <ugv_reset_safety/reset_path.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ugv_reset_safety {

enum class ScheduleStatus {
    Uninitialized,
    Ready,
    Complete,
    InvalidInput,
    ConflictingTargets,
    OccupiedTarget,
    UnsupportedCoordination
};

struct ScheduleResult {
    ScheduleStatus status{ScheduleStatus::Uninitialized};
    std::vector<std::size_t> selected;
    std::string detail;
    bool ok() const {
        return status == ScheduleStatus::Ready || status == ScheduleStatus::Complete;
    }
};

inline double resetPathCross(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return a.x() * b.y() - a.y() * b.x();
}

// Interior segment crossing, or collinear opposite-direction overlap (a swap).
// Same-direction convoys that only share an endpoint are allowed in one wave.
inline bool resetPathsConflict(const Eigen::Vector2d& a0, const Eigen::Vector2d& a1,
                               const Eigen::Vector2d& b0, const Eigen::Vector2d& b1) {
    const Eigen::Vector2d da = a1 - a0;
    const Eigen::Vector2d db = b1 - b0;
    const double a_norm = da.norm();
    const double b_norm = db.norm();
    if (a_norm < 1e-6 || b_norm < 1e-6) {
        return false;
    }
    const double den = resetPathCross(da, db);
    if (std::abs(den) > 1e-6 * a_norm * b_norm) {
        const Eigen::Vector2d r = b0 - a0;
        const double s = resetPathCross(r, db) / den;
        const double t = resetPathCross(r, da) / den;
        return s > 1e-4 && s < 1.0 - 1e-4 && t > 1e-4 && t < 1.0 - 1e-4;
    }
    if (da.dot(db) >= 0.0) {
        return false;
    }
    const double a2 = da.squaredNorm();
    double t0 = (b0 - a0).dot(da) / a2;
    double t1 = (b1 - a0).dot(da) / a2;
    if (t0 > t1) {
        std::swap(t0, t1);
    }
    return t0 < 1.0 - 1e-4 && t1 > 1e-4;
}

// Batch admission for one Reset click. Target-overlap and a nonparticipant on a
// requested goal still reject before motion. Crossing start→goal segments are
// split into successive waves so a 一字 swap does not launch every robot at once.
// Non-requesting robots stay parked obstacles.
//
// A completion bit means BOTH original target arrival AND measured stop, not
// merely a zero desired input. This does not prove geometric route existence,
// safe continuous execution, deadlock freedom, or arbitrary fleet counts.
class FleetSchedule {
   public:
    explicit FleetSchedule(double clearance = 0.1) : clearance_(clearance) {}

    void clear() {
        result_ = ScheduleResult();
        groups_.clear();
        completed_.clear();
        roster_size_ = 0;
        group_ = 0;
    }

    ScheduleResult initialize(const std::vector<Robot>& robots,
                              const std::vector<ResetTarget>& targets) {
        clear();
        roster_size_ = robots.size();
        if (robots.size() != targets.size() || !std::isfinite(clearance_) || clearance_ < 0.0) {
            return reject(ScheduleStatus::InvalidInput, "Invalid scheduler roster or clearance");
        }
        std::set<std::string> identifiers;
        std::vector<std::size_t> requesting;
        std::vector<double> radius(robots.size(), 0.0);
        for (std::size_t i = 0; i < robots.size(); ++i) {
            const auto& robot = robots[i];
            if (robot.id.empty() || !identifiers.insert(robot.id).second ||
                !robot.position.allFinite() || !std::isfinite(robot.yaw) ||
                !robot.body_center_offset.allFinite() || !std::isfinite(robot.half_length) ||
                !std::isfinite(robot.half_width) || robot.half_length <= 0.0 ||
                robot.half_width <= 0.0) {
                return reject(ScheduleStatus::InvalidInput,
                              "Invalid scheduler robot geometry or ID");
            }
            radius[i] =
                std::hypot(robot.half_length, robot.half_width) + robot.body_center_offset.norm();
            if (robot.active) {
                if (!targets[i].position.allFinite() || !std::isfinite(targets[i].yaw)) {
                    return reject(ScheduleStatus::InvalidInput, "Invalid requested reset target");
                }
                requesting.push_back(i);
            }
        }
        auto lexical = [&](std::size_t left, std::size_t right) {
            return robots[left].id < robots[right].id;
        };
        std::sort(requesting.begin(), requesting.end(), lexical);
        for (std::size_t position = 0; position < requesting.size(); ++position) {
            const auto i = requesting[position];
            for (std::size_t other = position + 1; other < requesting.size(); ++other) {
                const auto j = requesting[other];
                if ((targets[i].position - targets[j].position).norm() <=
                    radius[i] + radius[j] + clearance_) {
                    return reject(ScheduleStatus::ConflictingTargets,
                                  "Requested target footprint clearances overlap: " + robots[i].id +
                                      ", " + robots[j].id);
                }
            }
            for (std::size_t j = 0; j < robots.size(); ++j) {
                if (i == j || (targets[i].position - robots[j].position).norm() >
                                  radius[i] + radius[j] + clearance_) {
                    continue;
                }
                if (!robots[j].active) {
                    return reject(
                        ScheduleStatus::OccupiedTarget,
                        "Requested target occupied by stationary nonparticipant: " + robots[j].id);
                }
            }
        }
        for (const auto i : requesting) {
            bool placed = false;
            for (auto& wave : groups_) {
                bool ok = true;
                for (const auto j : wave) {
                    if (resetPathsConflict(robots[i].position, targets[i].position,
                                           robots[j].position, targets[j].position)) {
                        ok = false;
                        break;
                    }
                }
                if (ok) {
                    wave.push_back(i);
                    placed = true;
                    break;
                }
            }
            if (!placed) {
                groups_.push_back({i});
            }
        }
        completed_.assign(robots.size(), false);
        result_.status = groups_.empty() ? ScheduleStatus::Complete : ScheduleStatus::Ready;
        return select(std::vector<bool>(robots.size(), false));
    }

    ScheduleResult select(const std::vector<bool>& arrived_and_stopped) {
        if (!result_.ok()) {
            return result_;
        }
        if (arrived_and_stopped.size() != roster_size_) {
            return reject(ScheduleStatus::InvalidInput, "Completion vector does not match roster");
        }
        for (std::size_t i = 0; i < completed_.size(); ++i) {
            if (completed_[i] && !arrived_and_stopped[i]) {
                return reject(ScheduleStatus::InvalidInput,
                              "Previously completed robot lost its arrived-and-stopped state");
            }
        }
        while (group_ < groups_.size()) {
            bool finished = true;
            for (const auto robot : groups_[group_]) {
                finished = finished && arrived_and_stopped[robot];
            }
            if (!finished) {
                break;
            }
            for (const auto robot : groups_[group_]) {
                completed_[robot] = true;
            }
            ++group_;
        }
        result_.selected.clear();
        result_.status =
            group_ == groups_.size() ? ScheduleStatus::Complete : ScheduleStatus::Ready;
        if (group_ < groups_.size()) {
            for (const auto robot : groups_[group_]) {
                if (!arrived_and_stopped[robot]) {
                    result_.selected.push_back(robot);
                }
            }
        }
        return result_;
    }

   private:
    ScheduleResult reject(ScheduleStatus status, std::string detail) {
        result_.status = status;
        result_.selected.clear();
        result_.detail = std::move(detail);
        return result_;
    }
    double clearance_;
    ScheduleResult result_;
    std::vector<std::vector<std::size_t>> groups_;
    std::vector<bool> completed_;
    std::size_t roster_size_{0};
    std::size_t group_{0};
};

// Exact rectangular footprints of every nonselected peer, frozen only for
// geometric route initialization. DWA rollouts must still include these
// peers at their fresh actual pose as hard stationary constraints.
inline std::vector<ConvexObstacle> parkedPeerObstacles(const std::vector<Robot>& robots,
                                                       const std::vector<bool>& selected_mask) {
    if (selected_mask.size() != robots.size()) {
        throw std::invalid_argument("Selected mask does not match the scheduler roster");
    }
    std::vector<ConvexObstacle> obstacles;
    for (std::size_t index = 0; index < robots.size(); ++index) {
        if (selected_mask[index]) {
            continue;
        }
        const auto& robot = robots[index];
        const double c = std::cos(robot.yaw), s = std::sin(robot.yaw);
        ConvexObstacle obstacle;
        obstacle.id = "held_robot/" + robot.id;
        for (const auto& corner :
             std::vector<Eigen::Vector2d>{{-robot.half_length, -robot.half_width},
                                          {robot.half_length, -robot.half_width},
                                          {robot.half_length, robot.half_width},
                                          {-robot.half_length, robot.half_width}}) {
            const Eigen::Vector2d local = corner + robot.body_center_offset;
            obstacle.vertices.emplace_back(robot.position.x() + c * local.x() - s * local.y(),
                                           robot.position.y() + s * local.x() + c * local.y());
        }
        obstacles.push_back(std::move(obstacle));
    }
    return obstacles;
}

}  // namespace ugv_reset_safety
