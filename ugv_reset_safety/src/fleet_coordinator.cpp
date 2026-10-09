#include <ugv_reset_safety/fleet_coordinator.h>
#include <ugv_reset_safety/scene_projection.h>

#include <Eigen/Geometry>
#include <chrono>
#include <set>
#include <stdexcept>
namespace ugv_reset_safety {
namespace {
Eigen::Quaterniond quaternion(const scene_model::Quaternion& q) {
    Eigen::Quaterniond r(q.w, q.x, q.y, q.z);
    if (!r.coeffs().allFinite() || !std::isfinite(r.norm()) || r.norm() < 1e-6)
        throw std::invalid_argument("invalid quaternion");
    r.normalize();
    return r;
}
double yaw(const Eigen::Quaterniond& q) {
    const auto r = q.toRotationMatrix();
    return std::atan2(r(1, 0), r(0, 0));
}
bool finitePose(const scene_model::Pose2& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.theta);
}
}  // namespace
class FleetCoordinator::Impl {
    struct Entry {
        Robot robot;
        ResetPath path;
        FleetCoordinator::Request request;
        NativeWallTime request_wall, pose_wall, state_wall;
        NativeStamp pose_stamp;
        Eigen::Vector2d measured_position = Eigen::Vector2d::Zero();
        double measured_yaw = 0, measured_speed = 0, measured_omega = 0;
        bool have_pose = false, have_request = false, have_generation = false, planned = false,
             rejected = false;
        uint32_t generation = 0;
        std::string state;
        scene_model::Pose2 frozen_target;
        std::string reason;
    };

   public:
    Impl(std::vector<Robot> robots, FleetCoordinator::Configuration config, NativeClock clock)
        : clock_(clock),
          world_frame_(config.world_frame),
          fence_(config.fence),
          dwa_config_(config.dwa),
          obstacle_avoidance_(config.obstacle_avoidance),
          frequency_(config.frequency),
          timeout_(config.input_timeout),
          state_timeout_(config.state_timeout) {
        if (robots.empty() || robots.size() > FleetCoordinator::kMaxRobots ||
            !std::isfinite(frequency_) || frequency_ < 10 || !std::isfinite(timeout_) ||
            timeout_ <= 0 || timeout_ > .5 || !std::isfinite(state_timeout_) ||
            state_timeout_ < .5 || state_timeout_ > 2 || world_frame_.empty() || !fence_.enabled ||
            !std::isfinite(fence_.xmin) || !std::isfinite(fence_.xmax) ||
            !std::isfinite(fence_.ymin) || !std::isfinite(fence_.ymax) ||
            fence_.xmin >= fence_.xmax || fence_.ymin >= fence_.ymax) {
            throw std::invalid_argument("invalid bounded roster, timing or explicit fence");
        }
        std::set<std::string> ids;
        entries_.reserve(robots.size());
        proposals_.resize(robots.size());
        for (auto& robot : robots) {
            if (robot.id.empty() || robot.id.find("..") != std::string::npos ||
                !ids.insert(robot.id).second || !std::isfinite(robot.half_length) ||
                !std::isfinite(robot.half_width) || robot.half_length <= 0 ||
                robot.half_width <= 0) {
                throw std::invalid_argument("invalid robot identity or footprint");
            }
            Entry entry;
            entry.robot = std::move(robot);
            entries_.push_back(std::move(entry));
        }
        setClock(clock);
        last_tick_ = wallClock();
        last_ros_tick_ = worldClock();
    }
    void setClock(NativeClock clock) {
        if (!std::isfinite(clock.monotonic.seconds) || clock.monotonic.seconds <= 0) {
            throw std::invalid_argument("finite positive monotonic time required");
        }
        clock_ = clock;
    }
    NativeStamp worldClock() const {
        return clock_.world;
    }
    NativeWallTime wallClock() const {
        return clock_.monotonic;
    }
    void state(std::size_t index, std::string state) {
        auto& entry = entries_.at(index);
        entry.state = std::move(state);
        entry.state_wall = wallClock();
    }

   public:
    void request(std::size_t n, const FleetCoordinator::Request& r) {
        auto& e = entries_.at(n);
        const auto now = worldClock();
        if (r.header.stamp.isZero() || r.pose_stamp.isZero() || !finitePose(r.pose) ||
            !finitePose(r.target) || (now - r.header.stamp).toSec() < 0 ||
            (now - r.header.stamp).toSec() > timeout_ || (now - r.pose_stamp).toSec() < 0 ||
            (now - r.pose_stamp).toSec() > timeout_) {
            return;
        }
        const auto& a = r.applied_command;
        Eigen::Vector3d applied(a.linear.x, a.linear.y, a.angular.z);
        if (!applied.allFinite() || a.linear.z != 0 || a.angular.x != 0 || a.angular.y != 0 ||
            (e.robot.type == RobotType::Unicycle && a.linear.y != 0)) {
            return;
        }
        if (r.header.frame_id != world_frame_ || r.applied_stamp.isZero() ||
            r.applied_stamp > r.header.stamp || (now - r.applied_stamp).toSec() > timeout_) {
            return;
        }
        if (std::abs(applied.x()) > e.robot.limits.max_vx + 1e-6 ||
            std::abs(applied.y()) > e.robot.limits.max_vy + 1e-6 ||
            std::abs(applied.z()) > e.robot.limits.max_omega + 1e-6) {
            return;
        }
        if (e.have_request && r.header.stamp <= e.request.header.stamp) {
            return;
        }
        if (!e.have_generation || r.generation != e.generation) {
            // Session identifiers are monotonically increasing within an owner.
            // Wall expiration alone never resets the command/rate state.
            e.generation = r.generation;
            e.have_generation = true;
            e.planned = false;
            e.rejected = false;
            e.reason.clear();
            e.frozen_target = r.target;
            e.path.clear();
            dwa_.clear();
            schedule_ready_ = false;
            schedule_.clear();
            scheduled_requested_.clear();
            completed_.assign(entries_.size(), false);
            // Every new owner generation reopens admission. Stored requests
            // from a previous Reset must not suppress the next batch window.
            last_admission_ = wallClock();
            const auto wall = wallClock();
            if (!e.have_pose || (wall - e.pose_wall).toSec() > timeout_ ||
                (now - e.pose_stamp).toSec() < 0 || (now - e.pose_stamp).toSec() > timeout_ ||
                (wall - e.state_wall).toSec() > state_timeout_) {
                e.rejected = true;
                e.reason = "reset pose/state unavailable at request";
            }
        }
        if (r.target.x != e.frozen_target.x || r.target.y != e.frozen_target.y ||
            r.target.theta != e.frozen_target.theta) {
            e.rejected = true;
            e.reason = "target changed inside reset session";
        }
        e.robot.previous = applied;
        e.request = r;
        e.have_request = true;
        e.request_wall = wallClock();
    }
    void pose(std::size_t n, const scene_model::PoseSample& p) {
        auto& e = entries_.at(n);
        try {
            if (p.header.frame_id != world_frame_ || p.header.stamp.isZero() ||
                !std::isfinite(p.pose.position.x) || !std::isfinite(p.pose.position.y) ||
                p.header.stamp > worldClock()) {
                return;
            }
            const auto heading = yaw(quaternion(p.pose.orientation));
            Eigen::Vector2d position(p.pose.position.x, p.pose.position.y);
            if (e.have_pose) {
                const double dt = (p.header.stamp - e.pose_stamp).toSec();
                if (dt <= 0) {
                    return;
                }
                e.measured_speed =
                    dt <= timeout_ ? (position - e.measured_position).norm() / dt : 1e9;
                e.measured_omega = dt > timeout_ ? 1e9
                                                 : std::atan2(std::sin(heading - e.measured_yaw),
                                                              std::cos(heading - e.measured_yaw)) /
                                                       dt;
            } else {
                e.measured_speed = 1e9;
                e.measured_omega = 1e9;
            }
            e.measured_position = position;
            e.measured_yaw = heading;
            e.pose_stamp = p.header.stamp;
            e.pose_wall = wallClock();
            e.have_pose = true;
        } catch (const std::exception& ex) {
        }
    }
    void sceneState(const scene_model::State& state) {
        if (scene_valid_ && state.simulation_time.epoch != scene_state_.simulation_time.epoch) {
            for (auto& entry : entries_)
                if (entry.have_request) {
                    entry.rejected = true;
                    entry.reason =
                        "simulation clock epoch changed; request a new reset after stopping";
                }
            scene_valid_ = false;
            last_state_stamp_ = {};
        }
        if (snapshot_.epoch.empty() || !scene_parsed_) {
            return;
        }
        if (state.epoch != snapshot_.epoch || state.revision != snapshot_.revision ||
            state.header.frame_id != world_frame_ || state.header.stamp.isZero()) {
            return;
        }
        if (!last_state_stamp_.isZero() && state.header.stamp <= last_state_stamp_) {
            return;
        }
        try {
            auto geometry = scene_projection::live(snapshot_, state, world_frame_);
            bool moved = geometry.live.size() != obstacles_.size();
            if (!moved) {
                for (std::size_t i = 0; i < geometry.live.size(); ++i) {
                    if (geometry.live[i].id != obstacles_[i].id ||
                        geometry.live[i].vertices.size() != obstacles_[i].vertices.size()) {
                        moved = true;
                        break;
                    }
                    for (std::size_t j = 0; j < geometry.live[i].vertices.size(); ++j) {
                        if ((geometry.live[i].vertices[j] - obstacles_[i].vertices[j]).norm() >
                            1e-3) {
                            moved = true;
                            break;
                        }
                    }
                }
            }
            obstacles_ = std::move(geometry.live);
            occupancy_ = std::move(geometry.occupancy);
            scene_state_ = state;
            last_state_stamp_ = state.header.stamp;
            scene_state_wall_ = wallClock();
            scene_valid_ = true;
            scene_error_.clear();
            scene_capability_ = "ok";
            if (moved) {
                for (auto& e : entries_) {
                    e.planned = false;
                }
            }
        } catch (const std::exception& e) {
            scene_valid_ = false;
            scene_error_ = e.what();
            classifyCapability(scene_error_);
        }
        publishStatus();
    }
    void scene(const scene_model::Snapshot& snapshot) {
        if (snapshot.epoch == snapshot_.epoch && snapshot.revision < snapshot_.revision) {
            return;
        }
        const bool definition_changed =
            !snapshot_.epoch.empty() &&
            (snapshot.epoch != snapshot_.epoch || snapshot.revision != snapshot_.revision);
        snapshot_ = snapshot;
        scene_parsed_ = false;
        scene_valid_ = false;
        scene_error_.clear();
        obstacles_.clear();
        occupancy_.clear();
        last_state_stamp_ = NativeStamp();
        scene_state_wall_ = NativeWallTime();
        try {
            std::set<std::string> ids;
            for (const auto& obstacle : snapshot_.obstacles) {
                scene_projection::validateObstacle(obstacle, &ids);
            }
            if (snapshot_.epoch.empty() || snapshot_.header.frame_id != world_frame_) {
                throw std::invalid_argument("scene epoch/frame mismatch");
            }
            scene_parsed_ = true;
            scene_capability_ = "ok";
            if (definition_changed) {
                for (auto& e : entries_) {
                    if (e.have_request) {
                        e.rejected = true;
                        e.reason = "scene changed; request a new reset after stopping";
                    }
                }
            }
        } catch (const std::exception& e) {
            scene_error_ = e.what();
            classifyCapability(scene_error_);
        }
        publishStatus();
    }
    void classifyCapability(const std::string& error) {
        scene_capability_ = error.find("unsupported") != std::string::npos ? "unsupported" : "";
    }
    void publishStatus() {
        scene_status_.epoch = snapshot_.epoch;
        scene_status_.revision = snapshot_.revision;
        scene_status_.simulation_time_epoch = scene_state_.simulation_time.epoch;
        scene_status_.applied = scene_parsed_;
        scene_status_.capability =
            scene_capability_.empty() ? (scene_parsed_ ? "ok" : "") : scene_capability_;
        scene_status_.operational =
            scene_parsed_ && scene_valid_ && scene_status_.capability != "unsupported" &&
            !scene_state_wall_.isZero() && (wallClock() - scene_state_wall_).toSec() >= 0 &&
            (wallClock() - scene_state_wall_).toSec() <= .5 && worldClock() >= last_state_stamp_ &&
            (worldClock() - last_state_stamp_).toSec() <= .5;
        scene_status_.message =
            scene_parsed_
                ? (scene_valid_
                       ? "applied live planar projection with finite-horizon occupancy"
                       : (scene_error_.empty() ? "waiting for matching scene state" : scene_error_))
                : scene_error_;
    }
    void reply(Entry& e, uint8_t status, const Eigen::Vector3d& command,
               const std::string& reason) {
        auto& proposal = proposals_[static_cast<std::size_t>(&e - entries_.data())];
        proposal = {true,
                    e.request.generation,
                    e.request.header.stamp,
                    static_cast<ResetSession::Status>(status),
                    command,
                    reason};
    }
    void commandDirect(const std::vector<Robot>& robots, const std::vector<ResetTarget>& targets,
                       double dt) {
        for (std::size_t i = 0; i < entries_.size(); ++i) {
            auto& e = entries_[i];
            if (!e.robot.active) {
                continue;
            }
            if (fence_.enabled &&
                (targets[i].position.x() < fence_.xmin || targets[i].position.x() > fence_.xmax ||
                 targets[i].position.y() < fence_.ymin || targets[i].position.y() > fence_.ymax)) {
                rejectActive("reset target outside fence");
                return;
            }
            const auto command = directResetCommand(robots[i], targets[i], dt);
            const bool arrived =
                withinTargetTolerance(robots[i], targets[i]) && command.isZero(0.0) &&
                e.robot.previous.cwiseAbs().maxCoeff() <= dwa_config_.feasibility_tolerance &&
                e.measured_speed <= 0.03 && std::abs(e.measured_omega) <= 0.05;
            if (arrived) {
                reply(e, ResetSession::ARRIVED, Eigen::Vector3d::Zero(), "");
            } else {
                reply(e, ResetSession::RUNNING, command, "");
            }
        }
    }
    void rejectActive(const std::string& reason) {
        for (auto& e : entries_) {
            if (e.have_request && e.robot.active) {
                e.rejected = true;
                e.reason = reason;
                reply(e, ResetSession::REJECTED, Eigen::Vector3d::Zero(), reason);
            }
        }
    }
    void tick() {
        const auto native_started = std::chrono::steady_clock::now();
        std::fill(proposals_.begin(), proposals_.end(), FleetCoordinator::Proposal{});
        const auto wall = wallClock();
        const auto now = worldClock();
        const double wall_dt = (wall - last_tick_).toSec();
        last_tick_ = wall;
        const double dt = (now - last_ros_tick_).toSec();
        last_ros_tick_ = now;
        bool active = false;
        for (auto& e : entries_) {
            e.robot.active =
                e.have_request && (wall - e.request_wall).toSec() <= timeout_ && e.state == "Reset";
            active = active || e.robot.active;
        }
        publishStatus();
        if (!active) {
            return;
        }
        if (obstacle_avoidance_ && !scene_parsed_) {
            rejectActive("scene unavailable: " + scene_error_);
            return;
        }
        // Requests and the 5 Hz public state do not arrive atomically.
        // Hold zero until new generations have finished joining, before
        // consulting any previous generation's rejection or freezing members.
        const bool awaiting_state =
            std::any_of(entries_.begin(), entries_.end(), [&](const auto& e) {
                const bool fresh_request =
                    e.have_request && (wall - e.request_wall).toSec() <= timeout_;
                return fresh_request != (e.state == "Reset");
            });
        if (!schedule_ready_ &&
            ((wall - last_admission_).toSec() < timeout_ ||
             (awaiting_state && (wall - last_admission_).toSec() < state_timeout_))) {
            for (auto& e : entries_) {
                if (e.robot.active) {
                    reply(e, ResetSession::RUNNING, Eigen::Vector3d::Zero(),
                          "collecting reset batch");
                }
            }
            return;
        }
        for (auto& e : entries_) {
            if (e.robot.active && e.rejected) {
                rejectActive(e.reason);
                return;
            }
        }
        if (obstacle_avoidance_ &&
            (scene_state_wall_.isZero() || (wall - scene_state_wall_).toSec() < 0 ||
             (wall - scene_state_wall_).toSec() > 0.5 || now < last_state_stamp_ ||
             (now - last_state_stamp_).toSec() > .5)) {
            rejectActive("shared scene heartbeat expired");
            return;
        }
        if (obstacle_avoidance_ && !scene_valid_) {
            rejectActive("scene unavailable: " + scene_error_);
            return;
        }
        if (dt <= 0 || dt > timeout_ || wall_dt <= 0 || wall_dt > timeout_) {
            rejectActive("coordinator deadline missed");
            return;
        }
        for (auto& e : entries_) {
            if (!e.have_pose || (wall - e.pose_wall).toSec() > timeout_ ||
                (now - e.pose_stamp).toSec() < 0 || (now - e.pose_stamp).toSec() > timeout_ ||
                (wall - e.state_wall).toSec() > state_timeout_) {
                rejectActive("fleet pose/state unavailable");
                return;
            }
            if (e.state != "SelfCheck" && e.state != "Ready" && e.state != "Reset") {
                rejectActive("fleet member is outside reset/stop states");
                return;
            }
            if (!e.robot.active && (e.measured_speed > 0.03 || std::abs(e.measured_omega) > 0.05)) {
                rejectActive("uncontrolled moving fleet member");
                return;
            }
            e.robot.position = e.measured_position;
            e.robot.yaw = e.measured_yaw;
            if (!e.robot.active) {
                e.robot.previous.setZero();
            }
            if (e.robot.active && e.rejected) {
                rejectActive(e.reason);
                return;
            }
        }
        std::vector<Robot> robots;
        std::vector<ResetTarget> targets;
        std::vector<bool> requested;
        for (const auto& e : entries_) {
            robots.push_back(e.robot);
            ResetTarget target;
            target.position = {e.frozen_target.x, e.frozen_target.y};
            target.yaw = e.frozen_target.theta;
            targets.push_back(target);
            requested.push_back(e.robot.active);
        }
        if (!obstacle_avoidance_) {
            commandDirect(robots, targets, dt);
            return;
        }
        if (!schedule_ready_) {
            schedule_ = FleetSchedule(dwa_config_.clearance + dwa_config_.uncertainty_margin);
            const auto initialized = schedule_.initialize(robots, targets);
            if (!initialized.ok()) {
                rejectActive("reset schedule: " + initialized.detail);
                return;
            }
            scheduled_requested_ = requested;
            completed_.assign(robots.size(), false);
            schedule_ready_ = true;
        }
        for (std::size_t i = 0; i < robots.size(); ++i) {
            if (scheduled_requested_[i] != requested[i] && !completed_[i]) {
                rejectActive("Reset batch membership changed before arrival");
                return;
            }
        }
        const auto group = schedule_.select(completed_);
        if (!group.ok()) {
            rejectActive("reset schedule: " + group.detail);
            return;
        }
        std::vector<bool> selected(robots.size(), false);
        for (const auto i : group.selected) {
            selected[i] = true;
        }
        // Completion changes a peer's command to zero, not its geometry or
        // anyone else's admission. All fleet bodies remain DWA constraints.
        // Static scene routes and learned motion survive another owner arriving.
        const auto& path_obstacles = occupancy_.empty() ? obstacles_ : occupancy_;
        std::vector<ResetPath> paths(robots.size());
        for (std::size_t i = 0; i < robots.size(); ++i) {
            auto& e = entries_[i];
            // Nonselected owners remain in Reset but receive an exact zero.
            // They are stationary obstacles, not free actuators.
            if (!selected[i] &&
                (e.measured_speed > 0.03 || std::abs(e.measured_omega) > 0.05 ||
                 robots[i].previous.cwiseAbs().maxCoeff() > dwa_config_.feasibility_tolerance)) {
                rejectActive("parked Reset member is moving");
                return;
            }
            robots[i].active = selected[i];
            robots[i].stop_requested = false;
            robots[i].brake_requested = false;
            robots[i].command.setZero();
            if (!selected[i]) {
                continue;
            }
            if (!e.planned) {
                const auto planned = e.path.setGoal(robots[i], targets[i], path_obstacles, fence_);
                if (planned.status == PathStatus::InvalidInput ||
                    planned.status == PathStatus::NoRoute) {
                    rejectActive(planned.message);
                    return;
                }
                e.planned = true;
            }
            const auto g = e.path.step(robots[i]);
            if (g.status == PathStatus::InvalidInput || g.status == PathStatus::NoRoute) {
                rejectActive(g.message);
                return;
            }
            paths[i] = e.path;
            // An inward instantaneous velocity must not cancel pose error at
            // release: the chassis can still settle outward after zero is
            // acknowledged. Reserve the observed motion magnitude over DWA's
            // horizon in either direction, using the unchanged goal tolerance.
            Robot coast = robots[i];
            Eigen::Vector2d outward = robots[i].position - e.path.target().position;
            outward = outward.norm() > 1e-9 ? Eigen::Vector2d(outward.normalized())
                                            : Eigen::Vector2d::UnitX();
            coast.position += outward * e.measured_speed * ResetDwa::predictionHorizon();
            const double yaw_error = std::atan2(std::sin(robots[i].yaw - e.path.target().yaw),
                                                std::cos(robots[i].yaw - e.path.target().yaw));
            coast.yaw += std::copysign(std::abs(e.measured_omega) * ResetDwa::predictionHorizon(),
                                       yaw_error);
            robots[i].brake_requested = g.status == PathStatus::Reached && e.path.reached(coast) &&
                                        e.measured_speed <= 0.03 &&
                                        std::abs(e.measured_omega) <= 0.05;
            robots[i].stop_requested =
                robots[i].brake_requested &&
                robots[i].previous.cwiseAbs().maxCoeff() <= dwa_config_.feasibility_tolerance;
        }
        dwa_config_.dt = dt;
        dwa_.apply(robots, paths, path_obstacles, fence_, dwa_config_);
        const double solve_seconds =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - native_started)
                .count();
        if (solve_seconds > 1.0 / frequency_) {
            rejectActive("reset solve deadline missed");
            return;
        }
        for (std::size_t i = 0; i < entries_.size(); ++i) {
            auto& e = entries_[i];
            if (!e.robot.active) {
                continue;
            }
            const bool arrived =
                robots[i].stop_requested && robots[i].command.isZero(0.0) &&
                e.robot.previous.cwiseAbs().maxCoeff() <= dwa_config_.feasibility_tolerance &&
                e.measured_speed <= 0.03 && std::abs(e.measured_omega) <= 0.05;
            if (arrived || completed_[i]) {
                completed_[i] = true;
                reply(e, ResetSession::ARRIVED, Eigen::Vector3d::Zero(), "");
            } else if (!robots[i].local_plan_feasible) {
                reply(e, ResetSession::RUNNING, Eigen::Vector3d::Zero(), "no feasible DWA sample");
            } else {
                reply(e, ResetSession::RUNNING, robots[i].command, "");
            }
        }
    }
    std::vector<Entry> entries_;
    std::vector<FleetCoordinator::Proposal> proposals_;
    FleetCoordinator::SceneStatus scene_status_;
    NativeClock clock_;
    scene_model::Snapshot snapshot_;
    scene_model::State scene_state_;
    NativeStamp last_state_stamp_;
    NativeWallTime scene_state_wall_;
    bool scene_parsed_ = false;
    bool scene_valid_ = false;
    std::string world_frame_, scene_namespace_, scene_error_, scene_capability_;
    std::vector<ConvexObstacle> obstacles_;
    std::vector<ConvexObstacle> occupancy_;
    Fence fence_;
    DwaConfig dwa_config_;
    ResetDwa dwa_;
    FleetSchedule schedule_;
    bool schedule_ready_ = false;
    std::vector<bool> scheduled_requested_, completed_;
    NativeWallTime last_admission_;
    bool obstacle_avoidance_ = true;
    double frequency_ = 50, timeout_ = .15, state_timeout_ = 1.0;
    uint32_t consumer_generation_ = 1;
    NativeWallTime last_tick_;
    NativeStamp last_ros_tick_;
};
FleetCoordinator::FleetCoordinator(std::vector<Robot> roster, Configuration configuration,
                                   NativeClock clock)
    : impl_(std::make_unique<Impl>(std::move(roster), std::move(configuration), clock)) {}
FleetCoordinator::~FleetCoordinator() = default;
void FleetCoordinator::request(std::size_t index, const Request& request, NativeClock clock) {
    impl_->setClock(clock);
    impl_->request(index, request);
}
void FleetCoordinator::pose(std::size_t index, const scene_model::PoseSample& pose,
                            NativeClock clock) {
    impl_->setClock(clock);
    impl_->pose(index, pose);
}
void FleetCoordinator::state(std::size_t index, std::string state, NativeClock clock) {
    impl_->setClock(clock);
    impl_->state(index, std::move(state));
}
void FleetCoordinator::scene(const scene_model::Snapshot& snapshot, NativeClock clock) {
    impl_->setClock(clock);
    impl_->scene(snapshot);
}
void FleetCoordinator::sceneState(const scene_model::State& state, NativeClock clock) {
    impl_->setClock(clock);
    impl_->sceneState(state);
}
void FleetCoordinator::tick(NativeClock clock) {
    impl_->setClock(clock);
    impl_->tick();
}
const std::vector<FleetCoordinator::Proposal>& FleetCoordinator::proposals() const {
    return impl_->proposals_;
}
const FleetCoordinator::SceneStatus& FleetCoordinator::sceneStatus() const {
    return impl_->scene_status_;
}
std::size_t FleetCoordinator::size() const {
    return impl_->entries_.size();
}
}  // namespace ugv_reset_safety
