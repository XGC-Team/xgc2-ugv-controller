#pragma once

#include <ugv_reset_safety/fleet_schedule.h>
#include <ugv_reset_safety/reset_dwa.h>
#include <ugv_reset_safety/reset_session.h>
#include <ugv_reset_safety/scene_model.h>

#include <memory>

namespace ugv_reset_safety {

// All methods run on the single native control owner. Management methods only
// submit bounded commands to that owner; proposal generation performs no RPC.
class FleetCoordinator {
   public:
    static constexpr std::size_t kMaxRobots = 128;
    struct Configuration {
        double frequency{50}, input_timeout{0.15}, state_timeout{1};
        std::string world_frame{"world"};
        bool obstacle_avoidance{true};
        Fence fence;
        DwaConfig dwa;
    };
    struct Request {
        scene_model::Header header;
        uint32_t generation{0};
        NativeStamp pose_stamp, applied_stamp;
        scene_model::Twist applied_command;
        scene_model::Pose2 pose, target;
    };
    struct Proposal {
        bool valid{false};
        uint32_t generation{0};
        NativeStamp stamp;
        ResetSession::Status status{ResetSession::RUNNING};
        Eigen::Vector3d command{Eigen::Vector3d::Zero()};
        std::string reason;
    };
    struct SceneStatus {
        std::string epoch, capability, message;
        uint64_t revision{0}, simulation_time_epoch{0};
        bool applied{false}, operational{false};
    };
    FleetCoordinator(std::vector<Robot> roster, Configuration configuration, NativeClock clock);
    ~FleetCoordinator();
    FleetCoordinator(const FleetCoordinator&) = delete;
    FleetCoordinator& operator=(const FleetCoordinator&) = delete;
    void request(std::size_t index, const Request& request, NativeClock clock);
    void pose(std::size_t index, const scene_model::PoseSample& pose, NativeClock clock);
    void state(std::size_t index, std::string state, NativeClock clock);
    void scene(const scene_model::Snapshot& snapshot, NativeClock clock);
    void sceneState(const scene_model::State& state, NativeClock clock);
    void tick(NativeClock clock);
    const std::vector<Proposal>& proposals() const;
    const SceneStatus& sceneStatus() const;
    std::size_t size() const;

   private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace ugv_reset_safety
