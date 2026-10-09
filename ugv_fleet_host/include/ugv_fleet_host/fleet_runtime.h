#pragma once
#include <ugv_fleet_host/reset_control.h>
#include <ugv_reset_safety/fleet_coordinator.h>
#include <ugv_reset_safety/fleet_edge.h>

#include <algorithm>
namespace ugv_fleet_host {
// A single native owner runs original controllers at their existing rate and
// the fleet solve at 50 Hz. No ROS reset messages or network calls are involved.
class FleetRuntime {
   public:
    FleetRuntime(std::vector<std::unique_ptr<ugv_reset_safety::FleetEdge>> edges,
                 std::vector<ugv_reset_safety::Robot> roster,
                 ugv_reset_safety::FleetCoordinator::Configuration config,
                 ugv_reset_safety::NativeClock clock)
        : edges_(std::move(edges)),
          coordinator_(std::move(roster), config, clock),
          control_(
              edges_.size(), [this](auto i) { return edges_[i]->reset(); },
              [this](auto i) { return edges_[i]->stop(); }),
          period_(1 / config.frequency),
          last_solve_(clock.monotonic.seconds) {
        if (edges_.size() != coordinator_.size())
            throw std::invalid_argument("edge and native roster mismatch");
        last_update_.resize(edges_.size(), 0);
        for (const auto& edge : edges_)
            if (!edge || !std::isfinite(edge->controlRate()) || edge->controlRate() <= 0)
                throw std::invalid_argument("null native edge");
    }
    ~FleetRuntime() {
        stopAll();
        for (auto& edge : edges_)
            edge->updateOnce();
    }
    void scene(const ugv_reset_safety::scene_model::Snapshot& definition,
               const ugv_reset_safety::scene_model::State& state,
               ugv_reset_safety::NativeClock clock) {
        scene_failed_ = false;
        const auto& status = coordinator_.sceneStatus();
        if (status.epoch != definition.epoch || status.revision != definition.revision)
            coordinator_.scene(definition, clock);
        coordinator_.sceneState(state, clock);
        syncScene(clock);
    }
    void invalidateScene(ugv_reset_safety::NativeClock clock) {
        scene_failed_ = true;
        syncScene(clock);
    }
    void turn(ugv_reset_safety::NativeClock clock) {
        for (std::size_t i = 0; i < edges_.size(); ++i)
            if (!last_update_[i] ||
                clock.monotonic.seconds - last_update_[i] >= 1 / edges_[i]->controlRate()) {
                last_update_[i] = clock.monotonic.seconds;
                edges_[i]->updateOnce();
            }
        if (clock.monotonic.seconds - last_solve_ >= period_) {
            last_solve_ = clock.monotonic.seconds;
            for (std::size_t i = 0; i < edges_.size(); ++i) {
                auto& edge = *edges_[i];
                const auto pose = edge.poseSample();
                coordinator_.pose(i, pose, clock);
                coordinator_.state(i, edge.controlState(), clock);
                if (!edge.healthReady() || pose.header.stamp.isZero())
                    continue;
                const auto q = pose.pose.orientation;
                const double heading =
                    std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
                const auto request =
                    edge.resetSession().issue({pose.pose.position.x, pose.pose.position.y, heading},
                                              clock.world.toNSec(), clock.monotonic.seconds);
                if (!request.valid)
                    continue;
                ugv_reset_safety::FleetCoordinator::Request input;
                input.header.stamp = {request.stamp};
                input.pose_stamp = pose.header.stamp;
                input.applied_stamp = {request.applied_stamp};
                input.generation = request.generation;
                input.pose = {request.pose.x, request.pose.y, request.pose.yaw};
                input.target = {request.target.x, request.target.y, request.target.yaw};
                input.applied_command.linear = {request.applied_command.x,
                                                request.applied_command.y, 0};
                input.applied_command.angular.z = request.applied_command.yaw;
                coordinator_.request(i, input, clock);
            }
            coordinator_.tick(clock);
            syncScene(clock);
            const auto& proposals = coordinator_.proposals();
            for (std::size_t i = 0; i < edges_.size(); ++i)
                if (proposals[i].valid) {
                    const auto& proposal = proposals[i];
                    edges_[i]->resetSession().accept(
                        proposal.generation, proposal.stamp.toNSec(), proposal.status,
                        {proposal.command.x(), proposal.command.y(), proposal.command.z()},
                        clock.world.toNSec(), clock.monotonic.seconds);
                }
        }
        for (std::size_t i = 0; i < edges_.size(); ++i) {
            auto& edge = *edges_[i];
            const auto command = edge.resetSession().appliedCommand();
            control_.sample(
                i, edge.resetSession(), edge.controlState() == "Reset", edge.admissionRejected(),
                command.x == 0 && command.y == 0 && command.yaw == 0,
                edge.resetSession().appliedStamp(), clock.world.toNSec(), clock.monotonic.seconds);
        }
    }
    ResetControl& control() {
        return control_;
    }
    const ugv_reset_safety::FleetCoordinator::SceneStatus& sceneStatus() const {
        return coordinator_.sceneStatus();
    }
    void stopAll() {
        for (auto& edge : edges_)
            edge->stop();
    }

   private:
    void syncScene(ugv_reset_safety::NativeClock clock) {
        const auto& status = coordinator_.sceneStatus();
        ResetControl::SceneFence fence;
        if (status.epoch.size() > fence.epoch.size()) {
            control_.scene({}, false, clock.world.toNSec(), clock.monotonic.seconds);
            return;
        }
        std::copy(status.epoch.begin(), status.epoch.end(), fence.epoch.begin());
        fence.revision = status.revision;
        fence.simulation_time_epoch = status.simulation_time_epoch;
        control_.scene(fence, status.operational && !scene_failed_, clock.world.toNSec(),
                       clock.monotonic.seconds);
    }
    std::vector<std::unique_ptr<ugv_reset_safety::FleetEdge>> edges_;
    ugv_reset_safety::FleetCoordinator coordinator_;
    ResetControl control_;
    std::vector<double> last_update_;
    double period_, last_solve_;
    bool scene_failed_{false};
};
}  // namespace ugv_fleet_host
