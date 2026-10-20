#pragma once

// The coordinator without ROS: the vehicles' Session messages in, the Grant messages out.
//
// It knows no roster. A vehicle is whoever sends sessions: its Session declares its footprint,
// limits, target and health, and the namespace it sends from names it. A vehicle that falls silent
// stays a parked body at its last pose (a vehicle does not vanish); a vehicle that sends something
// invalid is refused for itself. Every fault is isolated to the vehicle that has it: there is no
// fleet-wide rejection, the others keep their grants and treat the faulty one as a parked body.
//
// The node (reset_coordinator_node.cpp) only carries messages, the Scene and the clock; everything
// that decides is here and in the planner, so the closed-loop harness drives exactly this code.

#include <string>
#include <ugv_reset_client/reset_protocol.h>
#include <ugv_reset_safety/reset_planner.h>
#include <map>
#include <vector>

namespace ugv_reset_safety {

struct CoordinatorConfig {
    PlannerConfig planner;
    // No session message for this long: the vehicle is lost; it stays a parked body at its last pose.
    double lost_after{3.0};
    // An unchanged grant (or hold) is repeated at this period: the vehicles' heartbeat.
    double refresh_period{0.4};
    double ttl{1.0};
    // A body outside a session that moves faster than this is "uncontrolled": no new route is
    // granted while it moves (its movement is not planned).
    double moving_speed{0.10};
    // A rejection is repeated for this long, until the vehicle has ended its session.
    double reject_resend{5.0};
};

struct OutgoingGrant {
    std::string vehicle;
    ugv_reset_client::Grant grant;
};

struct FleetMember {
    std::string vehicle;
    uint32_t session{0};
    ugv_reset_client::Phase phase{ugv_reset_client::Phase::Idle};
    ugv_reset_client::GrantState grant_state{ugv_reset_client::GrantState::Hold};
    ugv_reset_client::Reason reason{ugv_reset_client::Reason::None};
    double progress{0.0};
    double remaining{0.0};
    bool lost{false};
    std::vector<std::string> blockers;
};

struct FleetSummary {
    bool scene_available{true};
    std::vector<FleetMember> members;
};

class ResetCoordinator {
   public:
    explicit ResetCoordinator(const CoordinatorConfig& config = CoordinatorConfig());

    // The statics (the live Scene occupancy, later non-participant circles) and the fence.
    void setWorld(const std::vector<ConvexObstacle>& statics, const Fence& fence);
    // Whether a Scene was authored for this Experiment, and whether its snapshot and heartbeat are
    // live. An authored Scene that is not live stops new grants (and says why); with no Scene
    // authored the statics are simply the fence.
    void setScene(bool authored, bool available);

    // A Session message from `vehicle` (its namespace), received at coordinator time `now`.
    void onSession(const std::string& vehicle, const ugv_reset_client::Session& session, double now);

    // The messages due at `now`: decisions of the planner and the heartbeat of unchanged ones.
    std::vector<OutgoingGrant> step(double now);

    FleetSummary fleet(double now) const;
    // True while a participant, a mover or a message in flight needs the 10 Hz tick.
    bool busy() const;
    const PlannerStats& stats() const {
        return planner_.stats();
    }
    const CoordinatorConfig& config() const {
        return config_;
    }

   private:
    struct Entry {
        ugv_reset_client::Session session;
        bool have{false};
        double received{0.0};
        // For the motion of a body outside a session.
        ugv_reset_client::Pose reference;
        double reference_stamp{0.0};
        double speed{0.0};
        // Refused for this vehicle only.
        bool bad{false};
        std::string bad_reason;
        // The last message sent to it, for the heartbeat.
        bool has_last{false};
        ugv_reset_client::Grant last;
        double last_sent{0.0};
        double first_reject{-1.0};
        bool lost{false};
        bool participant{false};
    };

    ugv_reset_client::Grant grantOf(const PlannerDecision& d, double now) const;
    static bool validSpec(const ugv_reset_client::VehicleSpec& spec);
    static bool participantPhase(ugv_reset_client::Phase phase);

    CoordinatorConfig config_;
    ResetPlanner planner_;
    std::map<std::string, Entry> entries_;
    bool scene_authored_{false};
    bool scene_available_{true};
};

}  // namespace ugv_reset_safety
