#pragma once

// The Reset planner: the ROS-free heart of the coordinator.
//
// It grants a frozen ordinary route and a free prefix using the declared body/spec truth.
// The geometric verifier samples ideal routes at three ell values for a unicycle, or one
// circumscribed omnidirectional footprint. These checks are not a proof of continuous real
// feedback/lag/coupling plant containment, heading coverage or braking/rear-release safety.
//
// Causality: one issued Plan remains reserved while an exact Brake is in flight. Only a
// matching (S,G,Brake id) post-zero stopping certificate retires it. Waiting/Idle/loss never
// retire it; only exact latest Go receipt permits the existing rear-cursor advancement.
// Replacement uses the same route planner, within initial plus two routes per S.
//
// Decisions are deterministic in id order, with no per-tick route replanning. Geometry and
// real-plant acceptance remain separate from the causal-retirement regression tests.

#include <Eigen/Core>
#include <map>
#include <memory>
#include <string>
#include <ugv_reset_client/reset_protocol.h>
#include <ugv_reset_client/reset_route.h>
#include <ugv_reset_safety/reset_geometry.h>
#include <vector>

namespace ugv_reset_safety {

struct PlannerConfig {
    // Required clearance of the continuous sweep. The check on the sampled sweep adds the sampling
    // error (SweepSampling) so that these numbers hold between the samples as well.
    double pair_margin{0.10};    // body to body
    double static_margin{0.12};  // body to static
    double fence_margin{0.05};   // body to the fence
    // Braking distance plus tracking slack kept free ahead of the granted prefix.
    double hold_margin{0.45};
    // A prefix shorter than this is not worth a grant.
    double min_prefix{0.25};
    // Sweep sampling: the route is sampled at most this far apart.
    double step{0.02};
    double step_yaw{1.5 * ugv_reset_client::kPi / 180.0};
    // Waiting vehicles are re-evaluated at this period, and prefixes of movers are extended at this
    // one (events trigger an evaluation at once).
    double replan_period{0.5};
    double extension_period{0.2};
    // Movers exist but nobody advanced for this long: the held movers are rejected as stalled.
    double stall_s{10.0};
    // A body outside any session that sits on a target is only a blocker if it stays for this long
    // (the members of one Reset click enter their sessions within a fraction of a second).
    double occupancy_settle_s{1.0};
    // Early refusal: the predicted duration of the own route above refuse_factor x timeout.
    double timeout{ugv_reset_client::kResetTimeout};
    double refuse_factor{1.5};
};

// One vehicle as the coordinator reports it (its latest Session, plus the lost/moving judgement).
struct PlannerVehicle {
    std::string id;
    uint32_t session{0};
    uint32_t request_generation{0};
    uint32_t acknowledged_grant_id{0};
    bool stop_certified{false};
    bool available{true};
    ugv_reset_client::VehicleSpec spec;
    ugv_reset_client::Pose pose;
    ugv_reset_client::Pose target;
    ugv_reset_client::Phase phase{ugv_reset_client::Phase::Idle};
    double ell_hat{0.0};
    double ell_spread{-1.0};
    // A body outside a session that is measured to be moving: it is a parked body at its pose, and
    // no new route is granted while it moves.
    bool uncontrolled_moving{false};
};

// The planner's answer for one vehicle; only changes are reported.
struct PlannerDecision {
    std::string id;
    uint32_t session{0};
    uint32_t request_generation{0};
    ugv_reset_client::GrantState state{ugv_reset_client::GrantState::Hold};
    ugv_reset_client::Reason reason{ugv_reset_client::Reason::None};
    std::string detail;
    std::vector<std::string> blockers;
    // Every state has a checked nonzero decision identity.
    uint32_t grant_id{0};
    ugv_reset_client::Pose start;
    ugv_reset_client::Pose goal;
    bool final_move{true};
    double ell_plan{0.0};
    std::vector<ugv_reset_client::Point> via;
    double s_limit{0.0};
};

// What the planner currently holds for a vehicle (for the fleet summary and for tests).
struct PlannerBodyReport {
    std::string id;
    bool participant{false};
    bool has_route{false};
    double progress{0.0};   // arc length travelled on the route
    double remaining{0.0};  // arc length left
    double s_limit{0.0};
    ugv_reset_client::GrantState state{ugv_reset_client::GrantState::Hold};
    ugv_reset_client::Reason reason{ugv_reset_client::Reason::None};
    std::vector<std::string> blockers;
};

struct PlannerStats {
    long events{0};
    long admissions{0};
    long extensions{0};
    long revocations{0};
    long parks{0};
    long rejections{0};
    double plan_ms_max{0.0};
    double plan_ms_sum{0.0};
    long plan_calls{0};
};

class ResetPlanner {
   public:
    explicit ResetPlanner(const PlannerConfig& config = PlannerConfig());
    ~ResetPlanner();
    ResetPlanner(const ResetPlanner&) = delete;
    ResetPlanner& operator=(const ResetPlanner&) = delete;

    // The statics (Scene geometry with its motion envelope, later non-participant circles) and
    // the fence. A change that touches a held route revokes it at the next update.
    void setWorld(const std::vector<ConvexObstacle>& statics, const Fence& fence);

    // One planning step at time `now` (the coordinator's clock) for all vehicles it knows.
    // Vehicles must be in id order. Without `admit` (an authored Scene is not available) the
    // issued Plans are braked/retained and nobody is granted a new route.
    std::vector<PlannerDecision> update(double now, const std::vector<PlannerVehicle>& vehicles,
                                        bool admit = true);

    std::vector<PlannerBodyReport> report() const;
    const PlannerStats& stats() const;
    const PlannerConfig& config() const;
    // All logical decisions, including Coordinator-generated ones, share this checked owner.
    uint32_t allocateGrantId();

    // The sampling error the sweep check adds to a margin, for a footprint whose farthest point
    // is `radius` from the rotation centre: half a translation step or half a rotation step.
    static double samplingError(const PlannerConfig& config, double radius);

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace ugv_reset_safety
