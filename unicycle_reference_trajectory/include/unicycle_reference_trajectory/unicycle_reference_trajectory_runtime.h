#pragma once

#include <memory>
#include <state_machine/state_machine.hpp>
#include <xgc2_math/trajectory.hpp>

#include "unicycle_reference_trajectory/ActivePolynomialReference.h"
#include "unicycle_reference_trajectory/AnalyticReference.h"
#include "unicycle_reference_trajectory/ReferenceStatus.h"
#include "unicycle_reference_trajectory/SampledReference.h"
#include "unicycle_reference_trajectory/WaypointReferenceRequest.h"
#include "unicycle_reference_trajectory/state_machine/event_types.h"

namespace unicycle_reference_trajectory {

namespace trajectory = xgc2_math::trajectory;

struct ReferenceTrajectoryConfig {
    double status_rate_hz{10.0};
    double active_publish_rate_hz{10.0};
    double validation_sample_dt{0.02};
    double trajectory_timeout{0.5};
    double min_lead_time{0.2};
    trajectory::TrajectoryLimits2 limits{};
};

class ReferenceTrajectoryRuntime {
   public:
    ReferenceTrajectoryRuntime();
    ReferenceTrajectoryRuntime(const ReferenceTrajectoryRuntime&) = delete;
    ReferenceTrajectoryRuntime& operator=(const ReferenceTrajectoryRuntime&) = delete;

    void setConfig(const ReferenceTrajectoryConfig& config);
    void reset();
    ::state_machine::Status postEvent(::state_machine::Event event);
    void update(double now_sec);

    bool acceptAnalytic(const AnalyticReference& msg);
    bool acceptSampled(const SampledReference& msg);
    bool acceptWaypoint(const WaypointReferenceRequest& msg);

    bool activatePending();
    bool planPendingWaypoint();
    bool activeExpired(double now_sec) const;

    void enterState(uint8_t state);
    uint8_t currentState() const {
        return state_;
    }
    double currentTime() const {
        return current_time_sec_;
    }
    const ReferenceTrajectoryConfig& config() const {
        return config_;
    }
    trajectory::TrajectoryModelType activeType() const {
        return active_type_;
    }
    uint32_t activeTrajectoryId() const {
        return active_trajectory_id_;
    }
    uint32_t activeRevision() const {
        return active_revision_;
    }
    uint32_t flags() const {
        return flags_;
    }

    const AnalyticReference& activeAnalyticMessage() const {
        return active_analytic_;
    }
    const SampledReference& activeSampledMessage() const {
        return active_sampled_;
    }
    const ActivePolynomialReference& activePolynomialMessage() const {
        return active_polynomial_;
    }
    ReferenceStatus makeStatus(double stamp_sec) const;
    const trajectory::TrajectoryEvaluator2* evaluator() const {
        return active_evaluator_.get();
    }
    ::state_machine::StateMachine& stateMachine() {
        return *machine_;
    }

   private:
    enum class PendingKind { kNone, kAnalytic, kSampled, kWaypoint };

    void setupMachine();
    std::unique_ptr<trajectory::TrajectoryEvaluator2> buildAnalyticEvaluator(
        const AnalyticReference& msg, uint32_t& flags) const;
    bool buildSampledEvaluator(const SampledReference& msg,
                               trajectory::SampledEvaluator2& evaluator, uint32_t& flags) const;
    bool buildWaypointProblem(const WaypointReferenceRequest& msg,
                              trajectory::WaypointProblem2& problem, uint32_t& flags) const;
    void setActiveAnalytic(const AnalyticReference& msg,
                           std::unique_ptr<trajectory::TrajectoryEvaluator2> evaluator,
                           uint32_t flags);
    void setActiveSampled(const SampledReference& msg,
                          std::unique_ptr<trajectory::TrajectoryEvaluator2> evaluator,
                          uint32_t flags);
    void setActivePolynomial(ActivePolynomialReference msg,
                             std::unique_ptr<trajectory::TrajectoryEvaluator2> evaluator,
                             uint32_t flags);

    ReferenceTrajectoryConfig config_{};
    std::unique_ptr<::state_machine::StateMachine> machine_;
    uint8_t state_{ReferenceStatus::STATE_SELF_CHECK};
    double current_time_sec_{0.0};
    uint32_t flags_{0U};

    PendingKind pending_kind_{PendingKind::kNone};
    AnalyticReference pending_analytic_;
    SampledReference pending_sampled_;
    WaypointReferenceRequest pending_waypoint_;

    trajectory::TrajectoryModelType active_type_{trajectory::TrajectoryModelType::kNone};
    uint32_t active_trajectory_id_{0U};
    uint32_t active_revision_{0U};
    double active_start_sec_{0.0};
    double active_duration_{0.0};
    std::unique_ptr<trajectory::TrajectoryEvaluator2> active_evaluator_;
    AnalyticReference active_analytic_;
    SampledReference active_sampled_;
    ActivePolynomialReference active_polynomial_;
};

}  // namespace unicycle_reference_trajectory
