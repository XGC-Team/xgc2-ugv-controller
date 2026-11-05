#pragma once

#include <cstdint>
#include <memory>
#include <state_machine/adt_machine.hpp>
#include <variant>
#include <vector>
#include <xgc2_math/trajectory.hpp>

#include "unicycle_reference_trajectory/reference_types.h"
#include "unicycle_reference_trajectory/state_machine/periodic_gate.h"

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

enum class ReferenceOutput : uint32_t {
    Status = 1000,
    Analytic = 1001,
    Polynomial = 1002,
    Sampled = 1003
};
struct ReferencePublication {
    ReferenceOutput kind;
    double timestamp;
};

class ReferenceTrajectoryRuntime {
   public:
    ReferenceTrajectoryRuntime();
    ReferenceTrajectoryRuntime(const ReferenceTrajectoryRuntime&) = delete;
    ReferenceTrajectoryRuntime& operator=(const ReferenceTrajectoryRuntime&) = delete;
    void setConfig(const ReferenceTrajectoryConfig& config);
    void reset();
    void update(double now_sec);
    bool acceptAnalytic(const reference::AnalyticReference& msg, double receipt_time);
    bool acceptSampled(const reference::SampledReference& msg, double receipt_time);
    bool acceptWaypoint(const reference::WaypointReferenceRequest& msg, double receipt_time);
    uint8_t currentState() const;
    double currentTime() const {
        return current_time_sec_;
    }
    double nextDeadline() const;
    const ReferenceTrajectoryConfig& config() const {
        return config_;
    }
    trajectory::TrajectoryModelType activeType() const;
    uint32_t activeTrajectoryId() const;
    uint32_t activeRevision() const;
    uint32_t flags() const {
        return flags_;
    }
    const reference::AnalyticReference& activeAnalyticMessage() const;
    const reference::SampledReference& activeSampledMessage() const;
    const reference::ActivePolynomialReference& activePolynomialMessage() const;
    reference::ReferenceStatus makeStatus(double stamp_sec) const;
    const trajectory::TrajectoryEvaluator2* evaluator() const;
    const std::vector<ReferencePublication>& outputs() const {
        return outputs_;
    }

   private:
    struct Analytic {
        reference::AnalyticReference message;
        std::unique_ptr<trajectory::TrajectoryEvaluator2> evaluator;
        uint32_t flags;
    };
    struct Sampled {
        reference::SampledReference message;
        std::unique_ptr<trajectory::TrajectoryEvaluator2> evaluator;
        uint32_t flags;
    };
    struct Polynomial {
        reference::ActivePolynomialReference message;
        std::unique_ptr<trajectory::TrajectoryEvaluator2> evaluator;
        uint32_t flags;
    };
    struct Waypoint {
        reference::WaypointReferenceRequest message;
        trajectory::WaypointProblem2 problem;
        uint32_t flags;
    };
    using Retained = std::variant<std::monostate, Analytic, Sampled, Polynomial>;
    using Request = std::variant<std::monostate, Analytic, Sampled, Waypoint>;
    struct SelfCheck {};
    struct Ready {
        PeriodicGate status;
    };
    struct Planning {
        Waypoint request;
    };
    struct Active {
        PeriodicGate status, publication;
    };
    struct Advance {};
    struct Spec {
        using State = std::variant<SelfCheck, Ready, Planning, Active>;
        using Time = double;
        using Change = ::state_machine::adt::Transition<SelfCheck, Ready, Planning, Active>;
        static Change handle(SelfCheck&, Advance, ReferenceTrajectoryRuntime&);
        static Change handle(Ready&, Advance, ReferenceTrajectoryRuntime&);
        static Change handle(Planning&, Advance, ReferenceTrajectoryRuntime&);
        static Change handle(Active&, Advance, ReferenceTrajectoryRuntime&);
    };
    bool activateRequest();
    bool planWaypoint(Waypoint& request);
    bool activeExpired(double now_sec) const;
    double activeEnd() const;
    void publish(PeriodicGate& gate, double rate, ReferenceOutput output);
    void publishActive(Active& state);
    std::unique_ptr<trajectory::TrajectoryEvaluator2> buildAnalyticEvaluator(
        const reference::AnalyticReference& msg, uint32_t& flags) const;
    bool buildSampledEvaluator(const reference::SampledReference& msg,
                               trajectory::SampledEvaluator2& evaluator, uint32_t& flags) const;
    bool buildWaypointProblem(const reference::WaypointReferenceRequest& msg,
                              trajectory::WaypointProblem2& problem, uint32_t& flags) const;

    ReferenceTrajectoryConfig config_{};
    ::state_machine::adt::AdtMachine<Spec> machine_{SelfCheck{}, 0.0};
    double current_time_sec_{0.0};
    uint32_t flags_{0U};
    // Latest replaceable trajectory request; transport commands are consumed on the owner.
    Request request_;
    // Last trajectory is still reported in Ready after expiry, until replacement/reset.
    Retained retained_;
    std::vector<ReferencePublication> outputs_;
};
}  // namespace unicycle_reference_trajectory
