#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include <xgc2_math/control.hpp>
#include <xgc2_math/trajectory.hpp>

#include "unicycle_ugv_controller/common/reference_types.h"
#include "unicycle_ugv_controller/common/time.h"

namespace unicycle_ugv_controller {

class ReferenceCache {
   public:
    bool updateAnalytic(const reference::AnalyticReference& msg);
    bool updatePolynomial(const reference::ActivePolynomialReference& msg);
    bool updateSampled(const reference::SampledReference& msg);
    void clear();
    bool valid() const;
    bool sampleHorizon(const Time& now, double stage_dt, int horizon_steps,
                       std::vector<xgc2_math::control::Se2Reference>& refs) const;

   private:
    bool activeLocked() const;

    mutable std::mutex mutex_;
    std::shared_ptr<const xgc2_math::trajectory::TrajectoryEvaluator2> evaluator_;
    Time start_time_;
    uint32_t trajectory_id_{0U};
    uint32_t revision_{0U};
    uint32_t flags_{0U};
};

}  // namespace unicycle_ugv_controller
