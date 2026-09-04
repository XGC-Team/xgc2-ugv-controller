#pragma once

#include <ros/time.h>
#include <unicycle_reference_trajectory_msgs/ActivePolynomialReference.h>
#include <unicycle_reference_trajectory_msgs/AnalyticReference.h>
#include <unicycle_reference_trajectory_msgs/SampledReference.h>

#include <memory>
#include <mutex>
#include <vector>
#include <xgc2_math/control.hpp>
#include <xgc2_math/trajectory.hpp>

namespace unicycle_ugv_controller {

class ReferenceCache {
   public:
    bool updateAnalytic(const unicycle_reference_trajectory_msgs::AnalyticReference& msg);
    bool updatePolynomial(const unicycle_reference_trajectory_msgs::ActivePolynomialReference& msg);
    bool updateSampled(const unicycle_reference_trajectory_msgs::SampledReference& msg);
    void clear();
    bool valid() const;
    bool sampleHorizon(const ros::Time& now, double stage_dt, int horizon_steps,
                       std::vector<xgc2_math::control::Se2Reference>& refs) const;

   private:
    bool activeLocked() const;

    mutable std::mutex mutex_;
    std::shared_ptr<const xgc2_math::trajectory::TrajectoryEvaluator2> evaluator_;
    ros::Time start_time_;
    uint32_t trajectory_id_{0U};
    uint32_t revision_{0U};
    uint32_t flags_{0U};
};

}  // namespace unicycle_ugv_controller
