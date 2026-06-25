#pragma once

#include <ros/ros.h>

#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <string>

#include "unicycle_reference_trajectory/ActivePolynomialReference.h"
#include "unicycle_reference_trajectory/AnalyticReference.h"
#include "unicycle_reference_trajectory/ReferenceStatus.h"
#include "unicycle_reference_trajectory/SampledReference.h"
#include "unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h"

namespace unicycle_reference_trajectory {

class ReferenceOutputConsumer final : public ::state_machine::runtime::EventConsumer {
   public:
    ReferenceOutputConsumer(ros::NodeHandle& nh,
                            ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
                            ReferenceTrajectoryRuntime& runtime, const std::string& status_topic,
                            const std::string& active_analytic_topic,
                            const std::string& active_polynomial_topic,
                            const std::string& active_sampled_topic, uint32_t queue_size);

    std::string name() const override {
        return "ReferenceOutputConsumer";
    }
    bool handle(const ::state_machine::Event& event) override;

   private:
    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor_;
    ReferenceTrajectoryRuntime& runtime_;
    ros::Publisher status_pub_;
    ros::Publisher active_analytic_pub_;
    ros::Publisher active_polynomial_pub_;
    ros::Publisher active_sampled_pub_;
};

}  // namespace unicycle_reference_trajectory
