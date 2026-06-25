#pragma once

#include <ros/ros.h>

#include <memory>
#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <string>
#include <vector>

#include "unicycle_ugv_controller/input/command_input_producer.h"
#include "unicycle_ugv_controller/input/reference_input_producer.h"
#include "unicycle_ugv_controller/input/state_input_producer.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

class UnicycleUgvRosNode {
   public:
    explicit UnicycleUgvRosNode(ros::NodeHandle& nh);
    ~UnicycleUgvRosNode();

    void run(double frequency_hz);

   private:
    void loadParams();
    void updateOnce();
    void dispatchOutputEvents(const std::vector<::state_machine::Event>& events);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    UgvState state_;
    UnicycleUgvController controller_;
    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle> output_executor_;
    ::state_machine::runtime::EventDispatcher output_dispatcher_;
    std::unique_ptr<CommandInputProducer> command_input_;
    std::unique_ptr<StateInputProducer> state_input_;
    std::unique_ptr<ReferenceInputProducer> reference_input_;

    ControllerConfig config_{};
    uint32_t queue_size_{10U};
    std::string state_topic_{"alg/state_estimator/state"};
    std::string active_analytic_topic_{"alg/unicycle_reference_trajectory/active/analytic"};
    std::string active_polynomial_topic_{"alg/unicycle_reference_trajectory/active/polynomial"};
    std::string active_sampled_topic_{"alg/unicycle_reference_trajectory/active/sampled"};
    std::string cmd_vel_topic_{"cmd_vel"};
};

}  // namespace unicycle_ugv_controller
