#pragma once

#include <ros/ros.h>
#include <ros1_utils/adaptive_owner_loop.h>
#include <std_msgs/String.h>
#include <ugv_reset_client/reset_ros.h>
#include <ugv_reset_client/schedule.h>

#include <memory>
#include <string>
#include <vector>

#include "unicycle_ugv_controller/input/command_input_producer.h"
#include "unicycle_ugv_controller/input/pva_reference_input_producer.h"
#include "unicycle_ugv_controller/input/reference_input_producer.h"
#include "unicycle_ugv_controller/input/reset_target_input_producer.h"
#include "unicycle_ugv_controller/input/state_input_producer.h"
#include "unicycle_ugv_controller/output/cmd_vel_output_consumer.h"
#include "unicycle_ugv_controller/output/nmpc_output_consumer.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

class UnicycleUgvRosNode {
   public:
    explicit UnicycleUgvRosNode(ros::NodeHandle& nh);
    ~UnicycleUgvRosNode();

    void run(double frequency_hz);

   private:
    void loadParams();
    void seedResetTarget();
    void updateOnce();
    void dispatchOutputs(const std::vector<ControllerOutput>& outputs);
    void publishStatusIfDue();
    void logStateChanges();

    ros1_utils::AdaptiveOwnerLoop loop_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    UgvState state_;
    UnicycleUgvController controller_;
    std::unique_ptr<ugv_reset_client::ResetRosEdge> reset_edge_;
    std::unique_ptr<CmdVelOutputConsumer> command_output_;
    std::unique_ptr<NmpcOutputConsumer> nmpc_output_;
    bool command_turn_{false};
    std::unique_ptr<CommandInputProducer> command_input_;
    std::unique_ptr<StateInputProducer> state_input_;
    std::unique_ptr<ReferenceInputProducer> reference_input_;
    std::unique_ptr<PvaReferenceInputProducer> pva_reference_input_;
    std::unique_ptr<ResetTargetInputProducer> reset_target_input_;

    ControllerConfig config_{};
    uint32_t queue_size_{10U};
    std::string state_topic_{"alg/state_estimator/state"};
    std::string platform_pose_topic_{"pose"};
    std::string reset_pose_topic_{"reset_pose"};
    std::string active_analytic_topic_{"alg/unicycle_reference_trajectory/active/analytic"};
    std::string active_polynomial_topic_{"alg/unicycle_reference_trajectory/active/polynomial"};
    std::string active_sampled_topic_{"alg/unicycle_reference_trajectory/active/sampled"};
    std::string pva_reference_topic_{"alg/reference/pva"};
    std::string cmd_vel_topic_{"cmd_vel"};
    std::string control_state_topic_{"custom/statustext"};
    ros::Publisher control_state_pub_;
    std::optional<ControlMode> last_logged_control_state_;
    std::string last_logged_reset_miss_;
};

}  // namespace unicycle_ugv_controller
