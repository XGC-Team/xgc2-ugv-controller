#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "mecanum_ugv_controller/mecanum_ugv_controller.h"

namespace mecanum_ugv_controller {
namespace {

constexpr uint32_t kMinQueueSize = 1U;

double finitePositiveOr(double value, double fallback) {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}

std::string normalize(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

}  // namespace

class MecanumUgvRosNode {
   public:
    explicit MecanumUgvRosNode(ros::NodeHandle& nh)
        : nh_(nh), private_nh_("~"), controller_(state_) {
        loadParams();
        controller_.setConfig(config_);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, queue_size_);
        control_state_pub_ = nh_.advertise<std_msgs::UInt32>(control_state_topic_, queue_size_);
        namespaced_command_sub_ =
            nh_.subscribe("command", queue_size_, &MecanumUgvRosNode::commandCallback, this);
        command_sub_ =
            nh_.subscribe("/command", queue_size_, &MecanumUgvRosNode::commandCallback, this);
        pose_sub_ = nh_.subscribe(pose_topic_, queue_size_, &MecanumUgvRosNode::poseCallback, this);
        twist_sub_ =
            nh_.subscribe(twist_topic_, queue_size_, &MecanumUgvRosNode::twistCallback, this);
        reset_pose_sub_ = nh_.subscribe(reset_pose_topic_, queue_size_,
                                        &MecanumUgvRosNode::resetPoseCallback, this);
        reference_twist_sub_ = nh_.subscribe(reference_twist_topic_, queue_size_,
                                             &MecanumUgvRosNode::referenceTwistCallback, this);
        ROS_INFO("[MecanumUgvRosNode] pose=%s twist=%s reset_pose=%s reference=%s cmd_vel=%s",
                 pose_topic_.c_str(), twist_topic_.c_str(), reset_pose_topic_.c_str(),
                 reference_twist_topic_.c_str(), cmd_vel_topic_.c_str());
    }

    void run() {
        ros::Rate rate(finitePositiveOr(config_.control_rate_hz, 50.0));
        while (ros::ok()) {
            ros::spinOnce();
            controller_.update(ros::Time::now().toSec());
            for (const auto& event : controller_.stateMachine().currentOutputEvents()) {
                if (event.id == output_event_type::PUBLISH_CMD_VEL) {
                    cmd_vel_pub_.publish(makeTwist(controller_.command()));
                } else if (event.id == output_event_type::PUBLISH_ZERO_CMD_VEL) {
                    cmd_vel_pub_.publish(geometry_msgs::Twist{});
                }
            }
            std_msgs::UInt32 status;
            status.data = static_cast<uint32_t>(
                controller_.stateMachine().currentState(region_type::CONTROL));
            control_state_pub_.publish(status);
            rate.sleep();
        }
    }

   private:
    void loadParams() {
        int queue_size = static_cast<int>(queue_size_);
        private_nh_.param("queue_size", queue_size, queue_size);
        queue_size_ = std::max(kMinQueueSize, static_cast<uint32_t>(std::max(1, queue_size)));
        private_nh_.param("pose_topic", pose_topic_, pose_topic_);
        private_nh_.param("twist_topic", twist_topic_, twist_topic_);
        private_nh_.param("reset_pose_topic", reset_pose_topic_, reset_pose_topic_);
        private_nh_.param("reference_twist_topic", reference_twist_topic_, reference_twist_topic_);
        private_nh_.param("cmd_vel_topic", cmd_vel_topic_, cmd_vel_topic_);
        private_nh_.param("control_state_topic", control_state_topic_, control_state_topic_);
        private_nh_.param("control_rate_hz", config_.control_rate_hz, config_.control_rate_hz);
        private_nh_.param("state_timeout", config_.state_timeout, config_.state_timeout);
        private_nh_.param("command_publish_rate_hz", config_.command_publish_rate_hz,
                          config_.command_publish_rate_hz);
        private_nh_.param("placement_idle_silent", config_.placement_idle_silent,
                          config_.placement_idle_silent);
        private_nh_.param("auto_start_tracking", config_.auto_start_tracking,
                          config_.auto_start_tracking);
        private_nh_.param("reference_timeout", config_.reference_timeout,
                          config_.reference_timeout);
        private_nh_.param("heading_target_yaw", config_.heading_target_yaw,
                          config_.heading_target_yaw);
        private_nh_.param("track/kp_yaw", config_.track_kp_yaw, config_.track_kp_yaw);
        private_nh_.param("track/max_speed", config_.track_max_speed, config_.track_max_speed);
        private_nh_.param("track/max_yaw_rate", config_.track_max_yaw_rate,
                          config_.track_max_yaw_rate);
        private_nh_.param("reset/timeout", config_.reset_timeout, config_.reset_timeout);
        private_nh_.param("reset/arrive_position", config_.reset_arrive_position,
                          config_.reset_arrive_position);
        private_nh_.param("reset/arrive_yaw", config_.reset_arrive_yaw, config_.reset_arrive_yaw);
        private_nh_.param("reset/settle_speed", config_.reset_settle_speed,
                          config_.reset_settle_speed);
        private_nh_.param("reset/settle_yaw_rate", config_.reset_settle_yaw_rate,
                          config_.reset_settle_yaw_rate);
        private_nh_.param("reset/kp_xy", config_.reset_kp_xy, config_.reset_kp_xy);
        private_nh_.param("reset/kp_yaw", config_.reset_kp_yaw, config_.reset_kp_yaw);
        private_nh_.param("reset/max_speed", config_.reset_max_speed, config_.reset_max_speed);
        private_nh_.param("reset/max_yaw_rate", config_.reset_max_yaw_rate,
                          config_.reset_max_yaw_rate);
        int settle_frames = config_.reset_settle_frames;
        private_nh_.param("reset/settle_frames", settle_frames, settle_frames);
        config_.reset_settle_frames = settle_frames > 0 ? settle_frames : 8;
        config_.control_rate_hz = finitePositiveOr(config_.control_rate_hz, 50.0);
        config_.state_timeout = finitePositiveOr(config_.state_timeout, 0.2);
        config_.command_publish_rate_hz = finitePositiveOr(config_.command_publish_rate_hz, 50.0);
        config_.reset_timeout = finitePositiveOr(config_.reset_timeout, 45.0);
        config_.reset_arrive_position = finitePositiveOr(config_.reset_arrive_position, 0.40);
        config_.reset_arrive_yaw = finitePositiveOr(config_.reset_arrive_yaw, 0.60);
        config_.reference_timeout = finitePositiveOr(config_.reference_timeout, 0.5);
        config_.track_kp_yaw = finitePositiveOr(config_.track_kp_yaw, 1.2);
        config_.track_max_speed = finitePositiveOr(config_.track_max_speed, 0.8);
        config_.track_max_yaw_rate = finitePositiveOr(config_.track_max_yaw_rate, 0.6);
        if (!std::isfinite(config_.heading_target_yaw)) {
            config_.heading_target_yaw = 0.0;
        }
    }

    void commandCallback(const std_msgs::String::ConstPtr& msg) {
        if (!msg || msg->data.empty()) {
            return;
        }
        const std::string command = normalize(msg->data);
        ::state_machine::EventId id = 0;
        if (command == "reset") {
            id = event_type::RESET_REQUESTED;
        } else if (command == "track" || command == "tracking" || command == "custom" ||
                   command == "custom1" || command == "start") {
            id = event_type::TRACKING_REQUESTED;
        } else if (command == "hold" || command == "stop") {
            id = event_type::HOLD_REQUESTED;
        } else {
            return;
        }
        ::state_machine::Event event(id, ::state_machine::EventTimestamp{ros::Time::now().toSec()});
        event.source = "command";
        event.category = ::state_machine::EventCategory::kInput;
        (void)controller_.postEvent(std::move(event));
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!msg) {
            return;
        }
        double yaw = 0.0;
        if (!tryYawFromQuaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                                  msg->pose.orientation.z, msg->pose.orientation.w, yaw)) {
            return;
        }
        state_.x = msg->pose.position.x;
        state_.y = msg->pose.position.y;
        state_.yaw = yaw;
        pose_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        pose_received_ = true;
        updateState();
    }

    void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        if (!msg) {
            return;
        }
        state_.vx = msg->twist.linear.x;
        state_.vy = msg->twist.linear.y;
        state_.yaw_rate = msg->twist.angular.z;
        twist_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        twist_received_ = true;
        updateState();
    }

    void resetPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
        if (!msg || !std::isfinite(msg->x) || !std::isfinite(msg->y) ||
            !std::isfinite(msg->theta)) {
            return;
        }
        ResetTarget target;
        target.x = msg->x;
        target.y = msg->y;
        target.yaw = wrapAngle(msg->theta);
        target.valid = true;
        controller_.setResetTarget(target);
    }

    void referenceTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        if (!msg || !std::isfinite(msg->twist.linear.x) || !std::isfinite(msg->twist.linear.y)) {
            return;
        }
        WorldVelocityReference reference;
        reference.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        reference.vx = msg->twist.linear.x;
        reference.vy = msg->twist.linear.y;
        reference.valid = true;
        controller_.setWorldReference(reference);
        ::state_machine::Event event(event_type::INPUT_REFERENCE_UPDATED,
                                     ::state_machine::EventTimestamp{reference.stamp.toSec()});
        event.source = "reference_twist";
        event.category = ::state_machine::EventCategory::kInput;
        (void)controller_.postEvent(std::move(event));
    }

    void updateState() {
        if (!pose_received_ || !twist_received_) {
            return;
        }
        state_.stamp = pose_stamp_ < twist_stamp_ ? pose_stamp_ : twist_stamp_;
        state_.received = true;
        ::state_machine::Event event(event_type::INPUT_STATE_UPDATED,
                                     ::state_machine::EventTimestamp{state_.stamp.toSec()});
        event.source = "platform_pose";
        event.category = ::state_machine::EventCategory::kInput;
        (void)controller_.postEvent(std::move(event));
    }

    geometry_msgs::Twist makeTwist(const ControlCommand& command) const {
        geometry_msgs::Twist msg;
        if (!command.valid) {
            return msg;
        }
        msg.linear.x = command.linear_x;
        msg.linear.y = command.linear_y;
        msg.angular.z = command.angular_z;
        return msg;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    UgvState state_;
    MecanumUgvController controller_;
    ControllerConfig config_{};
    uint32_t queue_size_{10U};
    std::string pose_topic_{"pose"};
    std::string twist_topic_{"twist"};
    std::string reset_pose_topic_{"reset_pose"};
    std::string reference_twist_topic_{"alg/reference/twist"};
    std::string cmd_vel_topic_{"cmd_vel"};
    std::string control_state_topic_{"alg/mecanum_ugv_controller/status/control_state"};
    ros::Publisher cmd_vel_pub_;
    ros::Publisher control_state_pub_;
    ros::Subscriber command_sub_;
    ros::Subscriber namespaced_command_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber twist_sub_;
    ros::Subscriber reset_pose_sub_;
    ros::Subscriber reference_twist_sub_;
    ros::Time pose_stamp_;
    ros::Time twist_stamp_;
    bool pose_received_{false};
    bool twist_received_{false};
};

}  // namespace mecanum_ugv_controller

int main(int argc, char** argv) {
    ros::init(argc, argv, "mecanum_ugv_controller");
    ros::NodeHandle nh;
    mecanum_ugv_controller::MecanumUgvRosNode node(nh);
    node.run();
    return 0;
}
