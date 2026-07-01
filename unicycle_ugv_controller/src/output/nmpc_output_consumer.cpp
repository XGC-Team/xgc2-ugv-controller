#include "unicycle_ugv_controller/output/nmpc_output_consumer.h"

#include <ros/console.h>

#include <cmath>
#include <utility>

#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {
namespace {

geometry_msgs::Pose poseFromState(const Se2StateVector& state) {
    geometry_msgs::Pose pose;
    pose.position.x = state(0);
    pose.position.y = state(1);
    pose.position.z = 0.05;
    const double yaw = state(2);
    if (std::isfinite(yaw)) {
        pose.orientation.z = std::sin(0.5 * yaw);
        pose.orientation.w = std::cos(0.5 * yaw);
    } else {
        pose.orientation.w = 1.0;
    }
    return pose;
}

void unwrapReferenceYaw(std::vector<Se2Reference>& refs, double anchor_yaw) {
    if (!std::isfinite(anchor_yaw)) {
        return;
    }
    double previous_yaw = anchor_yaw;
    for (auto& ref : refs) {
        if (!std::isfinite(ref.state.yaw)) {
            continue;
        }
        ref.state.yaw = previous_yaw + wrapAngle(ref.state.yaw - previous_yaw);
        previous_yaw = ref.state.yaw;
    }
}

}  // namespace

NmpcOutputConsumer::NmpcOutputConsumer(ros::NodeHandle& nh, UnicycleUgvController& controller,
                                       EventSink event_sink, uint32_t queue_size)
    : controller_(controller), event_sink_(std::move(event_sink)) {
    predicted_path_pub_ = nh.advertise<nav_msgs::Path>("alg/nmpc/predicted_path", queue_size);
    predicted_poses_pub_ =
        nh.advertise<geometry_msgs::PoseArray>("alg/nmpc/predicted_poses", queue_size);
    backend_.configure(controller_.config());
    worker_ = std::thread(&NmpcOutputConsumer::workerLoop, this);
}

NmpcOutputConsumer::~NmpcOutputConsumer() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    condition_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    backend_.exit();
}

bool NmpcOutputConsumer::handle(const ::state_machine::Event& event) {
    if (event.id != output_event_type::REQUEST_NMPC_SOLVE) {
        return false;
    }

    const ControllerConfig config = controller_.config();
    const ros::Time now(event.timestamp > 0.0 ? event.timestamp : ros::Time::now().toSec());
    Request request;
    request.sequence = event.correlation_id;
    request.now = now;
    request.state = controller_.state();
    request.config = config;
    const double stage_dt =
        config.prediction_horizon / static_cast<double>(UnicycleNmpcSolver::horizonSteps());
    if (!controller_.referenceCache().sampleHorizon(now, stage_dt,
                                                    UnicycleNmpcSolver::horizonSteps(),
                                                    config.reference_timeout, request.references)) {
        ROS_WARN_THROTTLE(
            1.0,
            "[UgvNmpcOutputConsumer] Reject solve seq=%lu: reference horizon unavailable "
            "now=%.3f stage_dt=%.3f timeout=%.3f",
            static_cast<unsigned long>(request.sequence), now.toSec(), stage_dt,
            config.reference_timeout);
        reject(request.sequence);
        return true;
    }
    unwrapReferenceYaw(request.references, request.state.yaw);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (busy_ || has_pending_) {
            ROS_WARN_THROTTLE(1.0, "[UgvNmpcOutputConsumer] Reject solve seq=%lu: backend busy",
                              static_cast<unsigned long>(request.sequence));
            reject(request.sequence);
            return true;
        }
        pending_ = std::move(request);
        has_pending_ = true;
    }
    condition_.notify_one();
    return true;
}

void NmpcOutputConsumer::workerLoop() {
    bool entered = false;
    while (true) {
        Request request;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this] { return stop_ || has_pending_; });
            if (stop_) {
                return;
            }
            request = std::move(pending_);
            has_pending_ = false;
            busy_ = true;
        }

        backend_.configure(request.config);
        if (!entered) {
            entered = backend_.enter();
            if (!entered) {
                ROS_WARN("[UgvNmpcOutputConsumer] Failed to enter NMPC backend");
            }
        }
        ControlCommand command;
        const bool ok =
            entered && backend_.compute(request.state, request.references, request.now, command);
        if (ok) {
            controller_.setCommand(command);
            publishPrediction(request.now);
            ROS_INFO_THROTTLE(1.0,
                              "[UgvNmpcOutputConsumer] Solve ok seq=%lu linear=%.3f angular=%.3f "
                              "solve=%.3f ms",
                              static_cast<unsigned long>(request.sequence), command.linear_speed,
                              command.angular_speed, backend_.solveTimeMs());
        } else {
            ROS_WARN_THROTTLE(1.0,
                              "[UgvNmpcOutputConsumer] Solve failed seq=%lu status=%d "
                              "solve=%.3f ms refs=%zu",
                              static_cast<unsigned long>(request.sequence), backend_.status(),
                              backend_.solveTimeMs(), request.references.size());
        }
        postResultEvent(request.sequence, ok);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            busy_ = false;
        }
    }
}

void NmpcOutputConsumer::reject(uint64_t sequence) {
    postResultEvent(sequence, false);
}

void NmpcOutputConsumer::postResultEvent(uint64_t sequence, bool success) {
    if (!event_sink_) {
        return;
    }
    ::state_machine::Event event(
        success ? event_type::INPUT_NMPC_SOLVE_SUCCEEDED : event_type::INPUT_NMPC_SOLVE_FAILED,
        ::state_machine::EventTimestamp{ros::Time::now().toSec()});
    event.source = "nmpc_output_consumer";
    event.category = ::state_machine::EventCategory::kInput;
    event.correlation_id = sequence;
    (void)event_sink_(std::move(event));
}

void NmpcOutputConsumer::publishPrediction(const ros::Time& stamp) {
    nav_msgs::Path path;
    geometry_msgs::PoseArray poses;
    path.header.stamp = stamp;
    path.header.frame_id = "world";
    poses.header = path.header;

    const auto& predicted_states = backend_.predictedStates();
    const size_t count = backend_.predictedStateCount();
    path.poses.reserve(count);
    poses.poses.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const geometry_msgs::Pose pose = poseFromState(predicted_states[i]);
        geometry_msgs::PoseStamped stamped_pose;
        stamped_pose.header = path.header;
        stamped_pose.pose = pose;
        path.poses.push_back(stamped_pose);
        poses.poses.push_back(pose);
    }

    predicted_path_pub_.publish(path);
    predicted_poses_pub_.publish(poses);
}

}  // namespace unicycle_ugv_controller
