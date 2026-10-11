// ugv_ros_edge: the ROS1 side of a unicycle vehicle entity.
//
// The vehicle's modules (ugv_unicycle_reference, ugv_unicycle_controller) exchange typed payloads
// in the host process; everything the vehicle exchanges with ROS passes through this module, with
// the topics and the messages of the ROS nodes it stands in for:
//
//   ROS in   alg/state_estimator/state or pose   -> state            (RigidStateEstimate /
//   PoseStamped)
//            command and /command                -> command          (std_msgs/String, parsed here)
//            reset_pose                          -> reset_target     (geometry_msgs/Pose2D)
//            reset/response                      -> reset_clearance  (validated by the Reset lease)
//            alg/reference/pva                   -> pva              (flatness strategy only)
//            .../request/{analytic,sampled,waypoint}, .../reset -> the generator's request ports
//            /clock                              -> clock            (simulation time only)
//   ROS out  cmd_vel                             <- cmd_vel          (geometry_msgs/Twist)
//            custom/statustext                   <- controller_status
//            reset/request                       <- reset_session    (with the pose of the latest
//            state)
//            .../status, .../active/{analytic,polynomial,sampled} <- the generator's outputs
//
// ROS runs on a private callback queue served by one thread. That thread is the only writer of the
// output ports (all of them are ASYNC_WRITER); the instance's step reads the input ports and
// publishes. The step is also the period of the Reset lease: it asks the lease for a request about
// every 20 ms, so the instance is run by a period (period_ms = 10 in the manifests).
//
// Clocks. The host clock is CLOCK_MONOTONIC, or in simulation the /clock topic (the manifest's
// clock mode "external" with the channel of this module's clock port, and sim_time = true here).
// The ROS stamps of the messages are converted to the host clock at the receipt time of the message
// (ros::MessageEvent::getReceiptTime), and the times of the messages published back are converted
// the other way; a time that is zero stays zero. With sim_time the two clocks are the same and
// nothing is converted. sim_time has to agree with /use_sim_time: the module refuses to start
// otherwise.
//
// ROS can be started once in a process: the first instance initializes roscpp (node name and master
// from its configuration), later instances share it, and it is shut down when the process ends.
// The library is linked -z nodelete, so unloading it, or replacing it by a newer file, leaves ROS
// running for the instance that continues.
//
// The configuration is a JSON object with the keys below; every key is optional, the defaults are
// those of the ROS nodes. A key the module does not know is an error. A live configure reconnects.
//   node_name and master_uri (used by the instance that starts roscpp), namespace (the vehicle's:
//   the relative topics below, and those of the Reset lease, resolve under it), sim_time,
//   queue_size, state_source ("state_estimator" | "platform_pose"), tracking_strategy ("nmpc" |
//   "flatness"), state_estimate_topic, platform_pose_topic, reset_pose_topic, cmd_vel_topic,
//   control_state_topic, pva_reference_topic, reset_request_topic, reset_response_topic,
//   analytic_topic, waypoint_topic, sampled_topic, reset_topic, status_topic,
//   active_analytic_topic, active_polynomial_topic, active_sampled_topic.

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <ugv_reset_client/reset_client.h>
#include <unicycle_reference_trajectory_msgs/AnalyticReference.h>
#include <unicycle_reference_trajectory_msgs/PlanarPvaReference.h>
#include <unicycle_reference_trajectory_msgs/SampledReference.h>
#include <unicycle_reference_trajectory_msgs/WaypointReferenceRequest.h>
#include <xgc2/module.h>

#include <atomic>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "module_support.h"
#include "reference_payloads.h"
#include "unicycle_reference_trajectory/ros_reference_conversion.h"
#include "unicycle_ugv_controller/common/operator_command.h"
#include "unicycle_ugv_controller/common/rigid_to_unicycle.h"
#include "unicycle_ugv_controller/common/types.h"

#ifndef UGV_MODULES_VERSION
#define UGV_MODULES_VERSION "0.0.0"
#endif

namespace {

using namespace ugv_modules;
namespace rp = ugv_modules::reference_payloads;
namespace urt = unicycle_reference_trajectory;
namespace ugv = unicycle_ugv_controller;
namespace msgs = unicycle_reference_trajectory_msgs;

constexpr const char* kClockSchema = "xgc2.clock.v1";

enum Port : uint32_t {
    // from the other modules
    kCmdVel = 0,
    kControllerStatus,
    kResetSession,
    kReferenceStatus,
    kActiveAnalytic,
    kActivePolynomial,
    kActiveSampled,
    // to the other modules
    kState,
    kCommand,
    kResetTarget,
    kResetClearance,
    kPva,
    kAnalyticRequest,
    kSampledRequest,
    kWaypointRequest,
    kReferenceReset,
    kClock,
    kPortCount
};
constexpr uint32_t kInputCount = kState;

constexpr uint32_t kAsync = XGC2_PORT_ASYNC_WRITER;

const xgc2_port_desc kPorts[kPortCount] = {
    port<xgc2_ugv_cmd_vel>("cmd_vel", XGC2_PORT_IN, XGC2_PORT_STATE, XGC2_UGV_SCHEMA_CMD_VEL),
    port<xgc2_ugv_controller_status>("controller_status", XGC2_PORT_IN, XGC2_PORT_STATE,
                                     XGC2_UGV_SCHEMA_CONTROLLER_STATUS),
    port<xgc2_ugv_reset_session>("reset_session", XGC2_PORT_IN, XGC2_PORT_STATE,
                                 XGC2_UGV_SCHEMA_RESET_SESSION),
    port<xgc2_ugv_reference_status>("reference_status", XGC2_PORT_IN, XGC2_PORT_STATE,
                                    XGC2_UGV_SCHEMA_REFERENCE_STATUS),
    port<xgc2_ugv_analytic_reference>("active_analytic", XGC2_PORT_IN, XGC2_PORT_STATE,
                                      XGC2_UGV_SCHEMA_ANALYTIC_REFERENCE),
    port<xgc2_ugv_polynomial_reference>("active_polynomial", XGC2_PORT_IN, XGC2_PORT_STATE,
                                        XGC2_UGV_SCHEMA_POLYNOMIAL_REFERENCE),
    port<xgc2_ugv_sampled_reference>("active_sampled", XGC2_PORT_IN, XGC2_PORT_STATE,
                                     XGC2_UGV_SCHEMA_SAMPLED_REFERENCE),
    port<xgc2_ugv_planar_state>("state", XGC2_PORT_OUT, XGC2_PORT_STATE,
                                XGC2_UGV_SCHEMA_PLANAR_STATE, 0U, kAsync),
    port<xgc2_ugv_command>("command", XGC2_PORT_OUT, XGC2_PORT_EVENT, XGC2_UGV_SCHEMA_COMMAND, 8U,
                           kAsync),
    port<xgc2_ugv_reset_target>("reset_target", XGC2_PORT_OUT, XGC2_PORT_STATE,
                                XGC2_UGV_SCHEMA_RESET_TARGET, 0U, kAsync),
    port<xgc2_ugv_reset_clearance>("reset_clearance", XGC2_PORT_OUT, XGC2_PORT_STATE,
                                   XGC2_UGV_SCHEMA_RESET_CLEARANCE, 0U, kAsync),
    port<xgc2_ugv_planar_pva>("pva", XGC2_PORT_OUT, XGC2_PORT_STATE, XGC2_UGV_SCHEMA_PLANAR_PVA, 0U,
                              kAsync),
    port<xgc2_ugv_analytic_reference>("analytic_request", XGC2_PORT_OUT, XGC2_PORT_EVENT,
                                      XGC2_UGV_SCHEMA_ANALYTIC_REFERENCE, 8U, kAsync),
    port<xgc2_ugv_sampled_reference>("sampled_request", XGC2_PORT_OUT, XGC2_PORT_EVENT,
                                     XGC2_UGV_SCHEMA_SAMPLED_REFERENCE, 4U, kAsync),
    port<xgc2_ugv_waypoint_request>("waypoint_request", XGC2_PORT_OUT, XGC2_PORT_EVENT,
                                    XGC2_UGV_SCHEMA_WAYPOINT_REQUEST, 4U, kAsync),
    port<xgc2_ugv_reference_reset>("reference_reset", XGC2_PORT_OUT, XGC2_PORT_EVENT,
                                   XGC2_UGV_SCHEMA_REFERENCE_RESET, 2U, kAsync),
    port<int64_t>("clock", XGC2_PORT_OUT, XGC2_PORT_STATE, kClockSchema, 0U, kAsync),
};

// ---- configuration
// ---------------------------------------------------------------------------------

struct EdgeConfig {
    std::string node_name{"ugv_ros_edge"};
    std::string ns;
    std::string master_uri;
    bool sim_time{false};
    uint32_t queue_size{10U};
    bool pose_source{false};  // state_source: platform_pose
    bool flatness{false};     // tracking_strategy: flatness

    std::string state_estimate_topic{"alg/state_estimator/state"};
    std::string platform_pose_topic{"pose"};
    std::string reset_pose_topic{"reset_pose"};
    std::string cmd_vel_topic{"cmd_vel"};
    std::string control_state_topic{"custom/statustext"};
    std::string pva_reference_topic{"alg/reference/pva"};
    std::string reset_request_topic{"reset/request"};
    std::string reset_response_topic{"reset/response"};
    std::string analytic_topic{"alg/unicycle_reference_trajectory/request/analytic"};
    std::string waypoint_topic{"alg/unicycle_reference_trajectory/request/waypoint"};
    std::string sampled_topic{"alg/unicycle_reference_trajectory/request/sampled"};
    std::string reset_topic{"alg/unicycle_reference_trajectory/reset"};
    std::string status_topic{"alg/unicycle_reference_trajectory/status"};
    std::string active_analytic_topic{"alg/unicycle_reference_trajectory/active/analytic"};
    std::string active_polynomial_topic{"alg/unicycle_reference_trajectory/active/polynomial"};
    std::string active_sampled_topic{"alg/unicycle_reference_trajectory/active/sampled"};

    bool operator==(const EdgeConfig& other) const {
        return node_name == other.node_name && ns == other.ns && master_uri == other.master_uri &&
               sim_time == other.sim_time && queue_size == other.queue_size &&
               pose_source == other.pose_source && flatness == other.flatness &&
               state_estimate_topic == other.state_estimate_topic &&
               platform_pose_topic == other.platform_pose_topic &&
               reset_pose_topic == other.reset_pose_topic && cmd_vel_topic == other.cmd_vel_topic &&
               control_state_topic == other.control_state_topic &&
               pva_reference_topic == other.pva_reference_topic &&
               reset_request_topic == other.reset_request_topic &&
               reset_response_topic == other.reset_response_topic &&
               analytic_topic == other.analytic_topic && waypoint_topic == other.waypoint_topic &&
               sampled_topic == other.sampled_topic && reset_topic == other.reset_topic &&
               status_topic == other.status_topic &&
               active_analytic_topic == other.active_analytic_topic &&
               active_polynomial_topic == other.active_polynomial_topic &&
               active_sampled_topic == other.active_sampled_topic;
    }
};

EdgeConfig loadEdgeConfig(const JsonSource& source) {
    EdgeConfig config;
    source.get("node_name", config.node_name);
    source.get("namespace", config.ns);
    source.get("master_uri", config.master_uri);
    source.get("sim_time", config.sim_time);
    int queue_size = static_cast<int>(config.queue_size);
    source.get("queue_size", queue_size);
    if (queue_size < 1) {
        throw std::invalid_argument("queue_size must be at least 1");
    }
    config.queue_size = static_cast<uint32_t>(queue_size);

    std::string state_source = "state_estimator";
    source.get("state_source", state_source);
    if (state_source != "state_estimator" && state_source != "platform_pose") {
        throw std::invalid_argument("Unknown state_source: " + state_source);
    }
    config.pose_source = state_source == "platform_pose";
    std::string strategy = "nmpc";
    source.get("tracking_strategy", strategy);
    if (strategy != "nmpc" && strategy != "flatness") {
        throw std::invalid_argument("Unknown tracking_strategy: " + strategy);
    }
    config.flatness = strategy == "flatness";

    source.get("state_estimate_topic", config.state_estimate_topic);
    source.get("platform_pose_topic", config.platform_pose_topic);
    source.get("reset_pose_topic", config.reset_pose_topic);
    source.get("cmd_vel_topic", config.cmd_vel_topic);
    source.get("control_state_topic", config.control_state_topic);
    source.get("pva_reference_topic", config.pva_reference_topic);
    source.get("reset_request_topic", config.reset_request_topic);
    source.get("reset_response_topic", config.reset_response_topic);
    source.get("analytic_topic", config.analytic_topic);
    source.get("waypoint_topic", config.waypoint_topic);
    source.get("sampled_topic", config.sampled_topic);
    source.get("reset_topic", config.reset_topic);
    source.get("status_topic", config.status_topic);
    source.get("active_analytic_topic", config.active_analytic_topic);
    source.get("active_polynomial_topic", config.active_polynomial_topic);
    source.get("active_sampled_topic", config.active_sampled_topic);
    return config;
}

// ---- roscpp, once per process
// ------------------------------------------------------------------------

class RosProcess {
   public:
    // Starts roscpp on the first call (or finds it started by the process) and returns once it runs
    // against a master that answers.
    static void ensure(const EdgeConfig& config) {
        static std::mutex mutex;
        std::lock_guard<std::mutex> lock(mutex);
        if (ros::isShuttingDown()) {
            throw std::runtime_error(
                "roscpp was shut down in this process and cannot be restarted");
        }
        if (!ros::isInitialized()) {
            std::map<std::string, std::string> remappings;
            if (!config.master_uri.empty()) {
                remappings["__master"] = config.master_uri;
            }
            // The host owns the signals.
            ros::init(remappings, config.node_name, ros::init_options::NoSigintHandler);
        }
        // Asked before roscpp starts: a subscriber would wait for a master that is not there.
        if (!ros::master::check()) {
            throw std::runtime_error("the ROS master is not reachable: " + ros::master::getURI());
        }
        // Started explicitly: the last node handle going away must not shut roscpp down.
        ros::start();
    }
};

// Ends roscpp at process exit, before the libraries it uses. The library is linked -z nodelete, so
// this does not run when the host unloads the module.
struct RosShutdown {
    ~RosShutdown() {
        if (ros::isStarted() && !ros::isShuttingDown()) {
            ros::shutdown();
        }
    }
};
RosShutdown g_ros_shutdown;

// ---- the two clocks
// ------------------------------------------------------------------------------------

// ROS time <-> host time. Without simulation time the offset between the two is read when a time is
// converted: ROS time is the wall clock, the host clock is monotonic.
class ClockBridge {
   public:
    ClockBridge() = default;
    ClockBridge(const Host& host, bool same) : host_(host), same_(same) {}

    int64_t toHost(const ros::Time& t) const {
        if (t.isZero()) {
            return 0;
        }
        const int64_t ros_ns = static_cast<int64_t>(t.toNSec());
        if (same_) {
            return ros_ns;
        }
        const int64_t shifted =
            host_.nowNs() - (static_cast<int64_t>(ros::Time::now().toNSec()) - ros_ns);
        return shifted > 0 ? shifted : 1;  // a time that was set stays set
    }

    ros::Time toRos(int64_t host_ns) const {
        if (host_ns <= 0) {
            return ros::Time();
        }
        if (same_) {
            return ros::Time().fromNSec(static_cast<uint64_t>(host_ns));
        }
        const int64_t ros_ns =
            static_cast<int64_t>(ros::Time::now().toNSec()) + (host_ns - host_.nowNs());
        return ros_ns > 0 ? ros::Time().fromNSec(static_cast<uint64_t>(ros_ns)) : ros::Time(0, 1);
    }

    urt::Time toHost(const urt::Time& t) const {
        return rp::timeOfNs(toHost(ros::Time(t.sec, t.nsec)));
    }
    urt::Time toRos(const urt::Time& t) const {
        const ros::Time out = toRos(rp::nsOfTime(t));
        return urt::Time(out.sec, out.nsec);
    }

   private:
    Host host_;
    bool same_{false};
};

// ---- the instance
// --------------------------------------------------------------------------------------

class Instance {
   public:
    explicit Instance(const Host& host) : host_(host) {}

    ~Instance() {
        disconnect();
    }

    const Host& host() const {
        return host_;
    }

    void configure(const xgc2_config* config) {
        JsonSource source(config);
        EdgeConfig next = loadEdgeConfig(source);
        source.rejectUnused();
        if (connected_ && !(next == config_)) {
            disconnect();
            config_ = next;
            connect();
            return;
        }
        config_ = next;
    }

    void start() {
        connect();
        host_.report(kHealthOk, "connected");
    }

    void stop() {
        disconnect();
    }

    void step(const xgc2_step_ctx& ctx) {
        if (!connected_) {
            return;
        }
        publishCmdVel(ctx.now_ns);
        publishControllerStatus();
        publishReferenceOutputs();
        updateResetClient(ctx.now_ns);
    }

   private:
    // ---- ROS handles --------------------------------------------------------------------------

    void connect() {
        if (connected_) {
            return;
        }
        RosProcess::ensure(config_);
        if (ros::Time::isSimTime() != config_.sim_time) {
            throw std::runtime_error(
                config_.sim_time ? "sim_time is set but /use_sim_time is not set on the ROS master"
                                 : "/use_sim_time is set on the ROS master but sim_time is not");
        }
        bridge_ = ClockBridge(host_, config_.sim_time);
        seen_.forget();
        queue_ = std::make_unique<ros::CallbackQueue>();
        // The vehicle's namespace is the one of the node handle: relative topics, the lease's
        // included, resolve under it; "/command" does not.
        nh_ = std::make_unique<ros::NodeHandle>(config_.ns);
        nh_->setCallbackQueue(queue_.get());
        const uint32_t q = config_.queue_size;

        if (config_.sim_time) {
            clock_sub_ = nh_->subscribe("/clock", 100, &Instance::onClock, this,
                                        ros::TransportHints().tcpNoDelay());
        }
        if (config_.pose_source) {
            pose_sub_ = nh_->subscribe(config_.platform_pose_topic, q, &Instance::onPose, this);
        } else {
            state_sub_ =
                nh_->subscribe(config_.state_estimate_topic, q, &Instance::onEstimate, this);
        }
        command_sub_ = nh_->subscribe("command", q, &Instance::onNamespacedCommand, this);
        public_command_sub_ = nh_->subscribe("/command", q, &Instance::onPublicCommand, this);
        reset_target_sub_ =
            nh_->subscribe(config_.reset_pose_topic, q, &Instance::onResetTarget, this);
        if (config_.flatness) {
            pva_sub_ = nh_->subscribe(config_.pva_reference_topic, q, &Instance::onPva, this);
        }
        analytic_sub_ = nh_->subscribe(config_.analytic_topic, q, &Instance::onAnalytic, this);
        waypoint_sub_ = nh_->subscribe(config_.waypoint_topic, q, &Instance::onWaypoint, this);
        sampled_sub_ = nh_->subscribe(config_.sampled_topic, q, &Instance::onSampled, this);
        reset_sub_ = nh_->subscribe(config_.reset_topic, q, &Instance::onReferenceReset, this);

        cmd_vel_pub_ = nh_->advertise<geometry_msgs::Twist>(config_.cmd_vel_topic, 1);
        control_state_pub_ = nh_->advertise<std_msgs::String>(config_.control_state_topic, q);
        status_pub_ = nh_->advertise<msgs::ReferenceStatus>(config_.status_topic, q, true);
        analytic_pub_ =
            nh_->advertise<msgs::AnalyticReference>(config_.active_analytic_topic, q, true);
        polynomial_pub_ = nh_->advertise<msgs::ActivePolynomialReference>(
            config_.active_polynomial_topic, q, true);
        sampled_pub_ =
            nh_->advertise<msgs::SampledReference>(config_.active_sampled_topic, q, true);

        reset_client_ = std::make_unique<ugv_reset_client::ResetClient>(
            *nh_,
            [this](const ugv_reset_client::ResetLease::Clearance& clearance) {
                onClearance(clearance);
            },
            config_.reset_request_topic, config_.reset_response_topic);
        spinner_ = std::make_unique<ros::AsyncSpinner>(1, queue_.get());
        spinner_->start();
        connected_ = true;
        host_.log(kLogInfo, format("Connected to ROS as %s%s", ros::this_node::getName().c_str(),
                                   config_.sim_time ? " on simulation time" : ""));
    }

    // Nothing of the module runs on a ROS thread once this returns.
    void disconnect() {
        if (spinner_) {
            spinner_->stop();
            spinner_.reset();
        }
        reset_client_.reset();
        clock_sub_.shutdown();
        state_sub_.shutdown();
        pose_sub_.shutdown();
        command_sub_.shutdown();
        public_command_sub_.shutdown();
        reset_target_sub_.shutdown();
        pva_sub_.shutdown();
        analytic_sub_.shutdown();
        waypoint_sub_.shutdown();
        sampled_sub_.shutdown();
        reset_sub_.shutdown();
        cmd_vel_pub_.shutdown();
        control_state_pub_.shutdown();
        status_pub_.shutdown();
        analytic_pub_.shutdown();
        polynomial_pub_.shutdown();
        sampled_pub_.shutdown();
        nh_.reset();
        if (queue_) {
            queue_->clear();
            queue_.reset();
        }
        connected_ = false;
    }

    // ---- ROS in: the callbacks run on the one ROS thread, the only writer of the output ports
    // ----

    void dropped(const char* what) {
        if (drop_throttle_.due(static_cast<double>(host_.nowNs()) * 1e-9)) {
            host_.log(kLogWarn, std::string("Dropped ") + what + ": the channel does not take it");
        }
    }
    void refused(const char* what, const char* why) {
        if (refuse_throttle_.due(static_cast<double>(host_.nowNs()) * 1e-9)) {
            host_.log(kLogWarn, std::string("Refused ") + what + ": " + why);
        }
    }

    void rememberPose(double x, double y, double yaw, const ros::Time& stamp) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        pose_ = {x, y, yaw};
        pose_stamp_ = stamp;
    }

    void onClock(const rosgraph_msgs::Clock::ConstPtr& msg) {
        const int64_t ns = static_cast<int64_t>(msg->clock.toNSec());
        auto slot = host_.write<int64_t>(kClock);
        if (!slot) {
            return;
        }
        *slot = ns;
        slot.commit(ns);
    }

    void onEstimate(
        const ros::MessageEvent<const rigid_state_estimator_msgs::RigidStateEstimate>& event) {
        const auto& msg = *event.getConstMessage();
        const ros::Time receipt = event.getReceiptTime();
        const ros::Time stamp = msg.header.stamp.isZero() ? receipt : msg.header.stamp;
        const ugv::UnicycleProjection planar = ugv::projectRigidToUnicycle(msg);
        auto slot = host_.write<xgc2_ugv_planar_state>(kState);
        if (!slot) {
            dropped("a state estimate");
            return;
        }
        slot->stamp_ns = bridge_.toHost(stamp);
        slot->x = planar.x;
        slot->y = planar.y;
        slot->yaw = planar.yaw;
        slot->vx = msg.velocity.x;
        slot->vy = msg.velocity.y;
        slot->speed = planar.speed;
        slot->yaw_rate = planar.yaw_rate;
        slot->estimator_state = msg.estimator_state;
        slot->estimator_flags = msg.flags;
        slot->source = XGC2_UGV_STATE_SOURCE_ESTIMATE;
        slot->velocity_valid =
            std::isfinite(msg.velocity.x) && std::isfinite(msg.velocity.y) ? 1 : 0;
        slot.commit(bridge_.toHost(receipt));
        rememberPose(planar.x, planar.y, planar.yaw, stamp);
    }

    void onPose(const ros::MessageEvent<const geometry_msgs::PoseStamped>& event) {
        const auto& msg = *event.getConstMessage();
        const ros::Time receipt = event.getReceiptTime();
        const ros::Time stamp = msg.header.stamp.isZero() ? receipt : msg.header.stamp;
        double yaw = 0.0;
        if (!ugv::tryYawFromQuaternion(msg.pose.orientation.x, msg.pose.orientation.y,
                                       msg.pose.orientation.z, msg.pose.orientation.w, yaw)) {
            refused("a pose", "invalid quaternion");
            return;
        }
        if (!std::isfinite(msg.pose.position.x) || !std::isfinite(msg.pose.position.y)) {
            return;
        }
        auto slot = host_.write<xgc2_ugv_planar_state>(kState);
        if (!slot) {
            dropped("a pose");
            return;
        }
        slot->stamp_ns = bridge_.toHost(stamp);
        slot->x = msg.pose.position.x;
        slot->y = msg.pose.position.y;
        slot->yaw = yaw;
        slot->source = XGC2_UGV_STATE_SOURCE_POSE;
        slot.commit(bridge_.toHost(receipt));
        rememberPose(msg.pose.position.x, msg.pose.position.y, yaw, stamp);
    }

    void onNamespacedCommand(const ros::MessageEvent<const std_msgs::String>& event) {
        handleCommand(event, XGC2_UGV_COMMAND_SOURCE_NAMESPACED, "command");
    }
    void onPublicCommand(const ros::MessageEvent<const std_msgs::String>& event) {
        handleCommand(event, XGC2_UGV_COMMAND_SOURCE_PUBLIC, "/command");
    }
    void handleCommand(const ros::MessageEvent<const std_msgs::String>& event, uint32_t source,
                       const char* topic) {
        const std::string& text = event.getConstMessage()->data;
        if (text.empty()) {
            host_.log(kLogWarn, format("Ignoring empty command on %s", topic));
            return;
        }
        uint32_t kind = 0;
        switch (ugv::parseOperatorCommand(text)) {
            case ugv::OperatorCommand::kCustom1:
                kind = XGC2_UGV_COMMAND_CUSTOM1;
                break;
            case ugv::OperatorCommand::kStop:
                kind = XGC2_UGV_COMMAND_STOP;
                break;
            case ugv::OperatorCommand::kReset:
                kind = XGC2_UGV_COMMAND_RESET;
                break;
            case ugv::OperatorCommand::kUnknown:
                host_.log(kLogWarn, format("Unknown command: %s on %s", text.c_str(), topic));
                return;
        }
        auto slot = host_.write<xgc2_ugv_command>(kCommand);
        if (!slot) {
            dropped("a command");
            return;
        }
        const int64_t receipt = bridge_.toHost(event.getReceiptTime());
        slot->stamp_ns = receipt;
        slot->kind = kind;
        slot->source = source;
        slot.commit(receipt);
        host_.log(kLogInfo, format("Accepted command %s on %s", text.c_str(), topic));
    }

    void onResetTarget(const ros::MessageEvent<const geometry_msgs::Pose2D>& event) {
        const auto& msg = *event.getConstMessage();
        auto slot = host_.write<xgc2_ugv_reset_target>(kResetTarget);
        if (!slot) {
            dropped("a reset target");
            return;
        }
        slot->x = msg.x;
        slot->y = msg.y;
        slot->yaw = msg.theta;
        slot.commit(bridge_.toHost(event.getReceiptTime()));
    }

    // The lease validated the response; the controller gets it in its own clock.
    void onClearance(const ugv_reset_client::ResetLease::Clearance& clearance) {
        auto slot = host_.write<xgc2_ugv_reset_clearance>(kResetClearance);
        if (!slot) {
            dropped("a reset clearance");
            return;
        }
        slot->stamp_ns = bridge_.toHost(ros::Time().fromNSec(clearance.stamp));
        slot->issue_wall = clearance.issue_wall;
        slot->linear_x = clearance.command.x;
        slot->linear_y = clearance.command.y;
        slot->yaw_rate = clearance.command.yaw;
        slot->lease_seconds = clearance.lease_seconds;
        slot->generation = clearance.generation;
        slot->status = clearance.status;
        slot.commit(host_.nowNs());
    }

    void onPva(const ros::MessageEvent<const msgs::PlanarPvaReference>& event) {
        const auto& msg = *event.getConstMessage();
        auto slot = host_.write<xgc2_ugv_planar_pva>(kPva);
        if (!slot) {
            dropped("a PVA reference");
            return;
        }
        const int64_t receipt = bridge_.toHost(event.getReceiptTime());
        slot->stamp_ns = receipt;
        slot->x = msg.x;
        slot->y = msg.y;
        slot->yaw = msg.yaw;
        slot->vx = msg.vx;
        slot->vy = msg.vy;
        slot->ax = msg.ax;
        slot->ay = msg.ay;
        slot.commit(receipt);
    }

    // A request of the reference generator: the message in the plain form, its times in the host
    // clock, then the payload.
    template <typename Payload, typename Message>
    void forwardRequest(const ros::MessageEvent<const Message>& event, uint32_t port_index,
                        const char* what) {
        auto plain = urt::toCore(*event.getConstMessage());
        plain.header.stamp = bridge_.toHost(plain.header.stamp);
        mapStartTime(plain);
        if (!rp::fits(plain)) {
            refused(what, "it does not fit the payload");
            return;
        }
        auto slot = host_.write<Payload>(port_index);
        if (!slot) {
            dropped(what);
            return;
        }
        rp::toPayload(*slot, plain);
        slot.commit(bridge_.toHost(event.getReceiptTime()));
    }
    void mapStartTime(urt::reference::AnalyticReference& plain) const {
        plain.start_time = bridge_.toHost(plain.start_time);
    }
    void mapStartTime(urt::reference::SampledReference& plain) const {
        plain.start_time = bridge_.toHost(plain.start_time);
    }
    void mapStartTime(urt::reference::WaypointReferenceRequest&) const {}

    void onAnalytic(const ros::MessageEvent<const msgs::AnalyticReference>& event) {
        forwardRequest<xgc2_ugv_analytic_reference>(event, kAnalyticRequest,
                                                    "an analytic reference");
    }
    void onSampled(const ros::MessageEvent<const msgs::SampledReference>& event) {
        forwardRequest<xgc2_ugv_sampled_reference>(event, kSampledRequest, "a sampled reference");
    }
    void onWaypoint(const ros::MessageEvent<const msgs::WaypointReferenceRequest>& event) {
        forwardRequest<xgc2_ugv_waypoint_request>(event, kWaypointRequest, "a waypoint request");
    }
    void onReferenceReset(const ros::MessageEvent<const std_msgs::Empty>& event) {
        auto slot = host_.write<xgc2_ugv_reference_reset>(kReferenceReset);
        if (!slot) {
            dropped("a reference reset");
            return;
        }
        const int64_t receipt = bridge_.toHost(event.getReceiptTime());
        slot->stamp_ns = receipt;
        slot.commit(receipt);
    }

    // ---- ROS out: the step ----------------------------------------------------------------------

    void publishCmdVel(int64_t) {
        const auto sample = host_.latest<xgc2_ugv_cmd_vel>(kCmdVel);
        if (!seen_.fresh(kCmdVel, sample)) {
            return;
        }
        geometry_msgs::Twist twist;
        twist.linear.x = sample->linear_x;
        twist.linear.y = sample->linear_y;
        twist.angular.z = sample->angular_z;
        cmd_vel_pub_.publish(twist);
        if (reset_client_) {
            reset_client_->noteApplied({twist.linear.x, twist.linear.y, twist.angular.z});
        }
    }

    void publishControllerStatus() {
        const auto sample = host_.latest<xgc2_ugv_controller_status>(kControllerStatus);
        if (!seen_.fresh(kControllerStatus, sample)) {
            return;
        }
        std_msgs::String text;
        text.data.assign(sample->control_state_name,
                         strnlen(sample->control_state_name, sizeof(sample->control_state_name)));
        if (text.data.empty()) {
            text.data = "Unknown";
        }
        control_state_pub_.publish(text);
    }

    template <typename Payload, typename Plain, typename Publisher>
    void publishActive(uint32_t port_index, Publisher& publisher, const char* what) {
        const auto sample = host_.latest<Payload>(port_index);
        if (!seen_.fresh(port_index, sample)) {
            return;
        }
        Plain plain;
        if (!rp::toPlain(*sample, plain)) {
            host_.log(kLogWarn,
                      std::string("Refused the active ") + what + ": a count exceeds its payload");
            return;
        }
        plain.header.frame_id = "world";
        plain.header.stamp = bridge_.toRos(plain.header.stamp);
        mapStartTimeToRos(plain);
        publisher.publish(urt::toRos(plain));
    }
    void mapStartTimeToRos(urt::reference::AnalyticReference& plain) const {
        plain.start_time = bridge_.toRos(plain.start_time);
    }
    void mapStartTimeToRos(urt::reference::SampledReference& plain) const {
        plain.start_time = bridge_.toRos(plain.start_time);
    }
    void mapStartTimeToRos(urt::reference::ActivePolynomialReference& plain) const {
        plain.start_time = bridge_.toRos(plain.start_time);
    }

    void publishReferenceOutputs() {
        const auto status = host_.latest<xgc2_ugv_reference_status>(kReferenceStatus);
        if (seen_.fresh(kReferenceStatus, status)) {
            auto plain = rp::toPlain(*status);
            plain.header.stamp = bridge_.toRos(plain.header.stamp);
            status_pub_.publish(urt::toRos(plain));
        }
        publishActive<xgc2_ugv_analytic_reference, urt::reference::AnalyticReference>(
            kActiveAnalytic, analytic_pub_, "analytic reference");
        publishActive<xgc2_ugv_polynomial_reference, urt::reference::ActivePolynomialReference>(
            kActivePolynomial, polynomial_pub_, "polynomial reference");
        publishActive<xgc2_ugv_sampled_reference, urt::reference::SampledReference>(
            kActiveSampled, sampled_pub_, "sampled reference");
    }

    // The lease follows the controller's Reset session and asks the coordinator for a clearance
    // with the latest pose of the vehicle. It is called at every step: the lease limits itself to a
    // request about every 20 ms.
    void updateResetClient(int64_t) {
        const auto session = host_.latest<xgc2_ugv_reset_session>(kResetSession);
        if (!session || !reset_client_) {
            return;
        }
        ugv_reset_client::ResetLease::Pose pose;
        ros::Time pose_stamp;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            pose = pose_;
            pose_stamp = pose_stamp_;
        }
        reset_client_->update((session->flags & XGC2_UGV_RESET_SESSION_ACTIVE) != 0U,
                              session->generation,
                              {session->target_x, session->target_y, session->target_yaw}, pose,
                              pose_stamp, (session->flags & XGC2_UGV_RESET_SESSION_HEALTHY) != 0U);
    }

    Host host_;
    EdgeConfig config_;
    bool connected_{false};
    ClockBridge bridge_;
    SeenSamples seen_;
    LogThrottle drop_throttle_;
    LogThrottle refuse_throttle_;

    std::mutex pose_mutex_;
    ugv_reset_client::ResetLease::Pose pose_;
    ros::Time pose_stamp_;

    std::unique_ptr<ros::CallbackQueue> queue_;
    std::unique_ptr<ros::NodeHandle> nh_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;
    std::unique_ptr<ugv_reset_client::ResetClient> reset_client_;
    ros::Subscriber clock_sub_, state_sub_, pose_sub_, command_sub_, public_command_sub_,
        reset_target_sub_, pva_sub_, analytic_sub_, waypoint_sub_, sampled_sub_, reset_sub_;
    ros::Publisher cmd_vel_pub_, control_state_pub_, status_pub_, analytic_pub_, polynomial_pub_,
        sampled_pub_;
};

const xgc2_module_desc kDescriptor = {XGC2_MODULE_ABI_MAJOR,
                                      XGC2_MODULE_ABI_MINOR,
                                      "ugv_ros_edge",
                                      UGV_MODULES_VERSION,
                                      kPorts,
                                      kPortCount,
                                      Lifecycle<Instance>::create,
                                      Lifecycle<Instance>::configure,
                                      Lifecycle<Instance>::start,
                                      Lifecycle<Instance>::step,
                                      Lifecycle<Instance>::stop,
                                      Lifecycle<Instance>::destroy};

}  // namespace

UGV_MODULE_EXPORT const xgc2_module_desc* xgc2_module_entry(void) {
    return &kDescriptor;
}
