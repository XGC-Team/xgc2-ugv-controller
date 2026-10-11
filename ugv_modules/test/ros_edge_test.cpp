// The ROS edge as a shared library in an in-test host, against a ROS master that rostest starts:
// the topics the vehicle talks, the payloads they become, and the other way round.

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <ugv_reset_safety/ResetRequest.h>
#include <ugv_reset_safety/ResetResponse.h>
#include <unicycle_reference_trajectory_msgs/AnalyticReference.h>
#include <unicycle_reference_trajectory_msgs/PlanarPvaReference.h>
#include <unicycle_reference_trajectory_msgs/SampledReference.h>
#include <unicycle_reference_trajectory_msgs/WaypointReferenceRequest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "edge_test_support.h"
#include "reference_payloads.h"
#include "test_host.h"
#include "unicycle_reference_trajectory/ros_reference_conversion.h"
#include "xgc2_ugv/payloads.h"

namespace {

using edge_test::connected;
using edge_test::waitUntil;
using ugv_modules_test::ModuleLibrary;
using ugv_modules_test::TestHost;
namespace msgs = unicycle_reference_trajectory_msgs;
namespace rp = ugv_modules::reference_payloads;
namespace urt = unicycle_reference_trajectory;

constexpr int64_t kMs = 1000000LL;

// The library stays loaded for the whole run: roscpp ends with the process, not with a test.
ModuleLibrary& library() {
    static ModuleLibrary library(EDGE_MODULE_PATH);
    return library;
}

double wallSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// The messages of a topic, received on the test's own ROS callback thread.
template <typename M>
class Probe {
   public:
    Probe(ros::NodeHandle& nh, const std::string& topic) {
        sub_ = nh.subscribe(topic, 20, &Probe::callback, this);
    }
    size_t count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return messages_.size();
    }
    M last() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return messages_.empty() ? M() : messages_.back();
    }
    M at(size_t i) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return messages_.at(i);
    }
    bool waitFor(size_t n, int timeout_ms = 3000) const {
        return waitUntil([&] { return count() >= n; }, timeout_ms);
    }

   private:
    void callback(const boost::shared_ptr<const M>& message) {
        std::lock_guard<std::mutex> lock(mutex_);
        messages_.push_back(*message);
    }
    ros::Subscriber sub_;
    mutable std::mutex mutex_;
    std::vector<M> messages_;
};

class EdgeTest : public ::testing::Test {
   protected:
    EdgeTest() : host_(library().desc()) {
        host_.useMonotonicClock();
    }
    ~EdgeTest() override {
        host_.stop();
        host_.destroy();
    }

    void startEdge(const std::string& config = "{}") {
        ASSERT_EQ(host_.create(config), XGC2_OK);
        ASSERT_EQ(host_.start(), XGC2_OK);
    }
    void step() {
        ASSERT_EQ(host_.step(XGC2_STEP_TIMER), XGC2_OK);
    }
    template <typename T>
    std::vector<T> waitOutputs(const std::string& name, size_t n, int timeout_ms = 3000) {
        EXPECT_TRUE(waitUntil([&] { return host_.outputs(name).size() >= n; }, timeout_ms))
            << "no " << n << " sample(s) on " << name;
        return host_.outputsAs<T>(name);
    }

    static rigid_state_estimator_msgs::RigidStateEstimate estimate(const ros::Time& stamp) {
        rigid_state_estimator_msgs::RigidStateEstimate e;
        e.header.stamp = stamp;
        e.position.x = 1.0;
        e.position.y = 2.0;
        const double yaw = 0.6;
        e.orientation.z = std::sin(0.5 * yaw);
        e.orientation.w = std::cos(0.5 * yaw);
        e.velocity.x = 0.3;
        e.velocity.y = 0.4;
        e.angular_velocity.z = 0.25;
        e.estimator_state = 3;
        e.flags = 256;
        return e;
    }

    ros::NodeHandle nh_;
    TestHost host_;
};

TEST_F(EdgeTest, DescribesItsPortsForTheHost) {
    const xgc2_module_desc* d = library().desc();
    EXPECT_EQ(d->abi_major, XGC2_MODULE_ABI_MAJOR);
    EXPECT_STREQ(d->name, "ugv_ros_edge");
    ASSERT_EQ(d->port_count, 17u);
    const char* inputs[] = {"cmd_vel",          "controller_status", "reset_session",
                            "reference_status", "active_analytic",   "active_polynomial",
                            "active_sampled"};
    const char* outputs[] = {"state",
                             "command",
                             "reset_target",
                             "reset_clearance",
                             "pva",
                             "analytic_request",
                             "sampled_request",
                             "waypoint_request",
                             "reference_reset",
                             "clock"};
    for (uint32_t i = 0; i < 7; ++i) {
        EXPECT_STREQ(d->ports[i].name, inputs[i]);
        EXPECT_EQ(d->ports[i].direction, static_cast<uint32_t>(XGC2_PORT_IN));
        EXPECT_EQ(d->ports[i].kind, static_cast<uint32_t>(XGC2_PORT_STATE));
        EXPECT_EQ(d->ports[i].flags, 0u);
    }
    for (uint32_t i = 0; i < 10; ++i) {
        const xgc2_port_desc& p = d->ports[7 + i];
        EXPECT_STREQ(p.name, outputs[i]);
        EXPECT_EQ(p.direction, static_cast<uint32_t>(XGC2_PORT_OUT));
        // Everything the edge writes, ROS threads write.
        EXPECT_EQ(p.flags, static_cast<uint32_t>(XGC2_PORT_ASYNC_WRITER)) << p.name;
        EXPECT_EQ(p.align, 8u) << p.name;
        EXPECT_EQ(p.kind, std::string(p.name).find("request") != std::string::npos ||
                                  std::string(p.name) == "command" ||
                                  std::string(p.name) == "reference_reset"
                              ? static_cast<uint32_t>(XGC2_PORT_EVENT)
                              : static_cast<uint32_t>(XGC2_PORT_STATE))
            << p.name;
    }
    EXPECT_STREQ(d->ports[16].schema_id, "xgc2.clock.v1");
    EXPECT_EQ(d->ports[16].size, 8u);
    EXPECT_STREQ(d->ports[7].schema_id, "xgc2.ugv.planar_state.v1");
    EXPECT_STREQ(d->ports[12].schema_id, "xgc2.ugv.unicycle_reference.analytic.v1");
}

TEST_F(EdgeTest, TheConfigurationIsStrict) {
    TestHost host(library().desc());
    EXPECT_EQ(host.create("{\"state_topic\": \"x\"}"), XGC2_ERR_INVALID);
    EXPECT_TRUE(host.logged(3, "unknown configuration key 'state_topic'"));
    EXPECT_EQ(host.create("{\"state_source\": \"gps\"}"), XGC2_ERR_INVALID);
    EXPECT_TRUE(host.logged(3, "Unknown state_source: gps"));
    EXPECT_EQ(host.create("{\"tracking_strategy\": \"lqr\"}"), XGC2_ERR_INVALID);
    EXPECT_EQ(host.create("{\"queue_size\": 0}"), XGC2_ERR_INVALID);
    EXPECT_EQ(host.create("{\"queue_size\": 1.5}"), XGC2_ERR_INVALID);
    EXPECT_EQ(host.create("{\"sim_time\": \"yes\"}"), XGC2_ERR_INVALID);
    EXPECT_EQ(host.create("{\"cmd_vel_topic\": 3}"), XGC2_ERR_INVALID);
    EXPECT_EQ(host.create("{\"queue_size\": 5, \"namespace\": \"\", \"sim_time\": false}"),
              XGC2_OK);
}

TEST_F(EdgeTest, ASimTimeConfigurationAgainstAWallClockMasterDoesNotStart) {
    ASSERT_EQ(host_.create("{\"sim_time\": true}"), XGC2_OK);
    EXPECT_EQ(host_.start(), XGC2_ERR_INTERNAL);
    EXPECT_TRUE(host_.logged(3, "sim_time is set but /use_sim_time is not set"));
}

TEST_F(EdgeTest, AStateEstimateBecomesThePlanarStateInTheHostClock) {
    startEdge();
    ros::Publisher pub = nh_.advertise<rigid_state_estimator_msgs::RigidStateEstimate>(
        "alg/state_estimator/state", 5);
    ASSERT_TRUE(connected(pub));

    const int64_t before = host_.now();
    auto message = estimate(ros::Time::now() - ros::Duration(0.5));
    pub.publish(message);
    const auto states = waitOutputs<xgc2_ugv_planar_state>("state", 1);
    const int64_t after = host_.now();
    ASSERT_EQ(states.size(), 1u);
    const xgc2_ugv_planar_state& s = states[0];
    EXPECT_DOUBLE_EQ(s.x, 1.0);
    EXPECT_DOUBLE_EQ(s.y, 2.0);
    EXPECT_NEAR(s.yaw, 0.6, 1e-12);
    EXPECT_DOUBLE_EQ(s.vx, 0.3);
    EXPECT_DOUBLE_EQ(s.vy, 0.4);
    EXPECT_NEAR(s.speed, std::cos(0.6) * 0.3 + std::sin(0.6) * 0.4, 1e-12);  // along the heading
    EXPECT_DOUBLE_EQ(s.yaw_rate, 0.25);
    EXPECT_EQ(s.estimator_state, 3u);
    EXPECT_EQ(s.estimator_flags, 256u);
    EXPECT_EQ(s.source, XGC2_UGV_STATE_SOURCE_ESTIMATE);
    EXPECT_EQ(s.velocity_valid, 1);
    // The measurement time is half a second before the receipt, in the host clock.
    EXPECT_GE(s.stamp_ns, before - 500 * kMs - 5 * kMs);
    EXPECT_LE(s.stamp_ns, after - 500 * kMs + 5 * kMs);
    // The commit is stamped with the receipt.
    const auto samples = host_.outputs("state");
    EXPECT_GE(samples[0].stamp_ns, before);
    EXPECT_LE(samples[0].stamp_ns, after);
}

TEST_F(EdgeTest, AnUnstampedEstimateIsStampedWithItsReceiptTime) {
    startEdge();
    ros::Publisher pub = nh_.advertise<rigid_state_estimator_msgs::RigidStateEstimate>(
        "alg/state_estimator/state", 5);
    ASSERT_TRUE(connected(pub));
    auto message = estimate(ros::Time());
    message.velocity.x = std::nan("");
    const int64_t before = host_.now();
    pub.publish(message);
    const auto states = waitOutputs<xgc2_ugv_planar_state>("state", 1);
    const int64_t after = host_.now();
    EXPECT_GE(states[0].stamp_ns, before);
    EXPECT_LE(states[0].stamp_ns, after);
    EXPECT_EQ(states[0].velocity_valid, 0);
}

TEST_F(EdgeTest, APoseBecomesThePlanarStateOfAPoseSource) {
    startEdge("{\"state_source\": \"platform_pose\"}");
    ros::Publisher pub = nh_.advertise<geometry_msgs::PoseStamped>("pose", 5);
    ros::Publisher estimates = nh_.advertise<rigid_state_estimator_msgs::RigidStateEstimate>(
        "alg/state_estimator/state", 5);
    ASSERT_TRUE(connected(pub));
    EXPECT_FALSE(waitUntil([&] { return estimates.getNumSubscribers() > 0; }, 300))
        << "a pose source does not listen to the estimator";

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = -1.5;
    pose.pose.position.y = 0.5;
    pose.pose.orientation.z = 2.0 * std::sin(0.25);  // not normalized
    pose.pose.orientation.w = 2.0 * std::cos(0.25);
    pub.publish(pose);
    const auto states = waitOutputs<xgc2_ugv_planar_state>("state", 1);
    EXPECT_DOUBLE_EQ(states[0].x, -1.5);
    EXPECT_DOUBLE_EQ(states[0].y, 0.5);
    EXPECT_NEAR(states[0].yaw, 0.5, 1e-12);
    EXPECT_EQ(states[0].source, XGC2_UGV_STATE_SOURCE_POSE);
    EXPECT_EQ(states[0].velocity_valid, 0);
    EXPECT_EQ(states[0].speed, 0.0);

    // A pose without a rotation, or without a position, is not a state.
    geometry_msgs::PoseStamped bad = pose;
    bad.pose.orientation = geometry_msgs::Quaternion();
    pub.publish(bad);
    bad = pose;
    bad.pose.position.x = std::nan("");
    pub.publish(bad);
    EXPECT_TRUE(waitUntil([&] { return host_.logged(2, "Refused a pose: invalid quaternion"); }));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_EQ(host_.outputs("state").size(), 1u);
}

TEST_F(EdgeTest, OperatorCommandsOnBothTopicsBecomeEvents) {
    startEdge("{\"namespace\": \"ugv1\"}");
    ros::Publisher namespaced = nh_.advertise<std_msgs::String>("ugv1/command", 5);
    ros::Publisher open = nh_.advertise<std_msgs::String>("/command", 5);
    ASSERT_TRUE(connected(namespaced));
    ASSERT_TRUE(connected(open));
    auto send = [](ros::Publisher& pub, const char* text) {
        std_msgs::String message;
        message.data = text;
        pub.publish(message);
    };
    const int64_t before = host_.now();
    send(namespaced, "Track");
    ASSERT_TRUE(waitUntil([&] { return host_.outputs("command").size() == 1; }));
    send(open, "stop");
    ASSERT_TRUE(waitUntil([&] { return host_.outputs("command").size() == 2; }));
    send(open, "reset");  // (ROS orders the messages of one topic, not of two)
    ASSERT_TRUE(waitUntil([&] { return host_.outputs("command").size() == 3; }));
    send(namespaced, "bogus");
    send(namespaced, "");
    send(namespaced, "start");
    ASSERT_TRUE(waitUntil([&] { return host_.outputs("command").size() == 4; }));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const auto commands = host_.outputsAs<xgc2_ugv_command>("command");
    ASSERT_EQ(commands.size(), 4u);
    EXPECT_EQ(commands[0].kind, XGC2_UGV_COMMAND_CUSTOM1);
    EXPECT_EQ(commands[0].source, XGC2_UGV_COMMAND_SOURCE_NAMESPACED);
    EXPECT_EQ(commands[1].kind, XGC2_UGV_COMMAND_STOP);
    EXPECT_EQ(commands[1].source, XGC2_UGV_COMMAND_SOURCE_PUBLIC);
    EXPECT_EQ(commands[2].kind, XGC2_UGV_COMMAND_RESET);
    EXPECT_EQ(commands[2].source, XGC2_UGV_COMMAND_SOURCE_PUBLIC);
    EXPECT_EQ(commands[3].kind, XGC2_UGV_COMMAND_CUSTOM1);
    EXPECT_EQ(commands[3].source, XGC2_UGV_COMMAND_SOURCE_NAMESPACED);
    for (const auto& c : commands) {
        EXPECT_GE(c.stamp_ns, before);
        EXPECT_LE(c.stamp_ns, host_.now());
    }
    const auto samples = host_.outputs("command");
    for (size_t i = 0; i < samples.size(); ++i) {
        EXPECT_EQ(samples[i].stamp_ns, commands[i].stamp_ns);
    }
    EXPECT_TRUE(host_.logged(2, "Unknown command: bogus on command"));
    EXPECT_TRUE(host_.logged(2, "Ignoring empty command on command"));
    // Without a namespace the two topics are one, and a command arrives on both.
    ASSERT_EQ(host_.configure("{}"), XGC2_OK);
    ros::Publisher flat = nh_.advertise<std_msgs::String>("command", 5);
    ASSERT_TRUE(connected(flat));
    send(flat, "stop");
    ASSERT_TRUE(waitUntil([&] { return host_.outputs("command").size() == 6; }));
}

TEST_F(EdgeTest, TheResetPoseBecomesTheResetTarget) {
    startEdge();
    ros::Publisher pub = nh_.advertise<geometry_msgs::Pose2D>("reset_pose", 5);
    ASSERT_TRUE(connected(pub));
    geometry_msgs::Pose2D pose;
    pose.x = 2.5;
    pose.y = -1.0;
    pose.theta = 7.0;  // not wrapped here: the controller owns that
    pub.publish(pose);
    const auto targets = waitOutputs<xgc2_ugv_reset_target>("reset_target", 1);
    EXPECT_DOUBLE_EQ(targets[0].x, 2.5);
    EXPECT_DOUBLE_EQ(targets[0].y, -1.0);
    EXPECT_DOUBLE_EQ(targets[0].yaw, 7.0);
}

TEST_F(EdgeTest, ReferenceRequestsBecomePayloadsWithTheirTimesInTheHostClock) {
    startEdge();
    ros::Publisher analytic = nh_.advertise<msgs::AnalyticReference>(
        "alg/unicycle_reference_trajectory/request/analytic", 5);
    ros::Publisher sampled = nh_.advertise<msgs::SampledReference>(
        "alg/unicycle_reference_trajectory/request/sampled", 5);
    ros::Publisher waypoint = nh_.advertise<msgs::WaypointReferenceRequest>(
        "alg/unicycle_reference_trajectory/request/waypoint", 5);
    ros::Publisher reset =
        nh_.advertise<std_msgs::Empty>("alg/unicycle_reference_trajectory/reset", 5);
    ASSERT_TRUE(connected(analytic));
    ASSERT_TRUE(connected(sampled));
    ASSERT_TRUE(connected(waypoint));
    ASSERT_TRUE(connected(reset));

    // An analytic request: a start time in the future is shifted into the host clock, zero stays
    // zero.
    msgs::AnalyticReference a;
    a.header.stamp = ros::Time::now();
    a.request_id = 11;
    a.trajectory_id = 3;
    a.revision = 2;
    a.analytic_type = msgs::AnalyticReference::ANALYTIC_CIRCLE;
    a.start_time = ros::Time::now() + ros::Duration(2.0);
    a.duration = 6.0;
    a.origin.position.x = 0.5;
    a.origin.orientation.w = 1.0;
    a.params = {2.0, 0.8, 3.0, 0.0, 1.0};
    const int64_t before = host_.now();
    analytic.publish(a);
    auto analytics = waitOutputs<xgc2_ugv_analytic_reference>("analytic_request", 1);
    const int64_t after = host_.now();
    EXPECT_EQ(analytics[0].trajectory_id, 3u);
    EXPECT_EQ(analytics[0].revision, 2u);
    EXPECT_EQ(analytics[0].request_id, 11u);
    EXPECT_EQ(analytics[0].param_count, 5u);
    EXPECT_DOUBLE_EQ(analytics[0].params[0], 2.0);
    EXPECT_DOUBLE_EQ(analytics[0].origin_x, 0.5);
    EXPECT_GE(analytics[0].stamp_ns, before - 5 * kMs);
    EXPECT_LE(analytics[0].stamp_ns, after + 5 * kMs);
    EXPECT_GE(analytics[0].start_time_ns, before + 2000 * kMs - 5 * kMs);
    EXPECT_LE(analytics[0].start_time_ns, after + 2000 * kMs + 5 * kMs);
    a.start_time = ros::Time();
    analytic.publish(a);
    analytics = waitOutputs<xgc2_ugv_analytic_reference>("analytic_request", 2);
    EXPECT_EQ(analytics[1].start_time_ns, 0);

    // A sampled request.
    msgs::SampledReference s;
    s.header.stamp = ros::Time::now();
    s.trajectory_id = 5;
    s.revision = 1;
    s.flags = msgs::SampledReference::FLAG_EXPLICIT_PLANAR_KINEMATICS;
    s.sample_dt = 0.05;
    for (int k = 0; k < 40; ++k) {
        msgs::PlanarReferencePoint p;
        p.t_from_start = 0.05 * k;
        p.x = 0.1 * k;
        p.y = 0.01 * k;
        p.yaw = 0.02 * k;
        p.speed = 0.5;
        p.jy = -1.0 * k;
        s.points.push_back(p);
    }
    sampled.publish(s);
    const auto sampleds = waitOutputs<xgc2_ugv_sampled_reference>("sampled_request", 1);
    EXPECT_EQ(sampleds[0].point_count, 40u);
    EXPECT_DOUBLE_EQ(sampleds[0].points[39].x, 3.9);
    EXPECT_DOUBLE_EQ(sampleds[0].points[39].jy, -39.0);
    EXPECT_EQ(sampleds[0].flags, msgs::SampledReference::FLAG_EXPLICIT_PLANAR_KINEMATICS);
    EXPECT_EQ(sampleds[0].start_time_ns, 0);

    // A waypoint request: the header stamp is the requested start time.
    msgs::WaypointReferenceRequest w;
    w.header.stamp = ros::Time::now() + ros::Duration(1.5);
    w.request_id = 21;
    w.trajectory_id = 4;
    for (int k = 0; k < 3; ++k) {
        geometry_msgs::Pose pose;
        pose.position.x = 1.0 * k;
        pose.orientation.w = 1.0;
        w.waypoints.push_back(pose);
    }
    w.segment_times = {2.0, 2.0};
    w.start_velocity.x = 0.2;
    w.desired_speed = 0.6;
    w.objective = 1;
    const int64_t before_w = host_.now();
    waypoint.publish(w);
    auto waypoints = waitOutputs<xgc2_ugv_waypoint_request>("waypoint_request", 1);
    const int64_t after_w = host_.now();
    EXPECT_EQ(waypoints[0].waypoint_count, 3u);
    EXPECT_EQ(waypoints[0].segment_time_count, 2u);
    EXPECT_DOUBLE_EQ(waypoints[0].waypoints[2].x, 2.0);
    EXPECT_DOUBLE_EQ(waypoints[0].start_velocity[0], 0.2);
    EXPECT_DOUBLE_EQ(waypoints[0].desired_speed, 0.6);
    EXPECT_EQ(waypoints[0].objective, 1u);
    EXPECT_GE(waypoints[0].stamp_ns, before_w + 1500 * kMs - 5 * kMs);
    EXPECT_LE(waypoints[0].stamp_ns, after_w + 1500 * kMs + 5 * kMs);
    w.header.stamp = ros::Time();
    waypoint.publish(w);
    waypoints = waitOutputs<xgc2_ugv_waypoint_request>("waypoint_request", 2);
    EXPECT_EQ(waypoints[1].stamp_ns, 0) << "an unset start time stays unset";

    // A reset.
    reset.publish(std_msgs::Empty());
    const auto resets = waitOutputs<xgc2_ugv_reference_reset>("reference_reset", 1);
    EXPECT_GE(resets[0].stamp_ns, before);
}

TEST_F(EdgeTest, ARequestThatDoesNotFitItsPayloadIsRefusedNotTruncated) {
    startEdge();
    ros::Publisher analytic = nh_.advertise<msgs::AnalyticReference>(
        "alg/unicycle_reference_trajectory/request/analytic", 5);
    ros::Publisher sampled = nh_.advertise<msgs::SampledReference>(
        "alg/unicycle_reference_trajectory/request/sampled", 5);
    ASSERT_TRUE(connected(analytic));
    ASSERT_TRUE(connected(sampled));
    msgs::AnalyticReference a;
    a.params.assign(XGC2_UGV_MAX_ANALYTIC_PARAMS + 1, 1.0);
    analytic.publish(a);
    msgs::SampledReference s;
    s.points.resize(XGC2_UGV_MAX_SAMPLED_POINTS + 1);
    sampled.publish(s);
    EXPECT_TRUE(waitUntil([&] { return host_.logged(2, "Refused an analytic reference"); }));
    // (the second refusal is throttled with the first; the payload of the sampled one is not
    // written)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_TRUE(host_.outputs("analytic_request").empty());
    EXPECT_TRUE(host_.outputs("sampled_request").empty());
    // A request that fits is still taken afterwards.
    a.params.assign(5, 1.0);
    analytic.publish(a);
    waitOutputs<xgc2_ugv_analytic_reference>("analytic_request", 1);
}

TEST_F(EdgeTest, APvaReferenceIsListenedToOnlyByTheFlatnessStrategy) {
    {
        startEdge();
        ros::Publisher pub = nh_.advertise<msgs::PlanarPvaReference>("alg/reference/pva", 5);
        EXPECT_FALSE(waitUntil([&] { return pub.getNumSubscribers() > 0; }, 300));
    }
    host_.stop();
    host_.destroy();
    TestHost flat(library().desc());
    flat.useMonotonicClock();
    ASSERT_EQ(flat.create("{\"tracking_strategy\": \"flatness\"}"), XGC2_OK);
    ASSERT_EQ(flat.start(), XGC2_OK);
    ros::Publisher pub = nh_.advertise<msgs::PlanarPvaReference>("alg/reference/pva", 5);
    ASSERT_TRUE(connected(pub));
    msgs::PlanarPvaReference pva;
    pva.x = 1.0;
    pva.y = 2.0;
    pva.yaw = 0.3;
    pva.vx = 0.4;
    pva.vy = 0.5;
    pva.ax = 0.6;
    pva.ay = 0.7;
    const int64_t before = flat.now();
    pub.publish(pva);
    ASSERT_TRUE(waitUntil([&] { return flat.outputs("pva").size() == 1; }));
    const auto p = flat.outputsAs<xgc2_ugv_planar_pva>("pva")[0];
    EXPECT_DOUBLE_EQ(p.x, 1.0);
    EXPECT_DOUBLE_EQ(p.yaw, 0.3);
    EXPECT_DOUBLE_EQ(p.ay, 0.7);
    EXPECT_GE(p.stamp_ns, before);  // the receipt time
    EXPECT_LE(p.stamp_ns, flat.now());
    flat.stop();
}

TEST_F(EdgeTest, TheControllersTwistAndStatusAreTheChassisTopics) {
    startEdge();
    Probe<geometry_msgs::Twist> twists(nh_, "cmd_vel");
    Probe<std_msgs::String> statuses(nh_, "custom/statustext");
    ASSERT_TRUE(waitUntil([&] { return true; }));

    xgc2_ugv_cmd_vel cmd;
    std::memset(&cmd, 0, sizeof cmd);
    cmd.stamp_ns = host_.now();
    cmd.linear_x = 0.4;
    cmd.linear_y = 0.0;
    cmd.angular_z = -0.2;
    cmd.kind = XGC2_UGV_CMD_VEL_COMMAND;
    host_.push("cmd_vel", cmd, cmd.stamp_ns);
    xgc2_ugv_controller_status status;
    std::memset(&status, 0, sizeof status);
    status.stamp_ns = host_.now();
    status.control_state = XGC2_UGV_CONTROL_READY;
    std::strcpy(status.control_state_name, "Ready");
    host_.push("controller_status", status, status.stamp_ns);

    ASSERT_TRUE(waitUntil([&] {
        step();
        return twists.count() >= 1 && statuses.count() >= 1;
    }));
    EXPECT_DOUBLE_EQ(twists.at(0).linear.x, 0.4);
    EXPECT_DOUBLE_EQ(twists.at(0).linear.y, 0.0);
    EXPECT_DOUBLE_EQ(twists.at(0).angular.z, -0.2);
    EXPECT_EQ(statuses.at(0).data, "Ready");

    // A sample is published once, however often the step runs; the next one is published again,
    // also when it is the same twist (the chassis watches the stream).
    for (int k = 0; k < 5; ++k) {
        step();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_EQ(twists.count(), 1u);
    cmd.stamp_ns = host_.now();
    host_.push("cmd_vel", cmd, cmd.stamp_ns);
    step();
    EXPECT_TRUE(twists.waitFor(2));
}

TEST_F(EdgeTest, TheGeneratorsOutputsAreTheReferenceTopicsInRosTime) {
    startEdge();
    Probe<msgs::ReferenceStatus> status(nh_, "alg/unicycle_reference_trajectory/status");
    Probe<msgs::AnalyticReference> analytic(nh_,
                                            "alg/unicycle_reference_trajectory/active/analytic");
    Probe<msgs::ActivePolynomialReference> polynomial(
        nh_, "alg/unicycle_reference_trajectory/active/polynomial");
    Probe<msgs::SampledReference> sampled(nh_, "alg/unicycle_reference_trajectory/active/sampled");

    const int64_t now = host_.now();
    xgc2_ugv_reference_status st;
    std::memset(&st, 0, sizeof st);
    st.stamp_ns = now;
    st.state = XGC2_UGV_REFERENCE_STATE_ACTIVE;
    st.flags = 4;
    st.active_trajectory_id = 9;
    st.active_revision = 3;
    st.active_type = XGC2_UGV_REFERENCE_TYPE_ANALYTIC;
    host_.push("reference_status", st, now);

    xgc2_ugv_analytic_reference an;
    std::memset(&an, 0, sizeof an);
    an.stamp_ns = now;
    an.start_time_ns = now + 1500 * kMs;
    an.trajectory_id = 9;
    an.revision = 3;
    an.analytic_type = XGC2_UGV_ANALYTIC_CIRCLE;
    an.duration = 8.0;
    an.origin_qw = 1.0;
    an.param_count = 3;
    an.params[0] = 2.0;
    an.params[1] = 0.8;
    an.params[2] = 3.0;
    host_.push("active_analytic", an, now);

    xgc2_ugv_polynomial_reference po;
    std::memset(&po, 0, sizeof po);
    po.stamp_ns = now;
    po.start_time_ns = 0;  // as soon as possible: stays unset
    po.trajectory_id = 4;
    po.order = 7;
    po.segment_count = 2;
    po.segment_durations[0] = 2.0;
    po.segment_durations[1] = 2.0;
    po.coeff_x_count = 16;
    po.coeff_y_count = 16;
    po.coeff_x[15] = 1.25;
    host_.push("active_polynomial", po, now);

    xgc2_ugv_sampled_reference sa;
    std::memset(&sa, 0, sizeof sa);
    sa.stamp_ns = now;
    sa.start_time_ns = now + 500 * kMs;
    sa.trajectory_id = 5;
    sa.sample_dt = 0.05;
    sa.point_count = 2;
    sa.points[1].x = 0.7;
    host_.push("active_sampled", sa, now);

    ASSERT_TRUE(waitUntil([&] {
        step();
        return status.count() >= 1 && analytic.count() >= 1 && polynomial.count() >= 1 &&
               sampled.count() >= 1;
    }));
    const auto s = status.at(0);
    EXPECT_EQ(s.state, msgs::ReferenceStatus::STATE_ACTIVE);
    EXPECT_EQ(s.flags, 4u);
    EXPECT_EQ(s.active_trajectory_id, 9u);
    EXPECT_EQ(s.active_revision, 3u);
    EXPECT_EQ(s.active_type, msgs::ReferenceStatus::TYPE_ANALYTIC);
    // The host clock is monotonic here and ROS time is the wall clock: the times are shifted.
    EXPECT_NEAR((ros::Time::now() - s.header.stamp).toSec(),
                static_cast<double>(host_.now() - now) * 1e-9, 0.02);

    const auto a = analytic.at(0);
    EXPECT_EQ(a.header.frame_id, "world");
    EXPECT_EQ(a.trajectory_id, 9u);
    EXPECT_EQ(a.params.size(), 3u);
    EXPECT_DOUBLE_EQ(a.params[1], 0.8);
    EXPECT_NEAR((a.start_time - a.header.stamp).toSec(), 1.5, 1e-6);
    EXPECT_NEAR((ros::Time::now() - a.header.stamp).toSec(),
                static_cast<double>(host_.now() - now) * 1e-9, 0.02);

    const auto p = polynomial.at(0);
    EXPECT_EQ(p.header.frame_id, "world");
    EXPECT_TRUE(p.start_time.isZero());
    EXPECT_EQ(p.order, 7);
    EXPECT_EQ(p.segment_durations.size(), 2u);
    EXPECT_EQ(p.coeff_x.size(), 16u);
    EXPECT_DOUBLE_EQ(p.coeff_x[15], 1.25);
    EXPECT_TRUE(p.coeff_yaw.empty());

    const auto q = sampled.at(0);
    EXPECT_EQ(q.points.size(), 2u);
    EXPECT_DOUBLE_EQ(q.points[1].x, 0.7);
    EXPECT_NEAR((q.start_time - q.header.stamp).toSec(), 0.5, 1e-6);

    // Latched, like the topics of the node: a late subscriber gets the last of each.
    Probe<msgs::ReferenceStatus> late(nh_, "alg/unicycle_reference_trajectory/status");
    EXPECT_TRUE(late.waitFor(1));
}

TEST_F(EdgeTest, TheResetLeaseRunsBetweenTheControllerAndTheCoordinator) {
    startEdge(
        "{\"namespace\": \"ugv1\", \"reset_request_topic\": \"station/ask\", "
        "\"reset_response_topic\": \"/coordinator/answers\"}");
    ros::Publisher states = nh_.advertise<rigid_state_estimator_msgs::RigidStateEstimate>(
        "ugv1/alg/state_estimator/state", 5);
    Probe<ugv_reset_safety::ResetRequest> requests(nh_, "ugv1/station/ask");
    ros::Publisher responses =
        nh_.advertise<ugv_reset_safety::ResetResponse>("/coordinator/answers", 5);
    Probe<geometry_msgs::Twist> twists(nh_, "ugv1/cmd_vel");
    ASSERT_TRUE(connected(states));
    ASSERT_TRUE(connected(responses));
    ASSERT_TRUE(waitUntil([&] { return requests.count() == 0; }));

    const ros::Time pose_stamp = ros::Time::now();
    states.publish(estimate(pose_stamp));
    ASSERT_TRUE(waitUntil([&] { return host_.outputs("state").size() == 1; }));

    // Not active: no request.
    xgc2_ugv_reset_session session;
    std::memset(&session, 0, sizeof session);
    host_.push("reset_session", session, host_.now());
    for (int k = 0; k < 5; ++k) {
        step();
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    EXPECT_EQ(requests.count(), 0u);

    // Active but not healthy: no request either.
    session.target_x = 2.0;
    session.target_y = 0.5;
    session.generation = 7;
    session.flags = XGC2_UGV_RESET_SESSION_ACTIVE;
    host_.push("reset_session", session, host_.now());
    for (int k = 0; k < 5; ++k) {
        step();
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    EXPECT_EQ(requests.count(), 0u);

    // Active and healthy: a request per about 20 ms, with the pose of the latest state.
    session.flags = XGC2_UGV_RESET_SESSION_ACTIVE | XGC2_UGV_RESET_SESSION_HEALTHY;
    host_.push("reset_session", session, host_.now());
    ASSERT_TRUE(waitUntil([&] {
        step();
        return requests.count() >= 2;
    }));
    const auto request = requests.at(0);
    EXPECT_EQ(request.generation, 7u);
    EXPECT_EQ(request.header.frame_id, "world");
    EXPECT_DOUBLE_EQ(request.pose.x, 1.0);
    EXPECT_DOUBLE_EQ(request.pose.y, 2.0);
    EXPECT_NEAR(request.pose.theta, 0.6, 1e-12);
    EXPECT_DOUBLE_EQ(request.target.x, 2.0);
    EXPECT_DOUBLE_EQ(request.target.y, 0.5);
    EXPECT_EQ(request.pose_stamp, pose_stamp);
    EXPECT_LT((ros::Time::now() - request.header.stamp).toSec(), 1.0);
    EXPECT_GT(requests.at(1).header.stamp, request.header.stamp);

    // The coordinator answers the second request: the controller gets it in its clock, with the
    // wall time the request was issued at.
    const auto answered = requests.at(1);
    ugv_reset_safety::ResetResponse response;
    response.header.stamp = answered.header.stamp;
    response.header.frame_id = "world";
    response.generation = 7;
    response.status = 0;
    response.command.linear.x = 0.2;
    response.command.angular.z = 0.1;
    const double wall_before = wallSeconds();
    const int64_t host_before = host_.now();
    responses.publish(response);
    const auto clearances = waitOutputs<xgc2_ugv_reset_clearance>("reset_clearance", 1);
    EXPECT_EQ(clearances[0].generation, 7u);
    EXPECT_EQ(clearances[0].status, XGC2_UGV_RESET_RUNNING);
    EXPECT_DOUBLE_EQ(clearances[0].linear_x, 0.2);
    EXPECT_DOUBLE_EQ(clearances[0].linear_y, 0.0);
    EXPECT_DOUBLE_EQ(clearances[0].yaw_rate, 0.1);
    EXPECT_DOUBLE_EQ(clearances[0].lease_seconds, 0.15);
    // The request was stamped in ROS time; the clearance carries that instant in the host clock.
    const double age = (ros::Time::now() - answered.header.stamp).toSec();
    EXPECT_NEAR(static_cast<double>(host_.now() - clearances[0].stamp_ns) * 1e-9, age, 0.02);
    EXPECT_LT(clearances[0].issue_wall, wall_before);
    EXPECT_GT(clearances[0].issue_wall, wall_before - 0.15);
    EXPECT_LE(clearances[0].stamp_ns, host_before);

    // Answers the lease does not know are not forwarded: another generation, a stamp that was never
    // issued, a response that is not in the world frame.
    ugv_reset_safety::ResetResponse wrong = response;
    wrong.generation = 8;
    responses.publish(wrong);
    wrong = response;
    wrong.header.stamp = ros::Time::now();
    responses.publish(wrong);
    wrong = response;
    wrong.header.frame_id = "map";
    responses.publish(wrong);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    EXPECT_EQ(host_.outputs("reset_clearance").size(), 1u);

    // The command the chassis was really given is what the next request reports.
    xgc2_ugv_cmd_vel cmd;
    std::memset(&cmd, 0, sizeof cmd);
    cmd.stamp_ns = host_.now();
    cmd.linear_x = 0.2;
    cmd.angular_z = 0.1;
    host_.push("cmd_vel", cmd, cmd.stamp_ns);
    step();
    const size_t seen = requests.count();
    ASSERT_TRUE(waitUntil([&] {
        step();
        return requests.count() >= seen + 2;
    }));
    const auto latest = requests.last();
    EXPECT_DOUBLE_EQ(latest.applied_command.linear.x, 0.2);
    EXPECT_DOUBLE_EQ(latest.applied_command.angular.z, 0.1);
    EXPECT_FALSE(latest.applied_stamp.isZero());
    EXPECT_TRUE(twists.waitFor(1));

    // The session ends: the requests stop.
    session.flags = 0;
    host_.push("reset_session", session, host_.now());
    step();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const size_t after_end = requests.count();
    for (int k = 0; k < 5; ++k) {
        step();
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    EXPECT_EQ(requests.count(), after_end);
}

TEST_F(EdgeTest, ALiveConfigureReconnectsToTheNewTopicsAndStopCutsTheRosSide) {
    startEdge();
    ros::Publisher old_pub = nh_.advertise<geometry_msgs::Pose2D>("reset_pose", 5);
    ASSERT_TRUE(connected(old_pub));
    ASSERT_EQ(host_.configure("{\"reset_pose_topic\": \"goal_pose\"}"), XGC2_OK);
    ros::Publisher new_pub = nh_.advertise<geometry_msgs::Pose2D>("goal_pose", 5);
    ASSERT_TRUE(connected(new_pub));
    ASSERT_TRUE(waitUntil([&] { return old_pub.getNumSubscribers() == 0; }));
    geometry_msgs::Pose2D pose;
    pose.x = 1.0;
    old_pub.publish(pose);
    pose.x = 3.0;
    new_pub.publish(pose);
    const auto targets = waitOutputs<xgc2_ugv_reset_target>("reset_target", 1);
    EXPECT_DOUBLE_EQ(targets[0].x, 3.0);
    EXPECT_EQ(targets.size(), 1u);

    // An unchanged configuration does not reconnect: the publisher keeps its subscriber.
    ASSERT_EQ(host_.configure("{\"reset_pose_topic\": \"goal_pose\"}"), XGC2_OK);
    EXPECT_EQ(new_pub.getNumSubscribers(), 1u);
    // A refused one changes nothing.
    EXPECT_EQ(host_.configure("{\"reset_pose_topic\": \"goal_pose\", \"nope\": 1}"),
              XGC2_ERR_INVALID);
    EXPECT_EQ(new_pub.getNumSubscribers(), 1u);

    // stop: no ROS thread is left to write; start: the same topics again.
    ASSERT_EQ(host_.stop(), XGC2_OK);
    EXPECT_TRUE(waitUntil([&] { return new_pub.getNumSubscribers() == 0; }));
    pose.x = 4.0;
    new_pub.publish(pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_EQ(host_.outputs("reset_target").size(), 1u);
    ASSERT_EQ(host_.start(), XGC2_OK);
    ASSERT_TRUE(connected(new_pub));
    pose.x = 5.0;
    new_pub.publish(pose);
    const auto again = waitOutputs<xgc2_ugv_reset_target>("reset_target", 2);
    EXPECT_DOUBLE_EQ(again[1].x, 5.0);
}

TEST_F(EdgeTest, AFullChannelCostsTheSampleNotTheEdge) {
    startEdge();
    ros::Publisher pub = nh_.advertise<std_msgs::String>("command", 20);
    ASSERT_TRUE(connected(pub));
    host_.limitOutput("command", 2);
    std_msgs::String message;
    message.data = "track";
    for (int k = 0; k < 6; ++k) {
        pub.publish(message);
    }
    ASSERT_TRUE(waitUntil([&] { return host_.refusedWrites("command") >= 4; }));
    EXPECT_EQ(host_.outputs("command").size(), 2u);
    EXPECT_TRUE(host_.logged(2, "Dropped a command"));
    EXPECT_EQ(host_.openSlots(), 0u);
    EXPECT_TRUE(host_.reports().empty() || host_.reports().back().first == 0);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ugv_ros_edge_test");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    const int result = RUN_ALL_TESTS();
    spinner.stop();
    return result;
}
