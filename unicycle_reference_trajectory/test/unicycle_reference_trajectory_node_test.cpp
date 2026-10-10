// The ROS node over the core: topics, defaults and behavior as before the split.

#include <geometry_msgs/Pose.h>
#include <gtest/gtest.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <unicycle_reference_trajectory_msgs/ActivePolynomialReference.h>
#include <unicycle_reference_trajectory_msgs/AnalyticReference.h>
#include <unicycle_reference_trajectory_msgs/ReferenceStatus.h>
#include <unicycle_reference_trajectory_msgs/SampledReference.h>
#include <unicycle_reference_trajectory_msgs/WaypointReferenceRequest.h>

#include <cmath>
#include <functional>
#include <mutex>
#include <string>

namespace {

namespace msgs = unicycle_reference_trajectory_msgs;
const std::string kBase = "alg/unicycle_reference_trajectory/";

bool waitFor(const std::function<bool()>& predicate, double seconds) {
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(seconds);
    while (ros::ok() && ros::WallTime::now() < deadline) {
        if (predicate()) {
            return true;
        }
        ros::WallDuration(0.01).sleep();
    }
    return predicate();
}

// The outputs of one node, as a subscriber sees them.
class Probe {
   public:
    explicit Probe(const std::string& ns) : ns_("/" + ns + "/") {
        status_ = nh_.subscribe(ns_ + kBase + "status", 10, &Probe::onStatus, this);
        analytic_ = nh_.subscribe(ns_ + kBase + "active/analytic", 10, &Probe::onAnalytic, this);
        polynomial_ =
            nh_.subscribe(ns_ + kBase + "active/polynomial", 10, &Probe::onPolynomial, this);
        sampled_ = nh_.subscribe(ns_ + kBase + "active/sampled", 10, &Probe::onSampled, this);
        path_ =
            nh_.subscribe(ns_ + kBase + "visualization/reference_path", 10, &Probe::onPath, this);
        analytic_pub_ = nh_.advertise<msgs::AnalyticReference>(ns_ + kBase + "request/analytic", 1);
        sampled_pub_ = nh_.advertise<msgs::SampledReference>(ns_ + kBase + "request/sampled", 1);
        waypoint_pub_ =
            nh_.advertise<msgs::WaypointReferenceRequest>(ns_ + kBase + "request/waypoint", 1);
        reset_pub_ = nh_.advertise<std_msgs::Empty>(ns_ + kBase + "reset", 1);
    }

    bool connected() const {
        return analytic_pub_.getNumSubscribers() > 0 && sampled_pub_.getNumSubscribers() > 0 &&
               waypoint_pub_.getNumSubscribers() > 0 && reset_pub_.getNumSubscribers() > 0 &&
               status_.getNumPublishers() > 0;
    }
    msgs::ReferenceStatus status() {
        std::lock_guard<std::mutex> lock(mutex_);
        return status_msg_;
    }
    bool state(uint8_t state) {
        return status().state == state && status_seen_;
    }

    ros::NodeHandle nh_;
    ros::Publisher analytic_pub_, sampled_pub_, waypoint_pub_, reset_pub_;
    std::mutex mutex_;
    msgs::ReferenceStatus status_msg_;
    msgs::AnalyticReference analytic_msg_;
    msgs::ActivePolynomialReference polynomial_msg_;
    msgs::SampledReference sampled_msg_;
    nav_msgs::Path path_msg_;
    int analytic_count_{0}, polynomial_count_{0}, sampled_count_{0}, path_count_{0};
    bool status_seen_{false};

   private:
    void onStatus(const msgs::ReferenceStatus::ConstPtr& m) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_msg_ = *m;
        status_seen_ = true;
    }
    void onAnalytic(const msgs::AnalyticReference::ConstPtr& m) {
        std::lock_guard<std::mutex> lock(mutex_);
        analytic_msg_ = *m;
        ++analytic_count_;
    }
    void onPolynomial(const msgs::ActivePolynomialReference::ConstPtr& m) {
        std::lock_guard<std::mutex> lock(mutex_);
        polynomial_msg_ = *m;
        ++polynomial_count_;
    }
    void onSampled(const msgs::SampledReference::ConstPtr& m) {
        std::lock_guard<std::mutex> lock(mutex_);
        sampled_msg_ = *m;
        ++sampled_count_;
    }
    void onPath(const nav_msgs::Path::ConstPtr& m) {
        std::lock_guard<std::mutex> lock(mutex_);
        path_msg_ = *m;
        ++path_count_;
    }

    std::string ns_;
    ros::Subscriber status_, analytic_, polynomial_, sampled_, path_;
};

msgs::AnalyticReference circle(uint32_t id) {
    msgs::AnalyticReference m;
    m.trajectory_id = id;
    m.revision = 5U;
    m.analytic_type = msgs::AnalyticReference::ANALYTIC_CIRCLE;
    m.duration = 20.0;
    m.origin.orientation.w = 1.0;
    m.params = {2.0, 0.8, 3.0, 0.0, 1.0};
    return m;
}

TEST(ReferenceNode, RequestsActivateReferencesAndResetReturnsToReady) {
    Probe probe("ref_test");
    ASSERT_TRUE(waitFor([&] { return probe.connected(); }, 10.0));
    ASSERT_TRUE(waitFor([&] { return probe.state(msgs::ReferenceStatus::STATE_READY); }, 5.0));

    // Analytic: Active, the active message echoes the request with the lead time applied.
    const ros::Time sent = ros::Time::now();
    auto request = circle(7U);
    request.header.stamp = sent;
    ASSERT_TRUE(waitFor(
        [&] {
            probe.analytic_pub_.publish(request);
            return probe.state(msgs::ReferenceStatus::STATE_ACTIVE) && probe.analytic_count_ > 0;
        },
        5.0));
    {
        std::lock_guard<std::mutex> lock(probe.mutex_);
        EXPECT_EQ(probe.status_msg_.active_trajectory_id, 7U);
        EXPECT_EQ(probe.status_msg_.active_revision, 5U);
        EXPECT_EQ(probe.status_msg_.active_type, msgs::ReferenceStatus::TYPE_ANALYTIC);
        EXPECT_EQ(probe.analytic_msg_.trajectory_id, 7U);
        EXPECT_EQ(probe.analytic_msg_.params, request.params);
        EXPECT_GE((probe.analytic_msg_.start_time - sent).toSec(), 0.2 - 1e-6);
    }
    ASSERT_TRUE(waitFor([&] { return probe.path_count_ > 0; }, 5.0));
    {
        std::lock_guard<std::mutex> lock(probe.mutex_);
        EXPECT_EQ(probe.path_msg_.header.frame_id, "world");
        EXPECT_GE(probe.path_msg_.poses.size(), 10U);
    }

    // Waypoints: planned into the seventh-order polynomial.
    msgs::WaypointReferenceRequest waypoints;
    waypoints.trajectory_id = 8U;
    waypoints.desired_speed = 0.6;
    for (int k = 0; k < 3; ++k) {
        geometry_msgs::Pose pose;
        pose.position.x = k;
        pose.position.y = k % 2 == 0 ? 0.0 : 0.5;
        pose.orientation.w = 1.0;
        waypoints.waypoints.push_back(pose);
    }
    waypoints.segment_times = {2.0, 2.0};
    ASSERT_TRUE(waitFor(
        [&] {
            probe.waypoint_pub_.publish(waypoints);
            return probe.polynomial_count_ > 0;
        },
        5.0));
    {
        std::lock_guard<std::mutex> lock(probe.mutex_);
        EXPECT_EQ(probe.polynomial_msg_.trajectory_id, 8U);
        EXPECT_EQ(probe.polynomial_msg_.order, 7U);
        EXPECT_EQ(probe.polynomial_msg_.segment_durations.size(), 2U);
        EXPECT_EQ(probe.status_msg_.active_type, msgs::ReferenceStatus::TYPE_POLYNOMIAL);
    }

    // Sampled: an arc with explicit planar kinematics.
    msgs::SampledReference sampled;
    sampled.trajectory_id = 9U;
    sampled.flags = msgs::SampledReference::FLAG_EXPLICIT_PLANAR_KINEMATICS;
    sampled.sample_dt = 0.05;
    for (int k = 0; k < 100; ++k) {
        msgs::PlanarReferencePoint p;
        p.t_from_start = 0.05 * k;
        p.yaw = 0.25 * p.t_from_start;
        p.x = 2.0 * std::sin(p.yaw);
        p.y = 2.0 * (1.0 - std::cos(p.yaw));
        p.speed = 0.5;
        p.yaw_rate = 0.25;
        p.curvature = 0.5;
        p.vx = 0.5 * std::cos(p.yaw);
        p.vy = 0.5 * std::sin(p.yaw);
        sampled.points.push_back(p);
    }
    ASSERT_TRUE(waitFor(
        [&] {
            probe.sampled_pub_.publish(sampled);
            return probe.sampled_count_ > 0;
        },
        5.0));
    {
        std::lock_guard<std::mutex> lock(probe.mutex_);
        EXPECT_EQ(probe.sampled_msg_.points.size(), 100U);
        EXPECT_EQ(probe.status_msg_.active_type, msgs::ReferenceStatus::TYPE_SAMPLED);
        EXPECT_EQ(probe.status_msg_.flags, msgs::SampledReference::FLAG_EXPLICIT_PLANAR_KINEMATICS);
    }

    // An invalid request is refused and changes nothing.
    msgs::SampledReference empty;
    empty.trajectory_id = 10U;
    probe.sampled_pub_.publish(empty);
    ros::WallDuration(0.3).sleep();
    EXPECT_EQ(probe.status().active_trajectory_id, 9U);

    // Reset: back to Ready with nothing active.
    ASSERT_TRUE(waitFor(
        [&] {
            probe.reset_pub_.publish(std_msgs::Empty());
            return probe.state(msgs::ReferenceStatus::STATE_READY) &&
                   probe.status().active_trajectory_id == 0U;
        },
        5.0));
    EXPECT_EQ(probe.status().active_type, msgs::ReferenceStatus::TYPE_NONE);
}

TEST(ReferenceNode, DefaultAnalyticRequestStartsTheReferenceByItself) {
    Probe probe("ref_default");
    ASSERT_TRUE(waitFor([&] { return probe.state(msgs::ReferenceStatus::STATE_ACTIVE); }, 10.0));
    ASSERT_TRUE(waitFor([&] { return probe.analytic_count_ > 0; }, 5.0));
    std::lock_guard<std::mutex> lock(probe.mutex_);
    EXPECT_EQ(probe.status_msg_.active_trajectory_id, 9U);
    EXPECT_EQ(probe.status_msg_.active_revision, 4U);
    EXPECT_EQ(probe.analytic_msg_.analytic_type, msgs::AnalyticReference::ANALYTIC_CIRCLE);
    // radius, line speed, entry duration and centre of the shipped configuration
    ASSERT_EQ(probe.analytic_msg_.params.size(), 5U);
    EXPECT_DOUBLE_EQ(probe.analytic_msg_.params[0], 3.0);
    EXPECT_DOUBLE_EQ(probe.analytic_msg_.params[1], 1.0);
    EXPECT_DOUBLE_EQ(probe.analytic_msg_.duration, 120.0);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "unicycle_reference_trajectory_node_test");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    const int result = RUN_ALL_TESTS();
    spinner.stop();
    return result;
}
