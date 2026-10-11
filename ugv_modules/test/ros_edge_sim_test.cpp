// The ROS edge on simulation time: /clock becomes the host clock, and no stamp is shifted.

#include <geometry_msgs/Pose2D.h>
#include <gtest/gtest.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <unicycle_reference_trajectory_msgs/AnalyticReference.h>
#include <unicycle_reference_trajectory_msgs/ReferenceStatus.h>

#include <atomic>
#include <thread>

#include "edge_test_support.h"
#include "test_host.h"
#include "xgc2_ugv/payloads.h"

namespace {

using edge_test::connected;
using edge_test::waitUntil;
using ugv_modules_test::ModuleLibrary;
using ugv_modules_test::TestHost;
namespace msgs = unicycle_reference_trajectory_msgs;

constexpr int64_t kSecond = 1000000000LL;

ModuleLibrary& library() {
    static ModuleLibrary library(EDGE_MODULE_PATH);
    return library;
}

// A simulator: /clock advances 5 ms per message, 200 messages per second.
class Simulator {
   public:
    explicit Simulator(ros::NodeHandle& nh)
        : pub_(nh.advertise<rosgraph_msgs::Clock>("/clock", 100)) {
        thread_ = std::thread([this] {
            while (!stop_.load()) {
                rosgraph_msgs::Clock clock;
                clock.clock.fromNSec(static_cast<uint64_t>(now_.load()));
                pub_.publish(clock);
                now_.fetch_add(5 * 1000000LL);
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        });
    }
    ~Simulator() {
        stop_.store(true);
        thread_.join();
    }
    int64_t now() const {
        return now_.load();
    }

   private:
    ros::Publisher pub_;
    std::atomic<int64_t> now_{100 * kSecond};
    std::atomic<bool> stop_{false};
    std::thread thread_;
};

class EdgeSimTest : public ::testing::Test {
   protected:
    EdgeSimTest() : host_(library().desc()), simulator_(nh_) {
        host_.setNow(0);
    }
    ~EdgeSimTest() override {
        host_.stop();
        host_.destroy();
    }

    ros::NodeHandle nh_;
    TestHost host_;
    Simulator simulator_;
};

TEST_F(EdgeSimTest, TheClockOfTheSimulatorIsPublishedToTheHost) {
    ASSERT_TRUE(waitUntil([&] { return ros::Time::now().toNSec() > 0; }));
    ASSERT_EQ(host_.create("{\"sim_time\": true}"), XGC2_OK);
    ASSERT_EQ(host_.start(), XGC2_OK);
    ASSERT_TRUE(waitUntil([&] { return host_.outputs("clock").size() >= 20; }));
    const auto clocks = host_.outputsAs<int64_t>("clock");
    const auto samples = host_.outputs("clock");
    for (size_t i = 0; i < clocks.size(); ++i) {
        EXPECT_EQ(clocks[i] % (5 * 1000000LL), 0) << i;
        EXPECT_EQ(samples[i].stamp_ns, clocks[i])
            << "the commit is stamped with the time it carries";
        if (i > 0) {
            EXPECT_GT(clocks[i], clocks[i - 1]);
        }
    }
    EXPECT_GE(clocks.front(), 100 * kSecond);
    EXPECT_LE(clocks.back(), simulator_.now());
}

TEST_F(EdgeSimTest, AWallClockConfigurationAgainstASimulationMasterDoesNotStart) {
    ASSERT_EQ(host_.create("{}"), XGC2_OK);
    EXPECT_EQ(host_.start(), XGC2_ERR_INTERNAL);
    EXPECT_TRUE(host_.logged(3, "/use_sim_time is set on the ROS master but sim_time is not"));
}

TEST_F(EdgeSimTest, NoStampIsShiftedOnSimulationTime) {
    ASSERT_TRUE(waitUntil([&] { return ros::Time::now().toNSec() > 0; }));
    ASSERT_EQ(host_.create("{\"sim_time\": true}"), XGC2_OK);
    ASSERT_EQ(host_.start(), XGC2_OK);
    ros::Publisher states = nh_.advertise<rigid_state_estimator_msgs::RigidStateEstimate>(
        "alg/state_estimator/state", 5);
    ros::Publisher analytic = nh_.advertise<msgs::AnalyticReference>(
        "alg/unicycle_reference_trajectory/request/analytic", 5);
    ros::Publisher targets = nh_.advertise<geometry_msgs::Pose2D>("reset_pose", 5);
    ASSERT_TRUE(connected(states));
    ASSERT_TRUE(connected(analytic));
    ASSERT_TRUE(connected(targets));

    // A stamp is the instant it names, to the nanosecond.
    rigid_state_estimator_msgs::RigidStateEstimate estimate;
    estimate.header.stamp = ros::Time(123, 456);
    estimate.orientation.w = 1.0;
    states.publish(estimate);
    ASSERT_TRUE(waitUntil([&] { return host_.outputs("state").size() == 1; }));
    EXPECT_EQ(host_.outputsAs<xgc2_ugv_planar_state>("state")[0].stamp_ns, 123 * kSecond + 456);

    // An unstamped one gets the receipt time: the simulation time when it arrived.
    const int64_t before = simulator_.now();
    estimate.header.stamp = ros::Time();
    states.publish(estimate);
    ASSERT_TRUE(waitUntil([&] { return host_.outputs("state").size() == 2; }));
    const int64_t after = simulator_.now();
    const int64_t receipt = host_.outputsAs<xgc2_ugv_planar_state>("state")[1].stamp_ns;
    EXPECT_GE(receipt, before - 10 * 1000000LL);
    EXPECT_LE(receipt, after);

    // Request times as stamped, an unset start stays unset.
    msgs::AnalyticReference a;
    a.header.stamp = ros::Time(130, 5);
    a.start_time = ros::Time(150, 0);
    a.params = {1.0, 1.0, 1.0};
    analytic.publish(a);
    a.start_time = ros::Time();
    analytic.publish(a);
    ASSERT_TRUE(waitUntil([&] { return host_.outputs("analytic_request").size() == 2; }));
    const auto requests = host_.outputsAs<xgc2_ugv_analytic_reference>("analytic_request");
    EXPECT_EQ(requests[0].stamp_ns, 130 * kSecond + 5);
    EXPECT_EQ(requests[0].start_time_ns, 150 * kSecond);
    EXPECT_EQ(requests[1].start_time_ns, 0);

    // And the other way round.
    host_.setNow(simulator_.now());
    xgc2_ugv_reference_status status;
    std::memset(&status, 0, sizeof status);
    status.stamp_ns = 130 * kSecond + 7;
    status.state = XGC2_UGV_REFERENCE_STATE_READY;
    host_.push("reference_status", status, status.stamp_ns);
    ros::Subscriber sub;
    std::atomic<int> received{0};
    ros::Time stamp;
    sub = nh_.subscribe<msgs::ReferenceStatus>("alg/unicycle_reference_trajectory/status", 5,
                                               [&](const msgs::ReferenceStatus::ConstPtr& m) {
                                                   stamp = m->header.stamp;
                                                   received.fetch_add(1);
                                               });
    ASSERT_TRUE(waitUntil([&] {
        EXPECT_EQ(host_.step(XGC2_STEP_TIMER), XGC2_OK);
        return received.load() > 0;
    }));
    EXPECT_EQ(stamp, ros::Time(130, 7));
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ugv_ros_edge_sim_test");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    const int result = RUN_ALL_TESTS();
    spinner.stop();
    return result;
}
