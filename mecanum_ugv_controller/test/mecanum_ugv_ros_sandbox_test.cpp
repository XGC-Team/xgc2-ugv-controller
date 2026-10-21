#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ugv_reset_msgs/ResetGrant.h>
#include <ugv_reset_msgs/ResetSession.h>

#include <cmath>
#include <functional>
#include <string>

namespace {

geometry_msgs::PoseStamped makePose(double x, double y, double yaw, const ros::Time& stamp) {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.orientation.z = std::sin(yaw * 0.5);
    msg.pose.orientation.w = std::cos(yaw * 0.5);
    return msg;
}

bool waitFor(const std::function<bool()>& pred, double seconds) {
    const ros::Time deadline = ros::Time::now() + ros::Duration(seconds);
    ros::Rate rate(50.0);
    while (ros::ok() && ros::Time::now() < deadline) {
        ros::spinOnce();
        if (pred()) {
            return true;
        }
        rate.sleep();
    }
    return pred();
}

}  // namespace

TEST(MecanumRosSandbox, PoseCommandFenceAndDrop) {
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ugv_test/pose", 10);
    ros::Publisher command_pub = nh.advertise<std_msgs::String>("/ugv_test/command", 10);
    ros::Publisher reset_pub = nh.advertise<geometry_msgs::Pose2D>("/ugv_test/reset_pose", 10);

    // A transport-only station fixture. No Go is valid before the exact current
    // G/Brake receipt and the real node's post-zero stop certificate.
    ros::Publisher grant_pub = nh.advertise<ugv_reset_msgs::ResetGrant>("/ugv_test/reset/grant", 5);
    bool grant_response = false;
    uint32_t station_session=0,generation=0,decision=0,publication=0,brake=0,go=0,hold=0;
    ugv_reset_msgs::ResetGrant frozen_go;
    ros::Subscriber session_sub = nh.subscribe<ugv_reset_msgs::ResetSession>(
        "/ugv_test/reset/session", 5, [&](const ugv_reset_msgs::ResetSession::ConstPtr& session) {
            if (!grant_response || session->request_generation==0) return;
            if (session->session!=station_session || session->request_generation!=generation) {
                station_session=session->session;generation=session->request_generation;
                brake=++decision;go=0;hold=0;
            }
            const bool terminal=session->phase==ugv_reset_msgs::ResetSession::PHASE_IDLE ||
                                session->phase==ugv_reset_msgs::ResetSession::PHASE_DONE;
            ugv_reset_msgs::ResetGrant grant;
            grant.session=station_session;grant.request_generation=generation;
            grant.state=ugv_reset_msgs::ResetGrant::STATE_BRAKE;grant.grant_id=brake;
            grant.ttl=1.0;
            if (session->acknowledged_grant_id==brake && session->stop_certified) {
                if (terminal) {
                    if (hold==0) hold=++decision;
                    grant.state=ugv_reset_msgs::ResetGrant::STATE_HOLD;grant.grant_id=hold;
                } else {
                    if (go==0) {
                        go=++decision;frozen_go=grant;
                        frozen_go.state=ugv_reset_msgs::ResetGrant::STATE_GO;frozen_go.grant_id=go;
                        frozen_go.start=session->pose;frozen_go.goal=session->target;
                        frozen_go.final_move=true;frozen_go.s_limit=1.0e9;
                    }
                    grant=frozen_go;
                }
            } else if (!terminal && go!=0 && session->acknowledged_grant_id==go && !session->stop_certified) {
                grant=frozen_go; // Repeat the identical decision, never mutate its frozen route.
            } else if (terminal && hold!=0 && session->acknowledged_grant_id==hold && !session->stop_certified) {
                return;
            }
            grant.header.stamp=ros::Time::now();grant.header.frame_id="world";
            grant.header.seq=++publication;
            grant_pub.publish(grant);
        });
    std::string control_state;
    geometry_msgs::Twist last_cmd;
    ros::Subscriber state_sub = nh.subscribe<std_msgs::String>(
        "/ugv_test/custom/statustext", 10,
        [&](const std_msgs::String::ConstPtr& msg) { control_state = msg->data; });
    ros::Subscriber cmd_sub = nh.subscribe<geometry_msgs::Twist>(
        "/ugv_test/cmd_vel", 10,
        [&](const geometry_msgs::Twist::ConstPtr& msg) { last_cmd = *msg; });

    ASSERT_TRUE(waitFor(
        [&]() {
            return pose_pub.getNumSubscribers() > 0 && command_pub.getNumSubscribers() > 0 &&
                   reset_pub.getNumSubscribers() > 0 && state_sub.getNumPublishers() > 0 &&
                   cmd_sub.getNumPublishers() > 0;
        },
        5.0));

    ASSERT_TRUE(waitFor(
        [&]() {
            pose_pub.publish(makePose(0.0, 0.0, 0.0, ros::Time::now()));
            return control_state == "Ready";
        },
        3.0))
        << "expected Ready";

    geometry_msgs::Pose2D goal;
    goal.x = 0.8;
    goal.y = 0.0;
    reset_pub.publish(goal);
    ros::Duration(0.05).sleep();
    ros::spinOnce();
    EXPECT_EQ(control_state, "Ready") << "reset_pose must not drive the state machine";

    std_msgs::String reset;
    reset.data = "reset";
    command_pub.publish(reset);
    ASSERT_TRUE(waitFor(
        [&]() {
            pose_pub.publish(makePose(0.0, 0.0, 0.0, ros::Time::now()));
            return control_state == "Reset";
        },
        2.0))
        << "expected Reset";
    EXPECT_DOUBLE_EQ(last_cmd.linear.x, 0.0) << "a vehicle without a grant must never drive";
    grant_response = true;
    ASSERT_TRUE(waitFor(
        [&]() {
            pose_pub.publish(makePose(0.0, 0.0, 0.0, ros::Time::now()));
            return last_cmd.linear.x > 0.0;
        },
        2.0));
    grant_response = false;
    ASSERT_TRUE(waitFor(
        [&]() {
            pose_pub.publish(makePose(0.0, 0.0, 0.0, ros::Time::now()));
            return last_cmd.linear.x == 0.0;
        },
        3.0))
        << "a lapsed grant heartbeat must stop (it brakes from a second after the last message and the "
           "session ends with an exact zero at two seconds of silence)";
    EXPECT_TRUE(waitFor([&]() {
        // Keep localization healthy while isolating coordinator loss. The final
        // section below deliberately drops this stream to test pose health.
        pose_pub.publish(makePose(0.0,0.0,0.0,ros::Time::now()));
        return control_state == "Ready";
    }, 3.0))
        << "the vehicle ends its own session when the heartbeat lapses";

    ASSERT_TRUE(waitFor(
        [&]() {
            pose_pub.publish(makePose(50.0, 0.0, 0.0, ros::Time::now()));
            return control_state == "SelfCheck";
        },
        2.0))
        << "fence should SelfCheck";

    ASSERT_TRUE(waitFor(
        [&]() {
            pose_pub.publish(makePose(0.0, 0.0, 0.0, ros::Time::now()));
            return control_state == "Ready";
        },
        3.0))
        << "back inside fence -> Ready";

    // HEALTH state_timeout is 0.5 s; leave slack after the last fresh pose.
    ros::Duration(1.2).sleep();
    ros::spinOnce();
    EXPECT_EQ(control_state, "SelfCheck") << "dropped pose should SelfCheck";
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "mecanum_ugv_ros_sandbox_test");
    return RUN_ALL_TESTS();
}
