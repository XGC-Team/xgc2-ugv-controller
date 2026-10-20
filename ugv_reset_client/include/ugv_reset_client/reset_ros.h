#pragma once

// The ROS edge of the vehicle's Reset session: publishes the Session (`reset/session`, 5 Hz and on
// change) and the latched Result (`reset/result`), and hands every Grant of `reset/grant` to the
// owner. Topic names are relative to the vehicle's namespace unless they are given. The message
// types are those of ugv_reset_msgs; the executor and the planner use the plain types of
// reset_protocol.h, converted here.

#include <ros/ros.h>
#include <ugv_reset_client/reset_protocol.h>
#include <ugv_reset_msgs/ResetGrant.h>
#include <ugv_reset_msgs/ResetResult.h>
#include <ugv_reset_msgs/ResetSession.h>

#include <functional>
#include <string>
#include <utility>

namespace ugv_reset_client {

inline ugv_reset_msgs::ResetSession toMessage(const Session& s, const ros::Time& stamp) {
    ugv_reset_msgs::ResetSession m;
    m.header.stamp = stamp;
    m.header.frame_id = "world";
    m.session = s.session;
    m.pose_stamp = ros::Time(s.pose_stamp);
    m.pose.x = s.pose.x;
    m.pose.y = s.pose.y;
    m.pose.theta = s.pose.yaw;
    m.target.x = s.target.x;
    m.target.y = s.target.y;
    m.target.theta = s.target.yaw;
    m.phase = static_cast<uint8_t>(s.phase);
    m.progress = s.progress;
    m.cross_track = s.cross_track;
    m.model = s.spec.model == Model::Omni ? ugv_reset_msgs::ResetSession::MODEL_OMNI
                                          : ugv_reset_msgs::ResetSession::MODEL_UNICYCLE;
    m.half_length = s.spec.footprint.half_length;
    m.half_width = s.spec.footprint.half_width;
    m.offset_x = s.spec.footprint.offset_x;
    m.offset_y = s.spec.footprint.offset_y;
    m.max_v = s.spec.max_v;
    m.max_vy = s.spec.max_vy;
    m.max_w = s.spec.max_w;
    m.acc_v = s.spec.acc_v;
    m.acc_w = s.spec.acc_w;
    m.ell_hat = s.ell_hat;
    m.ell_spread = s.ell_spread;
    m.ell_bound = s.spec.ell_bound;
    m.healthy = s.healthy;
    return m;
}

inline ugv_reset_msgs::ResetResult toMessage(const Result& r, const ros::Time& stamp) {
    ugv_reset_msgs::ResetResult m;
    m.header.stamp = stamp;
    m.header.frame_id = "world";
    m.session = r.session;
    m.outcome = static_cast<uint8_t>(r.outcome);
    m.reason = static_cast<uint8_t>(r.reason);
    m.final_planar_error = r.final_planar_error;
    m.final_yaw_error = r.final_yaw_error;
    m.duration = r.duration;
    m.detail = r.detail;
    return m;
}

inline Grant toGrant(const ugv_reset_msgs::ResetGrant& m) {
    Grant g;
    g.stamp = m.header.stamp.toSec();
    g.ttl = m.ttl;
    g.session = m.session;
    g.state = m.state <= ugv_reset_msgs::ResetGrant::STATE_REJECT ? static_cast<GrantState>(m.state)
                                                                  : GrantState::Hold;
    g.grant_id = m.grant_id;
    g.start = {m.start.x, m.start.y, m.start.theta};
    g.goal = {m.goal.x, m.goal.y, m.goal.theta};
    g.final_move = m.final_move;
    g.ell_plan = m.ell_plan;
    for (const auto& p : m.via) {
        g.via.push_back({p.x, p.y});
    }
    g.s_limit = m.s_limit;
    g.reason = static_cast<Reason>(m.reason);
    g.detail = m.detail;
    g.blockers = m.blockers;
    return g;
}

class ResetRosEdge {
   public:
    using GrantSink = std::function<void(const Grant&)>;

    ResetRosEdge(ros::NodeHandle& nh, GrantSink sink, const std::string& session_topic = "reset/session",
                 const std::string& grant_topic = "reset/grant",
                 const std::string& result_topic = "reset/result")
        : sink_(std::move(sink)) {
        session_pub_ = nh.advertise<ugv_reset_msgs::ResetSession>(session_topic, 5);
        result_pub_ = nh.advertise<ugv_reset_msgs::ResetResult>(result_topic, 1, true);
        // Every grant matters (a brake and a new route can follow each other at once).
        grant_sub_ = nh.subscribe(grant_topic, 50, &ResetRosEdge::receive, this);
    }

    void publish(const Session& session) {
        session_pub_.publish(toMessage(session, ros::Time::now()));
    }
    void publish(const Result& result) {
        result_pub_.publish(toMessage(result, ros::Time::now()));
    }

   private:
    void receive(const ugv_reset_msgs::ResetGrant::ConstPtr& message) {
        if (message) {
            sink_(toGrant(*message));
        }
    }

    GrantSink sink_;
    ros::Publisher session_pub_, result_pub_;
    ros::Subscriber grant_sub_;
};

}  // namespace ugv_reset_client
