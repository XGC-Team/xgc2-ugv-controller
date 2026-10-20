// The Reset coordinator node: the ROS edge of ResetCoordinator.
//
// Event driven: it sleeps in the callback queue (idle CPU about zero) and wakes on a Session, a
// Scene message or, while a vehicle is in a session or moving, every 100 ms. It has no roster: it
// finds the vehicles by their `<ns>/reset/session` topics on the master and answers each on
// `<ns>/reset/grant`. It never subscribes /command and never publishes cmd_vel.
//
// Parameters (private, all optional): scene_namespace (empty: no Scene was authored for this
// Experiment, and a missing Scene is not an error), fence_x_min / fence_x_max / fence_y_min /
// fence_y_max (all four or none; the Session's frozen world boundary), world_frame, the planner
// margins pair_margin / static_margin / hold_margin, and fleet_topic.
//
// A startup error is the node's own non-zero exit, so a Session-managed instance that is
// misconfigured fails its Run instead of idling.

#include <ros/callback_queue.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <ugv_reset_msgs/ResetFleet.h>
#include <ugv_reset_msgs/ResetGrant.h>
#include <ugv_reset_msgs/ResetSession.h>
#include <ugv_reset_safety/reset_coordinator.h>
#include <ugv_reset_safety/scene_projection.h>
#include <xgc2_geometry_msgs/SceneState.h>

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace ugv_reset_safety {
namespace {

namespace rc = ugv_reset_client;
namespace msgs = ugv_reset_msgs;

double wallSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// A private parameter that may be absent, an empty string (unset by a process definition that
// passes every argument) or a number.
bool optionalNumber(ros::NodeHandle& nh, const std::string& key, double* value) {
    XmlRpc::XmlRpcValue v;
    if (!nh.getParam(key, v)) {
        return false;
    }
    if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        *value = static_cast<double>(v);
        return true;
    }
    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        *value = static_cast<int>(v);
        return true;
    }
    if (v.getType() == XmlRpc::XmlRpcValue::TypeString && static_cast<std::string>(v).empty()) {
        return false;
    }
    throw std::invalid_argument("numeric parameter required: " + key);
}

rc::Session toSession(const msgs::ResetSession& m) {
    rc::Session s;
    s.stamp = m.header.stamp.toSec();
    s.session = m.session;
    s.pose_stamp = m.pose_stamp.toSec();
    s.pose = {m.pose.x, m.pose.y, m.pose.theta};
    s.target = {m.target.x, m.target.y, m.target.theta};
    s.phase = m.phase <= msgs::ResetSession::PHASE_DONE ? static_cast<rc::Phase>(m.phase) : rc::Phase::Idle;
    s.progress = m.progress;
    s.cross_track = m.cross_track;
    s.spec.model = m.model == msgs::ResetSession::MODEL_OMNI ? rc::Model::Omni : rc::Model::Unicycle;
    s.spec.footprint = {m.half_length, m.half_width, m.offset_x, m.offset_y};
    s.spec.max_v = m.max_v;
    s.spec.max_vy = m.max_vy;
    s.spec.max_w = m.max_w;
    s.spec.acc_v = m.acc_v;
    s.spec.acc_w = m.acc_w;
    s.spec.ell_bound = m.ell_bound;
    s.ell_hat = m.ell_hat;
    s.ell_spread = m.ell_spread;
    s.healthy = m.healthy;
    return s;
}

msgs::ResetGrant toMessage(const rc::Grant& g, const std::string& frame) {
    msgs::ResetGrant m;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = frame;
    m.session = g.session;
    m.state = static_cast<uint8_t>(g.state);
    m.grant_id = g.grant_id;
    m.start.x = g.start.x;
    m.start.y = g.start.y;
    m.start.theta = g.start.yaw;
    m.goal.x = g.goal.x;
    m.goal.y = g.goal.y;
    m.goal.theta = g.goal.yaw;
    m.final_move = g.final_move;
    m.ell_plan = g.ell_plan;
    for (const rc::Point& p : g.via) {
        geometry_msgs::Point q;
        q.x = p.x;
        q.y = p.y;
        m.via.push_back(q);
    }
    m.s_limit = g.s_limit;
    m.ttl = g.ttl;
    m.reason = static_cast<uint8_t>(g.reason);
    m.detail = g.detail;
    m.blockers = g.blockers;
    return m;
}

}  // namespace

class CoordinatorNode {
   public:
    CoordinatorNode() : private_("~") {
        nh_.setCallbackQueue(&queue_);
        private_.param("world_frame", world_frame_, std::string("world"));
        private_.param("scene_namespace", scene_namespace_, std::string());
        private_.param("fleet_topic", fleet_topic_, std::string("/ugv/reset/fleet"));
        CoordinatorConfig config;
        private_.param("pair_margin", config.planner.pair_margin, config.planner.pair_margin);
        private_.param("static_margin", config.planner.static_margin, config.planner.static_margin);
        private_.param("hold_margin", config.planner.hold_margin, config.planner.hold_margin);
        core_ = std::make_unique<ResetCoordinator>(config);

        double bounds[4] = {0, 0, 0, 0};
        const char* keys[4] = {"fence_x_min", "fence_x_max", "fence_y_min", "fence_y_max"};
        int given = 0;
        for (int i = 0; i < 4; ++i) {
            given += optionalNumber(private_, keys[i], &bounds[i]) ? 1 : 0;
        }
        if (given != 0 && given != 4) {
            throw std::invalid_argument("the fence needs all four bounds or none");
        }
        fence_.enabled = given == 4;
        if (fence_.enabled) {
            fence_.xmin = bounds[0];
            fence_.xmax = bounds[1];
            fence_.ymin = bounds[2];
            fence_.ymax = bounds[3];
            if (!(fence_.xmin < fence_.xmax) || !(fence_.ymin < fence_.ymax)) {
                throw std::invalid_argument("the fence bounds are empty");
            }
        } else {
            ROS_WARN("No fence given: routes are only kept clear of the statics and the bodies");
        }
        scene_authored_ = !scene_namespace_.empty();
        if (scene_authored_) {
            scene_sub_ = nh_.subscribe(scene_namespace_ + "/snapshot", 1, &CoordinatorNode::scene, this);
            scene_state_sub_ =
                nh_.subscribe(scene_namespace_ + "/state", 1, &CoordinatorNode::sceneState, this);
        }
        fleet_pub_ = nh_.advertise<msgs::ResetFleet>(fleet_topic_, 1, true);
        core_->setWorld({}, fence_);
        core_->setScene(scene_authored_, !scene_authored_);
        ROS_INFO("Reset coordinator ready: scene %s, fence %s",
                 scene_authored_ ? scene_namespace_.c_str() : "none authored",
                 fence_.enabled ? "set" : "none");
        publishFleet(wallSeconds());
    }

    void run() {
        double next_discovery = 0.0;
        double next_fleet = 0.0;
        double last_step = 0.0;
        while (ros::ok()) {
            const bool busy = core_->busy();
            queue_.callAvailable(ros::WallDuration(busy ? 0.05 : 0.5));
            const double now = wallSeconds();
            if (now >= next_discovery) {
                next_discovery = now + 1.0;
                discover();
            }
            updateSceneAvailability(now);
            // A message wakes the step at once; otherwise every 100 ms while there is work, and
            // every 500 ms for the heartbeat of the lost-vehicle clock when there is none.
            const double period = core_->busy() ? 0.1 : 0.5;
            if (event_ || now - last_step >= period) {
                if (!event_ || now - last_step >= 0.02) {
                    last_step = now;
                    event_ = false;
                    for (const OutgoingGrant& g : core_->step(now)) {
                        auto found = grant_pubs_.find(g.vehicle);
                        if (found != grant_pubs_.end()) {
                            found->second.publish(toMessage(g.grant, world_frame_));
                        }
                        logGrant(g);
                    }
                }
            }
            if (now >= next_fleet) {
                next_fleet = now + (core_->busy() ? 0.2 : 1.0);
                publishFleet(now);
            }
        }
    }

   private:
    struct Vehicle {
        ros::Subscriber sub;
    };

    void discover() {
        ros::master::V_TopicInfo topics;
        if (!ros::master::getTopics(topics)) {
            return;
        }
        static const std::string suffix = "/reset/session";
        for (const auto& t : topics) {
            if (t.datatype != "ugv_reset_msgs/ResetSession" || t.name.size() <= suffix.size() ||
                t.name.compare(t.name.size() - suffix.size(), suffix.size(), suffix) != 0) {
                continue;
            }
            std::string vehicle = t.name.substr(0, t.name.size() - suffix.size());
            if (!vehicle.empty() && vehicle.front() == '/') {
                vehicle.erase(0, 1);
            }
            if (vehicle.empty() || vehicles_.count(vehicle) != 0) {
                continue;
            }
            grant_pubs_[vehicle] = nh_.advertise<msgs::ResetGrant>("/" + vehicle + "/reset/grant", 5);
            vehicles_[vehicle].sub = nh_.subscribe<msgs::ResetSession>(
                t.name, 10, [this, vehicle](const msgs::ResetSession::ConstPtr& m) { session(vehicle, *m); });
            ROS_INFO("Reset coordinator: vehicle %s joined", vehicle.c_str());
        }
    }

    void session(const std::string& vehicle, const msgs::ResetSession& m) {
        if (m.header.frame_id != world_frame_) {
            ROS_WARN_THROTTLE(5, "Ignoring a Reset session of %s in frame '%s' (expected '%s')",
                              vehicle.c_str(), m.header.frame_id.c_str(), world_frame_.c_str());
            return;
        }
        core_->onSession(vehicle, toSession(m), wallSeconds());
        event_ = true;
    }

    void logGrant(const OutgoingGrant& g) {
        // Changes only: the heartbeat repeats grants and is not worth a log line.
        auto& last = logged_[g.vehicle];
        const auto key = std::make_tuple(g.grant.session, static_cast<int>(g.grant.state),
                                         g.grant.grant_id, static_cast<int>(g.grant.reason));
        if (last == key) {
            return;
        }
        last = key;
        switch (g.grant.state) {
            case rc::GrantState::Go:
                ROS_INFO("Reset %s session %u: GO %zu via point(s), prefix %.2f m%s", g.vehicle.c_str(),
                         g.grant.session, g.grant.via.size(), g.grant.s_limit > 1e6 ? -1.0 : g.grant.s_limit,
                         g.grant.final_move ? "" : " (parking move)");
                break;
            case rc::GrantState::Hold:
                ROS_INFO("Reset %s session %u: HOLD %s %s", g.vehicle.c_str(), g.grant.session,
                         rc::name(g.grant.reason), g.grant.detail.c_str());
                break;
            case rc::GrantState::Brake:
                ROS_WARN("Reset %s session %u: BRAKE %s %s", g.vehicle.c_str(), g.grant.session,
                         rc::name(g.grant.reason), g.grant.detail.c_str());
                break;
            case rc::GrantState::Reject:
                ROS_ERROR("Reset %s session %u: REJECT %s: %s", g.vehicle.c_str(), g.grant.session,
                          rc::name(g.grant.reason), g.grant.detail.c_str());
                break;
        }
    }

    void publishFleet(double now) {
        const FleetSummary summary = core_->fleet(now);
        msgs::ResetFleet m;
        m.header.stamp = ros::Time::now();
        m.header.frame_id = world_frame_;
        m.scene_available = summary.scene_available;
        for (const FleetMember& f : summary.members) {
            msgs::ResetFleetMember member;
            member.vehicle = f.vehicle;
            member.session = f.session;
            member.phase = static_cast<uint8_t>(f.phase);
            member.grant_state = static_cast<uint8_t>(f.grant_state);
            member.reason = static_cast<uint8_t>(f.reason);
            member.progress = f.progress;
            member.remaining = f.remaining;
            member.lost = f.lost;
            member.blockers = f.blockers;
            m.members.push_back(std::move(member));
        }
        fleet_pub_.publish(m);
    }

    // ---- the Scene (kept as it was: scene_projection.h is correct and tested) --------------------

    void pushWorld() {
        core_->setWorld(occupancy_.empty() ? obstacles_ : occupancy_, fence_);
    }

    void updateSceneAvailability(double now) {
        if (!scene_authored_) {
            return;
        }
        const bool live = scene_parsed_ && scene_valid_ && scene_state_wall_ > 0.0 &&
                          now - scene_state_wall_ <= 1.0;
        if (live != scene_live_) {
            scene_live_ = live;
            ROS_INFO("Reset coordinator: the Scene is %s", live ? "available" : "unavailable");
            event_ = true;
        }
        core_->setScene(true, live);
    }

    void sceneState(const xgc2_geometry_msgs::SceneState::ConstPtr& state) {
        if (snapshot_.epoch.empty() || !scene_parsed_) {
            return;
        }
        if (state->epoch != snapshot_.epoch || state->revision != snapshot_.revision ||
            state->header.frame_id != world_frame_ || state->header.stamp.isZero()) {
            return;
        }
        if (!last_state_stamp_.isZero() && state->header.stamp <= last_state_stamp_) {
            return;
        }
        try {
            auto geometry = scene_projection::live(prepared_scene_, *state, world_frame_);
            bool moved = geometry.live.size() != obstacles_.size();
            if (!moved) {
                for (std::size_t i = 0; i < geometry.live.size() && !moved; ++i) {
                    moved = geometry.live[i].id != obstacles_[i].id ||
                            geometry.live[i].vertices.size() != obstacles_[i].vertices.size();
                    for (std::size_t j = 0; !moved && j < geometry.live[i].vertices.size(); ++j) {
                        moved = (geometry.live[i].vertices[j] - obstacles_[i].vertices[j]).norm() > 1e-3;
                    }
                }
            }
            obstacles_ = std::move(geometry.live);
            occupancy_ = std::move(geometry.occupancy);
            last_state_stamp_ = state->header.stamp;
            scene_state_wall_ = wallSeconds();
            scene_valid_ = true;
            if (moved || !world_pushed_) {
                world_pushed_ = true;
                pushWorld();
                event_ = true;
            }
        } catch (const std::exception& e) {
            scene_valid_ = false;
            ROS_WARN_THROTTLE(2, "Reset live scene rejected: %s", e.what());
        }
    }

    void scene(const xgc2_geometry_msgs::SceneSnapshot::ConstPtr& snapshot) {
        if (snapshot->epoch == snapshot_.epoch && snapshot->revision < snapshot_.revision) {
            return;
        }
        if (scene_parsed_ && snapshot->epoch == snapshot_.epoch &&
            snapshot->revision == snapshot_.revision) {
            if (snapshot->obstacles == snapshot_.obstacles && snapshot->scene_id == snapshot_.scene_id &&
                snapshot->header.frame_id == snapshot_.header.frame_id) {
                return;
            }
            scene_valid_ = false;
            ROS_WARN("Reset scene rejected: the scene definition changed at the same revision");
            return;
        }
        snapshot_ = *snapshot;
        scene_parsed_ = false;
        scene_valid_ = false;
        obstacles_.clear();
        occupancy_.clear();
        world_pushed_ = false;
        last_state_stamp_ = ros::Time();
        scene_state_wall_ = 0.0;
        try {
            prepared_scene_ = scene_projection::prepare(snapshot_, world_frame_, geometry_cache_);
            scene_parsed_ = true;
        } catch (const std::exception& e) {
            ROS_WARN("Reset scene rejected: %s", e.what());
        }
        event_ = true;
    }

    ros::NodeHandle nh_, private_;
    ros::CallbackQueue queue_;
    std::unique_ptr<ResetCoordinator> core_;
    std::map<std::string, Vehicle> vehicles_;
    std::map<std::string, ros::Publisher> grant_pubs_;
    std::map<std::string, std::tuple<uint32_t, int, uint32_t, int>> logged_;
    ros::Publisher fleet_pub_;
    ros::Subscriber scene_sub_, scene_state_sub_;
    std::string world_frame_, scene_namespace_, fleet_topic_;
    Fence fence_;
    bool event_{false};
    bool scene_authored_{false};
    bool scene_live_{false};
    bool scene_parsed_{false};
    bool scene_valid_{false};
    bool world_pushed_{false};
    double scene_state_wall_{0.0};
    xgc2_geometry_msgs::SceneSnapshot snapshot_;
    scene_projection::PreparedScene prepared_scene_;
    xgc2_math::geometry::PreparedGeometryCache geometry_cache_;
    ros::Time last_state_stamp_;
    std::vector<ConvexObstacle> obstacles_;
    std::vector<ConvexObstacle> occupancy_;
};

}  // namespace ugv_reset_safety

int main(int argc, char** argv) {
    ros::init(argc, argv, "xgc_reset_coordinator");
    try {
        ugv_reset_safety::CoordinatorNode node;
        node.run();
    } catch (const std::exception& e) {
        ROS_FATAL("Reset coordinator startup failed: %s", e.what());
        return 1;
    }
    return 0;
}
