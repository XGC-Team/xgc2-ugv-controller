#include <ugv_reset_safety/reset_coordinator.h>

#include <algorithm>
#include <cmath>

#include "planner_test_support.h"

namespace reset_test {
namespace {

using ugv_reset_safety::CoordinatorConfig;
using ugv_reset_safety::OutgoingGrant;
// Legacy geometry/heartbeat cases use explicitly ideal stopped endpoints. The separate
// ResetCausal tests and real Executor harness exercise the actual asynchronous barrier.
class StoppedCoordinatorFixture : public ugv_reset_safety::ResetCoordinator {
    struct Wire {
        rc::Session latest;
        uint32_t generation{0}, sequence{0}, ack{0};
        bool cert{false};
    };
    std::map<std::string, Wire> wire_;

   public:
    explicit StoppedCoordinatorFixture(std::vector<std::string> names = {"ugv1"})
        : ugv_reset_safety::ResetCoordinator(config(std::move(names))) {}
    explicit StoppedCoordinatorFixture(CoordinatorConfig c)
        : ugv_reset_safety::ResetCoordinator(c) {}
    static CoordinatorConfig config(std::vector<std::string> names) {
        CoordinatorConfig c;
        c.expected_namespaces = std::move(names);
        return c;
    }
    void onSession(const std::string& id, rc::Session v, double now) {
        auto& w = wire_[id];
        auto active = [](rc::Phase p) { return p != rc::Phase::Idle && p != rc::Phase::Done; };
        if ((active(v.phase) && !w.generation) || (w.generation && v.session != w.latest.session) ||
            (!active(v.phase) && active(w.latest.phase))) {
            ++w.generation;
            w.ack = 0;
            w.cert = false;
        }
        v.request_generation = w.generation;
        v.publication_seq = ++w.sequence;
        v.acknowledged_grant_id = w.ack;
        v.stop_certified = w.cert;
        w.latest = v;
        ugv_reset_safety::ResetCoordinator::onSession(id, v, now);
    }
    std::vector<OutgoingGrant> step(double now) {
        std::vector<OutgoingGrant> result;
        for (unsigned turn = 0; turn < 4; ++turn) {
            bool barrier = false;
            for (const auto& out : ugv_reset_safety::ResetCoordinator::step(now)) {
                auto& w = wire_.at(out.vehicle);
                const auto& g = out.grant;
                if (g.state == rc::GrantState::Brake) {
                    barrier = true;
                    w.ack = g.grant_id;
                    w.cert = true;
                    auto v = w.latest;
                    v.acknowledged_grant_id = w.ack;
                    v.stop_certified = true;
                    v.publication_seq = ++w.sequence;
                    w.latest = v;
                    ugv_reset_safety::ResetCoordinator::onSession(out.vehicle, v, now);
                } else {
                    result.push_back(out);
                    if (g.state == rc::GrantState::Go || g.state == rc::GrantState::Hold) {
                        w.ack = g.grant_id;
                        w.cert = false;
                    }
                }
            }
            if (!barrier)
                break;
        }
        return result;
    }
};
using ResetCoordinator = StoppedCoordinatorFixture;

rc::VehicleSpec scout() {
    rc::VehicleSpec spec;
    spec.footprint = {0.45, 0.40, 0.0, 0.0};
    spec.ell_bound = 0.25;
    return spec;
}

rc::Session session(uint32_t id, rc::Phase phase, const rc::Pose& pose, const rc::Pose& target,
                    double stamp) {
    rc::Session s;
    s.session = id;
    s.phase = phase;
    s.pose = pose;
    s.target = target;
    s.spec = scout();
    s.pose_stamp = stamp;
    s.healthy = true;
    return s;
}

std::vector<OutgoingGrant> only(const std::vector<OutgoingGrant>& all, const std::string& vehicle) {
    std::vector<OutgoingGrant> out;
    for (const auto& g : all) {
        if (g.vehicle == vehicle) {
            out.push_back(g);
        }
    }
    return out;
}

TEST(ResetCoordinator,
     DeclaredVehiclesAreFoundByTheirSessionsAndEveryParticipantHearsWithinTheRefreshPeriod) {
    ResetCoordinator c({"ugv1", "ugv2"});
    c.setWorld({}, ugv_reset_safety::Fence());
    double t = 0.0;
    c.onSession("ugv1", session(1, rc::Phase::Requesting, {-4, 0, 0}, {4, 0, 0}, t), t);
    c.onSession("ugv2", session(1, rc::Phase::Idle, {0, 6, 0}, {0, 0, 0}, t), t);
    auto first = c.step(t);
    ASSERT_EQ(only(first, "ugv1").size(), 1U);
    EXPECT_EQ(only(first, "ugv1")[0].grant.state, rc::GrantState::Go);
    EXPECT_EQ(only(first, "ugv2").size(), 0U) << "an idle vehicle is a body, not a participant";
    // The unchanged grant is repeated as the heartbeat, no more often than the refresh period.
    int repeats = 0;
    for (t = 0.05; t < 2.0; t += 0.05) {
        c.onSession("ugv1", session(1, rc::Phase::Moving, {-4, 0, 0}, {4, 0, 0}, t), t);
        c.onSession("ugv2", session(1, rc::Phase::Idle, {0, 6, 0}, {0, 0, 0}, t), t);
        repeats += static_cast<int>(only(c.step(t), "ugv1").size());
    }
    EXPECT_GE(repeats, 4);
    EXPECT_LE(repeats, 6) << "about every 0.4 s";
}

TEST(ResetCoordinator, AFaultAfterValidDiscoveryNeverTouchesTheOthers) {
    ResetCoordinator c({"ugv1", "ugv2", "ugv3"});
    c.setWorld({}, ugv_reset_safety::Fence());
    // Every declared body is observed first. Initial missing/invalid body data is
    // deliberately covered as a fleet admission hold in ResetCausal.
    c.onSession("ugv2", session(1, rc::Phase::Idle, {-4, 3, 0}, {4, 3, 0}, 0), 0);
    c.onSession("ugv3", session(1, rc::Phase::Idle, {-4, -3, 0}, {4, -3, 0}, 0), 0);
    // ugv1 drives; ugv2 sends an invalid footprint; ugv3 has a non-finite target.
    c.onSession("ugv1", session(1, rc::Phase::Requesting, {-4, 0, 0}, {4, 0, 0}, 0.0), 0.0);
    rc::Session bad = session(1, rc::Phase::Requesting, {-4, 3, 0}, {4, 3, 0}, 0.0);
    bad.spec.footprint.half_length = -1.0;
    c.onSession("ugv2", bad, 0.0);
    rc::Session nan_target =
        session(1, rc::Phase::Requesting, {-4, -3, 0}, {std::nan(""), -3, 0}, 0.0);
    c.onSession("ugv3", nan_target, 0.0);
    const auto out = c.step(0.0);
    ASSERT_EQ(only(out, "ugv1").size(), 1U);
    EXPECT_EQ(only(out, "ugv1")[0].grant.state, rc::GrantState::Go);
    ASSERT_GE(only(out, "ugv2").size(), 1U);
    EXPECT_EQ(only(out, "ugv2")[0].grant.state, rc::GrantState::Reject);
    EXPECT_EQ(only(out, "ugv2")[0].grant.reason, rc::Reason::BadSession);
    ASSERT_GE(only(out, "ugv3").size(), 1U);
    EXPECT_EQ(only(out, "ugv3")[0].grant.state, rc::GrantState::Reject);
    // ugv1 keeps its grant (no brake, no re-plan) while the faulty ones are refused again.
    uint32_t id = only(out, "ugv1")[0].grant.grant_id;
    for (double t = 0.1; t < 1.5; t += 0.1) {
        c.onSession("ugv1", session(1, rc::Phase::Moving, {-4 + t * 0.3, 0, 0}, {4, 0, 0}, t), t);
        c.onSession("ugv2", bad, t);
        c.onSession("ugv3", nan_target, t);
        for (const auto& g : only(c.step(t), "ugv1")) {
            EXPECT_EQ(g.grant.state, rc::GrantState::Go);
            EXPECT_EQ(g.grant.grant_id, id);
        }
    }
}

TEST(ResetCoordinator, ANewSessionOfOneVehicleDoesNotDisturbAMover) {
    ResetCoordinator c({"ugv1", "ugv2"});
    c.setWorld({}, ugv_reset_safety::Fence());
    c.onSession("ugv1", session(5, rc::Phase::Requesting, {-4, 0, 0}, {4, 0, 0}, 0.0), 0.0);
    c.onSession("ugv2", session(1, rc::Phase::Idle, {0, 5, 0}, {0, 5, 0}, 0.0), 0.0);
    const uint32_t id = only(c.step(0.0), "ugv1").at(0).grant.grant_id;
    // ugv2 starts a session (a second click on one vehicle): ugv1 neither brakes nor is re-planned.
    for (double t = 0.1; t < 1.2; t += 0.1) {
        c.onSession("ugv1", session(5, rc::Phase::Moving, {-4 + t * 0.3, 0, 0}, {4, 0, 0}, t), t);
        c.onSession("ugv2", session(2, rc::Phase::Requesting, {0, 5, 0}, {0, 5, 0}, t), t);
        for (const auto& g : only(c.step(t), "ugv1")) {
            EXPECT_EQ(g.grant.state, rc::GrantState::Go) << "t=" << t;
            EXPECT_EQ(g.grant.grant_id, id);
            EXPECT_EQ(g.grant.session, 5U);
        }
    }
}

TEST(ResetCoordinator, ASilentVehicleStaysABodyAndAMovingOutsiderHoldsNewRoutes) {
    CoordinatorConfig config;
    config.expected_namespaces = {"ugv1", "ugv2"};
    ResetCoordinator c(config);
    c.setWorld({}, ugv_reset_safety::Fence());
    // ugv2 sits in the way, in Custom1 (outside any Reset session), and drives about.
    c.onSession("ugv2", session(1, rc::Phase::Idle, {0, 0, 0}, {0, 0, 0}, 0.0), 0.0);
    c.onSession("ugv2", session(1, rc::Phase::Idle, {0.1, 0, 0}, {0, 0, 0}, 0.3), 0.3);
    c.onSession("ugv1", session(1, rc::Phase::Requesting, {-4, 4, 0}, {4, 4, 0}, 0.3), 0.3);
    auto out = c.step(0.3);
    ASSERT_GE(only(out, "ugv1").size(), 1U);
    EXPECT_EQ(only(out, "ugv1")[0].grant.state, rc::GrantState::Hold);
    EXPECT_EQ(only(out, "ugv1")[0].grant.reason, rc::Reason::NonParticipantMoving);
    // It stops: the route is granted.
    double t = 0.6;
    c.onSession("ugv2", session(1, rc::Phase::Idle, {0.1, 0, 0}, {0, 0, 0}, t), t);
    c.onSession("ugv1", session(1, rc::Phase::Requesting, {-4, 4, 0}, {4, 4, 0}, t), t);
    out = c.step(t);
    bool granted = false;
    for (const auto& g : only(out, "ugv1")) {
        granted = granted || g.grant.state == rc::GrantState::Go;
    }
    for (t = 0.7; t < 1.5 && !granted; t += 0.1) {
        c.onSession("ugv2", session(1, rc::Phase::Idle, {0.1, 0, 0}, {0, 0, 0}, t), t);
        c.onSession("ugv1", session(1, rc::Phase::Requesting, {-4, 4, 0}, {4, 4, 0}, t), t);
        for (const auto& g : only(c.step(t), "ugv1")) {
            granted = granted || g.grant.state == rc::GrantState::Go;
        }
    }
    EXPECT_TRUE(granted);
    // ugv2 goes silent for 5 s: lost, but still a parked body at its last pose.
    c.onSession("ugv1", session(2, rc::Phase::Requesting, {-4, 0, 0}, {4, 0, 0}, 7.0), 7.0);
    out = c.step(7.0);
    const auto fleet = c.fleet(7.0);
    bool lost = false;
    for (const auto& m : fleet.members) {
        lost = lost || (m.vehicle == "ugv2" && m.lost);
    }
    EXPECT_TRUE(lost);
    // Its pose (0.1, 0) is on ugv1's straight line to (4, 0): ugv1 goes around, never through.
    bool detour = false;
    for (const auto& g : only(out, "ugv1")) {
        detour = detour || (g.grant.state == rc::GrantState::Go && !g.grant.via.empty());
    }
    EXPECT_TRUE(detour);
}

TEST(ResetCoordinator, AnAuthoredSceneThatIsNotAvailableHoldsNewGrantsAndSaysWhy) {
    ResetCoordinator c;
    c.setWorld({}, ugv_reset_safety::Fence());
    c.setScene(true, false);
    c.onSession("ugv1", session(1, rc::Phase::Requesting, {-4, 0, 0}, {4, 0, 0}, 0.0), 0.0);
    auto out = c.step(0.0);
    ASSERT_EQ(only(out, "ugv1").size(), 1U);
    EXPECT_EQ(only(out, "ugv1")[0].grant.state, rc::GrantState::Hold);
    EXPECT_EQ(only(out, "ugv1")[0].grant.reason, rc::Reason::SceneUnavailable);
    // The hold is repeated as the heartbeat, and lifted when the scene arrives.
    int holds = 0;
    for (double t = 0.1; t < 1.0; t += 0.1) {
        c.onSession("ugv1", session(1, rc::Phase::Waiting, {-4, 0, 0}, {4, 0, 0}, t), t);
        for (const auto& g : only(c.step(t), "ugv1")) {
            holds += g.grant.state == rc::GrantState::Hold ? 1 : 0;
        }
    }
    EXPECT_GE(holds, 2);
    c.setScene(true, true);
    c.onSession("ugv1", session(1, rc::Phase::Waiting, {-4, 0, 0}, {4, 0, 0}, 1.0), 1.0);
    out = c.step(1.0);
    bool go = false;
    for (const auto& g : only(out, "ugv1")) {
        go = go || g.grant.state == rc::GrantState::Go;
    }
    EXPECT_TRUE(go);
    // No Scene authored: nothing to wait for.
    ResetCoordinator open;
    open.setWorld({}, ugv_reset_safety::Fence());
    open.setScene(false, false);
    open.onSession("ugv1", session(1, rc::Phase::Requesting, {-4, 0, 0}, {4, 0, 0}, 0.0), 0.0);
    EXPECT_EQ(only(open.step(0.0), "ugv1").at(0).grant.state, rc::GrantState::Go);
}

TEST(ResetCoordinator, ARejectionIsRepeatedUntilTheVehicleEndsItsSessionAndThenForgotten) {
    ResetCoordinator c;
    c.setWorld({}, ugv_reset_safety::Fence());
    c.onSession("ugv1", session(1, rc::Phase::Requesting, {-4, 0, 0}, {std::nan(""), 0, 0}, 0.0),
                0.0);
    int rejects = 0;
    for (double t = 0.0; t < 2.0; t += 0.1) {
        c.onSession("ugv1", session(1, rc::Phase::Requesting, {-4, 0, 0}, {std::nan(""), 0, 0}, t),
                    t);
        for (const auto& g : only(c.step(t), "ugv1")) {
            rejects += g.grant.state == rc::GrantState::Reject ? 1 : 0;
        }
    }
    EXPECT_GE(rejects, 4);
    // It ended; a new session is answered afresh.
    c.onSession("ugv1", session(1, rc::Phase::Idle, {-4, 0, 0}, {0, 0, 0}, 2.1), 2.1);
    const auto retired = only(c.step(2.1), "ugv1");
    ASSERT_EQ(retired.size(), 1u);
    EXPECT_EQ(retired[0].grant.state, rc::GrantState::Hold);
    c.onSession("ugv1", session(2, rc::Phase::Requesting, {-4, 0, 0}, {4, 0, 0}, 2.2), 2.2);
    const auto out = only(c.step(2.2), "ugv1");
    ASSERT_FALSE(out.empty());
    EXPECT_EQ(out[0].grant.state, rc::GrantState::Go);
    EXPECT_EQ(out[0].grant.session, 2U);
}

// The coordinator against ideal followers over the TASE family: the same soundness and completion
// as the planner alone, through the message layer (heartbeat, sessions, grants).
TEST(ResetCoordinator, ClosedLoopWithIdealFollowersOnTheTaseFamily) {
    const ScenarioFile world =
        loadScenarios(std::string(RESET_TEST_DATA_DIR) + "/tase_scenarios_subset80.txt");
    int completed = 0;
    int run = 0;
    for (size_t index = 0; index < world.scenarios.size(); index += 4) {
        const Scenario& sc = world.scenarios[index];
        CoordinatorConfig config;
        for (size_t i = 0; i < sc.starts.size(); ++i)
            config.expected_namespaces.push_back("ugv" + std::to_string(i + 1));
        ugv_reset_safety::ResetCoordinator c(config);
        std::vector<ugv_reset_safety::ConvexObstacle> statics;
        for (const auto& p : world.obstacles) {
            ugv_reset_safety::ConvexObstacle o;
            o.id = "obstacle" + std::to_string(statics.size());
            o.vertices = p;
            statics.push_back(o);
        }
        c.setWorld(statics, ugv_reset_safety::Fence());
        std::vector<IdealFollower> robots;
        for (size_t i = 0; i < sc.starts.size(); ++i) {
            robots.emplace_back("ugv" + std::to_string(i + 1), scout(),
                                rc::Pose{sc.starts[i].x(), sc.starts[i].y(), sc.starts[i].z()},
                                rc::Pose{sc.goals[i].x(), sc.goals[i].y(), sc.goals[i].z()}, -0.229,
                                true);
            robots.back().beginSession(1);
        }
        double min_clear = 1e9;
        bool all = false;
        for (double t = 0.0; t < 90.0 && !all; t += 0.002) {
            if (std::fmod(t + 1e-9, 0.1) < 0.002) {
                for (auto& r : robots) {
                    c.onSession(r.id(), r.session(), t);
                }
                for (const auto& g : c.step(t)) {
                    for (auto& r : robots) {
                        if (r.id() == g.vehicle) {
                            r.onGrant(g.grant);
                        }
                    }
                }
            }
            all = true;
            for (auto& r : robots) {
                r.step(0.002);
                all = all && r.done();
            }
            if (std::fmod(t + 1e-9, 0.02) < 0.002) {
                for (size_t i = 0; i < robots.size(); ++i) {
                    for (size_t j = i + 1; j < robots.size(); ++j) {
                        const auto a = rect(robots[i].pose().x, robots[i].pose().y,
                                            robots[i].pose().yaw, 0.31, 0.26);
                        const auto b = rect(robots[j].pose().x, robots[j].pose().y,
                                            robots[j].pose().yaw, 0.31, 0.26);
                        min_clear = std::min(min_clear, clearance(a, b));
                    }
                }
            }
        }
        ++run;
        EXPECT_GE(min_clear, 0.0) << sc.name;
        bool arrived = true;
        for (const auto& r : robots) {
            arrived = arrived && r.arrived();
        }
        completed += arrived ? 1 : 0;
    }
    std::printf("coordinator closed loop: %d/%d completed\n", completed, run);
    EXPECT_GE(completed, static_cast<int>(0.75 * run));
}

}  // namespace
}  // namespace reset_test
