#include <gtest/gtest.h>
#include <ugv_reset_safety/reset_coordinator.h>
#include <ugv_reset_safety/reset_scope.h>

#include <map>
#include <stdexcept>
namespace rs = ugv_reset_safety;
namespace rc = ugv_reset_client;
namespace {
struct WireRig {
    rs::ResetCoordinator c;
    std::map<std::string, rc::Session> s;
    double t{1.0};
    explicit WireRig(std::vector<std::string> names = {"a", "b"}) : c(config(std::move(names))) {
        c.setWorld({}, rs::Fence{});
    }
    static rs::CoordinatorConfig config(std::vector<std::string> names) {
        rs::CoordinatorConfig c;
        c.expected_namespaces = std::move(names);
        return c;
    }
    void body(const std::string& id, rc::Pose p, rc::Pose target, uint32_t session = 0,
              uint32_t generation = 0) {
        rc::Session v;
        v.session = session;
        v.request_generation = generation;
        v.phase = generation ? rc::Phase::Requesting : rc::Phase::Idle;
        v.pose = p;
        v.target = target;
        v.spec.footprint = {.2, .2, 0, 0};
        v.spec.ell_bound = 0;
        v.ell_spread = 0;
        v.healthy = true;
        s[id] = v;
        publish(id);
    }
    void publish(const std::string& id) {
        auto& v = s.at(id);
        ++v.publication_seq;
        v.stamp = t;
        v.pose_stamp = t;
        c.onSession(id, v, t);
    }
    std::vector<rs::OutgoingGrant> tick(double dt = .05) {
        t += dt;
        return c.step(t);
    }
    static rc::Grant get(const std::vector<rs::OutgoingGrant>& out, const std::string& id,
                         rc::GrantState state) {
        for (const auto& g : out)
            if (g.vehicle == id && g.grant.state == state)
                return g.grant;
        ADD_FAILURE() << "missing decision for " << id << " state " << int(state);
        return {};
    }
    void certify(const std::string& id, const rc::Grant& b) {
        auto& v = s.at(id);
        v.acknowledged_grant_id = b.grant_id;
        v.stop_certified = true;
        publish(id);
    }
    void receipt(const std::string& id, const rc::Grant& go) {
        auto& v = s.at(id);
        v.acknowledged_grant_id = go.grant_id;
        v.stop_certified = false;
        v.phase = rc::Phase::Moving;
        publish(id);
    }
    rc::Grant start() {
        body("a", {-4, 0, 0}, {4, 0, 0}, 11, 1);
        body("b", {0, 6, 0}, {0, 6, 0});
        auto b = get(tick(), "a", rc::GrantState::Brake);
        certify("a", b);
        auto go = get(tick(), "a", rc::GrantState::Go);
        receipt("a", go);
        tick();
        return go;
    }
    rs::FleetMember a() {
        for (auto& r : c.fleet(t).members)
            if (r.vehicle == "a")
                return r;
        return {};
    }
};
TEST(ResetCausal, ScopeAndDelayedFirstPeerNeverAssumeAnEmptySafeFleet) {
    WireRig r;
    r.body("a", {-4, 0, 0}, {4, 0, 0}, 11, 1);
    auto b = WireRig::get(r.tick(), "a", rc::GrantState::Brake);
    r.certify("a", b);
    auto held = r.tick();
    for (auto& g : held)
        EXPECT_NE(g.grant.state, rc::GrantState::Go);
    EXPECT_EQ(r.c.stats().admissions, 0);
    r.body("b", {0, 6, 0}, {0, 6, 0});
    auto go = WireRig::get(r.tick(), "a", rc::GrantState::Go);
    EXPECT_GT(go.grant_id, b.grant_id);
    WireRig missing({});
    missing.body("a", {-4, 0, 0}, {4, 0, 0}, 11, 1);
    b = WireRig::get(missing.tick(), "a", rc::GrantState::Brake);
    missing.certify("a", b);
    for (auto& g : missing.tick())
        EXPECT_NE(g.grant.state, rc::GrantState::Go);
    EXPECT_EQ(missing.c.stats().admissions, 0);
}
TEST(ResetCausal, ExactBarrierRetiresOnceAndOnlyAfterReceipt) {
    WireRig r;
    auto old = r.start();
    const double reserved = r.a().remaining;
    ASSERT_GT(reserved, 1);
    auto& v = r.s["a"];
    ++v.request_generation;
    v.phase = rc::Phase::Requesting;
    v.pose = {3.9, .1, 0};
    v.acknowledged_grant_id = 0;
    v.stop_certified = false;
    r.publish("a");
    auto b = WireRig::get(r.tick(), "a", rc::GrantState::Brake);
    EXPECT_DOUBLE_EQ(r.a().remaining, reserved);
    EXPECT_EQ(r.c.stats().admissions, 1);
    v.phase = rc::Phase::Waiting;
    v.stop_certified = true;
    v.acknowledged_grant_id = old.grant_id;
    r.publish("a");
    r.tick();
    EXPECT_DOUBLE_EQ(r.a().remaining, reserved);
    EXPECT_EQ(r.c.stats().admissions, 1);
    v.phase = rc::Phase::Requesting;
    r.certify("a", b);
    auto next = WireRig::get(r.tick(), "a", rc::GrantState::Go);
    EXPECT_EQ(r.c.stats().admissions, 2);
    EXPECT_GT(next.grant_id, b.grant_id);
    EXPECT_NEAR(next.start.x, 3.9, 1e-9);
    r.certify("a", b);
    r.tick();
    EXPECT_EQ(r.c.stats().admissions, 2);
    EXPECT_GT(r.a().remaining, 0);
}
TEST(ResetCausal, WaitingIdleLostAndOldPublicationNeverFreeHighestIssuedPlan) {
    WireRig r;
    auto go = r.start();
    const double reserved = r.a().remaining;
    auto stale = r.s["a"];
    stale.phase = rc::Phase::Idle;
    stale.stop_certified = true;
    stale.acknowledged_grant_id = go.grant_id;
    r.c.onSession("a", stale, r.t + 2.9);  // exact old publication; must not refresh received time
    r.tick(3.2);
    EXPECT_TRUE(r.a().lost);
    EXPECT_DOUBLE_EQ(r.a().remaining, reserved);
    auto& v = r.s["a"];
    v.phase = rc::Phase::Idle;
    v.stop_certified = false;
    r.publish("a");
    r.tick();
    EXPECT_DOUBLE_EQ(r.a().remaining, reserved);
}
TEST(ResetCausal, RearReleaseNeedsTheExactGoReceiptNotAHigherDecisionNumber) {
    WireRig r;
    r.body("a", {-4, 0, 0}, {4, 0, 0}, 11, 1);
    r.body("b", {0, 6, 0}, {0, 6, 0});
    auto brake = WireRig::get(r.tick(), "a", rc::GrantState::Brake);
    r.certify("a", brake);
    auto go = WireRig::get(r.tick(), "a", rc::GrantState::Go);
    auto& v = r.s["a"];
    v.pose = {-3.7, 0, 0};
    v.phase = rc::Phase::Moving;
    v.stop_certified = false;
    for (uint32_t wrong : {brake.grant_id, go.grant_id + 1}) {
        v.acknowledged_grant_id = wrong;
        r.publish("a");
        r.tick();
        EXPECT_DOUBLE_EQ(r.a().progress, 0);
    }
    r.receipt("a", go);
    r.tick();
    EXPECT_GT(r.a().progress, .25);
}
TEST(ResetCausal, SceneRevocationRetainsAnUnreceivedGoAndOrdinaryWaitingIsNotReceipt) {
    WireRig r;
    r.body("a", {-4, 0, 0}, {4, 0, 0}, 11, 1);
    r.body("b", {0, 6, 0}, {0, 6, 0});
    auto b = WireRig::get(r.tick(), "a", rc::GrantState::Brake);
    r.certify("a", b);
    auto go =
        WireRig::get(r.tick(), "a", rc::GrantState::Go);  // deliberately still queued, unreceived
    const double reserved = r.a().remaining;
    auto& v = r.s["a"];
    v.phase = rc::Phase::Waiting;
    v.stop_certified = false;
    r.publish("a");
    rs::ConvexObstacle wall;
    wall.id = "changed";
    wall.vertices = {{1, -1}, {2, -1}, {2, 1}, {1, 1}};
    r.c.setWorld({wall}, rs::Fence{});
    auto revoke = WireRig::get(r.tick(.25), "a", rc::GrantState::Brake);
    EXPECT_GT(revoke.grant_id, go.grant_id);
    EXPECT_DOUBLE_EQ(r.a().remaining, reserved);
    // The queued Go can have reached the vehicle before this Brake. Its receipt/Waiting is
    // not the newer exact Brake certificate and cannot retire the highest issued sweep.
    v.acknowledged_grant_id = go.grant_id;
    r.publish("a");
    r.tick();
    EXPECT_DOUBLE_EQ(r.a().remaining, reserved);
}
TEST(ResetCausal, OneLaterInvalidDeclarationDoesNotRejectAnAlreadyValidPeer) {
    WireRig r;
    auto go = r.start();
    r.s["b"].spec.footprint.half_length = -1;
    r.publish("b");
    auto out = r.tick(.45);
    for (const auto& d : out)
        if (d.vehicle == "a") {
            EXPECT_EQ(d.grant.state, rc::GrantState::Go);
            EXPECT_EQ(d.grant.grant_id, go.grant_id);
        }
    EXPECT_EQ(r.c.stats().admissions, 1);
}
TEST(ResetCausal, AValidBodySurvivesABadReportBeforeTheFirstPlannerTick) {
    WireRig r;
    r.body("a", {-4, 0, 0}, {4, 0, 0}, 11, 1);
    r.body("b", {0, 0, 0}, {0, 0, 0});
    r.s["b"].spec.footprint.half_length = -1;
    r.publish("b");
    auto brake = WireRig::get(r.tick(), "a", rc::GrantState::Brake);
    r.certify("a", brake);
    auto go = WireRig::get(r.tick(), "a", rc::GrantState::Go);
    EXPECT_FALSE(go.via.empty()) << "the last valid body still blocks the straight route";
}
TEST(ResetCausal, ANewSAndLowerNumericSessionStillNeedExactNewGStop) {
    WireRig r;
    r.start();
    const double reserved = r.a().remaining;
    auto& v = r.s["a"];
    v.session = 2;
    ++v.request_generation;
    v.phase = rc::Phase::Requesting;
    v.stop_certified = false;
    v.acknowledged_grant_id = 0;
    r.publish("a");
    auto b = WireRig::get(r.tick(), "a", rc::GrantState::Brake);
    EXPECT_EQ(b.session, 2u);
    EXPECT_DOUBLE_EQ(r.a().remaining, reserved);
    r.certify("a", b);
    auto go = WireRig::get(r.tick(), "a", rc::GrantState::Go);
    EXPECT_EQ(go.session, 2u);
    EXPECT_EQ(r.c.stats().admissions, 2);
}
TEST(ResetCausal, InitialPlusTwoReplacementsCannotBeResetByGeneration) {
    WireRig r;
    r.start();
    for (uint32_t g = 2; g <= 4; ++g) {
        auto& v = r.s["a"];
        v.request_generation = g;
        v.phase = rc::Phase::Requesting;
        v.acknowledged_grant_id = 0;
        v.stop_certified = false;
        r.publish("a");
        auto b = WireRig::get(r.tick(), "a", rc::GrantState::Brake);
        r.certify("a", b);
        auto out = r.tick();
        if (g < 4) {
            auto go = WireRig::get(out, "a", rc::GrantState::Go);
            r.receipt("a", go);
            r.tick();
        } else {
            auto reject = WireRig::get(out, "a", rc::GrantState::Reject);
            EXPECT_EQ(reject.reason, rc::Reason::NoRoute);
            EXPECT_EQ(reject.detail, "Reset replan budget exhausted");
        }
    }
    EXPECT_EQ(r.c.stats().admissions, 3);
}
TEST(ResetCausal, TerminalCertificateHoldAndLostIdleAreNotOneShotProof) {
    WireRig r;
    r.start();
    auto& v = r.s["a"];
    ++v.request_generation;
    v.phase = rc::Phase::Done;
    v.stop_certified = false;
    v.acknowledged_grant_id = 0;
    r.publish("a");
    auto b = WireRig::get(r.tick(), "a", rc::GrantState::Brake);
    EXPECT_GT(r.a().remaining, 0);
    r.certify("a", b);
    auto h = WireRig::get(r.tick(), "a", rc::GrantState::Hold);
    EXPECT_GT(h.grant_id, b.grant_id);
    EXPECT_EQ(r.a().remaining, 0);
    auto repeat = WireRig::get(r.tick(.45), "a", rc::GrantState::Hold);
    EXPECT_EQ(repeat.grant_id, h.grant_id);
    EXPECT_GT(repeat.publication_seq, h.publication_seq);
    v.acknowledged_grant_id = h.grant_id;
    v.stop_certified = false;
    v.phase = rc::Phase::Idle;
    r.publish("a");
    EXPECT_TRUE(r.tick(.45).empty());
}
TEST(ResetCausal, ScopeReadsOnlyFrozenKindNamespaceAndRejectsMalformedDeclarations) {
    EXPECT_EQ(
        rs::expectedResetNamespaces(
            R"([{"kind":"scout_mini","namespace":"/ugv1","half_length":999},{"kind":"mecanum_ugv","namespace":"/ugv2"},{"kind":"px4_multirotor","namespace":"/uav1"}])"),
        (std::vector<std::string>{"ugv1", "ugv2"}));
    EXPECT_TRUE(rs::expectedResetNamespaces("").empty());
    EXPECT_TRUE(rs::expectedResetNamespaces("[]").empty());
    EXPECT_THROW(rs::expectedResetNamespaces("{}"), std::exception);
    EXPECT_THROW(
        rs::expectedResetNamespaces(
            R"([{"kind":"scout_mini","namespace":"/a"},{"kind":"scout_mini","namespace":"a"}])"),
        std::exception);
}

// Protocol-only endpoints explicitly declare initial stopped certificates. Real publication,
// fresh-sample stopping and physical clearance are exercised by the Executor/plant harness.
struct FrozenSweepRig : WireRig {
    rc::Grant highest, barrier;
    double retained{0};

    FrozenSweepRig(bool received = true, bool retiring = true) {
        declare("a", {-4, 0, 0}, {4, 0, 0}, false, 1);
        declare("b", {0, -4, 0}, {0, 4, 0}, true, 0);
        auto initial = get(tick(), "a", rc::GrantState::Brake);
        certify("a", initial);
        highest = get(tick(), "a", rc::GrantState::Go);
        if (received) {
            receipt("a", highest);
            tick();
        }
        retained = a().remaining;
        if (retiring) {
            auto& v = s["a"];
            v.session = 2;  // numeric S may decrease; live G still advances
            ++v.request_generation;
            v.phase = rc::Phase::Idle;
            v.healthy = false;
            v.stop_certified = false;
            v.acknowledged_grant_id = 0;
            publish("a");
            barrier = get(tick(), "a", rc::GrantState::Brake);
        }
    }

    void declare(const std::string& id, rc::Pose pose, rc::Pose target, bool omni,
                 uint32_t generation) {
        rc::Session v;
        v.session = 11;
        v.request_generation = generation;
        v.phase = generation ? rc::Phase::Requesting : rc::Phase::Idle;
        v.pose = pose;
        v.target = target;
        v.healthy = true;
        v.spec.model = omni ? rc::Model::Omni : rc::Model::Unicycle;
        v.spec.footprint = omni ? rc::Footprint{.345, .32, 0, 0} : rc::Footprint{.45, .40, 0, 0};
        v.spec.ell_bound = omni ? 0 : .25;
        v.spec.max_v = .35;
        v.spec.max_vy = omni ? .35 : 0;
        v.spec.max_w = .5;
        v.spec.acc_v = .35;
        v.spec.acc_w = .6;
        v.ell_hat = omni ? 0 : -.229;
        v.ell_spread = omni ? 0 : .02;
        s[id] = v;
        publish(id);
    }

    std::vector<rs::OutgoingGrant> requestB() {
        auto& v = s["b"];
        v.request_generation = 1;
        v.phase = rc::Phase::Requesting;
        v.acknowledged_grant_id = 0;
        v.stop_certified = false;
        publish("b");
        auto brake = get(tick(), "b", rc::GrantState::Brake);
        certify("b", brake);
        return tick();
    }

    void narrowFence() {
        rs::Fence f;
        f.enabled = true;
        f.xmin = -1;
        f.xmax = 1;
        f.ymin = -5;
        f.ymax = 5;
        c.setWorld({}, f);
    }
};

TEST(ResetCausal, FrozenWholeSweepFarFromTheReportedPoseRequiresADetour) {
    FrozenSweepRig r;
    auto go = WireRig::get(r.requestB(), "b", rc::GrantState::Go);
    // a's last pose (-4,0) does not intersect b's x=0 route; its issued sweep does.
    EXPECT_FALSE(go.via.empty());
    EXPECT_GT(go.s_limit, 1e17);
    EXPECT_DOUBLE_EQ(r.a().remaining, r.retained);
    EXPECT_EQ(r.c.stats().admissions, 2);
}

TEST(ResetCausal, QueuedHighestGoSurvivesLowerNewSAndAnUnrelatedCertificate) {
    FrozenSweepRig r(false);
    ASSERT_EQ(r.s["a"].session, 2u);
    EXPECT_EQ(r.barrier.session, 2u);
    auto go = WireRig::get(r.requestB(), "b", rc::GrantState::Go);
    EXPECT_FALSE(go.via.empty());
    EXPECT_GT(go.s_limit, 1e17);
    EXPECT_GT(r.retained, 7);
    auto& v = r.s["a"];
    v.healthy = true;
    v.stop_certified = true;
    v.acknowledged_grant_id = r.highest.grant_id;  // never the newer exact Brake
    r.publish("a");
    r.tick();
    EXPECT_DOUBLE_EQ(r.a().remaining, r.retained);
}

TEST(ResetCausal, AnExecutingMoverKeepsTheOriginalSoftPrefixRule) {
    FrozenSweepRig r(true, false);
    auto go = WireRig::get(r.requestB(), "b", rc::GrantState::Go);
    EXPECT_TRUE(go.via.empty());
    EXPECT_GT(go.s_limit, .25);
    EXPECT_LT(go.s_limit, 4);
    EXPECT_DOUBLE_EQ(r.a().remaining, r.retained);
}

TEST(ResetCausal, NoDetourNeverGrantsThroughOrRetiresTheFrozenSweep) {
    FrozenSweepRig r;
    r.narrowFence();
    auto out = r.requestB();
    for (int k = 0; k < 12; ++k) {
        for (const auto& d : out)
            if (d.vehicle == "b")
                EXPECT_NE(d.grant.state, rc::GrantState::Go);
        EXPECT_DOUBLE_EQ(r.a().remaining, r.retained);
        r.publish("a");
        r.publish("b");
        out = r.tick(.5);
    }
    EXPECT_EQ(r.c.stats().admissions, 1);
}

TEST(ResetCausal, OnlyFrozenPlansDoNotHideTheOriginalNoMoverDeadlock) {
    FrozenSweepRig r;
    r.narrowFence();
    auto out = r.requestB();
    bool deadlock = false;
    for (const auto& d : out)
        deadlock = deadlock || (d.vehicle == "b" && d.grant.state == rc::GrantState::Reject &&
                                d.grant.reason == rc::Reason::Deadlock);
    EXPECT_TRUE(deadlock);
    EXPECT_DOUBLE_EQ(r.a().remaining, r.retained);
    EXPECT_EQ(r.c.stats().admissions, 1);
}

TEST(ResetCausal, FrozenOnlyTimeDoesNotStallAnUnrelatedNewMover) {
    FrozenSweepRig r;
    auto& v = r.s["b"];
    v.pose = {0, -4, 0};
    v.target = {4, -4, 0};
    for (int k = 0; k < 120; ++k) {
        r.publish("a");
        r.publish("b");
        r.tick(.1);
    }
    auto go = WireRig::get(r.requestB(), "b", rc::GrantState::Go);
    v.acknowledged_grant_id = go.grant_id;
    v.stop_certified = false;
    v.phase = rc::Phase::Waiting;
    r.publish("b");
    for (const auto& d : r.tick(.25))
        if (d.vehicle == "b")
            EXPECT_NE(d.grant.reason, rc::Reason::Stalled);
    EXPECT_DOUBLE_EQ(r.a().remaining, r.retained);
}

TEST(ResetCausal, AnExactStopProofDoesNotOpenGoWhileTheSceneIsUnavailable) {
    WireRig r({"a"});
    r.c.setScene(true, true);
    r.body("a", {-4, 0, 0}, {4, 0, 0}, 11, 1);
    auto brake = WireRig::get(r.tick(), "a", rc::GrantState::Brake);
    r.certify("a", brake);
    auto go = WireRig::get(r.tick(), "a", rc::GrantState::Go);
    r.receipt("a", go);
    r.tick();
    const double reserved = r.a().remaining;
    r.c.setScene(true, false);
    brake = WireRig::get(r.tick(), "a", rc::GrantState::Brake);
    EXPECT_EQ(brake.reason, rc::Reason::SceneUnavailable);
    EXPECT_DOUBLE_EQ(r.a().remaining, reserved);
    auto& v = r.s["a"];
    ++v.request_generation;
    v.phase = rc::Phase::Requesting;
    v.acknowledged_grant_id = 0;
    v.stop_certified = false;
    r.publish("a");
    brake = WireRig::get(r.tick(), "a", rc::GrantState::Brake);
    r.certify("a", brake);
    for (int k = 0; k < 4; ++k) {
        r.publish("a");
        for (const auto& d : r.tick(.1))
            EXPECT_NE(d.grant.state, rc::GrantState::Go);
    }
    EXPECT_DOUBLE_EQ(r.a().remaining, 0);  // exact stop proof retired it; availability did not
    rs::ConvexObstacle wall;
    wall.id = "wall";
    wall.vertices = {{0, -1}, {.3, -1}, {.3, 1}, {0, 1}};
    r.c.setWorld({wall}, {});
    r.c.setScene(true, true);
    go = WireRig::get(r.tick(), "a", rc::GrantState::Go);
    EXPECT_FALSE(go.via.empty());
    EXPECT_EQ(go.request_generation, v.request_generation);
}
}  // namespace
