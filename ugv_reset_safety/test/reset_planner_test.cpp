#include <algorithm>
#include <cmath>
#include <sstream>

#include "planner_test_support.h"

namespace reset_test {
namespace {

using ugv_reset_safety::ConvexObstacle;
using ugv_reset_safety::Fence;
using ugv_reset_safety::PlannerConfig;
using ugv_reset_safety::PlannerDecision;
using ugv_reset_safety::PlannerVehicle;
using ResetPlanner = StoppedPlannerFixture;

rc::VehicleSpec scoutSpec() {
    rc::VehicleSpec spec;
    spec.model = rc::Model::Unicycle;
    spec.footprint = {0.45, 0.40, 0.0, 0.0};
    spec.ell_bound = 0.25;
    return spec;
}

rc::VehicleSpec omniSpec() {
    rc::VehicleSpec spec;
    spec.model = rc::Model::Omni;
    spec.footprint = {0.345, 0.32, 0.0, 0.0};
    spec.max_vy = 0.35;
    spec.ell_bound = 0.0;
    return spec;
}

std::string dataFile(const std::string& name) {
    return std::string(RESET_TEST_DATA_DIR) + "/" + name;
}

Scenario scenario(const std::string& name, std::vector<Eigen::Vector3d> starts,
                  std::vector<Eigen::Vector3d> goals) {
    Scenario s;
    s.name = name;
    s.starts = std::move(starts);
    s.goals = std::move(goals);
    return s;
}

DriveOptions scoutOptions() {
    DriveOptions options;
    options.spec = scoutSpec();
    options.chassis_half_length = 0.31;
    options.chassis_half_width = 0.26;
    return options;
}

TEST(ResetPlanner, OneVehicleDrivesItsFrozenRouteInTheOpenAndArrives) {
    ScenarioFile world;
    const auto s = scenario("single", {{-3.0, 0.0, 0.0}}, {{3.0, 1.0, 1.0}});
    int go = 0;
    DriveOptions options = scoutOptions();
    options.observer = [&](double, const std::vector<PlannerDecision>& decisions) {
        for (const auto& d : decisions) {
            go += d.state == rc::GrantState::Go ? 1 : 0;
            if (d.state == rc::GrantState::Go) {
                EXPECT_EQ(d.via.size(), 0U) << "the open field needs no via point";
                EXPECT_TRUE(d.final_move);
                EXPECT_GE(d.s_limit, 6.0) << "nothing in the way: the whole route is granted";
            }
        }
    };
    const DriveResult r = drive(world, s, options);
    EXPECT_TRUE(r.all_arrived);
    EXPECT_EQ(go, 1) << "one grant, never re-planned";
    EXPECT_GE(r.min_chassis_clearance, 0.0);
}

TEST(ResetPlanner, TheRouteGoesAroundAStaticWithTheRequiredSweepClearance) {
    ScenarioFile world;
    world.obstacles.push_back({{-0.5, -1.0}, {0.5, -1.0}, {0.5, 1.0}, {-0.5, 1.0}});
    const auto s = scenario("around", {{-3.0, 0.0, 0.0}}, {{3.0, 0.0, 0.0}});
    DriveOptions options = scoutOptions();
    size_t via = 0;
    options.observer = [&](double, const std::vector<PlannerDecision>& decisions) {
        for (const auto& d : decisions) {
            if (d.state == rc::GrantState::Go) {
                via = std::max(via, d.via.size());
            }
        }
    };
    const DriveResult r = drive(world, s, options);
    EXPECT_TRUE(r.all_arrived);
    EXPECT_GE(via, 1U) << "a detour is needed";
    // The continuous sweep of the planning footprint keeps the static margin (the sampling error
    // is added by the verifier).
    EXPECT_GE(r.min_static_clearance, options.planner.static_margin - 1e-6);
}

TEST(ResetPlanner, TwoVehiclesCrossingAtTheCentreKeepThePairMargin) {
    ScenarioFile world;
    const auto s =
        scenario("cross", {{-3.0, 0.0, 0.0}, {0.0, -3.0, 1.5}}, {{3.0, 0.0, 0.0}, {0.0, 3.0, 1.5}});
    const DriveResult r = drive(world, s, scoutOptions());
    EXPECT_TRUE(r.all_arrived);
    EXPECT_GE(r.min_pair_clearance, 0.10 - 1e-6);
    EXPECT_GE(r.min_chassis_clearance, 0.0);
}

TEST(ResetPlanner, SwappingPlacesHeadOnIsResolvedWithoutAContact) {
    ScenarioFile world;
    const auto s = scenario("swap", {{-0.8, 0.0, 0.0}, {0.8, 0.0, 3.14159}},
                            {{0.8, 0.0, 0.0}, {-0.8, 0.0, 3.14159}});
    const DriveResult r = drive(world, s, scoutOptions());
    EXPECT_TRUE(r.all_arrived) << "reasons: " << r.reasons.size();
    EXPECT_GE(r.min_pair_clearance, 0.10 - 1e-6);
    EXPECT_GE(r.min_chassis_clearance, 0.0);
}

// The four-Scout fleet lawfully ends its drive with neighbours inside the pair margin (the
// recorded receipt: 3.2 cm / 3.5 cm edge gaps between ugv1-ugv2 and ugv3-ugv4). The parked
// overlap is inherited, not created by any route: one vehicle of each pair must be able to
// leave while never closing the inherited gap, then its neighbour follows.
TEST(ResetPlanner, ScoutsParkedInsideThePairMarginSequenceOutOfTheOverlap) {
    ScenarioFile world;
    const auto s =
        scenario("overlap-start",
                 {{-4.5613497, -1.6651842, -0.0872233},
                  {-5.5977380, -2.0857238, 0.6334592},
                  {-5.4382727, -4.7342245, 0.0046286},
                  {-4.5565166, -5.4201524, 0.4086611}},
                 {{-6.0, -1.2, -0.3929740},
                  {-6.0, -2.8, 1.0791080},
                  {-6.0, -4.4, -1.5279221},
                  {-6.0, -6.0, 0.5332720}});
    const DriveResult r = drive(world, s, scoutOptions());
    EXPECT_TRUE(r.all_arrived) << "rejections: " << r.rejected;
    EXPECT_EQ(r.rejected, 0);
    EXPECT_GE(r.min_chassis_clearance, 0.0);
}

// The escape allowance never becomes an approach allowance: a start overlap may only shrink,
// never stay level while another part of the route closes further than the inherited gap.
TEST(ResetPlanner, TheParkedEscapeNeverClosesTheInheritedGap) {
    ScenarioFile world;
    // Both scouts start inside the pair margin (~0.09 m edge gap) with goals pointing away
    // from each other: both have to pull out of the same overlap without a contact.
    const auto s = scenario("overlap-escape",
                            {{-0.94, 0.0, 0.0}, {0.0, 0.0, 1.5707963}},
                            {{-3.0, 0.0, 0.0}, {2.0, 0.0, 0.0}});
    const DriveResult r = drive(world, s, scoutOptions());
    EXPECT_TRUE(r.all_arrived) << "rejections: " << r.rejected;
    EXPECT_GE(r.min_chassis_clearance, 0.0);
    // The escape certificate's bound is the exact inherited gap of THIS pair, not merely
    // non-penetration: the planning footprints may never come closer than where they started.
    const double inherited = clearance(rect(-0.94, 0.0, 0.0, 0.45, 0.40),
                                       rect(0.0, 0.0, 1.5707963, 0.45, 0.40));
    ASSERT_LT(inherited, scoutOptions().planner.pair_margin);
    const auto it = r.pair_gaps.find({0, 1});
    ASSERT_NE(it, r.pair_gaps.end()) << "the start-overlapped pair must still be audited";
    EXPECT_GE(it->second, inherited - 1e-6);
}

// The first move of a unicycle route is a rotation about the vehicle's rotation centre. Starting
// beside a parked edge inside the pair margin, the required turn sweeps a moving vertex across
// the parked edge's interior while every sampled endpoint still clears the inherited gap - a
// vertex-only check would grant it. The admission certificate must refuse the sweep (or find an
// alternative that keeps the gap); here no alternative exists, so admission fails outright.
TEST(ResetPlanner, ATurnSweepingAVertexAcrossAParkedEdgeIsRefused) {
    auto turn = [](bool parked_peer) {
        ResetPlanner planner;
        planner.setWorld({}, Fence());
        std::vector<PlannerVehicle> vehicles;
        PlannerVehicle self;
        self.id = "ugv1";
        self.session = 7;
        self.request_generation = 1;
        self.stop_certified = true;
        self.spec = scoutSpec();
        self.pose = {0.0, 0.0, 0.0};
        self.target = {2.0, 2.0, 1.5707963};
        self.phase = rc::Phase::Requesting;
        self.ell_hat = -0.229;
        self.ell_spread = 0.0;
        vehicles.push_back(self);
        if (parked_peer) {
            PlannerVehicle peer;
            peer.id = "ugv2";
            peer.spec = scoutSpec();
            peer.pose = {-0.95, 0.0, 0.0};  // right edge 0.05 left of ugv1's left edge
            peer.phase = rc::Phase::Idle;
            vehicles.push_back(peer);
        }
        return planner.update(0.0, vehicles);
    };
    for (const auto& d : turn(true)) {
        EXPECT_NE(d.state, rc::GrantState::Go) << "the dipping sweep must never be granted";
    }
    bool go = false;
    for (const auto& d : turn(false)) {
        go = go || d.state == rc::GrantState::Go;
    }
    EXPECT_TRUE(go) << "the same turn is freely grantable without the parked edge";
}

// A start pose already in contact with a parked body (inherited gap <= 0) may not be issued an
// escape certificate: every candidate is refused and nobody moves.
TEST(ResetPlanner, AStartInContactWithAParkedBodyIsRefused) {
    ResetPlanner planner;
    planner.setWorld({}, Fence());
    std::vector<PlannerVehicle> vehicles(2);
    for (int i = 0; i < 2; ++i) {
        vehicles[i].id = "ugv" + std::to_string(i + 1);
        vehicles[i].session = 7;
        vehicles[i].request_generation = 1;
        vehicles[i].stop_certified = true;
        vehicles[i].spec = scoutSpec();
        vehicles[i].pose = {i == 0 ? -0.5 : 0.3, 0.0, 0.0};  // footprints interpenetrate
        vehicles[i].target = {i == 0 ? -3.0 : 3.0, 0.0, 0.0};
        vehicles[i].phase = rc::Phase::Requesting;
        vehicles[i].ell_hat = -0.229;
        vehicles[i].ell_spread = 0.0;
    }
    bool saw_contact = false;
    for (const auto& d : planner.update(0.0, vehicles)) {
        EXPECT_NE(d.state, rc::GrantState::Go) << "a contact start may not be escaped";
        saw_contact = saw_contact || d.detail.find("contact") != std::string::npos;
    }
    EXPECT_TRUE(saw_contact) << "the refusal must name the contact, not a generic failure";
}

// The inherited-gap certificate admits only escape, never a return below the pair margin: a
// route ending inside the margin of a peer that can never leave is refused, and the pair keeps
// the inherited gap for the whole drive.
TEST(ResetPlanner, AReturnInsideThePairMarginIsRefused) {
    ScenarioFile world;
    world.obstacles.push_back({{-1.5, 2.0}, {1.5, 2.0}, {1.5, 6.0}, {-1.5, 6.0}});
    const auto s = scenario("return-inside-margin",
                            {{-0.94, 0.0, 0.0}, {0.0, 0.0, 1.5707963}},
                            {{0.90, 0.0, 0.0}, {0.0, 4.0, 1.5707963}});
    const DriveResult r = drive(world, s, scoutOptions());
    EXPECT_FALSE(r.arrived.at("ugv1")) << "the route ends inside the margin: refused";
    const double inherited = clearance(rect(-0.94, 0.0, 0.0, 0.45, 0.40),
                                       rect(0.0, 0.0, 1.5707963, 0.45, 0.40));
    const auto it = r.pair_gaps.find({0, 1});
    ASSERT_NE(it, r.pair_gaps.end());
    EXPECT_GE(it->second, inherited - 1e-6);
    EXPECT_GE(r.min_chassis_clearance, 0.0);
}

// The certificate is owned by its admission: it lives on the admitted plan, matches only the
// peer id AND the frozen peer pose, and does not survive as a stale entitlement into the next
// session. After the escape, a new request that would park inside the same peer's margin meets
// the full pair margin, not the old inherited gap.
TEST(ResetPlanner, ANewSessionKeepsTheFullMarginAgainstTheSameParkedPeer) {
    ResetPlanner planner;
    planner.setWorld({}, Fence());
    auto vehicle = [](unsigned session, unsigned gen, rc::Phase phase, rc::Pose pose,
                      rc::Pose target) {
        PlannerVehicle v;
        v.id = "ugv1";
        v.session = session;
        v.request_generation = gen;
        v.stop_certified = true;
        v.spec = scoutSpec();
        v.pose = pose;
        v.target = target;
        v.phase = phase;
        v.ell_hat = -0.229;
        v.ell_spread = 0.0;
        return v;
    };
    PlannerVehicle peer;
    peer.id = "ugv2";
    peer.spec = scoutSpec();
    peer.pose = {0.0, 0.0, 1.5707963};
    peer.phase = rc::Phase::Idle;
    // Session 7: start overlapped (~0.09 gap), goal away - the escape is granted.
    const auto first = planner.update(
        0.0, {vehicle(7, 1, rc::Phase::Requesting, {-0.94, 0.0, 0.0}, {-3.0, 0.0, 0.0}), peer});
    bool go = false;
    for (const auto& d : first) {
        go = go || (d.id == "ugv1" && d.state == rc::GrantState::Go);
    }
    ASSERT_TRUE(go) << "the escape itself must be grantable";
    // Drive the granted route, then close the session out.
    planner.update(0.3, {vehicle(7, 1, rc::Phase::Moving, {-1.5, 0.0, 0.0}, {-3.0, 0.0, 0.0}), peer});
    planner.update(0.6, {vehicle(7, 1, rc::Phase::Moving, {-2.2, 0.0, 0.0}, {-3.0, 0.0, 0.0}), peer});
    planner.update(0.9, {vehicle(7, 1, rc::Phase::Done, {-3.0, 0.0, 0.0}, {-3.0, 0.0, 0.0}), peer});
    // Session 8: a goal 0.095 into the same peer's margin - above the old inherited gap (~0.09),
    // below the pair margin. A leaked d0 allowance would grant it; the full margin must not.
    for (int k = 0; k < 4; ++k) {
        for (const auto& d : planner.update(
                 1.4 + 0.2 * k,
                 {vehicle(8, 1, rc::Phase::Requesting, {-3.0, 0.0, 0.0}, {-0.945, 0.0, 0.0}),
                  peer})) {
            EXPECT_NE(d.state, rc::GrantState::Go)
                << "the inherited gap of the previous admission must not persist";
        }
    }
}

TEST(ResetPlanner, DecisionsAreADeterministicFunctionOfTheInputs) {
    const ScenarioFile world = loadScenarios(dataFile("tase_scenarios.txt"));
    for (size_t pick : {0U, 3U, 160U, 200U}) {
        std::string first, second;
        for (std::string* trace : {&first, &second}) {
            DriveOptions options = scoutOptions();
            options.horizon = 40.0;
            options.observer = [trace](double t, const std::vector<PlannerDecision>& decisions) {
                std::ostringstream text;
                text.precision(12);
                for (const auto& d : decisions) {
                    text << t << ' ' << d.id << ' ' << static_cast<int>(d.state) << ' '
                         << static_cast<int>(d.reason) << ' ' << d.grant_id << ' ' << d.s_limit
                         << ' ' << d.via.size();
                    for (const auto& p : d.via) {
                        text << ' ' << p.x << ',' << p.y;
                    }
                    text << '\n';
                }
                *trace += text.str();
            };
            drive(world, world.scenarios.at(pick), options);
        }
        EXPECT_FALSE(first.empty());
        EXPECT_EQ(first, second) << world.scenarios[pick].name;
    }
}

TEST(ResetPlanner, ANonFiniteTargetRejectsOnlyThatVehicle) {
    ScenarioFile world;
    const auto s = scenario("bad", {{-3.0, 0.0, 0.0}, {-3.0, 2.0, 0.0}},
                            {{3.0, 0.0, 0.0}, {std::nan(""), 2.0, 0.0}});
    const DriveResult r = drive(world, s, scoutOptions());
    EXPECT_TRUE(r.arrived.at("ugv1"));
    EXPECT_FALSE(r.arrived.at("ugv2"));
    EXPECT_EQ(r.rejected, 1);
    ASSERT_EQ(r.reasons.size(), 1U);
    EXPECT_EQ(r.reasons[0], rc::Reason::BadSession);
}

TEST(ResetPlanner, ATargetInsideAStaticIsRefusedForThatVehicleOnly) {
    ScenarioFile world;
    world.obstacles.push_back({{2.5, -0.5}, {3.5, -0.5}, {3.5, 0.5}, {2.5, 0.5}});
    const auto s = scenario("blocked", {{-3.0, 0.0, 0.0}, {-3.0, 3.0, 0.0}},
                            {{3.0, 0.0, 0.0}, {3.0, 3.0, 0.0}});
    const DriveResult r = drive(world, s, scoutOptions());
    EXPECT_FALSE(r.arrived.at("ugv1"));
    EXPECT_TRUE(r.arrived.at("ugv2"));
    ASSERT_EQ(r.reasons.size(), 1U);
    EXPECT_EQ(r.reasons[0], rc::Reason::GoalOccupied);
}

TEST(ResetPlanner, OverlappingTargetsRefuseBothVehiclesAndNameEachOther) {
    ResetPlanner planner;
    planner.setWorld({}, Fence());
    std::vector<PlannerVehicle> vehicles(3);
    for (int i = 0; i < 3; ++i) {
        vehicles[i].id = "ugv" + std::to_string(i + 1);
        vehicles[i].session = 1;
        vehicles[i].spec = scoutSpec();
        vehicles[i].pose = {-4.0, 1.5 * i, 0.0};
        vehicles[i].target = {3.0, 1.5 * i, 0.0};
        vehicles[i].phase = rc::Phase::Requesting;
    }
    vehicles[2].target = {3.0, 1.5 * 1 + 0.2, 0.0};  // on top of ugv2's target
    const auto decisions = planner.update(0.0, vehicles);
    std::map<std::string, PlannerDecision> by;
    for (const auto& d : decisions) {
        by[d.id] = d;
    }
    ASSERT_EQ(by.count("ugv1"), 1U);
    EXPECT_EQ(by["ugv1"].state, rc::GrantState::Go);
    for (const char* id : {"ugv2", "ugv3"}) {
        ASSERT_EQ(by.count(id), 1U) << id;
        EXPECT_EQ(by[id].state, rc::GrantState::Reject) << id;
        EXPECT_EQ(by[id].reason, rc::Reason::GoalOccupied) << id;
        ASSERT_EQ(by[id].blockers.size(), 1U) << id;
    }
    EXPECT_EQ(by["ugv2"].blockers[0], "ugv3");
    EXPECT_EQ(by["ugv3"].blockers[0], "ugv2");
}

TEST(ResetPlanner, ABodyOutsideTheResetOnATargetBlocksItOnlyIfItStays) {
    ResetPlanner planner;
    planner.setWorld({}, Fence());
    std::vector<PlannerVehicle> vehicles(2);
    vehicles[0].id = "ugv1";
    vehicles[0].session = 1;
    vehicles[0].spec = scoutSpec();
    vehicles[0].pose = {-4.0, 0.0, 0.0};
    vehicles[0].target = {3.0, 0.0, 0.0};
    vehicles[0].phase = rc::Phase::Requesting;
    vehicles[1].id = "ugv2";
    vehicles[1].spec = scoutSpec();
    vehicles[1].pose = {3.0, 0.0, 0.0};  // idle on ugv1's target
    vehicles[1].phase = rc::Phase::Idle;
    auto first = planner.update(0.0, vehicles);
    ASSERT_EQ(first.size(), 1U);
    EXPECT_EQ(first[0].state, rc::GrantState::Hold) << "a settle time before it counts";
    EXPECT_EQ(first[0].reason, rc::Reason::WaitingForPeer);
    ASSERT_EQ(first[0].blockers.size(), 1U);
    EXPECT_EQ(first[0].blockers[0], "ugv2");
    // It left within the settle time: the route is granted.
    vehicles[1].pose = {3.0, 4.0, 0.0};
    const auto second = planner.update(0.6, vehicles);
    ASSERT_EQ(second.size(), 1U);
    EXPECT_EQ(second[0].state, rc::GrantState::Go);
    // A fresh planner: it stays.
    ResetPlanner other;
    other.setWorld({}, Fence());
    vehicles[1].pose = {3.0, 0.0, 0.0};
    other.update(0.0, vehicles);
    const auto late = other.update(1.2, vehicles);
    ASSERT_EQ(late.size(), 1U);
    EXPECT_EQ(late[0].state, rc::GrantState::Reject);
    EXPECT_EQ(late[0].reason, rc::Reason::GoalOccupied);
}

TEST(ResetPlanner, ARouteThatNeedsMoreThanOneAndAHalfTimesTheLimitIsRefusedBeforeMotion) {
    ResetPlanner planner;
    Fence fence;
    fence.xmin = fence.ymin = -100.0;
    fence.xmax = fence.ymax = 100.0;
    planner.setWorld({}, fence);
    PlannerVehicle v;
    v.id = "ugv1";
    v.session = 1;
    v.spec = scoutSpec();
    v.pose = {-40.0, 0.0, 0.0};
    v.target = {40.0, 0.0, 0.0};
    v.phase = rc::Phase::Requesting;
    const auto decisions = planner.update(0.0, {v});
    ASSERT_EQ(decisions.size(), 1U);
    EXPECT_EQ(decisions[0].state, rc::GrantState::Reject);
    EXPECT_EQ(decisions[0].reason, rc::Reason::TooSlow);
    // 25 m at 0.7 x 0.35 m/s is about 100 s: slow, but within 1.5 x 90 s, so it runs.
    v.pose = {-12.5, 0.0, 0.0};
    v.target = {12.5, 0.0, 0.0};
    ResetPlanner other;
    other.setWorld({}, fence);
    const auto run = other.update(0.0, {v});
    ASSERT_EQ(run.size(), 1U);
    EXPECT_EQ(run[0].state, rc::GrantState::Go);
}

TEST(ResetPlanner, AParkedBodyThatBlocksTheRestOfAHeldRouteRevokesItAndTheVehicleIsReplanned) {
    ResetPlanner planner;
    planner.setWorld({}, Fence());
    PlannerVehicle one, two;
    one.id = "ugv1";
    one.session = 1;
    one.spec = scoutSpec();
    one.pose = {-3.0, 0.0, 0.0};
    one.target = {3.0, 0.0, 0.0};
    one.phase = rc::Phase::Requesting;
    two = one;
    two.id = "ugv2";
    two.pose = {0.0, -3.0, 1.5708};
    two.target = {0.0, 3.0, 1.5708};
    std::map<std::string, PlannerDecision> by;
    for (const auto& d : planner.update(0.0, {one, two})) {
        by[d.id] = d;
    }
    ASSERT_EQ(by["ugv1"].state, rc::GrantState::Go);
    ASSERT_GE(by["ugv1"].s_limit, 6.0) << "ugv1 is first: the whole route";
    ASSERT_EQ(by["ugv2"].state, rc::GrantState::Go);
    ASSERT_LT(by["ugv2"].s_limit, 6.0) << "ugv2 crosses ugv1's reservation: a prefix only";
    EXPECT_GT(by["ugv2"].s_limit, 0.25);
    // ugv1 stopped dead in the crossing point (its session ended there): it is a parked body now,
    // and ugv2, held at the end of its prefix, has the rest of its frozen route blocked.
    one.phase = rc::Phase::Idle;
    one.pose = {0.0, 0.0, 0.0};
    two.phase = rc::Phase::Waiting;
    two.pose = {0.0, -3.0, 1.5708};
    std::vector<PlannerDecision> revoked;
    for (int tick = 0; (0.3 + tick * 0.1) < 1.0 && revoked.empty(); ++tick) {
        const double t = 0.3 + tick * 0.1;
        for (const auto& d : planner.update(t, {one, two})) {
            if (d.id == "ugv2") {
                revoked.push_back(d);
            }
        }
    }
    ASSERT_FALSE(revoked.empty());
    EXPECT_EQ(revoked[0].id, "ugv2");
    EXPECT_EQ(revoked[0].state, rc::GrantState::Brake);
    EXPECT_EQ(revoked[0].reason, rc::Reason::Replan);
    ASSERT_EQ(revoked[0].blockers.size(), 1U);
    EXPECT_EQ(revoked[0].blockers[0], "ugv1");
    EXPECT_EQ(planner.stats().revocations, 1);
    // The ideal fixture performs the exact Brake/certificate wire turns before the detour.
    ASSERT_GE(revoked.size(), 2U);
    EXPECT_EQ(revoked[1].id, "ugv2");
    EXPECT_EQ(revoked[1].state, rc::GrantState::Go);
    EXPECT_GE(revoked[1].via.size(), 1U);
    EXPECT_GE(revoked[1].s_limit, 6.0);
}

TEST(ResetPlanner, AVehicleMayTurnInPlaceAtItsSlotBesideAParkedNeighbourAtTheSlotPitch) {
    // The TASE slot line: targets 1.6 m apart. ugv3 stands at its slot; ugv2 is two metres from
    // its own and must end with a 2.7 rad turn about its rotation centre. The footprints are
    // 0.9 x 0.8 m; the turn clears the neighbour's envelope by about 0.15 m at the pitch.
    for (const double spread : {0.0, 0.02, -1.0}) {
        ResetPlanner planner;
        planner.setWorld({}, Fence());
        PlannerVehicle two, three;
        two.id = "ugv2";
        two.session = 1;
        two.spec = scoutSpec();
        two.pose = {-6.09, -2.41, -1.60};
        two.target = {-6.0, -2.8, 1.08};
        two.phase = rc::Phase::Requesting;
        two.ell_hat = -0.229;
        two.ell_spread = spread;
        three = two;
        three.id = "ugv3";
        three.session = 2;
        three.pose = {-6.0, -4.4, -1.53};
        three.target = {-6.0, -4.4, -1.53};
        three.phase = rc::Phase::Idle;
        std::map<std::string, PlannerDecision> by;
        for (const auto& d : planner.update(0.0, {two, three})) {
            by[d.id] = d;
        }
        ASSERT_EQ(by.count("ugv2"), 1U);
        std::ostringstream text;
        text << "spread " << spread << ": state " << static_cast<int>(by["ugv2"].state)
             << " reason " << rc::name(by["ugv2"].reason) << " " << by["ugv2"].detail << " s_limit "
             << by["ugv2"].s_limit << " final " << by["ugv2"].final_move << " goal ("
             << by["ugv2"].goal.x << "," << by["ugv2"].goal.y << ")";
        EXPECT_EQ(by["ugv2"].state, rc::GrantState::Go) << text.str();
        EXPECT_TRUE(by["ugv2"].final_move) << text.str();
        EXPECT_GE(by["ugv2"].s_limit, 0.8) << text.str();
    }
}

TEST(ResetPlanner, ASlotNeighbourThatArrivesBesideAHeldVehicleDoesNotRevokeItsFinalTurn) {
    for (const double spread : {0.0, 0.02, -1.0}) {
        ResetPlanner planner;
        planner.setWorld({}, Fence());
        PlannerVehicle two, three;
        two.id = "ugv2";
        two.session = 1;
        two.spec = scoutSpec();
        two.pose = {-5.99, 0.74, -1.59};
        two.target = {-6.0, -2.8, 1.08};
        two.phase = rc::Phase::Requesting;
        two.ell_hat = -0.229;
        two.ell_spread = spread;
        three = two;
        three.id = "ugv3";
        three.session = 2;
        three.pose = {-6.0, -0.76, -1.49};
        three.target = {-6.0, -4.4, -1.53};
        three.ell_hat = -0.234;
        std::map<std::string, PlannerDecision> by;
        double t = 0.0;
        for (const auto& d : planner.update(t, {two, three})) {
            by[d.id] = d;
        }
        // ugv3 goes first and arrives; ugv2 follows down the slot line and is held at its prefix.
        three.phase = rc::Phase::Idle;
        three.pose = {-6.0, -4.4, -1.53};
        two.phase = rc::Phase::Waiting;
        two.pose = {-6.09, -2.41, -1.60};
        bool revoked = false;
        std::string what;
        for (int k = 0; k < 20; ++k) {
            t += 0.1;
            for (const auto& d : planner.update(t, {two, three})) {
                if (d.id == "ugv2" && d.state == rc::GrantState::Brake) {
                    revoked = true;
                    what = d.detail;
                }
            }
        }
        EXPECT_FALSE(revoked) << "spread " << spread << ": " << what;
    }
}

TEST(ResetPlanner, MoversThatNobodyAdvancesAreRejectedAsStalledAfterTenSeconds) {
    ResetPlanner planner;
    planner.setWorld({}, Fence());
    PlannerVehicle v;
    v.id = "ugv1";
    v.session = 1;
    v.spec = scoutSpec();
    v.pose = {-3.0, 0.0, 0.0};
    v.target = {3.0, 0.0, 0.0};
    v.phase = rc::Phase::Requesting;
    ASSERT_EQ(planner.update(0.0, {v}).at(0).state, rc::GrantState::Go);
    v.phase = rc::Phase::Waiting;  // never moves
    std::vector<PlannerDecision> last;
    for (int tick = 0; (0.2 + tick * 0.1) < 12.0; ++tick) {
        const double t = 0.2 + tick * 0.1;
        auto d = planner.update(t, {v});
        if (!d.empty()) {
            last = d;
            EXPECT_GT(t, 10.0) << "rejected too early";
            break;
        }
    }
    ASSERT_FALSE(last.empty());
    EXPECT_EQ(last[0].state, rc::GrantState::Reject);
    EXPECT_EQ(last[0].reason, rc::Reason::Stalled);
}

TEST(ResetPlanner, ASceneChangeUnderAGrantedRouteBrakesTheVehicle) {
    ResetPlanner planner;
    planner.setWorld({}, Fence());
    PlannerVehicle v;
    v.id = "ugv1";
    v.session = 1;
    v.spec = scoutSpec();
    v.pose = {-4.0, 0.0, 0.0};
    v.target = {4.0, 0.0, 0.0};
    v.phase = rc::Phase::Requesting;
    ASSERT_EQ(planner.update(0.0, {v}).at(0).state, rc::GrantState::Go);
    v.phase = rc::Phase::Moving;
    ConvexObstacle wall;
    wall.id = "new-wall";
    wall.vertices = {{-0.2, -1.0}, {0.2, -1.0}, {0.2, 1.0}, {-0.2, 1.0}};
    planner.setWorld({wall}, Fence());
    const auto decisions = planner.update(0.3, {v});
    ASSERT_FALSE(decisions.empty());
    EXPECT_EQ(decisions[0].state, rc::GrantState::Brake);
    EXPECT_EQ(decisions[0].reason, rc::Reason::SceneChanged);
    ASSERT_EQ(decisions[0].blockers.size(), 1U);
    EXPECT_EQ(decisions[0].blockers[0], "new-wall");
}

TEST(ResetPlanner, AParkedBodyThatMovesIntoAGrantedRouteBrakesTheVehicle) {
    // The peer never participates: declared once, then it moves into the certified
    // corridor mid-drive with no setWorld call. Its pose delta is itself a scene
    // change, so the extension sweep must re-verify and revoke the full grant.
    StoppedPlannerFixture planner;
    planner.setWorld({}, Fence());
    PlannerVehicle one, two;
    one.id = "ugv1";
    one.session = 1;
    one.spec = scoutSpec();
    one.pose = {0.0, 0.0, 0.0};
    one.target = {3.5, 0.0, 0.0};
    one.phase = rc::Phase::Requesting;
    two.id = "ugv2";
    two.session = 2;
    two.spec = scoutSpec();
    two.pose = {5.0, 5.0, 0.0};
    two.target = {5.0, 5.0, 0.0};
    two.phase = rc::Phase::Idle;
    const auto granted = planner.update(0.0, {one, two});
    ASSERT_EQ(granted.size(), 1U);
    ASSERT_EQ(granted[0].state, rc::GrantState::Go);
    ASSERT_GE(granted[0].s_limit, 3.5);
    one.phase = rc::Phase::Moving;
    one.pose = {0.7, 0.0, 0.0};
    EXPECT_TRUE(planner.update(0.3, {one, two}).empty());
    // ugv2 slides into the corridor dead ahead while ugv1 keeps driving.
    one.pose = {1.4, 0.0, 0.0};
    two.pose = {2.8, 0.0, 0.0};
    std::vector<PlannerDecision> brakes;
    for (const auto& d : planner.update(0.6, {one, two})) {
        if (d.id == "ugv1" && d.state == rc::GrantState::Brake) {
            brakes.push_back(d);
        }
    }
    ASSERT_FALSE(brakes.empty()) << "a moved parked body must invalidate the certified route";
    EXPECT_EQ(brakes[0].reason, rc::Reason::Replan);
    ASSERT_EQ(brakes[0].blockers.size(), 1U);
    EXPECT_EQ(brakes[0].blockers[0], "ugv2");
}

TEST(ResetPlanner, ATinyIncrementDriftOfACertPeerInvalidatesTheCertificate) {
    // The cert peer starts inside the pair margin (admission-owned EscapeCert at
    // the r7 geometry) and then creeps 0.5e-6 m per observation. Each step is
    // below the 1e-6 pose authority, but the reference is the last-validated
    // anchor - never overwritten by observations - so the increments accumulate
    // and the certificate stops matching. The escape samples then fall back to
    // the full margin, which the peer violates: Replan.
    StoppedPlannerFixture planner;
    planner.setWorld({}, Fence());
    PlannerVehicle one, two;
    one.id = "ugv1";
    one.session = 1;
    one.spec = scoutSpec();
    one.pose = {-0.94, 0.0, 0.0};
    one.target = {-3.0, 0.0, 0.0};
    one.phase = rc::Phase::Requesting;
    two.id = "ugv2";
    two.session = 2;
    two.spec = scoutSpec();
    two.pose = {0.0, 0.0, 1.5707963};
    two.target = {2.0, 0.0, 0.0};
    two.phase = rc::Phase::Idle;
    const auto granted = planner.update(0.0, {one, two});
    ASSERT_EQ(granted.size(), 1U);
    ASSERT_EQ(granted[0].state, rc::GrantState::Go);
    one.phase = rc::Phase::Moving;
    one.pose = {-0.95, 0.0, 0.0};
    std::vector<PlannerDecision> brakes;
    for (int k = 1; k <= 8 && brakes.empty(); ++k) {
        two.pose = {-0.5e-6 * k, 0.0, 1.5707963};
        for (const auto& d : planner.update(0.05 * k, {one, two})) {
            if (d.id == "ugv1" && d.state == rc::GrantState::Brake) {
                brakes.push_back(d);
            }
        }
    }
    ASSERT_FALSE(brakes.empty())
        << "0.5e-6 per-observation drift must accumulate past the 1e-6 anchor";
    EXPECT_EQ(brakes[0].reason, rc::Reason::Replan);
    ASSERT_EQ(brakes[0].blockers.size(), 1U);
    EXPECT_EQ(brakes[0].blockers[0], "ugv2");
}

TEST(ResetPlanner, APendingPeerChangeIsNotErasedByUnrelatedVerifications) {
    // Two consumers share the planner: ugv1 holds a full grant (the cert peer
    // ugv2 inside its margin) and ugv4 holds a contested prefix, so an
    // unrelated extension sweep runs every 0.2 s. ugv2 creeps 0.05e-6 m per
    // observation - below the 1e-6 anchor authority inside any single
    // extension window. Re-baselining ugv2 on every unrelated verification
    // would erase the pending change before ugv1's grant ever rechecks; the
    // anchor belongs to the body alone.
    StoppedPlannerFixture planner;
    planner.setWorld({}, Fence());
    PlannerVehicle one, two, three, four;
    one.id = "ugv1";
    one.session = 1;
    one.spec = scoutSpec();
    one.pose = {-0.94, 0.0, 0.0};
    one.target = {-3.0, 0.0, 0.0};
    one.phase = rc::Phase::Requesting;
    two.id = "ugv2";
    two.session = 2;
    two.spec = scoutSpec();
    two.pose = {0.0, 0.0, 1.5707963};
    two.phase = rc::Phase::Idle;
    three.id = "ugv3";
    three.session = 3;
    three.spec = scoutSpec();
    three.pose = {-3.0, 6.0, 0.0};
    three.target = {3.0, 6.0, 0.0};
    three.phase = rc::Phase::Requesting;
    four.id = "ugv4";
    four.session = 4;
    four.spec = scoutSpec();
    four.pose = {0.0, 3.0, 1.5708};
    four.target = {0.0, 9.0, 1.5708};
    four.phase = rc::Phase::Requesting;
    std::map<std::string, PlannerDecision> by;
    for (const auto& d : planner.update(0.0, {one, two, three, four})) {
        by[d.id] = d;
    }
    ASSERT_EQ(by["ugv1"].state, rc::GrantState::Go);
    ASSERT_GE(by["ugv1"].s_limit, 2.0);
    ASSERT_EQ(by["ugv3"].state, rc::GrantState::Go);
    ASSERT_EQ(by["ugv4"].state, rc::GrantState::Go);
    ASSERT_LT(by["ugv4"].s_limit, 6.0) << "ugv4 must hold a prefix, not the route";
    // ugv3 waits short of the contested crossing so its reservation keeps
    // ugv4's prefix open; ugv1 creeps forward as a committed mover.
    one.phase = rc::Phase::Moving;
    three.phase = rc::Phase::Moving;
    four.phase = rc::Phase::Moving;
    one.pose = {-0.95, 0.0, 0.0};
    three.pose = {-0.5, 6.0, 0.0};
    four.pose = {0.0, 3.5, 1.5708};
    std::vector<PlannerDecision> brakes;
    for (int k = 1; k <= 120 && brakes.empty(); ++k) {
        two.pose = {-0.05e-6 * k, 0.0, 1.5707963};
        for (const auto& d : planner.update(0.02 * k, {one, two, three, four})) {
            if (d.id == "ugv1" && d.state == rc::GrantState::Brake) {
                brakes.push_back(d);
            }
        }
    }
    ASSERT_FALSE(brakes.empty())
        << "unrelated sweeps must not erase drift accumulating toward the grant";
    EXPECT_EQ(brakes[0].reason, rc::Reason::Replan);
    ASSERT_EQ(brakes[0].blockers.size(), 1U);
    EXPECT_EQ(brakes[0].blockers[0], "ugv2");
}

// Root's scout-certification counterexample (a statics-free, limited case): a certified
// participant whose route is hard at s=0 only because a blocking participant still awaits
// its post-zero stop certificate is not deadlocked - that certificate can still arrive,
// and once it does the same blocker is a parking claim (second arm). The irreversible
// verdict must wait for the in-flight certification.
TEST(ResetPlanner, ABlockerAwaitingItsStopCertificateDefersTheDeadlockVerdict) {
    const rc::Pose poses[4] = {{-4.462397, -1.670390, -0.041435},
                               {-5.523253, -2.032793, 0.586472},
                               {-5.366939, -4.730242, 0.059541},
                               {-4.458062, -5.375016, 0.446796}};
    const rc::Pose goals[4] = {{-6.0, -1.2, -0.392974},
                               {-6.0, -2.8, 1.079108},
                               {-6.0, -4.4, -1.527922},
                               {-6.0, -6.0, 0.533272}};
    auto run = [&](bool both, bool ugv2_available = true) {
        PlannerConfig cfg;
        cfg.step_yaw = 0.035;
        ugv_reset_safety::ResetPlanner planner(cfg);
        Fence fence;
        fence.xmin = -10;
        fence.xmax = 10;
        fence.ymin = -10;
        fence.ymax = 10;
        planner.setWorld({}, fence);
        std::vector<PlannerVehicle> vehicles(4);
        for (size_t i = 0; i < vehicles.size(); ++i) {
            PlannerVehicle& v = vehicles[i];
            v.id = "ugv" + std::to_string(i + 1);
            v.session = 100 + i;
            v.request_generation = 1;
            v.spec = scoutSpec();
            v.pose = poses[i];
            v.target = goals[i];
            v.phase = rc::Phase::Requesting;
            v.ell_spread = -1.0;
        }
        const auto initial = planner.update(0.0, vehicles);
        for (const auto& d : initial) {
            for (auto& v : vehicles) {
                if (v.id == d.id && d.state == rc::GrantState::Brake) {
                    v.acknowledged_grant_id = d.grant_id;
                }
            }
        }
        vehicles[0].stop_certified = true;
        vehicles[1].stop_certified = both;
        vehicles[1].available = ugv2_available;
        return planner.update(0.268, vehicles);
    };
    // Only ugv1 certified: ugv2's certificate is still in flight, so the only fact against
    // ugv1 is a blocked route - a hold that names the pending peer, never a reject.
    bool held_by_awaiting = false;
    for (const auto& d : run(false)) {
        EXPECT_FALSE(d.state == rc::GrantState::Reject && d.reason == rc::Reason::Deadlock)
            << d.id << ": a blocker that can still certify is not deadlock evidence";
        held_by_awaiting =
            held_by_awaiting ||
            (d.id == "ugv1" && d.reason == rc::Reason::WaitingForPeer &&
             std::find(d.blockers.begin(), d.blockers.end(), "ugv2") != d.blockers.end());
    }
    EXPECT_TRUE(held_by_awaiting) << "the stuck body must stay held on its awaiting blocker";
    // A lost blocker can never land a certificate: it is not in flight, so the hold must
    // not persist - ugv1 is adjudicated on the facts it actually has.
    bool rejected_on_lost = false;
    for (const auto& d : run(false, false)) {
        rejected_on_lost =
            rejected_on_lost ||
            (d.id == "ugv1" && d.state == rc::GrantState::Reject &&
             d.reason == rc::Reason::Deadlock);
    }
    EXPECT_TRUE(rejected_on_lost)
        << "an unavailable blocker defers nothing - the verdict must not wait forever";
    // Both certified: the identical blocker becomes the parking move that frees ugv1.
    bool parked = false;
    for (const auto& d : run(true)) {
        if (d.id == "ugv2" && d.state == rc::GrantState::Go) {
            parked = true;
            EXPECT_FALSE(d.final_move);
            EXPECT_NEAR(d.goal.x, -7.12325, 1e-3);
            EXPECT_NEAR(d.goal.y, -2.03279, 1e-3);
        }
    }
    EXPECT_TRUE(parked) << "the certified blocker must be claimable by the parking move";
}

// The deferral is per-component, not global: a second pair that is genuinely mutually
// stuck with every blocker settled must still receive its deadlock verdict even while
// the awaiting-certification pair defers.
TEST(ResetPlanner, AnIndependentDeadlockedComponentStillGetsItsVerdict) {
    PlannerConfig cfg;
    cfg.step_yaw = 0.035;
    ugv_reset_safety::ResetPlanner planner(cfg);
    Fence fence;
    fence.xmin = -10;
    fence.xmax = 10;
    fence.ymin = -10;
    fence.ymax = 10;
    planner.setWorld({}, fence);
    std::vector<PlannerVehicle> vehicles(4);
    const rc::Pose poses[4] = {{-4.462397, -1.670390, -0.041435},
                               {-5.523253, -2.032793, 0.586472},
                               {5.0, 0.0, 0.0},
                               {5.6, 0.0, 0.0}};
    const rc::Pose goals[4] = {{-6.0, -1.2, -0.392974},
                               {-6.0, -2.8, 1.079108},
                               {8.0, 0.0, 0.0},
                               {2.0, 0.0, 0.0}};
    for (size_t i = 0; i < vehicles.size(); ++i) {
        PlannerVehicle& v = vehicles[i];
        v.id = "ugv" + std::to_string(i + 1);
        v.session = 100 + i;
        v.request_generation = 1;
        v.spec = scoutSpec();
        v.pose = poses[i];
        v.target = goals[i];
        v.phase = rc::Phase::Requesting;
        v.ell_spread = -1.0;
    }
    const auto initial = planner.update(0.0, vehicles);
    for (const auto& d : initial) {
        for (auto& v : vehicles) {
            if (v.id == d.id && d.state == rc::GrantState::Brake) {
                v.acknowledged_grant_id = d.grant_id;
            }
        }
    }
    // ugv1 certified with ugv2's certificate still in flight; ugv3/ugv4 are both
    // certified and sit inside each other's inflated start - a real deadlock that
    // must not be masked by the pending component.
    vehicles[0].stop_certified = true;
    vehicles[1].stop_certified = false;
    vehicles[2].stop_certified = true;
    vehicles[3].stop_certified = true;
    std::map<std::string, bool> deadlocked;
    for (const auto& d : planner.update(0.268, vehicles)) {
        deadlocked[d.id] =
            deadlocked[d.id] ||
            (d.state == rc::GrantState::Reject && d.reason == rc::Reason::Deadlock);
        if (d.id == "ugv1") {
            EXPECT_FALSE(d.state == rc::GrantState::Reject &&
                         d.reason == rc::Reason::Deadlock)
                << "the pending component still defers its verdict";
        }
    }
    EXPECT_FALSE(deadlocked["ugv1"]) << "ugv1's blocker is still certifying: deferred";
    EXPECT_TRUE(deadlocked["ugv3"])
        << "ugv3's component is unsolvable now - its verdict must arrive";
    EXPECT_TRUE(deadlocked["ugv4"])
        << "ugv4's component is unsolvable now - its verdict must arrive";
}

// The family of the design measurements (235 scenarios on the TASE scene, 4 Scouts, the Reset
// footprint 0.90 x 0.80): with ideal followers the planner alone must be sound, whatever it
// completes.
TEST(ResetPlannerFamily, EveryGrantOfTheTaseFamilyIsSoundAndMostScenariosComplete) {
    const ScenarioFile world = loadScenarios(dataFile("tase_scenarios_subset80.txt"));
    int completed = 0;
    int collisions = 0;
    std::vector<double> all_ms;
    double min_pair = 1e9, min_static = 1e9;
    for (const Scenario& s : world.scenarios) {
        DriveOptions options = scoutOptions();
        options.ell_known = true;
        const DriveResult r = drive(world, s, options);
        completed += r.all_arrived ? 1 : 0;
        collisions += r.min_chassis_clearance < -1e-6 ? 1 : 0;
        min_pair = std::min(min_pair, r.min_pair_clearance);
        min_static = std::min(min_static, r.min_static_clearance);
        all_ms.insert(all_ms.end(), r.work_ms.begin(), r.work_ms.end());
        EXPECT_GE(r.min_pair_clearance, options.planner.pair_margin - 1e-6) << s.name;
        EXPECT_GE(r.min_static_clearance, options.planner.static_margin - 1e-6) << s.name;
    }
    std::sort(all_ms.begin(), all_ms.end());
    std::printf(
        "family: %d/%zu completed, collisions %d, min pair %.3f, min static %.3f, "
        "planning calls ms p50 %.2f p95 %.2f max %.2f (n=%zu)\n",
        completed, world.scenarios.size(), collisions, min_pair, min_static,
        all_ms[all_ms.size() / 2], all_ms[all_ms.size() * 95 / 100], all_ms.back(), all_ms.size());
    EXPECT_LE(all_ms[all_ms.size() * 95 / 100], 50.0) << "planner budget per event (p95)";
    EXPECT_EQ(collisions, 0);
    EXPECT_GE(completed, static_cast<int>(0.85 * world.scenarios.size()));
}

// The vehicle's rotation-centre offset is unknown to the planner (it covers +-ell_bound): whatever
// the true value in that range, the verified sweeps contain the motion.
TEST(ResetPlannerFamily, TheSweepCoversEveryRotationCentreOffsetWithinTheDeclaredBound) {
    const ScenarioFile world = loadScenarios(dataFile("tase_scenarios_subset80.txt"));
    for (double ell : {-0.25, 0.0, 0.12, 0.25}) {
        int completed = 0;
        for (size_t i = 0; i < world.scenarios.size(); i += 3) {
            DriveOptions options = scoutOptions();
            options.ell_known = false;
            options.ell_true = ell;
            const DriveResult r = drive(world, world.scenarios[i], options);
            completed += r.all_arrived ? 1 : 0;
            EXPECT_GE(r.min_chassis_clearance, 0.0) << world.scenarios[i].name << " ell " << ell;
            EXPECT_GE(r.min_pair_clearance, options.planner.pair_margin - 1e-6)
                << world.scenarios[i].name << " ell " << ell;
            EXPECT_GE(r.min_static_clearance, options.planner.static_margin - 1e-6)
                << world.scenarios[i].name << " ell " << ell;
        }
        std::printf("unknown ell %.2f: %d of %zu completed\n", ell, completed,
                    (world.scenarios.size() + 2) / 3);
    }
}

// The omnidirectional vehicle (Wheeltec 0.41 x 0.36 plus the 0.14 m margin per side): a disc
// footprint, so any heading schedule of the vehicle is covered by the verified sweep.
TEST(ResetPlannerFamily, TheOmnidirectionalFootprintIsSoundOnTheSameFamily) {
    const ScenarioFile world = loadScenarios(dataFile("tase_scenarios_subset80.txt"));
    int completed = 0;
    for (const Scenario& s : world.scenarios) {
        DriveOptions options;
        options.spec = omniSpec();
        options.ell_true = 0.0;
        options.chassis_half_length = 0.205;
        options.chassis_half_width = 0.18;
        const DriveResult r = drive(world, s, options);
        completed += r.all_arrived ? 1 : 0;
        EXPECT_GE(r.min_chassis_clearance, 0.0) << s.name;
        EXPECT_GE(r.min_static_clearance, options.planner.static_margin - 1e-6) << s.name;
    }
    std::printf("omni family: %d/%zu completed\n", completed, world.scenarios.size());
    EXPECT_GE(completed, static_cast<int>(0.9 * world.scenarios.size()));
}

}  // namespace
}  // namespace reset_test
