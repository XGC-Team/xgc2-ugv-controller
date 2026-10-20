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
using ugv_reset_safety::ResetPlanner;

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
    const auto s = scenario("cross", {{-3.0, 0.0, 0.0}, {0.0, -3.0, 1.5}},
                            {{3.0, 0.0, 0.0}, {0.0, 3.0, 1.5}});
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
                         << static_cast<int>(d.reason) << ' ' << d.grant_id << ' ' << d.s_limit << ' '
                         << d.via.size();
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
    for (double t = 0.3; t < 1.0 && revoked.empty(); t += 0.1) {
        revoked = planner.update(t, {one, two});
    }
    ASSERT_FALSE(revoked.empty());
    EXPECT_EQ(revoked[0].id, "ugv2");
    EXPECT_EQ(revoked[0].state, rc::GrantState::Brake);
    EXPECT_EQ(revoked[0].reason, rc::Reason::Replan);
    ASSERT_EQ(revoked[0].blockers.size(), 1U);
    EXPECT_EQ(revoked[0].blockers[0], "ugv1");
    EXPECT_EQ(planner.stats().revocations, 1);
    // From where it stands, ugv2 is planned a detour around the parked body, in the same step.
    ASSERT_GE(revoked.size(), 2U);
    EXPECT_EQ(revoked[1].id, "ugv2");
    EXPECT_EQ(revoked[1].state, rc::GrantState::Go);
    EXPECT_GE(revoked[1].via.size(), 1U);
    EXPECT_GE(revoked[1].s_limit, 6.0);
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
    for (double t = 0.2; t < 12.0; t += 0.1) {
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
    std::printf("family: %d/%zu completed, collisions %d, min pair %.3f, min static %.3f, "
                "planning calls ms p50 %.2f p95 %.2f max %.2f (n=%zu)\n",
                completed, world.scenarios.size(), collisions, min_pair, min_static,
                all_ms[all_ms.size() / 2], all_ms[all_ms.size() * 95 / 100], all_ms.back(),
                all_ms.size());
    EXPECT_LE(all_ms[all_ms.size() * 95 / 100], 50.0) << "planner budget per event (p95)";
    EXPECT_EQ(collisions, 0);
    EXPECT_GE(completed, static_cast<int>(0.85 * world.scenarios.size()));
}

// The vehicle's rotation-centre offset is unknown to the planner (it covers +-ell_bound): whatever the
// true value in that range, the verified sweeps contain the motion.
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
