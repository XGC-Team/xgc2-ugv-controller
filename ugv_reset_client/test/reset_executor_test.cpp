#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

#include "executor_test_support.h"

namespace reset_test {
namespace {

// A route with a 90 degree corner and a reverse leg: (-3, 0, 0) -> (-1, 0) -> (-1, -2) -> goal.
struct Course {
    rc::Pose start{-3.0, 0.0, 0.0};
    std::vector<rc::Point> via{{-1.0, 0.0}};
    rc::Pose goal{-1.0, -2.0, -1.5708};
};

double lengthOf(const Course& c, double ell) {
    return rc::Route(c.start, c.via, c.goal, ell).length();
}

struct Outcome {
    bool ended{false};
    rc::Result result;
    double max_cross{0.0};
    double max_progress{0.0};
    double time{0.0};
    rc::Pose pose;
};

// Drive `course` with a whole-route grant, refreshed every `refresh` seconds.
Outcome drive(SoloRig& rig, const Course& c, double ell_plan, double s_limit, double seconds,
              double refresh = 0.4) {
    rig.exec().begin(1, c.goal, true, rig.measurement());
    Outcome out;
    double next = 0.0;
    uint32_t id = 1;
    out.ended = rig.run(seconds, [&](SoloRig& r, const rc::StepOutput&) {
        if (r.time() + 1e-12 >= next) {
            next = r.time() + refresh;
            r.deliver(goGrant(1, id, c.start, c.via, c.goal, ell_plan, s_limit), r.measurement());
        }
        out.max_cross = std::max(out.max_cross, std::abs(r.exec().tracker().crossTrack()));
        out.max_progress = std::max(out.max_progress, r.exec().tracker().progress());
    });
    rig.exec().takeResult(&out.result);
    out.time = rig.time();
    out.pose = rig.pose();
    return out;
}

TEST(ResetExecutor, FollowsAGrantedRouteToTheTargetAndEndsWithOneArrivedResult) {
    const Course c;
    PlantParams plant;
    SoloRig rig(scoutConfig(), c.start, plant);
    const Outcome o = drive(rig, c, plant.ell, 1e9, 90.0);
    ASSERT_TRUE(o.ended);
    EXPECT_EQ(o.result.outcome, rc::Outcome::Arrived) << o.result.detail;
    EXPECT_EQ(o.result.session, 1U);
    EXPECT_LE(o.result.final_planar_error, 0.05);
    EXPECT_LE(o.result.final_yaw_error, 5.0 * rc::kPi / 180.0);
    EXPECT_LT(o.result.duration, 90.0);
    rc::Result again;
    EXPECT_FALSE(rig.exec().takeResult(&again)) << "exactly one result per session";
    std::printf("course: arrived after %.1f s, max cross-track %.3f m\n", o.time, o.max_cross);
}

// The turns of Custom1 teach the offset before Reset starts (the estimator runs in every state).
void teach(SoloRig& rig) {
    rig.setManualCommand({0.0, 0.0, 0.4});
    rig.run(6.0);
    rig.setManualCommand({0.0, 0.0, -0.4});
    rig.run(6.0);
    rig.setManualCommand({});
    rig.run(1.0);
    rig.clearManualCommand();
}

TEST(ResetExecutor, TheTrackerConvergesOverLagCouplingNoiseAndDelay) {
    const Course c;
    int runs = 0;
    double worst_known = 0.0, worst_unknown = 0.0, slowest = 0.0;
    for (double lag : {0.5, 1.0, 2.0}) {
        for (double ell : {-0.25, -0.1, 0.0, 0.1, 0.25}) {
            for (double noise : {0.0, 0.001}) {
                for (bool known : {true, false}) {
                    PlantParams plant;
                    plant.ell = ell;
                    plant.lin_lag *= lag;
                    plant.ang_lag *= lag;
                    plant.pos_noise = noise;
                    plant.yaw_noise = noise / 0.3;
                    SoloRig rig(scoutConfig(), c.start, plant, 11 + runs);
                    if (known) {
                        teach(rig);
                    }
                    // The ell of the grant is the vehicle's own estimate: here the truth +- 0.05.
                    const Outcome o = drive(rig, c, ell + (runs % 2 ? 0.05 : -0.05), 1e9, 90.0);
                    ++runs;
                    ASSERT_TRUE(o.ended) << "lag " << lag << " ell " << ell << " noise " << noise
                                         << " known " << known;
                    EXPECT_EQ(o.result.outcome, rc::Outcome::Arrived)
                        << "lag " << lag << " ell " << ell << " noise " << noise << " known "
                        << known << ": " << o.result.detail;
                    EXPECT_LE(o.result.final_planar_error, 0.05);
                    EXPECT_LE(o.result.final_yaw_error, 5.0 * rc::kPi / 180.0);
                    (known ? worst_known : worst_unknown) =
                        std::max(known ? worst_known : worst_unknown, o.max_cross);
                    slowest = std::max(slowest, o.result.duration);
                }
            }
        }
    }
    std::printf(
        "tracker sweep: %d runs, worst cross-track %.3f m with the offset learned, %.3f m "
        "without, slowest %.1f s\n",
        runs, worst_known, worst_unknown, slowest);
    EXPECT_LT(worst_known, 0.15)
        << "inside the tube budget (0.10 + 2 x spread) once the offset is known";
    EXPECT_LT(slowest, 90.0);
}

// E2E 20261020T161318 (TASE-4UGVs-Wheeltec): two of four vehicles ended "ABORTED_STALLED: allowed
// to move but did not for 10 s" 6.1 and 4.9 cm from the slot with the heading right. They drove the
// 3.4 m line aligned with it and turned 66 degrees at the end; the tracker had entered its final
// stage on the distance of the rotation centre it believed in (offset not learned yet, true value
// +-0.04 m), the turn about the true centre left the origin 4-6 cm to the side, and the final stage
// can only move along the heading. A vehicle that may move and is farther than the arrival
// tolerance must move: it drives the lateral error out (teaching the offset with the turns) and
// arrives.
TEST(ResetExecutor, AFinalTurnAboutAWronglyBelievedCentreIsCorrectedInsteadOfStalling) {
    const rc::Pose start{-6.0, 2.2, -1.5943};
    const rc::Pose goal{-6.0, -1.2, -0.393};
    for (double ell : {-0.04, -0.02, 0.02, 0.04}) {
        PlantParams plant;
        plant.ell = ell;
        SoloRig rig(wheeltecConfig(), start, plant);
        rig.exec().begin(1, goal, true, rig.measurement());
        uint32_t id = 1;
        double next = 0.0;
        const bool ended = rig.run(90.0, [&](SoloRig& r, const rc::StepOutput&) {
            if (r.time() + 1e-12 >= next) {
                next = r.time() + 0.4;
                r.deliver(goGrant(1, id, start, {}, goal, 0.0, 1e18), r.measurement());
            }
        });
        rc::Result result;
        rig.exec().takeResult(&result);
        ASSERT_TRUE(ended) << "ell " << ell;
        EXPECT_EQ(result.outcome, rc::Outcome::Arrived) << "ell " << ell << ": " << result.detail;
        EXPECT_LE(result.final_planar_error, 0.05) << "ell " << ell;
        EXPECT_LE(result.final_yaw_error, 5.0 * rc::kPi / 180.0) << "ell " << ell;
    }
}

TEST(ResetExecutor, AnOmnidirectionalVehicleArrivesToo) {
    const Course c;
    PlantParams plant;
    plant.unicycle = false;
    SoloRig rig(omniConfig(), c.start, plant);
    const Outcome o = drive(rig, c, 0.0, 1e9, 90.0);
    ASSERT_TRUE(o.ended);
    EXPECT_EQ(o.result.outcome, rc::Outcome::Arrived) << o.result.detail;
    EXPECT_LE(o.result.final_planar_error, 0.05);
}

TEST(ResetExecutor, StopsAtTheEndOfTheGrantedPrefixAndContinuesWhenItGrows) {
    const Course c;
    PlantParams plant;
    SoloRig rig(scoutConfig(), c.start, plant);
    rig.exec().begin(1, c.goal, true, rig.measurement());
    double s_limit = 1.2;
    double max_progress = 0.0;
    bool waiting_at_limit = false;
    double next = 0.0;
    const bool ended = rig.run(90.0, [&](SoloRig& r, const rc::StepOutput& out) {
        if (r.time() + 1e-12 >= next) {
            next = r.time() + 0.4;
            r.deliver(goGrant(1, s_limit > 5.0 ? 2 : 1, c.start, c.via, c.goal, plant.ell, s_limit),
                      r.measurement());
        }
        max_progress = std::max(max_progress, r.exec().tracker().progress());
        if (s_limit < 5.0 && r.time() > 15.0) {
            waiting_at_limit = out.phase == rc::Phase::Waiting && r.speed() < 0.02;
            EXPECT_LE(max_progress, 1.2 + 0.05) << "the vehicle went past its prefix";
            s_limit = 1e9;
        }
    });
    EXPECT_TRUE(waiting_at_limit) << "it must report Waiting while held";
    ASSERT_TRUE(ended);
    EXPECT_EQ(rig.exec().result().outcome, rc::Outcome::Arrived);
}

TEST(ResetExecutor, AHeldVehicleDoesNotCountAsStalledAndAFrozenOneDoes) {
    const Course c;
    PlantParams plant;
    {
        SoloRig rig(scoutConfig(), c.start, plant);
        // Held at the very start for 40 s: nothing is allowed, nothing is expected.
        rig.exec().begin(1, c.goal, true, rig.measurement());
        double next = 0.0;
        const bool ended = rig.run(40.0, [&](SoloRig& r, const rc::StepOutput&) {
            if (r.time() + 1e-12 >= next) {
                next = r.time() + 0.4;
                r.deliver(goGrant(1, 1, c.start, c.via, c.goal, plant.ell, 0.0), r.measurement());
            }
        });
        EXPECT_FALSE(ended) << rig.exec().result().detail;
    }
    {
        SoloRig rig(scoutConfig(), c.start, plant);
        bool frozen = false;
        rig.exec().begin(1, c.goal, true, rig.measurement());
        double next = 0.0;
        const bool ended = rig.run(60.0, [&](SoloRig& r, const rc::StepOutput&) {
            if (r.time() + 1e-12 >= next) {
                next = r.time() + 0.4;
                r.deliver(goGrant(1, 1, c.start, c.via, c.goal, plant.ell, 1e9), r.measurement());
            }
            if (!frozen && r.time() > 1.0) {
                frozen = true;
                r.freezePlant(true);
            }
        });
        ASSERT_TRUE(ended);
        rc::Result result = rig.exec().result();
        EXPECT_EQ(result.outcome, rc::Outcome::AbortedStalled);
        EXPECT_NEAR(result.duration, 11.0, 1.5);
    }
}

// s7/s8 native receipts: a whole-route non-final grant ends with the vehicle standing at the
// route end while the tracker's terminal rotation creep (|w| = 1.5 |e|, |e| a degree or two)
// sits below the chassis actuation floor. The command never reads zero, so the whole-route
// handover could not fire and the executor's own stall watchdog ended the session ("allowed
// to move but did not for 10 s"). A completed parking leg opens a new request generation.
TEST(ResetExecutor, ACompletedParkingMoveHandsOverInsteadOfStallingOnItsOwnCreep) {
    const rc::Pose start{-5.574, -2.064, 0.616};
    const rc::Pose park{-7.174, -2.064, 0.616};
    const rc::Pose target{-6.0, -1.2, -0.393};
    PlantParams plant;
    plant.ell = 0.0;
    plant.ang_deadband = 0.04;
    SoloRig rig(scoutConfig(), start, plant);
    rig.exec().begin(1, target, true, rig.measurement());
    double next = 0.0;
    bool handed_over = false;
    const bool ended = rig.run(60.0, [&](SoloRig& r, const rc::StepOutput&) {
        if (r.time() + 1e-12 >= next) {
            next = r.time() + 0.4;
            r.deliver(goGrant(1, 1, start, {}, park, plant.ell, 1e9, false), r.measurement());
        }
        handed_over = handed_over || r.exec().requestGeneration() > 1;
    });
    EXPECT_TRUE(handed_over)
        << "a finished parking leg must open a new request generation, not wait on its own creep";
    EXPECT_FALSE(ended) << "it must not die on its own terminal creep: "
                        << rig.exec().result().detail;
}

TEST(ResetExecutor, LeavingTheRouteTubeBrakesAndReportsDeviation) {
    const Course c;
    PlantParams plant;
    SoloRig rig(scoutConfig(), c.start, plant);
    rig.exec().begin(1, c.goal, true, rig.measurement());
    bool pushed = false;
    double next = 0.0;
    const bool ended = rig.run(30.0, [&](SoloRig& r, const rc::StepOutput&) {
        if (r.time() + 1e-12 >= next) {
            next = r.time() + 0.4;
            r.deliver(goGrant(1, 1, c.start, c.via, c.goal, plant.ell, 1e9), r.measurement());
        }
        if (!pushed && r.time() > 3.0) {
            pushed = true;
            r.push(0.0, 1.0);  // dragged a metre sideways
        }
    });
    ASSERT_TRUE(ended);
    EXPECT_EQ(rig.exec().result().outcome, rc::Outcome::AbortedDeviation);
}

TEST(ResetExecutor, EveryFailsafeEndsTheSessionWithItsOwnTypedResult) {
    const Course c;
    PlantParams plant;
    auto fresh = [&](SoloRig& rig) { rig.exec().begin(1, c.goal, true, rig.measurement()); };
    {  // no coordinator: 2 s
        SoloRig rig(scoutConfig(), c.start, plant);
        fresh(rig);
        const bool ended = rig.run(5.0);
        ASSERT_TRUE(ended);
        EXPECT_EQ(rig.exec().result().outcome, rc::Outcome::RejectedNoCoordinator);
        EXPECT_NEAR(rig.exec().result().duration, 2.0, 0.1);
    }
    {  // no target: held 2 s, typed
        SoloRig rig(scoutConfig(), c.start, plant);
        rig.exec().begin(1, rc::Pose{}, false, rig.measurement());
        const bool ended = rig.run(5.0);
        ASSERT_TRUE(ended);
        EXPECT_EQ(rig.exec().result().outcome, rc::Outcome::RejectedNoTarget);
        EXPECT_NEAR(rig.exec().result().duration, 2.0, 0.1);
    }
    {  // the heartbeat lapses while moving: the vehicle brakes from a second after the last message
       // and the session ends typed once the coordinator has been silent for 2 s (exact zero
       // command)
        SoloRig rig(scoutConfig(), c.start, plant);
        fresh(rig);
        double last_message = -1.0;
        double speed_at_lapse = 0.0;
        const bool ended = rig.run(30.0, [&](SoloRig& r, const rc::StepOutput&) {
            if (r.time() < 4.0 && (last_message < 0.0 || r.time() - last_message >= 0.4)) {
                last_message = r.time();
                r.deliver(goGrant(1, 1, c.start, c.via, c.goal, plant.ell, 1e9), r.measurement());
            }
            if (last_message > 0.0 && r.time() < last_message + 1.0 &&
                r.time() > last_message + 0.9) {
                speed_at_lapse = r.speed();
            }
        });
        ASSERT_TRUE(ended);
        EXPECT_EQ(rig.exec().result().outcome, rc::Outcome::AbortedCoordinatorLost);
        EXPECT_NEAR(rig.time() - last_message, 2.0, 0.15)
            << "the typed result after 2 s of silence";
        EXPECT_GT(speed_at_lapse, 0.25) << "it was driving at the lapse";
        EXPECT_LT(rig.speed(), 0.6 * speed_at_lapse) << "and it had been braking for a second";
    }
    {  // a rejection of the planner is an infeasibility with its reason; a bad session is not
        SoloRig rig(scoutConfig(), c.start, plant);
        fresh(rig);
        rc::Grant g;
        g.session = 1;
        g.state = rc::GrantState::Reject;
        g.reason = rc::Reason::GoalOccupied;
        g.detail = "target occupied by ugv3";
        g.blockers = {"ugv3"};
        g.request_generation = rig.exec().requestGeneration();
        g.grant_id = 1;
        g.publication_seq = 0;
        rig.exec().onGrant(g, rig.measurement());
        const rc::StepOutput out = rig.exec().step(rig.measurement());
        EXPECT_TRUE(out.ended) << "the controller hears of a rejection at its next step";
        rc::Result result;
        ASSERT_TRUE(rig.exec().takeResult(&result));
        EXPECT_EQ(result.outcome, rc::Outcome::Infeasible);
        EXPECT_EQ(result.reason, rc::Reason::GoalOccupied);
        EXPECT_EQ(result.detail, "target occupied by ugv3");
        SoloRig other(scoutConfig(), c.start, plant);
        fresh(other);
        g.reason = rc::Reason::BadSession;
        other.deliver(g, other.measurement());
        rc::Result bad;
        ASSERT_TRUE(other.exec().takeResult(&bad));
        EXPECT_EQ(bad.outcome, rc::Outcome::RejectedByCoordinator);
    }
    {  // Stop and a health loss end it from the outside
        SoloRig rig(scoutConfig(), c.start, plant);
        fresh(rig);
        rig.deliver(goGrant(1, 1, c.start, c.via, c.goal, plant.ell, 1e9), rig.measurement());
        rig.exec().abort(rc::Outcome::Stopped, "stop", rig.measurement());
        rc::Result result;
        ASSERT_TRUE(rig.exec().takeResult(&result));
        EXPECT_EQ(result.outcome, rc::Outcome::Stopped);
        rig.exec().abort(rc::Outcome::AbortedUnhealthy, "late", rig.measurement());
        EXPECT_FALSE(rig.exec().takeResult(&result)) << "one result per session";
    }
    {  // an unhealthy vehicle stands still and does not end by itself
        SoloRig rig(scoutConfig(), c.start, plant);
        fresh(rig);
        rig.deliver(goGrant(1, 1, c.start, c.via, c.goal, plant.ell, 1e9), rig.measurement());
        rig.setUnhealthy(true);
        const rc::StepOutput out = rig.exec().step(rig.measurement(false));
        EXPECT_TRUE(out.command.zero());
        EXPECT_FALSE(out.ended);
    }
}

// The 90 s deadline is the vehicle's own, on the wall clock and on the ROS clock: a paused
// simulation clock and a stalled wall clock each end the session.
TEST(ResetExecutor, TheDeadlineRunsOnTheWallClockAndOnTheRosClock) {
    const Course c;
    for (int clock = 0; clock < 2; ++clock) {
        rc::ResetExecutor ex(scoutConfig());
        rc::Measurement m;
        m.ros_now = 10.0;
        m.wall_now = 100.0;
        m.pose = c.start;
        m.pose_stamp = 10.0;
        m.healthy = true;
        ex.begin(1, c.goal, true, m);
        rc::Grant hold;
        hold.session = 1;
        hold.request_generation = ex.requestGeneration();
        hold.grant_id = 1;
        hold.state = rc::GrantState::Brake;
        hold.publication_seq = 0;
        ex.onGrant(hold, m);  // a real pending stop barrier; no motion authority
        EXPECT_FALSE(ex.step(m).ended);
        // One clock advances 89.9 s, the other not at all: still going (heartbeat renewed).
        m.ros_now = clock == 0 ? 99.9 : 10.0;
        m.wall_now = clock == 0 ? 100.0 : 189.9;
        ++hold.publication_seq;
        ex.onGrant(hold, m);
        EXPECT_FALSE(ex.step(m).ended) << "clock " << clock;
        m.ros_now = clock == 0 ? 100.1 : 10.0;
        m.wall_now = clock == 0 ? 100.0 : 190.1;
        ++hold.publication_seq;
        ex.onGrant(hold, m);
        const rc::StepOutput out = ex.step(m);
        ASSERT_TRUE(out.ended) << "clock " << clock;
        rc::Result result;
        ASSERT_TRUE(ex.takeResult(&result));
        EXPECT_EQ(result.outcome, rc::Outcome::Timeout) << "clock " << clock;
        EXPECT_TRUE(out.command.zero());
    }
}

TEST(ResetExecutor, ARouteIsOnlyChangedFromStandstillAndAStaleGrantIsIgnored) {
    const Course c;
    PlantParams plant;
    SoloRig rig(scoutConfig(), c.start, plant);
    rig.exec().begin(1, c.goal, true, rig.measurement());
    const std::vector<rc::Point> other_via{{-3.0, 1.0}, {-1.0, 1.0}};
    bool changed_while_moving = false;
    bool checked = false;
    double next = 0.0;
    rig.run(8.0, [&](SoloRig& r, const rc::StepOutput&) {
        if (r.time() + 1e-12 >= next) {
            next = r.time() + 0.4;
            r.deliver(goGrant(1, 1, c.start, c.via, c.goal, plant.ell, 1e9), r.measurement());
            // A different route and a grant of another session arrive while the vehicle drives.
            if (r.time() > 3.0 && r.speed() > 0.1) {
                r.deliver(goGrant(1, 2, c.start, other_via, c.goal, plant.ell, 1e9),
                          r.measurement());
                r.deliver(goGrant(2, 3, c.start, other_via, c.goal, plant.ell, 1e9),
                          r.measurement());
                checked = true;
                changed_while_moving = r.exec().tracker().route().vertices().size() != 3;
            }
        }
    });
    EXPECT_TRUE(checked);
    EXPECT_FALSE(changed_while_moving);
    // Brake: the route is gone, the vehicle stands, and a new route is then accepted.
    rc::Grant brake;
    brake.session = 1;
    brake.state = rc::GrantState::Brake;
    brake.reason = rc::Reason::Replan;
    rig.deliver(brake, rig.measurement());
    EXPECT_TRUE(rig.exec().step(rig.measurement()).command.zero());
    double hold_next = 0.0;
    rig.run(3.0, [&](SoloRig& r, const rc::StepOutput&) {
        if (r.time() + 1e-12 >= hold_next) {
            hold_next = r.time() + 0.4;
            rc::Grant hold;
            hold.session = 1;
            hold.state = rc::GrantState::Hold;
            r.deliver(hold, r.measurement());
        }
    });
    EXPECT_LT(rig.speed(), 0.02);
    rc::Pose now = rig.pose();
    ASSERT_TRUE(rig.exec().active()) << rig.exec().result().detail;
    const auto stopped_measurement = rig.measurement();
    rig.exec().observe(
        stopped_measurement);  // actual controller observes before installing Go in its step
    rig.deliver(goGrant(1, 4, now, other_via, c.goal, plant.ell, 1e9), stopped_measurement);
    rig.exec().step(stopped_measurement);
    EXPECT_EQ(rig.exec().tracker().route().vertices().size(), 4U);
}

TEST(ResetExecutor, ALapsedHeartbeatStopsTheVehicleAndAShortOutageIsResumedFrom) {
    const Course c;
    PlantParams plant;
    struct Outcome {
        rc::Outcome outcome{rc::Outcome::None};
        double min_speed_during_gap{1e9};
        bool arrived{false};
    };
    auto run = [&](int lost_in_a_row) {
        SoloRig rig(scoutConfig(), c.start, plant);
        rig.exec().begin(1, c.goal, true, rig.measurement());
        double next = 0.0;
        int count = 0;
        double gap_from = -1.0, gap_to = -1.0;
        Outcome result;
        const bool ended = rig.run(60.0, [&](SoloRig& r, const rc::StepOutput&) {
            if (r.time() + 1e-12 >= next) {
                next = r.time() + 0.4;
                ++count;
                // Messages 10..(10+lost-1) are lost.
                if (count >= 10 && count < 10 + lost_in_a_row) {
                    if (gap_from < 0.0) {
                        gap_from = r.time();
                    }
                    return;
                }
                if (count == 10 + lost_in_a_row) {
                    gap_to = r.time();
                }
                r.deliver(goGrant(1, 1, c.start, c.via, c.goal, plant.ell, 1e9), r.measurement());
            }
            if (gap_from >= 0.0 && r.time() > gap_from + 1.4 &&
                (gap_to < 0.0 || r.time() < gap_to)) {
                result.min_speed_during_gap = std::min(result.min_speed_during_gap, r.speed());
            }
        });
        if (ended) {
            result.outcome = rig.exec().result().outcome;
        }
        return result;
    };
    EXPECT_EQ(run(0).outcome, rc::Outcome::Arrived);
    EXPECT_EQ(run(1).outcome, rc::Outcome::Arrived) << "0.8 s between receipts";
    const Outcome two = run(2);  // 1.2 s between receipts: stopped for a moment, then on
    EXPECT_EQ(two.outcome, rc::Outcome::Arrived);
    const Outcome four = run(4);  // 2.0 s of silence: ended, typed
    EXPECT_EQ(four.outcome, rc::Outcome::AbortedCoordinatorLost);
}

TEST(ResetExecutor, TheRotationCentreOffsetIsLearnedFromTurnsAndDoesNotFollowPoseNoise) {
    for (double ell : {-0.229, 0.0, 0.18}) {
        for (double noise : {0.0, 0.002}) {
            PlantParams plant;
            plant.ell = ell;
            plant.pos_noise = noise;
            plant.yaw_noise = noise / 0.3;
            SoloRig rig(scoutConfig(), {0.0, 0.0, 0.0}, plant, 3);
            EXPECT_LT(rig.exec().ell().spread(), 0.0) << "unknown at the start";
            // Standing still with noisy poses teaches nothing.
            rig.run(5.0);
            EXPECT_EQ(rig.exec().ell().ell(), 0.0);
            EXPECT_LT(rig.exec().ell().spread(), 0.0);
            // Turns in place, one each way, outside any Reset session.
            rig.setManualCommand({0.0, 0.0, 0.4});
            rig.run(6.0);
            rig.setManualCommand({0.0, 0.0, -0.4});
            rig.run(6.0);
            rig.setManualCommand({});
            rig.run(1.0);
            const double tolerance = noise > 0.0 ? 0.06 : 0.03;
            EXPECT_NEAR(rig.exec().ell().ell(), ell, tolerance) << "noise " << noise;
            EXPECT_GE(rig.exec().ell().spread(), 0.02) << "known after a few turns";
            EXPECT_LE(rig.exec().ell().spread(), 0.25);
        }
    }
}

TEST(ResetExecutor, TheSessionMessageDeclaresTheVehicleAndIsDueOnChangeAndAtFiveHertz) {
    const Course c;
    PlantParams plant;
    SoloRig rig(scoutConfig(), c.start, plant);
    rc::Measurement m = rig.measurement();
    EXPECT_TRUE(rig.exec().sessionDue(m)) << "the first one";
    rig.exec().markSessionPublished(m);
    EXPECT_FALSE(rig.exec().sessionDue(m));
    m.wall_now += 0.19;
    EXPECT_FALSE(rig.exec().sessionDue(m));
    m.wall_now += 0.02;
    EXPECT_TRUE(rig.exec().sessionDue(m)) << "every 0.2 s";
    rig.exec().markSessionPublished(m);
    rig.exec().begin(7, c.goal, true, m);
    EXPECT_TRUE(rig.exec().sessionDue(m)) << "a new session at once";
    const rc::Session s = rig.exec().makeSession(m);
    EXPECT_EQ(s.session, 7U);
    EXPECT_EQ(s.phase, rc::Phase::Requesting);
    EXPECT_EQ(s.target.x, c.goal.x);
    EXPECT_EQ(s.spec.footprint.half_length, 0.45);
    EXPECT_EQ(s.spec.model, rc::Model::Unicycle);
    EXPECT_TRUE(s.healthy);
    rig.exec().finish();
    EXPECT_EQ(rig.exec().makeSession(m).phase, rc::Phase::Idle);
}

}  // namespace
}  // namespace reset_test
