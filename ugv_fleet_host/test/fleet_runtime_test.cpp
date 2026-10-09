#include <gtest/gtest.h>
#include <ugv_fleet_host/fleet_runtime.h>
using namespace ugv_fleet_host;
using namespace ugv_reset_safety;
namespace {
struct Edge final : FleetEdge {
    ResetSession session;
    NativeClock clock{{10'000'000'000}, {1}};
    bool in_reset = false, stopping = false, zero = true;
    int publishes = 0, turns = 0;
    void updateOnce() override {
        ++turns;
        if (stopping) {
            session.cancel();
            in_reset = false;
            zero = true;
            stopping = false;
        } else if (in_reset) {
            const auto f = session.feedback(clock.world.toNSec(), clock.monotonic.seconds);
            zero = !f.valid || f.status != ResetSession::RUNNING ||
                   (f.command.x == 0 && f.command.y == 0 && f.command.yaw == 0);
            if (f.valid && f.status != ResetSession::RUNNING) {
                session.complete(f.status == ResetSession::ARRIVED
                                     ? ResetSession::Completion::Arrived
                                     : ResetSession::Completion::Rejected);
                session.cancel();
                in_reset = false;
            }
        }
        // Model the unchanged sole publisher's actual acknowledgment, including zero.
        session.noteApplied(zero ? ResetSession::Command{} : ResetSession::Command{.1, 0, 0},
                            clock.world.toNSec());
        ++publishes;
    }
    double controlRate() const override {
        return 500;
    }
    ResetSession& resetSession() override {
        return session;
    }
    scene_model::PoseSample poseSample() const override {
        scene_model::PoseSample p;
        p.header.stamp = clock.world;
        return p;
    }
    std::string controlState() const override {
        return in_reset ? "Reset" : "Ready";
    }
    bool healthReady() const override {
        return true;
    }
    bool reset() override {
        session.begin({});
        in_reset = true;
        return true;
    }
    bool stop() override {
        stopping = true;
        return true;
    }
    bool admissionRejected() const override {
        return false;
    }
};
struct Fixture {
    Edge* edge = nullptr;
    NativeClock clock{{10'000'000'000}, {1}};
    std::unique_ptr<FleetRuntime> runtime;
    Fixture() {
        std::vector<std::unique_ptr<FleetEdge>> edges;
        auto native = std::make_unique<Edge>();
        edge = native.get();
        edges.push_back(std::move(native));
        Robot robot;
        robot.id = "ugv0";
        robot.half_length = .45;
        robot.half_width = .4;
        robot.limits.max_vx = .35;
        robot.limits.max_omega = .5;
        robot.limits.accel_vx = .35;
        robot.limits.accel_omega = .6;
        robot.lateral_velocity_per_yaw_bound = .25;
        FleetCoordinator::Configuration config;
        config.fence = {true, -5, 5, -5, 5};
        config.obstacle_avoidance = false;
        runtime = std::make_unique<FleetRuntime>(std::move(edges), std::vector<Robot>{robot},
                                                 config, clock);
        scene_model::Snapshot d;
        d.epoch = "world";
        d.revision = 3;
        scene_model::State s;
        s.epoch = d.epoch;
        s.revision = d.revision;
        s.header.stamp = clock.world;
        s.simulation_time.epoch = 7;
        runtime->scene(d, s, clock);
    }
    ResetControl::SceneFence fence() {
        ResetControl::SceneFence f;
        std::copy_n("world", 5, f.epoch.begin());
        f.revision = 3;
        f.simulation_time_epoch = 7;
        return f;
    }
    void turn() {
        clock.world.nanoseconds += 2'000'000;
        clock.monotonic.seconds += .002;
        edge->clock = clock;
        runtime->turn(clock);
    }
};
}  // namespace
TEST(FleetRuntime, FiftyHertzHandoffPreservesExactStampAndActualTerminalZero) {
    Fixture f;
    std::array<bool, 128> robots{};
    robots[0] = true;
    ASSERT_TRUE(f.runtime->control().start(0, robots, f.clock.world.toNSec(),
                                           f.clock.monotonic.seconds, f.fence()));
    for (int n = 0; n < 150 && ResetControl::active(f.runtime->control().snapshot().state); ++n)
        f.turn();
    EXPECT_EQ(f.runtime->control().snapshot().state, ResetControl::State::Arrived);
    EXPECT_TRUE(f.edge->zero);
    EXPECT_GT(f.edge->publishes, 0);
    EXPECT_FALSE(f.edge->in_reset);
    EXPECT_EQ(f.runtime->control().snapshot().robots[0].generation, f.edge->session.generation());
}
TEST(FleetRuntime, SceneEpochChangeCancelsNativeMotionAndWaitsForSolePublisherZero) {
    Fixture f;
    std::array<bool, 128> robots{};
    robots[0] = true;
    ASSERT_TRUE(f.runtime->control().start(0, robots, f.clock.world.toNSec(),
                                           f.clock.monotonic.seconds, f.fence()));
    f.turn();
    scene_model::Snapshot d;
    d.epoch = "world";
    d.revision = 3;
    scene_model::State s;
    s.epoch = d.epoch;
    s.revision = d.revision;
    s.header.stamp = {f.clock.world.nanoseconds + 1};
    s.simulation_time.epoch = 8;
    f.runtime->scene(d, s, f.clock);
    EXPECT_EQ(f.runtime->control().snapshot().state, ResetControl::State::Cancelling);
    for (int n = 0; n < 10 && ResetControl::active(f.runtime->control().snapshot().state); ++n)
        f.turn();
    EXPECT_EQ(f.runtime->control().snapshot().state, ResetControl::State::Rejected);
    EXPECT_TRUE(f.edge->zero);
    EXPECT_FALSE(f.edge->in_reset);
}
