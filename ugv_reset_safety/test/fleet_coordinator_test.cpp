#include <gtest/gtest.h>
#include <ugv_reset_safety/fleet_coordinator.h>
#include <ugv_reset_safety/scene_projection.h>

#include <cmath>

using namespace ugv_reset_safety;
namespace {
struct Fixture {
    NativeClock clock{{10'000'000'000ULL}, {1}};
    FleetCoordinator::Configuration config;
    std::vector<Robot> robots;
    std::unique_ptr<FleetCoordinator> coordinator;
    explicit Fixture(bool avoid = false, std::size_t count = 1) {
        config.obstacle_avoidance = avoid;
        config.fence = {true, -5, 5, -5, 5};
        for (std::size_t i = 0; i < count; ++i) {
            Robot robot;
            robot.id = "ugv" + std::to_string(i);
            robot.type = i % 2 ? RobotType::Mecanum : RobotType::Unicycle;
            robot.half_length = .45;
            robot.half_width = .4;
            robot.limits.max_vx = .35;
            robot.limits.max_vy = i % 2 ? .35 : 0;
            robot.limits.max_omega = .5;
            robot.limits.accel_vx = .35;
            robot.limits.accel_vy = .35;
            robot.limits.accel_omega = .6;
            robot.lateral_velocity_per_yaw_bound = .25;
            robots.push_back(robot);
        }
        coordinator = std::make_unique<FleetCoordinator>(robots, config, clock);
        for (std::size_t i = 0; i < count; ++i)
            sample(i, {static_cast<double>(i) * 2, 0, 0});
    }
    void advance() {
        clock.world.nanoseconds += 20'000'000;
        clock.monotonic.seconds += .02;
    }
    void sample(std::size_t i, scene_model::Pose2 pose, std::string state = "Reset") {
        scene_model::PoseSample sample;
        sample.header.stamp = clock.world;
        sample.pose.position = {pose.x, pose.y, 0};
        sample.pose.orientation = {0, 0, std::sin(pose.theta / 2), std::cos(pose.theta / 2)};
        coordinator->pose(i, sample, clock);
        coordinator->state(i, std::move(state), clock);
    }
    void request(std::size_t i, scene_model::Pose2 pose, scene_model::Pose2 target,
                 double applied = .0, uint32_t generation = 1) {
        FleetCoordinator::Request request;
        request.header.stamp = clock.world;
        request.pose_stamp = clock.world;
        request.applied_stamp = clock.world;
        request.generation = generation;
        request.pose = pose;
        request.target = target;
        request.applied_command.linear.x = applied;
        coordinator->request(i, request, clock);
    }
    void run(std::size_t cycles, scene_model::Pose2 pose, scene_model::Pose2 target,
             double applied = 0) {
        for (std::size_t i = 0; i < cycles; ++i) {
            advance();
            sample(0, pose);
            request(0, pose, target, applied);
            coordinator->tick(clock);
        }
    }
};
}  // namespace

TEST(FleetCoordinator, OriginalGoalMeasuredStopAndActuallyPublishedZeroAreRequired) {
    Fixture fixture;
    fixture.run(10, {}, {});
    const auto arrived = fixture.coordinator->proposals()[0];
    ASSERT_TRUE(arrived.valid);
    EXPECT_EQ(arrived.status, ResetSession::ARRIVED);
    EXPECT_TRUE(arrived.command.isZero(0));
    EXPECT_EQ(arrived.stamp.toNSec(), fixture.clock.world.toNSec());
    EXPECT_EQ(arrived.generation, 1U);
    Fixture moving;
    moving.run(10, {}, {}, .1);
    EXPECT_EQ(moving.coordinator->proposals()[0].status, ResetSession::RUNNING);
    EXPECT_FALSE(moving.coordinator->proposals()[0].command.isZero(0));
    moving.run(2, {}, {}, .0);
    EXPECT_EQ(moving.coordinator->proposals()[0].status, ResetSession::ARRIVED);
    Fixture position;
    position.run(10, {.06, 0, 0}, {});
    EXPECT_EQ(position.coordinator->proposals()[0].status, ResetSession::RUNNING);
    Fixture heading;
    heading.run(10, {0, 0, .10}, {});
    EXPECT_EQ(heading.coordinator->proposals()[0].status, ResetSession::RUNNING);
}

TEST(FleetCoordinator, MeasuredMotionAndNonparticipantMotionCannotBecomeArrival) {
    Fixture fixture;
    fixture.run(9, {}, {});
    fixture.advance();
    fixture.sample(0, {.001, 0, 0});
    fixture.request(0, {.001, 0, 0}, {});
    fixture.coordinator->tick(fixture.clock);
    EXPECT_EQ(fixture.coordinator->proposals()[0].status, ResetSession::RUNNING);
    Fixture fleet(false, 2);
    for (int i = 0; i < 10; ++i) {
        fleet.advance();
        fleet.sample(0, {});
        fleet.sample(1, {2 + .001 * i, 0, 0}, "Ready");
        fleet.request(0, {}, {});
        fleet.coordinator->tick(fleet.clock);
    }
    const auto denied = fleet.coordinator->proposals()[0];
    ASSERT_TRUE(denied.valid);
    EXPECT_EQ(denied.status, ResetSession::REJECTED);
    EXPECT_TRUE(denied.command.isZero(0));
    EXPECT_EQ(denied.reason, "uncontrolled moving fleet member");
}

TEST(FleetCoordinator, PausedOrRewoundWorldClockAndMissedTurnStopAnActiveBatch) {
    for (int scenario = 0; scenario < 3; ++scenario) {
        Fixture fixture;
        fixture.run(10, {}, {1, 0, 0});
        ASSERT_EQ(fixture.coordinator->proposals()[0].status, ResetSession::RUNNING);
        if (scenario == 0)
            fixture.clock.monotonic.seconds += .02;
        if (scenario == 1) {
            fixture.clock.world.nanoseconds -= 1;
            fixture.clock.monotonic.seconds += .02;
        }
        if (scenario == 2) {
            fixture.clock.world.nanoseconds += 160'000'000;
            fixture.clock.monotonic.seconds += .16;
        }
        fixture.sample(0, {});
        fixture.request(0, {}, {1, 0, 0});
        fixture.coordinator->tick(fixture.clock);
        const auto rejected = fixture.coordinator->proposals()[0];
        ASSERT_TRUE(rejected.valid);
        EXPECT_EQ(rejected.status, ResetSession::REJECTED);
        EXPECT_TRUE(rejected.command.isZero(0));
        EXPECT_EQ(rejected.reason, "coordinator deadline missed");
    }
}

TEST(FleetCoordinator, ChangingFrozenTargetRejectsTheGenerationAndCannotReuseItsMotion) {
    Fixture fixture;
    fixture.run(10, {}, {1, 0, 0});
    fixture.advance();
    fixture.sample(0, {});
    fixture.request(0, {}, {2, 0, 0});
    fixture.coordinator->tick(fixture.clock);
    ASSERT_EQ(fixture.coordinator->proposals()[0].status, ResetSession::REJECTED);
    EXPECT_EQ(fixture.coordinator->proposals()[0].reason, "target changed inside reset session");
    for (int i = 0; i < 10; ++i) {
        fixture.advance();
        fixture.sample(0, {});
        fixture.request(0, {}, {1, 0, 0}, 0, 2);
        fixture.coordinator->tick(fixture.clock);
    }
    EXPECT_EQ(fixture.coordinator->proposals()[0].generation, 2U);
    EXPECT_EQ(fixture.coordinator->proposals()[0].status, ResetSession::RUNNING);
}

TEST(FleetCoordinator, SceneDefinitionAndStateMustBeAppliedFreshAndExact) {
    Fixture fixture(true);
    scene_model::Snapshot definition;
    definition.epoch = "world-boot";
    definition.revision = 1;
    scene_model::State state;
    state.epoch = definition.epoch;
    state.revision = definition.revision;
    fixture.coordinator->scene(definition, fixture.clock);
    EXPECT_TRUE(fixture.coordinator->sceneStatus().applied);
    EXPECT_FALSE(fixture.coordinator->sceneStatus().operational);
    fixture.advance();
    state.header.stamp = fixture.clock.world;
    fixture.coordinator->sceneState(state, fixture.clock);
    EXPECT_TRUE(fixture.coordinator->sceneStatus().operational);
    for (int i = 0; i < 10; ++i) {
        fixture.advance();
        fixture.sample(0, {});
        fixture.request(0, {}, {1, 0, 0});
        state.header.stamp = fixture.clock.world;
        fixture.coordinator->sceneState(state, fixture.clock);
        fixture.coordinator->tick(fixture.clock);
    }
    EXPECT_EQ(fixture.coordinator->proposals()[0].status, ResetSession::RUNNING);
    ++definition.revision;
    fixture.coordinator->scene(definition, fixture.clock);
    fixture.advance();
    fixture.sample(0, {});
    fixture.request(0, {}, {1, 0, 0});
    fixture.coordinator->tick(fixture.clock);
    EXPECT_EQ(fixture.coordinator->proposals()[0].status, ResetSession::REJECTED);
    EXPECT_EQ(fixture.coordinator->proposals()[0].reason,
              "scene changed; request a new reset after stopping");
    EXPECT_FALSE(fixture.coordinator->sceneStatus().operational);
}

TEST(FleetCoordinator, ProjectionKeepsCompoundGeometryAndTwoSecondMotionOccupancy) {
    scene_model::Snapshot definition;
    definition.epoch = "world-boot";
    scene_model::Obstacle obstacle;
    obstacle.id = "gate";
    obstacle.motion_type = "constant_twist";
    obstacle.dynamic = true;
    for (double x : {-1.0, 1.0}) {
        scene_model::Part part;
        part.id = x < 0 ? "left" : "right";
        part.pose.position.x = x;
        part.geometry.type = "sphere";
        part.geometry.radius = .5;
        obstacle.parts.push_back(part);
    }
    definition.obstacles.push_back(obstacle);
    scene_model::State state;
    state.epoch = definition.epoch;
    scene_model::ObstacleState body;
    body.id = obstacle.id;
    body.twist.linear.x = .2;
    state.obstacles.push_back(body);
    const auto projected = scene_projection::live(definition, state, "world");
    ASSERT_EQ(projected.live.size(), 2U);
    ASSERT_EQ(projected.occupancy.size(), 2U);
    double live_max = -1e9, occupancy_max = -1e9;
    for (const auto& part : projected.live)
        for (const auto& vertex : part.vertices)
            live_max = std::max(live_max, vertex.x());
    for (const auto& part : projected.occupancy)
        for (const auto& vertex : part.vertices)
            occupancy_max = std::max(occupancy_max, vertex.x());
    EXPECT_NEAR(occupancy_max - live_max, .4, 1e-10);
    state.obstacles.push_back(body);
    EXPECT_THROW(scene_projection::live(definition, state, "world"), std::invalid_argument);
}
TEST(FleetCoordinator, ScientificClockEpochAndBothSceneFreshnessClocksFenceActiveMotion) {
    for (int scenario = 0; scenario < 3; ++scenario) {
        Fixture fixture(true);
        scene_model::Snapshot definition;
        definition.epoch = "same-authoring-epoch";
        definition.revision = 3;
        scene_model::State state;
        state.epoch = definition.epoch;
        state.revision = 3;
        state.simulation_time.epoch = 8;
        fixture.coordinator->scene(definition, fixture.clock);
        for (int i = 0; i < 10; ++i) {
            fixture.advance();
            fixture.sample(0, {});
            fixture.request(0, {}, {1, 0, 0});
            state.header.stamp = fixture.clock.world;
            state.simulation_time.nanoseconds = fixture.clock.world;
            fixture.coordinator->sceneState(state, fixture.clock);
            fixture.coordinator->tick(fixture.clock);
        }
        ASSERT_EQ(fixture.coordinator->proposals()[0].status, ResetSession::RUNNING);
        fixture.advance();
        if (scenario == 0) {
            state.simulation_time.epoch++;
            state.header.stamp = fixture.clock.world;
            fixture.coordinator->sceneState(state, fixture.clock);
        }
        if (scenario == 1)
            fixture.clock.monotonic.seconds += .51;
        if (scenario == 2)
            fixture.clock.world.nanoseconds += 510'000'000;
        fixture.sample(0, {});
        fixture.request(0, {}, {1, 0, 0});
        fixture.coordinator->tick(fixture.clock);
        const auto result = fixture.coordinator->proposals()[0];
        ASSERT_TRUE(result.valid);
        EXPECT_EQ(result.status, ResetSession::REJECTED);
        EXPECT_TRUE(result.command.isZero(0));
        if (scenario != 0)
            EXPECT_FALSE(fixture.coordinator->sceneStatus().operational);
    }
}
