#include <gtest/gtest.h>
#include <ugv_reset_safety/scene_projection.h>

#include <algorithm>
#include <limits>
namespace {
xgc2_geometry_msgs::SceneSnapshot fixture(const std::string& type) {
    xgc2_geometry_msgs::SceneSnapshot scene;
    scene.epoch = "test";
    scene.header.frame_id = "world";
    xgc2_geometry_msgs::SceneObstacle obstacle;
    obstacle.id = "gate";
    obstacle.motion_type = "hold";
    obstacle.pose.orientation.w = 1;
    xgc2_geometry_msgs::ScenePart part;
    part.id = "left";
    part.pose.orientation.w = 1;
    using xgc2_math::geometry::Geometry;
    Geometry geometry;
    if (type == "box") {
        geometry = Geometry::box({.5, .5, 2});
    } else if (type == "sphere") {
        geometry = Geometry::sphere(.5);
    } else if (type == "cylinder") {
        geometry = Geometry::cylinder(.5, 1);
    } else if (type == "capsule") {
        geometry = Geometry::capsule(.5, 1);
    } else {
        throw std::invalid_argument("invalid fixture type");
    }
    const auto payload = xgc2_math::geometry::encodeGeometry(geometry);
    xgc2_geometry_msgs::SceneGeometry definition;
    definition.type = payload.type;
    definition.parameters = payload.parameters;
    scene.geometries.push_back(std::move(definition));
    part.geometry_index = 0;
    obstacle.parts.push_back(part);
    scene.obstacles.push_back(obstacle);
    return scene;
}
xgc2_geometry_msgs::SceneState liveState(const xgc2_geometry_msgs::SceneSnapshot& scene,
                                         double x = 0, double vx = 0, double omega = 0) {
    xgc2_geometry_msgs::SceneState state;
    state.header.frame_id = scene.header.frame_id;
    state.epoch = scene.epoch;
    state.revision = scene.revision;
    for (const auto& obstacle : scene.obstacles) {
        if (!obstacle.dynamic) {
            continue;
        }
        xgc2_geometry_msgs::SceneObstacleState body;
        body.id = obstacle.id;
        body.pose = obstacle.pose;
        body.pose.position.x += x;
        body.twist.linear.x = vx;
        body.twist.angular.z = omega;
        state.obstacles.push_back(body);
    }
    return state;
}
TEST(SceneProjection, CurvesAreCircumscribedInsteadOfInscribed) {
    for (const auto& type : {"sphere", "cylinder", "capsule"}) {
        auto scene = fixture(type);
        auto projected = ugv_reset_safety::scene_projection::project(scene, "world");
        ASSERT_EQ(projected.size(), 1U);
        for (const auto& vertex : projected[0].vertices) {
            EXPECT_GE(vertex.norm(), .5 - 1e-12);
            EXPECT_LE(vertex.norm(), .5 / std::cos(M_PI / 32) + 1e-10);
        }
    }
}
TEST(SceneProjection, KeepsCompoundOpeningAndStableIdentities) {
    auto scene = fixture("box");
    scene.obstacles[0].parts[0].pose.position.x = -1;
    auto right = scene.obstacles[0].parts[0];
    right.id = "right";
    right.pose.position.x = 1;
    scene.obstacles[0].parts.push_back(right);
    const auto parts = ugv_reset_safety::scene_projection::project(scene, "world");
    ASSERT_EQ(parts.size(), 2U);
    for (const auto& vertex : parts[0].vertices) {
        EXPECT_LT(vertex.x(), -.7);
    }
    for (const auto& vertex : parts[1].vertices) {
        EXPECT_GT(vertex.x(), .7);
    }
    scene.obstacles[0].parts.erase(scene.obstacles[0].parts.begin());
    const auto remaining = ugv_reset_safety::scene_projection::project(scene, "world");
    EXPECT_EQ(remaining[0].id, parts[1].id);
}
TEST(SceneProjection, RejectsUnknownGeometryNonfiniteDataAndFrameMismatch) {
    auto scene = fixture("sphere");
    scene.geometries[0].type = 0xffff;
    EXPECT_THROW(ugv_reset_safety::scene_projection::project(scene, "world"),
                 std::invalid_argument);
    scene = fixture("sphere");
    scene.obstacles[0].parts[0].geometry_index = scene.geometries.size();
    EXPECT_THROW(ugv_reset_safety::scene_projection::project(scene, "world"),
                 std::invalid_argument);
    scene = fixture("sphere");
    scene.obstacles[0].pose.position.x = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(ugv_reset_safety::scene_projection::project(scene, "world"),
                 std::invalid_argument);
    scene = fixture("sphere");
    scene.header.frame_id = "map";
    EXPECT_THROW(ugv_reset_safety::scene_projection::project(scene, "world"),
                 std::invalid_argument);
    scene = fixture("sphere");
    scene.obstacles[0].id.clear();
    EXPECT_THROW(ugv_reset_safety::scene_projection::project(scene, "world"),
                 std::invalid_argument);
    scene = fixture("sphere");
    scene.obstacles.clear();
    EXPECT_TRUE(ugv_reset_safety::scene_projection::project(scene, "world").empty());
}
TEST(SceneProjection, LiveStateMovesDynamicGeometryWithoutUsingRestPose) {
    auto scene = fixture("box");
    scene.obstacles[0].dynamic = true;
    scene.obstacles[0].motion_type = "constant_twist";
    EXPECT_THROW(ugv_reset_safety::scene_projection::project(scene, "world"),
                 std::invalid_argument);
    auto state = liveState(scene, 1.5, 0.4);
    const auto geometry = ugv_reset_safety::scene_projection::live(scene, state, "world");
    ASSERT_EQ(geometry.live.size(), 1U);
    double xmin = std::numeric_limits<double>::infinity();
    for (const auto& vertex : geometry.live[0].vertices) {
        xmin = std::min(xmin, vertex.x());
    }
    EXPECT_GT(xmin, 1.0);
    EXPECT_NEAR(geometry.live[0].velocity.x(), 0.4, 1e-12);
    EXPECT_GT(geometry.occupancy[0].vertices.size(), 2U);
    double occupancy_span = 0;
    for (const auto& vertex : geometry.occupancy[0].vertices) {
        occupancy_span = std::max(occupancy_span, (vertex - geometry.live[0].origin).norm());
    }
    double live_span = 0;
    for (const auto& vertex : geometry.live[0].vertices) {
        live_span = std::max(live_span, (vertex - geometry.live[0].origin).norm());
    }
    EXPECT_GT(occupancy_span, live_span + 0.5);
}
TEST(SceneProjection, PausedDynamicUsesLivePoseWithZeroTwist) {
    auto scene = fixture("cylinder");
    scene.obstacles[0].dynamic = true;
    scene.obstacles[0].motion_type = "ping_pong";
    auto state = liveState(scene, 2.0, 0.0);
    const auto geometry = ugv_reset_safety::scene_projection::live(scene, state, "world");
    ASSERT_EQ(geometry.live.size(), 1U);
    EXPECT_NEAR(geometry.live[0].origin.x(), 2.0, 1e-9);
    EXPECT_EQ(geometry.live[0].velocity, Eigen::Vector2d::Zero());
    EXPECT_NEAR(geometry.occupancy[0].vertices.front().x(), geometry.live[0].vertices.front().x(),
                1e-8);
}
TEST(SceneProjection, RejectsOldEpochMissingIdentitiesAndUnknownMotion) {
    auto scene = fixture("box");
    scene.obstacles[0].dynamic = true;
    scene.obstacles[0].motion_type = "circle";
    auto state = liveState(scene, 0.2, 0.1);
    state.epoch = "old";
    EXPECT_THROW(ugv_reset_safety::scene_projection::live(scene, state, "world"),
                 std::invalid_argument);
    state = liveState(scene, 0.2, 0.1);
    state.obstacles.clear();
    EXPECT_THROW(ugv_reset_safety::scene_projection::live(scene, state, "world"),
                 std::invalid_argument);
    scene.obstacles[0].motion_type = "spiral";
    EXPECT_THROW(ugv_reset_safety::scene_projection::project(scene, "world"),
                 std::invalid_argument);
}
TEST(SceneProjection, StaticDefinitionNeedsNoStaticStateAndPreparationIsReused) {
    namespace projection = ugv_reset_safety::scene_projection;
    auto scene = fixture("box");
    scene.obstacles[0].pose.position.x = -2;
    auto moving = scene.obstacles[0];
    moving.id = "moving";
    moving.dynamic = true;
    moving.motion_type = "constant_twist";
    moving.pose.position.x = 2;
    scene.obstacles.push_back(moving);
    xgc2_math::geometry::PreparedGeometryCache cache;
    const auto prepared = projection::prepare(scene, "world", cache);
    ASSERT_EQ(cache.size(), 1U);
    const auto shape = prepared.obstacles[0].parts[0].geometry;
    EXPECT_EQ(shape, prepared.obstacles[1].parts[0].geometry);
    auto state = liveState(scene, 1, .2);
    ASSERT_EQ(state.obstacles.size(), 1U);
    for (int i = 0; i < 5; ++i) {
        state.obstacles[0].pose.position.x = 3 + i;
        const auto current = projection::live(prepared, state, "world");
        ASSERT_EQ(current.live.size(), 2U);
        EXPECT_EQ(cache.size(), 1U);
        EXPECT_EQ(prepared.obstacles[0].parts[0].geometry, shape);
        const auto fixed = std::find_if(
            current.live.begin(), current.live.end(),
            [](const auto& value) { return value.id.find("gate") != std::string::npos; });
        ASSERT_NE(fixed, current.live.end());
        for (const auto& v : fixed->vertices) {
            EXPECT_LT(v.x(), -1.7);
        }
    }
    auto held = fixture("sphere");
    const auto held_prepared = projection::prepare(held, "world", cache);
    const auto heartbeat = liveState(held);
    EXPECT_TRUE(heartbeat.obstacles.empty());
    EXPECT_EQ(projection::live(held_prepared, heartbeat, "world").live.size(), 1U);
}
TEST(SceneProjection, RingProjectsEachConvexPartAndRetainsOpening) {
    namespace projection = ugv_reset_safety::scene_projection;
    auto scene = fixture("box");
    const auto payload =
        xgc2_math::geometry::encodeGeometry(xgc2_math::geometry::Geometry::ring({2, 1}, .1));
    auto& geometry = scene.geometries[0];
    geometry.type = payload.type;
    geometry.parameters = payload.parameters;
    xgc2_math::geometry::PreparedGeometryCache cache;
    const auto prepared = projection::prepare(scene, "world", cache);
    const auto parts = projection::project(prepared, "world");
    ASSERT_GT(parts.size(), 4U);
    std::set<std::string> ids;
    for (const auto& shape : parts) {
        EXPECT_TRUE(ids.insert(shape.id).second);
        // Every small convex part has a separating edge from the ring's free center.
        bool positive = false, negative = false;
        for (std::size_t i = 0; i < shape.vertices.size(); ++i) {
            const auto& a = shape.vertices[i];
            const auto& b = shape.vertices[(i + 1) % shape.vertices.size()];
            const double cross = (b.x() - a.x()) * (-a.y()) - (b.y() - a.y()) * (-a.x());
            positive = positive || cross > 1e-12;
            negative = negative || cross < -1e-12;
        }
        EXPECT_TRUE(positive && negative) << shape.id;
    }
    EXPECT_EQ(prepared.obstacles[0].parts[0].geometry->original, payload);
}
}  // namespace
