// This separate test binary compiles the real planner implementation in this TU.
// Only access control is exposed; no product API or state transition is replaced.
#include <gtest/gtest.h>
#include <ugv_reset_client/reset_protocol.h>
#include <ugv_reset_client/reset_route.h>
#include <ugv_reset_safety/reset_geometry.h>

#include <Eigen/Core>
#include <map>
#include <memory>
#include <string>
#include <vector>
#define private public
#include <ugv_reset_safety/reset_planner.h>
#undef private
#include "../src/reset_planner.cpp"
namespace {
namespace rs = ugv_reset_safety;
namespace rc = ugv_reset_client;
using rs::Body;
using rs::SampleSet;
Body line(rc::Model model) {
    Body b;
    b.id = "reserved";
    b.spec.model = model;
    b.spec.footprint = {.001, .001, 0, 0};
    b.shape = rs::makeShape(b.spec);
    b.pose = {1, 0, 3.0};
    b.plan_active = true;
    b.go_received = true;
    rs::ResetPlanner::Impl m{rs::PlannerConfig{}};
    SampleSet ss;
    for (size_t i = 0; i <= 100; ++i) {
        auto s = m.poseSample(b, {.02 * i, 0, i == 100 ? 3.0 : 0.0});
        s.s = .02 * i;
        ss.pts.push_back(s);
    }
    for (size_t i = 0; i < ss.pts.size(); i += rs::kChunk) {
        rs::Aabb box;
        for (size_t j = i; j < std::min(ss.pts.size(), i + rs::kChunk); ++j) {
            box.add(ss.pts[j].box);
        }
        ss.chunks.push_back(box);
    }
    b.plan.sets = {ss};
    b.plan.cursor = {0};
    b.plan.hi = {ss.pts.size()};
    return b;
}
TEST(Cursor, OmniConcurrentYawCannotReleaseTranslationAheadOfTheNearestPosition) {
    auto b = line(rc::Model::Omni);
    rs::ResetPlanner::Impl::advanceCursor(b);
    EXPECT_EQ(b.plan.cursor[0], 50u);
}
TEST(Cursor, UnicycleKeepsTheOriginalYawMetricForRepeatedPositionTurns) {
    auto b = line(rc::Model::Unicycle);
    rs::ResetPlanner::Impl::advanceCursor(b);
    EXPECT_EQ(b.plan.cursor[0], 100u);
    for (auto& q : b.plan.sets[0].pts) {
        q.x = 1;
        q.y = 0;
    }
    b.plan.cursor[0] = 0;
    rs::ResetPlanner::Impl::advanceCursor(b);
    EXPECT_EQ(b.plan.cursor[0], 100u);
}
TEST(Cursor, OmniFootprintReallyIsExactlyHeadingIndependent) {
    auto b = line(rc::Model::Omni);
    rs::Verts a, z;
    rs::place(b.shape, 1, 2, 0, &a);
    rs::place(b.shape, 1, 2, 2.3, &z);
    ASSERT_EQ(a.n, 16);
    for (int i = 0; i < a.n; ++i) {
        EXPECT_DOUBLE_EQ(a.x[i], z.x[i]);
        EXPECT_DOUBLE_EQ(a.y[i], z.y[i]);
    }
}
TEST(Cursor, OmniCursorStaysMonotoneWithTheSameSearchWindow) {
    auto b = line(rc::Model::Omni);
    b.plan.cursor[0] = 60;
    b.pose = {.5, 0, 0};
    rs::ResetPlanner::Impl::advanceCursor(b);
    EXPECT_EQ(b.plan.cursor[0], 60u);
    auto s = b.plan.sets[0].pts.back();
    for (size_t i = 101; i < 400; ++i) {
        s.x = .02 * i;
        s.s = s.x;
        b.plan.sets[0].pts.push_back(s);
    }
    b.plan.cursor[0] = 0;
    b.pose = {7, 0, 0};
    rs::ResetPlanner::Impl::advanceCursor(b);
    EXPECT_EQ(b.plan.cursor[0], 159u);
}
rs::Violation query(size_t cursor, size_t hi, double x) {
    rs::PlannerConfig cfg;
    cfg.pair_margin = 0;
    rs::ResetPlanner::Impl m{cfg};
    auto b = line(rc::Model::Unicycle);
    b.pose = {2, 1, 0};
    b.plan.cursor[0] = cursor;
    b.plan.hi[0] = hi;
    m.bodies.emplace(b.id, b);
    Body self;
    self.id = "candidate";
    self.spec = b.spec;
    self.shape = b.shape;
    auto q = m.poseSample(self, {x, 0, 0});
    q.s = 42;
    SampleSet candidate;
    candidate.pts = {q};
    candidate.chunks = {q.box};
    return m.firstViolation(self, {&candidate}, {0}, {});
}
TEST(Behind, ExactBehindIntervalStillConflictsWhenHiEqualsCursor) {
    auto v = query(50, 50, .90);
    EXPECT_TRUE(std::isfinite(v.s));
    EXPECT_EQ(v.blockers, std::vector<std::string>{"reserved"});
}
TEST(Behind, ExactBehindIntervalStillConflictsWhenHiIsBelowCursorButAboveLo) {
    auto v = query(50, 48, .90);
    EXPECT_TRUE(std::isfinite(v.s));
}
TEST(Behind, DoesNotIncludeSamplesBeforeOriginalBehindLoOrAtTheExclusiveHi) {
    EXPECT_FALSE(std::isfinite(query(50, 50, .80).s));
    EXPECT_FALSE(std::isfinite(query(50, 50, 1.00).s));
}
TEST(Behind, ATrulyEmptyRetainedIntervalStaysEmpty) {
    EXPECT_FALSE(std::isfinite(query(50, 44, .90).s));
    EXPECT_FALSE(std::isfinite(query(0, 0, 0).s));
}
TEST(Behind, RetiringRoutingAlsoIncludesTheExistingBehindFootprintAtHiEqualsCursor) {
    auto b = line(rc::Model::Unicycle);
    b.pose = {2, 1, 0};
    b.plan_retiring = true;
    b.plan.cursor[0] = 50;
    b.plan.hi[0] = 50;
    const auto obstacle = rs::ResetPlanner::Impl::retiringRoutingObstacle(b);
    ASSERT_EQ(obstacle.vertices.size(), 4u);
    // Sample45 at x=.90 is still in the original [44,50) interval, far from last pose.
    for (const Eigen::Vector2d& point :
         {Eigen::Vector2d(.899, -.001), {.901, -.001}, {.901, .001}, {.899, .001}}) {
        for (size_t i = 0; i < obstacle.vertices.size(); ++i) {
            const auto edge =
                obstacle.vertices[(i + 1) % obstacle.vertices.size()] - obstacle.vertices[i];
            const auto relative = point - obstacle.vertices[i];
            EXPECT_GE(edge.x() * relative.y() - edge.y() * relative.x(), -1e-10);
        }
    }
}
}  // namespace
