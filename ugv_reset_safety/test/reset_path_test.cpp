#include <gtest/gtest.h>
#include <ugv_reset_safety/reset_path.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace ugv_reset_safety {
namespace {

ConvexObstacle box(double xmin, double xmax, double ymin, double ymax) {
    ConvexObstacle obstacle;
    obstacle.id = "box";
    obstacle.vertices = {{xmin, ymin}, {xmax, ymin}, {xmax, ymax}, {xmin, ymax}};
    return obstacle;
}

double distanceToBox(const Eigen::Vector2d& point, double xmin, double xmax, double ymin,
                     double ymax) {
    const double dx = std::max({xmin - point.x(), 0.0, point.x() - xmax});
    const double dy = std::max({ymin - point.y(), 0.0, point.y() - ymax});
    return std::hypot(dx, dy);
}

TEST(ResetPath, VisibilityGraphRoutesAroundBlockingBox) {
    const auto obstacle = box(-0.4, 0.4, -0.7, 0.7);
    const auto path = planVisibilityPath({-2.0, 0.0}, {2.0, 0.0}, {obstacle}, Fence(), 0.3, 0.18);
    ASSERT_TRUE(path.valid) << path.message;
    ASSERT_GE(path.points.size(), 4U);
    for (std::size_t i = 1; i < path.points.size(); ++i) {
        for (int sample = 0; sample <= 100; ++sample) {
            const double ratio = sample / 100.0;
            const Eigen::Vector2d point =
                (1.0 - ratio) * path.points[i - 1] + ratio * path.points[i];
            EXPECT_GE(distanceToBox(point, -0.4, 0.4, -0.7, 0.7), 0.48 - 1e-8);
        }
    }
}

TEST(ResetPath, NoRouteThroughAWallThatSpansTheFence) {
    Fence fence;
    fence.xmin = -3.0;
    fence.xmax = 3.0;
    fence.ymin = -2.0;
    fence.ymax = 2.0;
    ASSERT_TRUE(planVisibilityPath({-2.0, 0.0}, {2.0, 0.0}, {}, fence, 0.3, 0.18).valid);
    const auto blocked =
        planVisibilityPath({-2.0, 0.0}, {2.0, 0.0}, {box(-0.2, 0.2, -3.0, 3.0)}, fence, 0.3, 0.18);
    EXPECT_FALSE(blocked.valid);
    EXPECT_FALSE(blocked.invalid_input);
    EXPECT_TRUE(blocked.points.empty());
}

TEST(ResetPath, RejectsInvalidGeometryAndOccupiedTarget) {
    const auto occupied =
        planVisibilityPath({-2.0, 0.0}, {0.0, 0.0}, {box(-0.5, 0.5, -0.5, 0.5)}, Fence(), 0.3, 0.1);
    EXPECT_FALSE(occupied.valid);
    EXPECT_FALSE(occupied.invalid_input);
    ConvexObstacle concave;
    concave.vertices = {{0, 0}, {1, 0}, {0.3, 0.3}, {1, 1}, {0, 1}};
    EXPECT_TRUE(planVisibilityPath({-2, 0}, {2, 0}, {concave}, Fence(), 0.3, 0.1).invalid_input);
    auto duplicate = box(-0.5, 0.5, -0.5, 0.5);
    duplicate.vertices.push_back(duplicate.vertices.front());
    EXPECT_TRUE(planVisibilityPath({-2, 0}, {2, 0}, {duplicate}, Fence(), 0.3, 0.1).invalid_input);
    EXPECT_TRUE(planVisibilityPath({std::numeric_limits<double>::quiet_NaN(), 0}, {2, 0}, {},
                                   Fence(), 0.3, 0.1)
                    .invalid_input);
}

TEST(ResetPath, ClearStartAndGoalAreNotRejectedByTheCorneringReserve) {
    const auto wall = box(-0.2, 0.2, -2.0, 2.0);
    // Radius 0.30 + clearance 0.18 fit at x = -0.8 and no more.
    EXPECT_TRUE(planVisibilityPath({-0.8, 0.0}, {-2.0, 0.0}, {wall}, Fence(), 0.3, 0.18).valid);
    EXPECT_TRUE(planVisibilityPath({-2.0, 0.0}, {-0.8, 0.0}, {wall}, Fence(), 0.3, 0.18).valid);
    EXPECT_FALSE(planVisibilityPath({-2.0, 0.0}, {-0.6, 0.0}, {wall}, Fence(), 0.3, 0.18).valid);
}

TEST(ResetPath, ClockwiseAndCollinearConvexGeometryHaveEquivalentRoutes) {
    auto obstacle = box(-0.5, 0.5, -0.5, 0.5);
    const auto baseline = planVisibilityPath({-2, 0}, {2, 0}, {obstacle}, Fence(), 0.3, 0.1);
    std::reverse(obstacle.vertices.begin(), obstacle.vertices.end());
    obstacle.vertices.insert(obstacle.vertices.begin() + 1, Eigen::Vector2d(0, 0.5));
    const auto alternative = planVisibilityPath({-2, 0}, {2, 0}, {obstacle}, Fence(), 0.3, 0.1);
    ASSERT_TRUE(baseline.valid);
    ASSERT_TRUE(alternative.valid) << alternative.message;
    auto length = [](const VisibilityPath& path) {
        double result = 0.0;
        for (std::size_t i = 1; i < path.points.size(); ++i) {
            result += (path.points[i] - path.points[i - 1]).norm();
        }
        return result;
    };
    EXPECT_NEAR(length(baseline), length(alternative), 1e-10);
}

TEST(ResetPath, SeededBoxPlacementsProduceGeometricallyClearRoutes) {
    std::mt19937 random(20260910);
    std::uniform_real_distribution<double> center(-0.8, 0.8);
    std::uniform_real_distribution<double> extent(0.1, 0.6);
    for (int trial = 0; trial < 100; ++trial) {
        const double x = center(random), y = center(random), hx = extent(random),
                     hy = extent(random);
        const auto path = planVisibilityPath(
            {-2.5, -0.2}, {2.5, 0.2}, {box(x - hx, x + hx, y - hy, y + hy)}, Fence(), 0.3, 0.1);
        ASSERT_TRUE(path.valid) << "trial=" << trial << " " << path.message;
        for (std::size_t i = 1; i < path.points.size(); ++i) {
            for (int sample = 0; sample <= 100; ++sample) {
                const double ratio = sample / 100.0;
                const Eigen::Vector2d p =
                    (1.0 - ratio) * path.points[i - 1] + ratio * path.points[i];
                ASSERT_GE(distanceToBox(p, x - hx, x + hx, y - hy, y + hy), 0.4 - 1e-8);
            }
        }
    }
}

}  // namespace
}  // namespace ugv_reset_safety
