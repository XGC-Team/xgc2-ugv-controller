#pragma once

// The geometric route of the station-side planner: a visibility graph around convex obstacles inside
// the fence. The vehicle's motion (tracking, speed, stopping) is the vehicle's own: ugv_reset_client.

#include <ugv_reset_safety/reset_geometry.h>

#include <string>
#include <vector>

namespace ugv_reset_safety {

struct VisibilityPath {
    bool valid = false;
    bool invalid_input = false;
    std::vector<Eigen::Vector2d> points;
    std::string message;
};

bool validConvexObstacle(const ConvexObstacle& obstacle);

// Shortest polyline from start to goal for a disc of `radius`, kept `clearance` away from every
// obstacle and inside the fence. Malformed geometry is an explicit invalid_input, not a repair.
VisibilityPath planVisibilityPath(const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
                                  const std::vector<ConvexObstacle>& obstacles, const Fence& fence,
                                  double radius, double clearance);

}  // namespace ugv_reset_safety
