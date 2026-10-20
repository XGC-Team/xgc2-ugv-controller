#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

namespace ugv_reset_safety {

struct ConvexObstacle {
    std::string id;
    // Clockwise or counterclockwise, without a repeated closing vertex.
    std::vector<Eigen::Vector2d> vertices;
    // World-frame origin and twist of the parent body. Zero for static shapes.
    Eigen::Vector2d origin{Eigen::Vector2d::Zero()};
    Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};
    double omega{0.0};
};

struct Fence {
    bool enabled{true};
    double xmin{-20.0};
    double xmax{20.0};
    double ymin{-20.0};
    double ymax{20.0};
};

}  // namespace ugv_reset_safety
