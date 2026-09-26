#pragma once

#include <cstdint>
#include <vector>

#include "unicycle_ugv_controller/common/time.h"

namespace unicycle_ugv_controller {
namespace reference {

// Plain forms of the unicycle_reference_trajectory_msgs references the
// ReferenceCache takes. Field names, meanings and defaults are the
// messages' own; the ROS edge converts with ros_reference_conversion.h.

struct Point {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

// All zero by default, as geometry_msgs/Quaternion is.
struct Quaternion {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double w{0.0};
};

struct Pose {
    Point position;
    Quaternion orientation;
};

struct AnalyticReference {
    static constexpr uint16_t ANALYTIC_HOLD = 0;
    static constexpr uint16_t ANALYTIC_CIRCLE = 1;
    static constexpr uint16_t ANALYTIC_CIRCLE_ENTRY = 3;
    static constexpr uint16_t ANALYTIC_FIGURE_EIGHT = 4;

    uint32_t request_id{0U};
    uint32_t trajectory_id{0U};
    uint32_t revision{0U};
    uint16_t analytic_type{0U};
    uint32_t flags{0U};
    Time start_time;
    double duration{0.0};
    Pose origin;
    std::vector<double> params;
};

struct PlanarReferencePoint {
    double t_from_start{0.0};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double speed{0.0};
    double linear_acceleration{0.0};
    double yaw_rate{0.0};
    double yaw_acceleration{0.0};
    double curvature{0.0};
    double vx{0.0};
    double vy{0.0};
    double ax{0.0};
    double ay{0.0};
    double jx{0.0};
    double jy{0.0};
};

struct SampledReference {
    static constexpr uint32_t FLAG_EXPLICIT_PLANAR_KINEMATICS = 32768U;

    uint32_t trajectory_id{0U};
    uint32_t revision{0U};
    uint32_t flags{0U};
    Time start_time;
    double sample_dt{0.0};
    std::vector<PlanarReferencePoint> points;
};

struct ActivePolynomialReference {
    uint32_t trajectory_id{0U};
    uint32_t revision{0U};
    uint32_t flags{0U};
    Time start_time;
    double duration{0.0};
    uint8_t order{0U};
    std::vector<double> segment_durations;
    std::vector<double> coeff_x;
    std::vector<double> coeff_y;
    std::vector<double> coeff_yaw;
};

}  // namespace reference
}  // namespace unicycle_ugv_controller
