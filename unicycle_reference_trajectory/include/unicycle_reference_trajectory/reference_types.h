#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "unicycle_reference_trajectory/time.h"

namespace unicycle_reference_trajectory {
namespace reference {

// The runtime's inputs and outputs, without ROS. They mirror the
// unicycle_reference_trajectory_msgs messages field for field (the ROS edge
// converts in ros_reference_conversion.h and pins the constants), so the
// runtime reads and writes exactly the values the messages carried. Defaults
// are zero, like ROS message defaults.

struct Point {
    double x{0.0}, y{0.0}, z{0.0};
};
struct Vector3 {
    double x{0.0}, y{0.0}, z{0.0};
};
struct Quaternion {
    double x{0.0}, y{0.0}, z{0.0}, w{0.0};
};
struct Pose {
    Point position;
    Quaternion orientation;
};
struct Header {
    uint32_t seq{0};
    Time stamp;
    std::string frame_id;
};

struct AnalyticReference {
    static constexpr uint16_t ANALYTIC_HOLD = 0;
    static constexpr uint16_t ANALYTIC_CIRCLE = 1;
    static constexpr uint16_t ANALYTIC_CIRCLE_ENTRY = 3;
    static constexpr uint16_t ANALYTIC_FIGURE_EIGHT = 4;

    Header header;
    uint32_t request_id{0};
    uint32_t trajectory_id{0};
    uint32_t revision{0};
    uint16_t analytic_type{0};
    uint32_t flags{0};
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

    Header header;
    uint32_t trajectory_id{0};
    uint32_t revision{0};
    uint32_t flags{0};
    Time start_time;
    double sample_dt{0.0};
    std::vector<PlanarReferencePoint> points;
};

struct WaypointReferenceRequest {
    static constexpr uint8_t OBJECTIVE_SEPTIC_INTERPOLATION = 1;

    Header header;
    uint32_t request_id{0};
    uint32_t trajectory_id{0};
    uint32_t revision{0};
    uint32_t flags{0};
    std::vector<Pose> waypoints;
    std::vector<double> segment_times;
    Vector3 start_velocity;
    Vector3 start_acceleration;
    Vector3 end_velocity;
    Vector3 end_acceleration;
    double desired_speed{0.0};
    double max_velocity{0.0};
    double max_acceleration{0.0};
    double max_yaw_rate{0.0};
    double max_linear_acceleration{0.0};
    uint8_t objective{0};
};

struct ActivePolynomialReference {
    Header header;
    uint32_t trajectory_id{0};
    uint32_t revision{0};
    uint32_t flags{0};
    Time start_time;
    double duration{0.0};
    uint8_t order{0};
    std::vector<double> segment_durations;
    std::vector<double> coeff_x;
    std::vector<double> coeff_y;
    std::vector<double> coeff_yaw;
};

struct ReferenceStatus {
    static constexpr uint8_t STATE_SELF_CHECK = 1;
    static constexpr uint8_t STATE_READY = 2;
    static constexpr uint8_t STATE_PLANNING = 3;
    static constexpr uint8_t STATE_ACTIVE = 4;
    static constexpr uint8_t STATE_FAULT = 9;
    static constexpr uint8_t TYPE_NONE = 0;
    static constexpr uint8_t TYPE_ANALYTIC = 1;
    static constexpr uint8_t TYPE_POLYNOMIAL = 2;
    static constexpr uint8_t TYPE_SAMPLED = 3;

    Header header;
    uint8_t state{0};
    uint32_t flags{0};
    uint32_t active_trajectory_id{0};
    uint32_t active_revision{0};
    uint8_t active_type{0};
};

}  // namespace reference
}  // namespace unicycle_reference_trajectory
