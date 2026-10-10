#include "unicycle_reference_trajectory/default_analytic.h"

#include <algorithm>
#include <cmath>

namespace unicycle_reference_trajectory {
namespace {

reference::Quaternion yawQuaternion(double yaw) {
    reference::Quaternion q;
    if (!std::isfinite(yaw)) {
        yaw = 0.0;
    }
    q.w = std::cos(0.5 * yaw);
    q.z = std::sin(0.5 * yaw);
    return q;
}

uint16_t normalizedAnalyticType(uint16_t type) {
    if (type == reference::AnalyticReference::ANALYTIC_HOLD ||
        type == reference::AnalyticReference::ANALYTIC_CIRCLE ||
        type == reference::AnalyticReference::ANALYTIC_CIRCLE_ENTRY ||
        type == reference::AnalyticReference::ANALYTIC_FIGURE_EIGHT) {
        return type;
    }
    return reference::AnalyticReference::ANALYTIC_CIRCLE;
}

}  // namespace

reference::AnalyticReference makeDefaultAnalytic(const DefaultAnalyticReferenceConfig& config,
                                                 double now_sec) {
    reference::AnalyticReference msg;
    msg.header.stamp = Time(now_sec);
    msg.request_id = config.request_id;
    msg.trajectory_id = config.trajectory_id;
    msg.revision = config.revision;
    msg.analytic_type = normalizedAnalyticType(config.analytic_type);
    msg.start_time = Time(now_sec + std::max(0.0, config.start_delay));
    msg.duration =
        std::isfinite(config.duration) && config.duration > 0.0 ? config.duration : 120.0;
    msg.origin.position.x = config.origin_x;
    msg.origin.position.y = config.origin_y;
    msg.origin.orientation = yawQuaternion(config.origin_yaw);
    msg.params = {config.radius, config.line_speed, config.entry_duration, config.center_x,
                  config.center_y};
    return msg;
}

}  // namespace unicycle_reference_trajectory
