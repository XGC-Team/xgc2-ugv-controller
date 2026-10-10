#pragma once

#include <algorithm>
#include <cstdint>

#include "unicycle_reference_trajectory/default_analytic.h"
#include "unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h"

namespace unicycle_reference_trajectory {

// Reads the runtime's parameters (config/unicycle_reference_trajectory.yaml)
// from any configuration source: the ROS parameter server in the node, a JSON
// object in the aggregator module. `Source::get(key, value)` returns true and
// sets `value` when `key` is present with a number (double or int) or a bool,
// and leaves `value` alone otherwise; nested keys are '/'-separated. A key
// that is absent keeps the value already in the structures, so their
// defaults are the defaults of both transports. The runtime clamps the
// values it cannot use in ReferenceTrajectoryRuntime::setConfig.
template <typename Source>
void loadReferenceConfig(const Source& source, ReferenceTrajectoryConfig& config,
                         DefaultAnalyticReferenceConfig& default_analytic) {
    source.get("status_rate", config.status_rate_hz);
    source.get("active_publish_rate", config.active_publish_rate_hz);
    source.get("validation_sample_dt", config.validation_sample_dt);
    source.get("trajectory_timeout", config.trajectory_timeout);
    source.get("min_lead_time", config.min_lead_time);
    source.get("max_velocity", config.limits.max_velocity);
    source.get("max_acceleration", config.limits.max_acceleration);
    source.get("max_yaw_rate", config.limits.max_yaw_rate);

    source.get("default_analytic/enabled", default_analytic.enabled);
    int type = static_cast<int>(default_analytic.analytic_type);
    int request_id = static_cast<int>(default_analytic.request_id);
    int trajectory_id = static_cast<int>(default_analytic.trajectory_id);
    int revision = static_cast<int>(default_analytic.revision);
    source.get("default_analytic/analytic_type", type);
    source.get("default_analytic/request_id", request_id);
    source.get("default_analytic/trajectory_id", trajectory_id);
    source.get("default_analytic/revision", revision);
    source.get("default_analytic/start_delay", default_analytic.start_delay);
    source.get("default_analytic/duration", default_analytic.duration);
    source.get("default_analytic/origin_x", default_analytic.origin_x);
    source.get("default_analytic/origin_y", default_analytic.origin_y);
    source.get("default_analytic/origin_yaw", default_analytic.origin_yaw);
    source.get("default_analytic/radius", default_analytic.radius);
    source.get("default_analytic/line_speed", default_analytic.line_speed);
    source.get("default_analytic/entry_duration", default_analytic.entry_duration);
    source.get("default_analytic/center_x", default_analytic.center_x);
    source.get("default_analytic/center_y", default_analytic.center_y);
    default_analytic.analytic_type = static_cast<uint16_t>(std::max(0, type));
    default_analytic.request_id = static_cast<uint32_t>(std::max(0, request_id));
    default_analytic.trajectory_id = static_cast<uint32_t>(std::max(0, trajectory_id));
    default_analytic.revision = static_cast<uint32_t>(std::max(0, revision));
}

}  // namespace unicycle_reference_trajectory
