#pragma once

#include <cstdint>

#include "unicycle_reference_trajectory/reference_types.h"

namespace unicycle_reference_trajectory {

// The analytic request the reference trajectory issues to itself once, when
// it first becomes Ready (the default_analytic/* parameters). It is the same
// request any planner could publish, so it goes through the same acceptance.
struct DefaultAnalyticReferenceConfig {
    bool enabled{false};
    uint32_t request_id{1U};
    uint32_t trajectory_id{1U};
    uint32_t revision{1U};
    uint16_t analytic_type{reference::AnalyticReference::ANALYTIC_CIRCLE};
    double start_delay{0.5};
    double duration{120.0};
    double origin_x{0.0};
    double origin_y{0.0};
    double origin_yaw{0.0};
    double radius{3.0};
    double line_speed{1.0};
    double entry_duration{2.0};
    double center_x{0.0};
    double center_y{0.0};
};

// The request for `config` at time `now_sec`.
reference::AnalyticReference makeDefaultAnalytic(const DefaultAnalyticReferenceConfig& config,
                                                 double now_sec);

}  // namespace unicycle_reference_trajectory
