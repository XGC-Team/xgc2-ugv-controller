#pragma once

#include <cstdint>

namespace unicycle_reference_trajectory {

namespace state_type {
constexpr uint32_t SelfCheck = 1;
constexpr uint32_t Ready = 2;
constexpr uint32_t Planning = 3;
constexpr uint32_t Active = 4;
constexpr uint32_t Fault = 9;
}  // namespace state_type

namespace region_type {
constexpr uint32_t REFERENCE = 1;
}  // namespace region_type

namespace event_type {
constexpr uint32_t CONFIG_READY = 100;
constexpr uint32_t ANALYTIC_RECEIVED = 101;
constexpr uint32_t WAYPOINT_RECEIVED = 102;
constexpr uint32_t SAMPLED_RECEIVED = 103;
constexpr uint32_t PLAN_SUCCEEDED = 104;
constexpr uint32_t PLAN_FAILED = 105;
constexpr uint32_t TRAJECTORY_EXPIRED = 106;
constexpr uint32_t RESET_REQUESTED = 107;
}  // namespace event_type

namespace output_event_type {
constexpr uint32_t PUBLISH_STATUS = 1000;
constexpr uint32_t PUBLISH_ACTIVE_ANALYTIC = 1001;
constexpr uint32_t PUBLISH_ACTIVE_POLYNOMIAL = 1002;
constexpr uint32_t PUBLISH_ACTIVE_SAMPLED = 1003;
}  // namespace output_event_type

namespace transition_priority {
constexpr int FAULT = 100;
constexpr int REQUEST = 50;
constexpr int AUTOMATIC = 10;
}  // namespace transition_priority

}  // namespace unicycle_reference_trajectory
