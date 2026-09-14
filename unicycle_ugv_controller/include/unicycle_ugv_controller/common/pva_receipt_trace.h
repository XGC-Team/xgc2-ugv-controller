#pragma once

#include <array>
#include <cstdint>

namespace unicycle_ugv_controller {

// The receipt timestamp is the controller's local ROS time, not transport
// latency or source activation time. PVA occupies indices 5..10 in world XY.
inline std::array<double, 12> pvaReceiptTrace(double receipt_time, double source_time,
                                              uint32_t source_sequence, uint32_t receipt_sequence,
                                              const std::array<double, 6>& pva, double yaw) {
    return {{1.0, receipt_time, source_time, static_cast<double>(source_sequence),
             static_cast<double>(receipt_sequence), pva[0], pva[1], pva[2], pva[3], pva[4],
             pva[5], yaw}};
}

inline const char* pvaReceiptTraceLayout() {
    return "schema_v1,receipt_ros_s,source_ros_s,source_seq,receipt_seq,x_m,y_m,vx_mps,vy_mps,"
           "ax_mps2,ay_mps2,yaw_rad";
}

}  // namespace unicycle_ugv_controller
