#pragma once

#include <array>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

namespace unicycle_ugv_controller {

// Observability only. These values never feed back into the controller.
struct FlatnessAuditSample {
    bool valid{false};
    std::array<double, 29> values{};

    std::string toJson() const {
        if (!valid) return {};
        static constexpr const char* keys[] = {
            "control_time_s", "state_source_time_s", "pva_receipt_time_s", "dt_s",
            "p_x_m", "p_y_m", "yaw_rad", "vx_world_m_s", "vy_world_m_s",
            "wz_measured_rad_s", "v_body_signed_m_s", "q_x_m", "q_y_m",
            "q_vx_m_s", "q_vy_m_s", "q_ax_m_s2", "q_ay_m_s2",
            "v_command_previous_m_s", "v_command_bounded_m_s", "w_command_bounded_rad_s",
            "a_parallel_m_s2", "v_epsilon_m_s", "lateral_response_length_m",
            "lateral_damping", "flatness_kp_s_inv2", "flatness_kv_s_inv",
            "chassis_limit_v_m_s", "chassis_limit_w_rad_s", "cached_state_speed_m_s"};
        static_assert(sizeof(keys) / sizeof(keys[0]) == 29, "audit schema mismatch");
        std::ostringstream out;
        out.imbue(std::locale::classic());
        out << std::setprecision(17) << "{\"schema\":\"xgc2.flatness_audit.v1\"";
        for (std::size_t i = 0; i < values.size(); ++i) {
            out << ",\"" << keys[i] << "\":";
            if (std::isfinite(values[i])) out << values[i];
            else out << "null";
        }
        out << '}';
        return out.str();
    }
};

// The output event can be queued. Never silently associate its sample with
// a newer command fetched by the consumer. The clock is ROS time, not wall time.
inline std::string flatnessAuditPublication(const std::string& sample, double now,
                                            double v, double w, double command_stamp,
                                            double sample_stamp) {
    if (sample.empty() || !std::isfinite(now) || !std::isfinite(v) || !std::isfinite(w) ||
        !std::isfinite(command_stamp) || !std::isfinite(sample_stamp)) return {};
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17) << "{\"schema\":\"xgc2.flatness_publication.v1\","
        << "\"publication_time_s\":" << now << ",\"published_v_m_s\":" << v
        << ",\"published_w_rad_s\":" << w << ",\"command_stamp_s\":" << command_stamp
        << ",\"sample_matches_command\":"
        << (std::fabs(command_stamp - sample_stamp) <= 1e-9 ? "true" : "false")
        << ",\"sample\":" << sample << '}';
    return out.str();
}

}  // namespace unicycle_ugv_controller
