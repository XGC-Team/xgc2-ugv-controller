#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>

namespace unicycle_ugv_controller {

// A value snapshot of one accepted Flatness evaluation, not a state estimator.
// Stored with ControlCommand under its existing mutex. No observer receipt time
// is substituted for the reference receipt time actually used by the tracker.
struct FlatnessTrace {
    bool valid{false};
    uint64_t evaluation_stamp_ns{0U};
    uint64_t state_stamp_ns{0U};
    uint64_t reference_source_stamp_ns{0U};
    uint64_t reference_receive_stamp_ns{0U};
    uint32_t reference_sequence{0U};
    double dt_s{0.0};
    double command_speed_before_mps{0.0};
    // State actually passed to computeFlatnessCommand: x,y,yaw,vx,vy,yaw_rate.
    // Positions and linear velocity are world-frame; yaw is body heading.
    std::array<double, 6> state{};
    // Exact lifted reference passed to the same call: x,y,vx,vy,ax,ay.
    std::array<double, 6> reference{};
    // kp [s^-2], kv [s^-1], epsilon [m/s], L [m], zeta [1],
    // command speed limit [m/s], command yaw-rate limit [rad/s].
    std::array<double, 7> parameters{};
};

// JSON over std_msgs/String avoids a new message package dependency. Nanosecond
// stamps are decimal strings, so consumers cannot silently round uint64 stamps
// through IEEE-754 doubles. Empty output means no valid diagnostic snapshot.
inline std::string flatnessTraceJson(const FlatnessTrace& trace,
                                    uint64_t publication_stamp_ns,
                                    double computed_v, double computed_w,
                                    double published_v, double published_w) {
    if (!trace.valid || !std::isfinite(trace.dt_s) || trace.dt_s <= 0.0 ||
        !std::isfinite(trace.command_speed_before_mps) ||
        !std::isfinite(computed_v) || !std::isfinite(computed_w) ||
        !std::isfinite(published_v) || !std::isfinite(published_w)) {
        return {};
    }
    for (double value : trace.state) if (!std::isfinite(value)) return {};
    for (double value : trace.reference) if (!std::isfinite(value)) return {};
    for (double value : trace.parameters) if (!std::isfinite(value)) return {};
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << std::setprecision(std::numeric_limits<double>::max_digits10);
    stream << "{\"schema\":\"xgc2.flatness.consumed.v1\","
           << "\"evaluation_stamp_ns\":\"" << trace.evaluation_stamp_ns << "\","
           << "\"publication_stamp_ns\":\"" << publication_stamp_ns << "\","
           << "\"state_stamp_ns\":\"" << trace.state_stamp_ns << "\","
           << "\"reference_source_stamp_ns\":\"" << trace.reference_source_stamp_ns << "\","
           << "\"reference_receive_stamp_ns\":\"" << trace.reference_receive_stamp_ns << "\","
           << "\"reference_sequence\":" << trace.reference_sequence << ','
           << "\"dt_s\":" << trace.dt_s << ','
           << "\"command_speed_before_mps\":" << trace.command_speed_before_mps << ',';
    const auto array = [&stream](const char* name, const auto& values) {
        stream << '\"' << name << "\":[";
        for (std::size_t index = 0; index < values.size(); ++index) {
            if (index != 0U) stream << ',';
            stream << values[index];
        }
        stream << "],";
    };
    array("state_xy_yaw_vx_vy_w", trace.state);
    array("consumed_q_xy_vx_vy_ax_ay", trace.reference);
    array("kp_kv_eps_L_zeta_vmax_wmax", trace.parameters);
    stream << "\"computed_v_mps\":" << computed_v << ','
           << "\"computed_w_radps\":" << computed_w << ','
           << "\"published_v_mps\":" << published_v << ','
           << "\"published_w_radps\":" << published_w << '}';
    return stream.str();
}

}  // namespace unicycle_ugv_controller
