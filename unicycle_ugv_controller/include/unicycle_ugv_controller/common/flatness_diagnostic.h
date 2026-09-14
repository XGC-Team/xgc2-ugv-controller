#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace unicycle_ugv_controller {

// Immutable inputs from one flatness evaluation. Observation only: none of
// these fields feed back into the controller or alter its low-speed law.
struct FlatnessDiagnostic {
    enum Field : std::size_t {
        TickTime, StateTime, ReferenceReceiptTime, Dt, CommandSpeedBefore,
        EstimatedLongitudinalSpeed, EstimatedLateralSpeed,
        Px, Py, Yaw, Vx, Vy, YawRate,
        Qx, Qy, Qvx, Qvy, Qax, Qay,
        SolvedLinearSpeed, SolvedYawRate, SolvedAcceleration,
        PublishTime, PublishedLinearSpeed, PublishedYawRate, Count
    };
    std::array<double, Count> values{};
    bool valid{false};
};

inline const char* flatnessDiagnosticSchema() {
    return "flatness_diagnostic_v1:tick_s,state_s,reference_receipt_s,dt_s,"
           "command_before_m_s,estimated_longitudinal_m_s,estimated_lateral_m_s,"
           "px_m,py_m,yaw_rad,vx_m_s,vy_m_s,yaw_rate_rad_s,"
           "qx_m,qy_m,qvx_m_s,qvy_m_s,qax_m_s2,qay_m_s2,"
           "solved_v_m_s,solved_w_rad_s,solved_a_m_s2,"
           "publish_s,published_v_m_s,published_w_rad_s";
}

template <class State, class Reference, class Output>
FlatnessDiagnostic makeFlatnessDiagnostic(double tick_time, double dt,
    double command_speed_before, const State& state, const Reference& lifted,
    const Output& output) {
    FlatnessDiagnostic sample;
    const double c = std::cos(state.yaw), s = std::sin(state.yaw);
    sample.values = {{tick_time, state.stamp.toSec(), lifted.stamp.toSec(), dt,
        command_speed_before, c*state.vx+s*state.vy, -s*state.vx+c*state.vy,
        state.x, state.y, state.yaw, state.vx, state.vy, state.yaw_rate,
        lifted.x, lifted.y, lifted.vx, lifted.vy, lifted.ax, lifted.ay,
        output.linear_speed, output.angular_speed, output.accel, 0.0, 0.0, 0.0}};
    sample.valid = output.valid && state.velocity_valid && lifted.valid;
    for (double value : sample.values) sample.valid = sample.valid && std::isfinite(value);
    return sample;
}

inline FlatnessDiagnostic atFlatnessPublication(const FlatnessDiagnostic& solved,
    double publish_time, double published_linear_speed, double published_yaw_rate) {
    FlatnessDiagnostic sample = solved;
    sample.values[FlatnessDiagnostic::PublishTime] = publish_time;
    sample.values[FlatnessDiagnostic::PublishedLinearSpeed] = published_linear_speed;
    sample.values[FlatnessDiagnostic::PublishedYawRate] = published_yaw_rate;
    sample.valid = sample.valid && std::isfinite(publish_time) &&
        std::isfinite(published_linear_speed) && std::isfinite(published_yaw_rate);
    return sample;
}

}  // namespace unicycle_ugv_controller
