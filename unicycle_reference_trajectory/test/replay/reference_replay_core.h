#pragma once

// The core side of the reference replay: the glue that drives
// ReferenceTrajectoryDriver through reference_replay_scenario.h, and the three
// runs. Shared by the trace writer (reference_replay.cpp) and the golden test.

#include <cstdint>
#include <cstdio>
#include <vector>

#include "reference_replay_scenario.h"
#include "unicycle_reference_trajectory/reference_driver.h"

namespace reference_replay_core {

namespace urt = unicycle_reference_trajectory;

struct CoreGlue {
    using Analytic = urt::reference::AnalyticReference;
    using Sampled = urt::reference::SampledReference;
    using Waypoint = urt::reference::WaypointReferenceRequest;
    using Pose = urt::reference::Pose;
    using Request = urt::ReferenceTrajectoryDriver::Request;

    urt::ReferenceTrajectoryDriver driver;
    std::vector<state_machine::Event> out;

    urt::Time time(double sec) const {
        return sec <= 0.0 ? urt::Time() : urt::Time(sec);
    }
    // The node reads ros::Time::now().toSec(): the clock at nanosecond resolution.
    static double clockSec(double now) {
        return urt::Time(now).toSec();
    }
    bool analytic(const Analytic& m, double now) {
        return driver.acceptAnalytic(m, clockSec(now)) == Request::kPosted;
    }
    bool sampled(const Sampled& m, double now) {
        return driver.acceptSampled(m, clockSec(now)) == Request::kPosted;
    }
    bool waypoint(const Waypoint& m, double now) {
        return driver.acceptWaypoint(m, clockSec(now)) == Request::kPosted;
    }
    void reset(double now) {
        (void)driver.reset(clockSec(now));
    }
    void update(double now) {
        out = driver.update(clockSec(now)).events;
    }
    const urt::ReferenceTrajectoryRuntime& runtime() const {
        return driver.runtime();
    }
    const std::vector<state_machine::Event>& events() const {
        return out;
    }
    uint64_t activeStartNs() const {
        namespace trajectory = xgc2_math::trajectory;
        const auto& rt = driver.runtime();
        if (rt.activeType() == trajectory::TrajectoryModelType::kAnalytic) {
            return rt.activeAnalyticMessage().start_time.toNSec();
        }
        if (rt.activeType() == trajectory::TrajectoryModelType::kSampled) {
            return rt.activeSampledMessage().start_time.toNSec();
        }
        if (rt.activeType() == trajectory::TrajectoryModelType::kPolynomial) {
            return rt.activePolynomialMessage().start_time.toNSec();
        }
        return 0;
    }
};

inline std::vector<reference_replay::Config> runs() {
    using reference_replay::Config;
    std::vector<Config> configs;
    configs.emplace_back();
    Config limits;
    limits.name = "limits";
    limits.status_rate_hz = 20.0;
    limits.active_publish_rate_hz = 5.0;
    limits.validation_sample_dt = 0.05;
    limits.trajectory_timeout = 0.2;
    limits.min_lead_time = 0.5;
    limits.max_velocity = 0.8;
    limits.max_acceleration = 1.0;
    limits.max_yaw_rate = 0.6;
    configs.push_back(limits);
    Config defaults;
    defaults.name = "default_analytic";
    defaults.default_analytic = true;
    configs.push_back(defaults);
    return configs;
}

// Runs the three scenarios and writes the trace to `out`.
inline void writeTrace(FILE* out) {
    for (const auto& run : runs()) {
        CoreGlue glue;
        urt::ReferenceTrajectoryConfig config;
        config.status_rate_hz = run.status_rate_hz;
        config.active_publish_rate_hz = run.active_publish_rate_hz;
        config.validation_sample_dt = run.validation_sample_dt;
        config.trajectory_timeout = run.trajectory_timeout;
        config.min_lead_time = run.min_lead_time;
        config.limits.max_velocity = run.max_velocity;
        config.limits.max_acceleration = run.max_acceleration;
        config.limits.max_yaw_rate = run.max_yaw_rate;
        urt::DefaultAnalyticReferenceConfig default_analytic;
        if (run.default_analytic) {
            default_analytic.enabled = true;
            default_analytic.analytic_type = 3;
            default_analytic.start_delay = 0.5;
            default_analytic.duration = 30.0;
            default_analytic.origin_x = 0.2;
            default_analytic.origin_y = 0.1;
            default_analytic.origin_yaw = 0.3;
            default_analytic.radius = 2.0;
            default_analytic.line_speed = 0.9;
            default_analytic.entry_duration = 2.5;
            default_analytic.center_x = 0.5;
            default_analytic.center_y = -0.5;
        }
        glue.driver.configure(config, default_analytic);
        reference_replay::run(glue, run, out);
    }
}

}  // namespace reference_replay_core
