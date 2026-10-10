#pragma once

// Scripted request stream and trace writer of the reference trajectory replay.
//
// The scenario is written once, as a template over the request/message types
// of a "glue" class, so the same text can be produced by any implementation
// whose types carry the unicycle_reference_trajectory_msgs field names: the
// original ROS runtime (ROS messages), the ROS-free core (reference_types.h)
// or a module. A glue provides:
//   types Analytic, Sampled, Waypoint, Pose              request/message types
//   Time time(double sec)                                 a time value from seconds
//   bool analytic/sampled/waypoint(msg, now_sec)         the producer: accept + post event
//   void reset(now_sec)                                   the reset producer
//   void update(now_sec)                                  one main-loop iteration
//   runtime()                                             the runtime (for the trace)
//   events()                                              this iteration's output events
//   uint64_t activeStartNs()                              start time of the active reference
// Every output event, the message the output consumer publishes for it, the
// runtime state and flags and the active evaluator are written with doubles
// as hex bits, so two implementations compare with `cmp`. Time is explicit:
// the harness sets every stamp; no wall-clock value reaches the output.

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include <xgc2_math/trajectory.hpp>

namespace reference_replay {

namespace trajectory = xgc2_math::trajectory;

constexpr double kT0 = 1000.0;  // epoch of the scenario, seconds
constexpr double kDt = 0.01;    // 100 Hz main loop
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// Event and message identifiers shared by all implementations.
constexpr uint32_t kPublishStatus = 1000;
constexpr uint32_t kPublishActiveAnalytic = 1001;
constexpr uint32_t kPublishActivePolynomial = 1002;
constexpr uint32_t kPublishActiveSampled = 1003;

// Runtime parameters of one run (config/unicycle_reference_trajectory.yaml).
struct Config {
    const char* name{"default"};
    double status_rate_hz{10.0};
    double active_publish_rate_hz{10.0};
    double validation_sample_dt{0.02};
    double trajectory_timeout{0.5};
    double min_lead_time{0.2};
    double max_velocity{0.0};
    double max_acceleration{0.0};
    double max_yaw_rate{0.0};
    bool default_analytic{false};
    double end{26.0};  // seconds after the first iteration
};

inline uint64_t bits(double v) {
    uint64_t b;
    std::memcpy(&b, &v, sizeof b);
    return b;
}

class Writer {
   public:
    explicit Writer(FILE* out) : out_(out) {}

    FILE* file() const {
        return out_;
    }
    void d(double v) {
        std::fprintf(out_, " %016" PRIx64, bits(v));
    }
    void doubles(const std::vector<double>& v) {
        std::fprintf(out_, " [%zu", v.size());
        for (double x : v) {
            d(x);
        }
        std::fputc(']', out_);
    }
    template <typename T>
    void time(const T& t) {
        std::fprintf(out_, " %u.%09u", static_cast<unsigned>(t.sec), static_cast<unsigned>(t.nsec));
    }
    template <typename H>
    void header(const H& h) {
        std::fprintf(out_, " hdr %u", static_cast<unsigned>(h.seq));
        time(h.stamp);
        std::fprintf(out_, " %s", h.frame_id.empty() ? "-" : h.frame_id.c_str());
    }
    template <typename V>
    void vec3(const V& v) {
        d(v.x);
        d(v.y);
        d(v.z);
    }
    template <typename Q>
    void quat(const Q& q) {
        d(q.x);
        d(q.y);
        d(q.z);
        d(q.w);
    }
    template <typename M>
    void status(const M& m) {
        std::fprintf(out_, " status");
        header(m.header);
        std::fprintf(out_, " st %u fl %u id %u rev %u type %u", static_cast<unsigned>(m.state),
                     static_cast<unsigned>(m.flags), static_cast<unsigned>(m.active_trajectory_id),
                     static_cast<unsigned>(m.active_revision),
                     static_cast<unsigned>(m.active_type));
    }
    template <typename M>
    void analytic(const M& m) {
        std::fprintf(out_, " analytic");
        header(m.header);
        std::fprintf(out_, " req %u id %u rev %u type %u fl %u",
                     static_cast<unsigned>(m.request_id), static_cast<unsigned>(m.trajectory_id),
                     static_cast<unsigned>(m.revision), static_cast<unsigned>(m.analytic_type),
                     static_cast<unsigned>(m.flags));
        time(m.start_time);
        d(m.duration);
        vec3(m.origin.position);
        quat(m.origin.orientation);
        doubles(m.params);
    }
    template <typename M>
    void sampled(const M& m) {
        std::fprintf(out_, " sampled");
        header(m.header);
        std::fprintf(out_, " id %u rev %u fl %u", static_cast<unsigned>(m.trajectory_id),
                     static_cast<unsigned>(m.revision), static_cast<unsigned>(m.flags));
        time(m.start_time);
        d(m.sample_dt);
        std::fprintf(out_, " [%zu", m.points.size());
        for (const auto& p : m.points) {
            d(p.t_from_start);
            d(p.x);
            d(p.y);
            d(p.yaw);
            d(p.speed);
            d(p.linear_acceleration);
            d(p.yaw_rate);
            d(p.yaw_acceleration);
            d(p.curvature);
            d(p.vx);
            d(p.vy);
            d(p.ax);
            d(p.ay);
            d(p.jx);
            d(p.jy);
        }
        std::fputc(']', out_);
    }
    template <typename M>
    void polynomial(const M& m) {
        std::fprintf(out_, " polynomial");
        header(m.header);
        std::fprintf(out_, " id %u rev %u fl %u", static_cast<unsigned>(m.trajectory_id),
                     static_cast<unsigned>(m.revision), static_cast<unsigned>(m.flags));
        time(m.start_time);
        d(m.duration);
        std::fprintf(out_, " order %u", static_cast<unsigned>(m.order));
        doubles(m.segment_durations);
        doubles(m.coeff_x);
        doubles(m.coeff_y);
        doubles(m.coeff_yaw);
    }

   private:
    FILE* out_;
};

// The active evaluator, sampled every 0.05 s over (at most) its first 20 s,
// as the reference path preview would read it.
inline void writeEvaluator(Writer& w, uint64_t k,
                           const trajectory::TrajectoryEvaluator2* evaluator) {
    FILE* out = w.file();
    std::fprintf(out, "%" PRIu64 " evaluator", k);
    if (evaluator == nullptr) {
        std::fprintf(out, " none\n");
        return;
    }
    std::fprintf(out, " type %d", static_cast<int>(evaluator->type()));
    w.d(evaluator->duration());
    std::fprintf(out, " fl %u\n", static_cast<unsigned>(evaluator->flags()));
    const double span =
        std::isfinite(evaluator->duration()) ? std::min(evaluator->duration(), 20.0) : 20.0;
    for (int i = 0; static_cast<double>(i) * 0.05 <= span + 1e-9; ++i) {
        const double at = static_cast<double>(i) * 0.05;
        trajectory::PlanarReference2 ref;
        const bool ok = evaluator->evaluate(at, ref);
        std::fprintf(out, "  %d %d", i, ok ? 1 : 0);
        for (const auto* v : {&ref.position, &ref.velocity, &ref.acceleration, &ref.jerk}) {
            w.d(v->x());
            w.d(v->y());
        }
        w.d(ref.yaw);
        w.d(ref.speed);
        w.d(ref.linear_acceleration);
        w.d(ref.yaw_rate);
        w.d(ref.yaw_acceleration);
        w.d(ref.curvature);
        std::fprintf(out, " fl %u\n", static_cast<unsigned>(ref.flags));
    }
}

enum class Kind { kAnalytic, kSampled, kWaypoint, kReset };

template <typename Glue>
struct Script {
    struct Item {
        double at{0.0};
        Kind kind{Kind::kReset};
        typename Glue::Analytic analytic{};
        typename Glue::Sampled sampled{};
        typename Glue::Waypoint waypoint{};
    };

    explicit Script(Glue& glue) : glue_(glue) {}

    std::vector<Item> items;

    void analytic(double at, uint16_t type, std::vector<double> params, double duration = 6.0,
                  double ox = 0.0, double oy = 0.0, double oyaw = 0.0, double start = 0.0) {
        Item item;
        item.at = at;
        item.kind = Kind::kAnalytic;
        auto& m = item.analytic;
        ++id_;
        m.header.stamp = glue_.time(kT0 + at);
        m.request_id = 100U + id_;
        m.trajectory_id = id_;
        m.revision = 1U;
        m.analytic_type = type;
        m.flags = 0U;
        m.start_time = glue_.time(start);
        m.duration = duration;
        m.origin.position.x = ox;
        m.origin.position.y = oy;
        m.origin.orientation.z = std::sin(0.5 * oyaw);
        m.origin.orientation.w = std::cos(0.5 * oyaw);
        m.params = std::move(params);
        items.push_back(std::move(item));
    }

    // A constant-curvature arc, optionally with a non-monotonic time or no points.
    void sampled(double at, int count, bool explicit_kinematics, bool monotonic = true,
                 double start = 0.0) {
        Item item;
        item.at = at;
        item.kind = Kind::kSampled;
        auto& m = item.sampled;
        ++id_;
        m.header.stamp = glue_.time(kT0 + at);
        m.trajectory_id = id_;
        m.revision = 2U;
        m.flags = explicit_kinematics ? 32768U : 0U;
        m.start_time = glue_.time(start);
        m.sample_dt = 0.05;
        const double v = 0.5, w = 0.25;
        for (int k = 0; k < count; ++k) {
            const double s = (monotonic || k != count / 2) ? k * 0.05 : 0.0;
            typename std::decay<decltype(m.points[0])>::type p;
            p.t_from_start = s;
            p.yaw = w * s;
            p.x = v / w * std::sin(p.yaw);
            p.y = v / w * (1.0 - std::cos(p.yaw));
            p.speed = v;
            p.yaw_rate = w;
            p.curvature = w / v;
            p.vx = v * std::cos(p.yaw);
            p.vy = v * std::sin(p.yaw);
            p.ax = -v * w * std::sin(p.yaw);
            p.ay = v * w * std::cos(p.yaw);
            p.jx = -v * w * w * std::cos(p.yaw);
            p.jy = -v * w * w * std::sin(p.yaw);
            m.points.push_back(p);
        }
        items.push_back(std::move(item));
    }

    // Waypoints on a line segment pair; the header stamp is the requested start time.
    void waypoint(double at, int count, std::vector<double> segment_times, double stamp_offset,
                  double desired_speed = 0.6, double max_velocity = 0.0) {
        Item item;
        item.at = at;
        item.kind = Kind::kWaypoint;
        auto& m = item.waypoint;
        ++id_;
        m.header.stamp = glue_.time(stamp_offset > 0.0 ? kT0 + at + stamp_offset : 0.0);
        m.request_id = 200U + id_;
        m.trajectory_id = id_;
        m.revision = 0U;
        m.flags = 0U;
        for (int k = 0; k < count; ++k) {
            typename Glue::Pose pose;
            pose.position.x = 1.0 * k;
            pose.position.y = (k % 2 == 0) ? 0.0 : 0.8;
            const double yaw = 0.4 * k;
            pose.orientation.z = std::sin(0.5 * yaw);
            pose.orientation.w = std::cos(0.5 * yaw);
            m.waypoints.push_back(pose);
        }
        m.segment_times = std::move(segment_times);
        m.start_velocity.x = 0.2;
        m.end_velocity.y = -0.1;
        m.desired_speed = desired_speed;
        m.max_velocity = max_velocity;
        m.max_acceleration = max_velocity > 0.0 ? 1.0 : 0.0;
        m.max_yaw_rate = max_velocity > 0.0 ? 0.5 : 0.0;
        m.objective = 1U;
        items.push_back(std::move(item));
    }

    void reset(double at) {
        Item item;
        item.at = at;
        item.kind = Kind::kReset;
        items.push_back(std::move(item));
    }

   private:
    Glue& glue_;
    uint32_t id_{0U};
};

// The requests of one run. kind constants: 0 hold, 1 circle, 3 circle entry, 4 figure eight.
template <typename Glue>
void buildScript(Script<Glue>& s, const Config& config) {
    s.analytic(0.35, 1, {2.0, 0.8, 3.0, 0.0, 1.0});                       // circle, start at once
    s.analytic(1.40, 0, {}, 3.0, 0.5, 0.2, 0.7);                          // hold
    s.analytic(2.00, 3, {1.5, 1.0, 1.2, 0.2, 0.3}, 8.0, 0.1, 0.0, 0.4);   // circle entry
    s.analytic(3.50, 4, {1.2, 0.9, 3.0}, 6.0, 0.0, 0.0, 0.0, kT0 + 9.0);  // figure eight, future
    s.analytic(4.00, 1, {kNaN, 0.8, kInf});                               // non-finite parameters
    s.analytic(4.50, 77, {1.0, 1.0, 1.0});                                // unknown type
    s.analytic(5.00, 1, {1.0, 0.8, 1.5}, -1.0);                           // default duration
    s.analytic(5.50, 1, {1.0, 0.8, 1.5}, 6.0, 0.0, 0.0, 0.0, kT0 + 5.5 + 0.05);  // inside lead time
    s.sampled(6.00, 120, true);                       // explicit planar kinematics
    s.sampled(6.50, 0, true);                         // empty: rejected
    s.sampled(7.00, 60, false, false);                // non-monotonic time: rejected
    s.sampled(7.50, 80, false);                       // derived kinematics
    s.waypoint(8.00, 3, {2.0, 2.0}, 0.0);             // header stamp unset
    s.waypoint(9.50, 4, {}, 1.5);                     // default segment times, start in the future
    s.waypoint(11.00, 1, {}, 0.0);                    // one waypoint: rejected
    s.waypoint(11.50, 3, {2.0}, 0.0);                 // segment count mismatch: rejected
    s.waypoint(12.00, 3, {2.0, 2.0}, 0.0, 0.6, 0.3);  // limit violations only flag
    s.analytic(14.00, 0, {}, 1.0);                    // short hold: expires back to Ready
    s.analytic(16.00, 1, {1.0, 0.8, 1.5}, 3.0);
    s.reset(17.00);  // Active -> SelfCheck -> Ready
    s.sampled(18.00, 100, true);
    s.reset(19.00);
    s.reset(19.05);                 // two resets in a row
    s.analytic(20.00, 3, {}, 5.0);  // default parameters
    s.waypoint(21.00, 3, {1.5, 1.5}, 0.0);
    s.analytic(22.50, 4, {1.0, 0.7, 1.0}, 4.0);  // replaces a polynomial
    if (config.default_analytic) {
        s.reset(24.00);  // re-arms the default analytic request
    }
}

// Run the scenario against `glue`, writing the trace to `out`.
template <typename Glue>
void run(Glue& glue, const Config& config, FILE* out) {
    Writer w(out);
    std::fprintf(out, "run %s\n", config.name);
    Script<Glue> script(glue);
    buildScript(script, config);
    std::stable_sort(script.items.begin(), script.items.end(),
                     [](const auto& a, const auto& b) { return a.at < b.at; });

    size_t next = 0;
    uint8_t last_state = 0xFFU;
    uint32_t last_flags = 0xFFFFFFFFU;
    std::tuple<int, uint32_t, uint32_t, uint64_t> last_active{-1, 0, 0, 0};
    uint64_t events_written = 0;
    const uint64_t steps = static_cast<uint64_t>(config.end / kDt);
    for (uint64_t k = 0; k <= steps; ++k) {
        const double now = kT0 + static_cast<double>(k) * kDt;
        // ros::spinOnce(): the callbacks for everything received so far.
        while (next < script.items.size() && kT0 + script.items[next].at <= now) {
            auto& item = script.items[next++];
            const double at = kT0 + item.at;
            std::fprintf(out, "%" PRIu64 " in %d\n", k, static_cast<int>(item.kind));
            bool ok = true;
            switch (item.kind) {
                case Kind::kAnalytic:
                    ok = glue.analytic(item.analytic, at);
                    break;
                case Kind::kSampled:
                    ok = glue.sampled(item.sampled, at);
                    break;
                case Kind::kWaypoint:
                    ok = glue.waypoint(item.waypoint, at);
                    break;
                case Kind::kReset:
                    glue.reset(at);
                    break;
            }
            if (!ok) {
                std::fprintf(out, "  rejected\n");
            }
        }
        glue.update(now);
        for (const auto& e : glue.events()) {
            std::fprintf(out, "%" PRIu64 " ev %u ts %016" PRIx64 " seq %" PRIu64 " cat %d src %s",
                         k, static_cast<unsigned>(e.id), bits(e.timestamp), e.sequence,
                         static_cast<int>(e.category), e.source.c_str());
            // The output consumer's message for the event.
            if (e.id == kPublishStatus) {
                w.status(glue.runtime().makeStatus(e.timestamp > 0.0 ? e.timestamp : now));
            } else if (e.id == kPublishActiveAnalytic) {
                w.analytic(glue.runtime().activeAnalyticMessage());
            } else if (e.id == kPublishActivePolynomial) {
                w.polynomial(glue.runtime().activePolynomialMessage());
            } else if (e.id == kPublishActiveSampled) {
                w.sampled(glue.runtime().activeSampledMessage());
            }
            std::fputc('\n', out);
            ++events_written;
        }
        const auto& runtime = glue.runtime();
        if (runtime.currentState() != last_state) {
            std::fprintf(out, "%" PRIu64 " state %u\n", k,
                         static_cast<unsigned>(runtime.currentState()));
            last_state = runtime.currentState();
        }
        if (runtime.flags() != last_flags) {
            std::fprintf(out, "%" PRIu64 " flags %u\n", k, static_cast<unsigned>(runtime.flags()));
            last_flags = runtime.flags();
        }
        const std::tuple<int, uint32_t, uint32_t, uint64_t> active{
            static_cast<int>(runtime.activeType()), runtime.activeTrajectoryId(),
            runtime.activeRevision(), glue.activeStartNs()};
        if (active != last_active) {
            writeEvaluator(w, k, runtime.evaluator());
            last_active = active;
        }
    }
    std::fprintf(out, "end %s events %" PRIu64 "\n", config.name, events_written);
}

}  // namespace reference_replay
