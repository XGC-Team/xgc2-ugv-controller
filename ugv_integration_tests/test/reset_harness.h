#pragma once

// The closed-loop scenario harness of the Reset design (RESET.md section 7), as a test.
//
// It runs the real product code, nothing else: one ResetCoordinator (planner included), one
// ResetExecutor per vehicle, a message bus with delay, loss and outages between them, and a
// physical plant per vehicle: a first-order lag on every axis, the uncommanded lateral velocity of
// a skid-steer (vy = -ell * omega), a delayed and noisy pose, and the command published at 30 Hz
// with zero-order hold. The Scout's physical chassis (0.62 x 0.52) and the Wheeltec's (0.41 x 0.36)
// are audited against the statics and each other every 10 ms, independently of the planner (its
// footprint is the larger Reset envelope: chassis plus 0.14 m a side).
//
// Scenario file: the TASE scene (7 obstacles) and 235 start/goal sets for four Scouts, from the
// recorded E2E state at the Reset click, uniform random starts, goal permutations on the slot line
// and dense clusters. ugv_reset_safety/test/data/README documents how it was made.

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <deque>
#include <ugv_reset_client/reset_executor.h>
#include <ugv_reset_safety/reset_coordinator.h>
#include <vector>

namespace reset_harness {

namespace rc = ugv_reset_client;
namespace rs = ugv_reset_safety;
using Polygon = std::vector<Eigen::Vector2d>;

inline double wrap(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

struct Scenario {
    std::string name;
    std::vector<Eigen::Vector3d> starts, goals;
};
struct ScenarioFile {
    std::vector<Polygon> obstacles;
    std::vector<Scenario> scenarios;
};

inline ScenarioFile loadScenarios(const std::string& path) {
    std::ifstream in(path);
    ScenarioFile file;
    int n = 0;
    in >> n;
    for (int i = 0; i < n; ++i) {
        int k;
        in >> k;
        Polygon p;
        for (int j = 0; j < k; ++j) {
            double x, y;
            in >> x >> y;
            p.emplace_back(x, y);
        }
        file.obstacles.push_back(p);
    }
    int m = 0;
    in >> m;
    for (int i = 0; i < m; ++i) {
        Scenario s;
        int robots;
        in >> s.name >> robots;
        for (int r = 0; r < robots; ++r) {
            double x, y, t;
            in >> x >> y >> t;
            s.starts.emplace_back(x, y, t);
        }
        for (int r = 0; r < robots; ++r) {
            double x, y, t;
            in >> x >> y >> t;
            s.goals.emplace_back(x, y, t);
        }
        file.scenarios.push_back(s);
    }
    return file;
}

inline Polygon rect(const Eigen::Vector3d& q, double half_length, double half_width) {
    const double c = std::cos(q.z()), s = std::sin(q.z());
    Polygon r;
    for (auto corner : {Eigen::Vector2d(-half_length, -half_width), Eigen::Vector2d(half_length, -half_width),
                        Eigen::Vector2d(half_length, half_width), Eigen::Vector2d(-half_length, half_width)}) {
        r.emplace_back(q.x() + c * corner.x() - s * corner.y(), q.y() + s * corner.x() + c * corner.y());
    }
    return r;
}

inline double segmentDistance(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    const Eigen::Vector2d e = b - a;
    const double t = std::clamp((p - a).dot(e) / e.squaredNorm(), 0.0, 1.0);
    return (p - a - t * e).norm();
}

// Independent SAT clearance of two convex polygons (negative: penetration depth).
inline double clearance(const Polygon& A, const Polygon& B) {
    bool separated = false;
    double penetration = std::numeric_limits<double>::infinity();
    for (const auto* P : {&A, &B}) {
        for (size_t e = 0; e < P->size(); ++e) {
            Eigen::Vector2d t = (*P)[(e + 1) % P->size()] - (*P)[e];
            Eigen::Vector2d n(-t.y(), t.x());
            n.normalize();
            double amin = 1e18, amax = -1e18, bmin = 1e18, bmax = -1e18;
            for (auto& p : A) {
                amin = std::min(amin, n.dot(p));
                amax = std::max(amax, n.dot(p));
            }
            for (auto& p : B) {
                bmin = std::min(bmin, n.dot(p));
                bmax = std::max(bmax, n.dot(p));
            }
            const double overlap = std::min(amax, bmax) - std::max(amin, bmin);
            if (overlap < 0) {
                separated = true;
            }
            penetration = std::min(penetration, overlap);
        }
    }
    if (!separated) {
        return -penetration;
    }
    double gap = 1e18;
    for (auto& p : A) {
        for (size_t e = 0; e < B.size(); ++e) {
            gap = std::min(gap, segmentDistance(p, B[e], B[(e + 1) % B.size()]));
        }
    }
    for (auto& p : B) {
        for (size_t e = 0; e < A.size(); ++e) {
            gap = std::min(gap, segmentDistance(p, A[e], A[(e + 1) % A.size()]));
        }
    }
    return gap;
}

struct Options {
    double horizon{90.0};
    double lag_scale{1.0};          // x the lag of the plant (0.12 s linear, 0.16 s angular)
    double ell{-0.229};             // uncommanded lateral velocity = -ell * omega (xsim sign; Gazebo +)
    bool teach{true};               // the offset was learned during the mission (turns of Custom1)
    double pos_noise{0.0};          // pose noise sigma [m]; yaw sigma = noise / 0.3
    double pose_delay{0.05};
    double bus_delay{0.02};         // one way between a vehicle and the coordinator
    double loss{0.0};               // probability that a bus message is lost
    double outage_start{-1.0};      // coordinator outage window
    double outage_length{0.0};
    bool mecanum{false};
    bool ell_in_planner{true};      // (kept for symmetry: the planner always gets the vehicle's own estimate)
    unsigned seed{7};
};

struct Outcome {
    std::string name;
    bool completed{false};
    double t_last{-1.0};
    bool collision{false};
    double min_clear{1e9};            // physical chassis, statics and pairs
    double min_planning_clear{1e9};   // planning footprints (chassis + margin) between pairs that started clear
    double min_planning_at{-1.0};     // simulation time of that minimum, and the pair
    std::string min_planning_pair;
    std::string details;              // why the vehicles that did not arrive ended: "ugvN OUTCOME reason: detail"
    std::string outcomes;             // one letter per vehicle: A arrived, T timeout, R rejected, I infeasible, X aborted, S stopped
    int reversals{0};
    double mean_speed{0.0};
    long parks{0}, admissions{0}, revocations{0};
    double plan_ms_max{0.0};
    std::vector<double> work_ms;      // planner calls that planned
    double max_perr{0.0}, max_yerr{0.0};
    double max_cross{0.0};
    std::string max_cross_where;      // when and where the tracker was farthest from its route
    bool stalled{false};
};

class Plant {
   public:
    Plant(const Eigen::Vector3d& q, const Options& o, bool mecanum)
        : q_(q), lin_lag_(0.12 * o.lag_scale), ang_lag_(0.16 * o.lag_scale), ell_(o.ell), mecanum_(mecanum) {}
    void step(const rc::Command& cmd, double dt) {
        const double want[3] = {cmd.vx, cmd.vy, cmd.wz};
        for (int axis = 0; axis < 3; ++axis) {
            const double lag = axis == 2 ? ang_lag_ : lin_lag_;
            const double f = lag > 0 ? 1.0 - std::exp(-dt / lag) : 1.0;
            actual_[axis] += f * (want[axis] - actual_[axis]);
        }
        const double vy = mecanum_ ? actual_[1] : -ell_ * actual_[2];
        const double c = std::cos(q_.z()), s = std::sin(q_.z());
        q_.x() += dt * (c * actual_[0] - s * vy);
        q_.y() += dt * (s * actual_[0] + c * vy);
        q_.z() = wrap(q_.z() + dt * actual_[2]);
    }
    const Eigen::Vector3d& pose() const {
        return q_;
    }
    void teleport(const Eigen::Vector3d& q) {
        q_ = q;
        actual_[0] = actual_[1] = actual_[2] = 0.0;
    }

   private:
    Eigen::Vector3d q_;
    double actual_[3]{0.0, 0.0, 0.0};
    double lin_lag_, ang_lag_, ell_;
    bool mecanum_;
};

inline rc::ExecutorConfig executorConfig(bool mecanum) {
    rc::ExecutorConfig cfg;
    cfg.spec.model = mecanum ? rc::Model::Omni : rc::Model::Unicycle;
    cfg.spec.footprint = mecanum ? rc::Footprint{0.345, 0.32, 0.0, 0.0} : rc::Footprint{0.45, 0.40, 0.0, 0.0};
    cfg.spec.max_v = 0.35;
    cfg.spec.max_vy = mecanum ? 0.35 : 0.0;
    cfg.spec.max_w = 0.5;
    cfg.spec.acc_v = 0.35;
    cfg.spec.acc_w = 0.6;
    cfg.spec.ell_bound = mecanum ? 0.0 : 0.25;
    return cfg;
}

inline Outcome run(const ScenarioFile& world, const Scenario& scenario, const Options& o) {
    const size_t n = scenario.starts.size();
    const double kWall = 1000.0;  // wall = simulation time + this
    std::mt19937 rng(o.seed);
    std::normal_distribution<double> gauss(0.0, 1.0);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    const std::pair<double, double> chassis = o.mecanum ? std::make_pair(0.205, 0.18) : std::make_pair(0.31, 0.26);
    const std::pair<double, double> planning = o.mecanum ? std::make_pair(0.345, 0.32) : std::make_pair(0.45, 0.40);

    Outcome out;
    out.name = scenario.name;
    rs::ResetCoordinator coordinator;
    std::vector<rs::ConvexObstacle> statics;
    for (const Polygon& p : world.obstacles) {
        rs::ConvexObstacle obstacle;
        obstacle.id = "obstacle" + std::to_string(statics.size());
        obstacle.vertices = p;
        statics.push_back(obstacle);
    }
    rs::Fence fence;
    fence.enabled = false;  // the TASE scene is open; the vehicles keep to the 12 x 12 m field themselves
    coordinator.setWorld(statics, fence);

    std::vector<Plant> plants;
    std::vector<rc::ResetExecutor> executors;
    std::vector<std::string> names;
    for (size_t i = 0; i < n; ++i) {
        plants.emplace_back(scenario.starts[i], o, o.mecanum);
        executors.emplace_back(executorConfig(o.mecanum));
        names.push_back("ugv" + std::to_string(i + 1));
    }

    // Pose stream: a sample every 20 ms, delayed and noisy; held between samples.
    struct PoseSample {
        rc::Pose pose;
        double stamp{0.0};
    };
    std::vector<std::deque<std::pair<double, Eigen::Vector3d>>> history(n);
    std::vector<PoseSample> pose_sample(n);
    auto measurement = [&](size_t i, double t) {
        rc::Measurement m;
        m.ros_now = t;
        m.wall_now = t + kWall;
        m.pose = pose_sample[i].pose;
        m.pose_stamp = pose_sample[i].stamp;
        m.healthy = true;
        return m;
    };
    auto samplePoses = [&](double t) {
        for (size_t i = 0; i < n; ++i) {
            history[i].emplace_back(t, plants[i].pose());
            while (history[i].size() > 1 && history[i][1].first <= t - o.pose_delay) {
                history[i].pop_front();
            }
            Eigen::Vector3d q = history[i].front().second;
            if (o.pos_noise > 0) {
                q.x() += o.pos_noise * gauss(rng);
                q.y() += o.pos_noise * gauss(rng);
                q.z() += o.pos_noise / 0.3 * gauss(rng);
            }
            pose_sample[i].pose = {q.x(), q.y(), q.z()};
            pose_sample[i].stamp = t - o.pose_delay;
        }
    };

    // The offset was learned during the mission: turns in place, as Custom1 makes them.
    double t = 0.0;
    if (o.teach && !o.mecanum) {
        for (size_t i = 0; i < n; ++i) {
            history[i].clear();
        }
        std::vector<rc::Command> manual(n);
        for (int k = 0; k < 7000; ++k) {  // 14 s at 2 ms
            if (k % 10 == 0) {
                samplePoses(t);
                for (size_t i = 0; i < n; ++i) {
                    executors[i].observe(measurement(i, t));
                }
            }
            const double w = k < 3000 ? 0.4 : (k < 6000 ? -0.4 : 0.0);
            for (size_t i = 0; i < n; ++i) {
                plants[i].step({0.0, 0.0, w}, 0.002);
            }
            t += 0.002;
        }
        for (size_t i = 0; i < n; ++i) {
            plants[i].teleport(scenario.starts[i]);
            history[i].clear();
        }
    } else {
        samplePoses(t);
        for (size_t i = 0; i < n; ++i) {
            executors[i].observe(measurement(i, t));
        }
    }
    // Everything from here on is the Reset: sessions begin at `t0`.
    const double t0 = t;
    for (size_t i = 0; i < n; ++i) {
        history[i].clear();
    }
    samplePoses(t);
    for (size_t i = 0; i < n; ++i) {
        executors[i].observe(measurement(i, t));
        executors[i].begin(1, {scenario.goals[i].x(), scenario.goals[i].y(), scenario.goals[i].z()}, true,
                           measurement(i, t));
    }

    struct Message {
        double at;
        size_t vehicle;
        bool to_coordinator;
        rc::Session session;
        rc::Grant grant;
    };
    std::vector<Message> bus;
    std::vector<rc::Command> published(n), latest(n);
    std::vector<double> next_publish(n, t);
    std::vector<bool> ended(n, false);
    std::vector<rc::Outcome> outcome(n, rc::Outcome::None);
    std::vector<double> end_time(n, -1.0);
    std::vector<double> last_vx(n, 0.0), speed_sum(n, 0.0);
    std::vector<rc::Pose> start_pose(n);
    // Planning-footprint pairs that started clear (the audit of the sweep itself).
    std::vector<std::vector<bool>> pair_ok(n, std::vector<bool>(n, false));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            pair_ok[i][j] = clearance(rect(scenario.starts[i], planning.first, planning.second),
                                      rect(scenario.starts[j], planning.first, planning.second)) >= 0.10;
        }
    }
    const char* trace_name = std::getenv("RESET_SCENARIOS_TRACE");
    const bool trace = trace_name != nullptr && scenario.name == trace_name;
    std::vector<double> next_trace(n, t);
    std::map<std::string, std::string> grant_signature;
    double next_sample = t, next_coordinator = t, next_audit = t, last_step = -1.0;
    bool event = false;
    const double kStep = 0.002;
    const double end = t0 + o.horizon + 1.0;
    for (; t < end; t += kStep) {
        if (t + 1e-9 >= next_sample) {
            next_sample += 0.02;
            samplePoses(t);
            for (size_t i = 0; i < n; ++i) {
                executors[i].observe(measurement(i, t));
            }
        }
        // Delivered messages.
        const bool outage = o.outage_start >= 0 && t - t0 >= o.outage_start && t - t0 < o.outage_start + o.outage_length;
        for (auto it = bus.begin(); it != bus.end();) {
            if (it->at > t + 1e-9) {
                ++it;
                continue;
            }
            if (it->to_coordinator) {
                if (!outage) {
                    coordinator.onSession(names[it->vehicle], it->session, t + kWall);
                    event = true;
                }
            } else if (!ended[it->vehicle]) {
                executors[it->vehicle].onGrant(it->grant, measurement(it->vehicle, t));
            }
            it = bus.erase(it);
        }
        // The coordinator: 10 Hz, and at once (at most every 20 ms) after a session arrived.
        if (!outage && ((event && t - last_step >= 0.02) || t + 1e-9 >= next_coordinator)) {
            if (t + 1e-9 >= next_coordinator) {
                next_coordinator += 0.1;
            }
            event = false;
            last_step = t;
            const auto t_start = std::chrono::steady_clock::now();
            const long before = coordinator.stats().events + coordinator.stats().extensions;
            const auto grants = coordinator.step(t + kWall);
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
            out.plan_ms_max = std::max(out.plan_ms_max, ms);
            if (coordinator.stats().events + coordinator.stats().extensions != before) {
                out.work_ms.push_back(ms);
            }
            for (const rs::OutgoingGrant& g : grants) {
                if (trace) {
                    char sig[160];
                    std::snprintf(sig, sizeof(sig), "%d/%u/%.1f/%d/%d", static_cast<int>(g.grant.state), g.grant.grant_id,
                                  std::min(g.grant.s_limit, 99.0), static_cast<int>(g.grant.reason), static_cast<int>(g.grant.via.size()));
                    if (grant_signature[g.vehicle] != sig) {
                        grant_signature[g.vehicle] = sig;
                        std::printf("  GRANT %5.1f %s state=%d reason=%s id=%u s_limit=%.2f via=%zu final=%d goal=(%.2f,%.2f,%.2f) %s\n", t - t0,
                                    g.vehicle.c_str(), static_cast<int>(g.grant.state), rc::name(g.grant.reason), g.grant.grant_id,
                                    std::min(g.grant.s_limit, 99.0), g.grant.via.size(), g.grant.final_move, g.grant.goal.x,
                                    g.grant.goal.y, g.grant.goal.yaw, g.grant.detail.c_str());
                        if (g.grant.state != rc::GrantState::Go) {
                            std::string who;
                            for (const std::string& b : g.grant.blockers) {
                                who += " " + b;
                            }
                            std::printf("        blockers:%s   poses:", who.c_str());
                            for (size_t k = 0; k < n; ++k) {
                                std::printf(" %s(%.2f,%.2f,%.2f%s)", names[k].c_str(), plants[k].pose().x(), plants[k].pose().y(),
                                            plants[k].pose().z(), ended[k] ? " done" : "");
                            }
                            std::printf("\n        ell:");
                            for (size_t k = 0; k < n; ++k) {
                                const rc::Session ss = executors[k].makeSession(measurement(k, t));
                                std::printf(" %s(hat %.3f spread %.3f phase %d)", names[k].c_str(), ss.ell_hat, ss.ell_spread, static_cast<int>(ss.phase));
                            }
                            std::printf("\n");
                        }
                    }
                }
                if (o.loss > 0 && uniform(rng) < o.loss) {
                    continue;
                }
                for (size_t i = 0; i < n; ++i) {
                    if (names[i] == g.vehicle) {
                        bus.push_back({t + o.bus_delay, i, false, rc::Session(), g.grant});
                    }
                }
            }
        }
        // The vehicles: the controller steps the executor at 500 Hz.
        for (size_t i = 0; i < n; ++i) {
            if (ended[i]) {
                latest[i] = rc::Command{};
                continue;
            }
            const rc::Measurement m = measurement(i, t);
            const rc::StepOutput step = executors[i].step(m);
            if (step.ended) {
                rc::Result result;
                executors[i].takeResult(&result);
                outcome[i] = result.outcome;
                if (result.outcome != rc::Outcome::Arrived) {
                    out.details += " [" + names[i] + " " + rc::name(result.outcome) + " " +
                                   rc::name(result.reason) + ": " + result.detail + "]";
                }
                end_time[i] = t - t0;
                ended[i] = true;
                latest[i] = rc::Command{};
                executors[i].finish();
                // The Session of the vehicle, now Idle, so that the coordinator sees the end.
                rc::Session s = executors[i].makeSession(m);
                bus.push_back({t + o.bus_delay, i, true, s, rc::Grant()});
                if (outcome[i] == rc::Outcome::Arrived) {
                    out.max_perr = std::max(out.max_perr, result.final_planar_error);
                    out.max_yerr = std::max(out.max_yerr, result.final_yaw_error);
                }
                continue;
            }
            latest[i] = step.command;
            if (trace && t + 1e-9 >= next_trace[i]) {
                next_trace[i] = t + 0.5;
                std::printf("  %s %5.1f pose=(%6.2f,%6.2f,%5.2f) cmd=(%5.2f,%5.2f) prog=%5.2f/%5.2f lim=%5.2f ect=%6.3f eh=%5.2f phase=%d ell=%.2f\n",
                            names[i].c_str(), t - t0, plants[i].pose().x(), plants[i].pose().y(), plants[i].pose().z(),
                            step.command.vx, step.command.wz, executors[i].tracker().progress(),
                            executors[i].tracker().route().length(), std::min(executors[i].sLimit(), 99.0),
                            executors[i].tracker().crossTrack(), executors[i].tracker().headingError(),
                            static_cast<int>(step.phase), executors[i].ell().ell());
            }
            if (std::abs(executors[i].tracker().crossTrack()) > out.max_cross) {
                out.max_cross = std::abs(executors[i].tracker().crossTrack());
                char text[200];
                std::snprintf(text, sizeof(text), "%s t=%.1f progress=%.2f/%.2f s_limit=%.2f cmd=(%.2f,%.2f) pose=(%.2f,%.2f,%.2f) ell=%.2f/%.2f",
                              names[i].c_str(), t - t0, executors[i].tracker().progress(),
                              executors[i].tracker().route().length(), executors[i].sLimit(), step.command.vx,
                              step.command.wz, plants[i].pose().x(), plants[i].pose().y(), plants[i].pose().z(),
                              executors[i].ell().ell(), executors[i].ell().spread());
                out.max_cross_where = text;
            }
            if (executors[i].sessionDue(m)) {
                if (!(o.loss > 0 && uniform(rng) < o.loss)) {
                    bus.push_back({t + o.bus_delay, i, true, executors[i].makeSession(m), rc::Grant()});
                }
            }
        }
        // The command is published at 30 Hz with zero-order hold; the plant follows it.
        for (size_t i = 0; i < n; ++i) {
            if (t + 1e-9 >= next_publish[i]) {
                next_publish[i] = t + 1.0 / 30.0;
                published[i] = latest[i];
                if (!ended[i]) {
                    speed_sum[i] += std::abs(published[i].vx) / 30.0;
                    if (last_vx[i] * published[i].vx < -1e-9) {
                        ++out.reversals;
                    }
                    if (std::abs(published[i].vx) > 1e-9) {
                        last_vx[i] = published[i].vx;
                    }
                }
            }
            plants[i].step(published[i], kStep);
        }
        // The physical audit, every 10 ms.
        if (t + 1e-9 >= next_audit) {
            next_audit += 0.01;
            for (size_t i = 0; i < n; ++i) {
                const Polygon body = rect(plants[i].pose(), chassis.first, chassis.second);
                for (const Polygon& obstacle : world.obstacles) {
                    out.min_clear = std::min(out.min_clear, clearance(body, obstacle));
                }
                for (size_t j = i + 1; j < n; ++j) {
                    out.min_clear = std::min(out.min_clear, clearance(body, rect(plants[j].pose(), chassis.first, chassis.second)));
                    if (pair_ok[i][j]) {
                        const double c = clearance(rect(plants[i].pose(), planning.first, planning.second),
                                                   rect(plants[j].pose(), planning.first, planning.second));
                        if (c < out.min_planning_clear) {
                            out.min_planning_clear = c;
                            out.min_planning_at = t - t0;
                            out.min_planning_pair = names[i] + "/" + names[j];
                        }
                    }
                }
            }
            if (out.min_clear < -1e-6) {
                out.collision = true;
                break;
            }
        }
        bool all = true;
        for (size_t i = 0; i < n; ++i) {
            all = all && ended[i];
        }
        if (all) {
            break;
        }
    }
    out.completed = true;
    double mean = 0.0;
    for (size_t i = 0; i < n; ++i) {
        out.completed = out.completed && outcome[i] == rc::Outcome::Arrived;
        out.t_last = std::max(out.t_last, end_time[i]);
        mean += speed_sum[i] / std::max(end_time[i] > 0 ? end_time[i] : o.horizon, 1e-6);
        char letter = 'U';
        switch (outcome[i]) {
            case rc::Outcome::Arrived: letter = 'A'; break;
            case rc::Outcome::Timeout: letter = 'T'; break;
            case rc::Outcome::Stopped: letter = 'S'; break;
            case rc::Outcome::Infeasible: letter = 'I'; break;
            case rc::Outcome::RejectedByCoordinator:
            case rc::Outcome::RejectedNoCoordinator:
            case rc::Outcome::RejectedNoTarget: letter = 'R'; break;
            case rc::Outcome::None: letter = '.'; break;
            default: letter = 'X'; break;
        }
        out.outcomes += letter;
    }
    out.mean_speed = mean / static_cast<double>(n);
    out.parks = coordinator.stats().parks;
    out.admissions = coordinator.stats().admissions;
    out.revocations = coordinator.stats().revocations;
    if (!out.completed) {
        out.t_last = -1.0;
    }
    return out;
}

}  // namespace reset_harness
