#pragma once

// Test support for the planner: an independent separating-axis audit (not the planner's code), the
// scenario files of the TASE scene, and an ideal follower that drives a granted route exactly.
// The ideal follower isolates the planner's soundness from the tracker's accuracy: it stays in the
// route tube by construction, so any collision it sees is the planner's.

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <ugv_reset_client/reset_protocol.h>
#include <ugv_reset_client/reset_route.h>
#include <ugv_reset_safety/reset_planner.h>
#include <vector>

namespace reset_test {

namespace rc = ugv_reset_client;
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

// scenarios file: n obstacles ("k x y x y ..."), m scenarios ("name robots starts... goals...").
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

inline Polygon rect(double x, double y, double yaw, double half_length, double half_width,
                    double offset_x = 0.0, double offset_y = 0.0) {
    const double c = std::cos(yaw), s = std::sin(yaw);
    Polygon r;
    for (auto corner : {Eigen::Vector2d(-half_length, -half_width), Eigen::Vector2d(half_length, -half_width),
                        Eigen::Vector2d(half_length, half_width), Eigen::Vector2d(-half_length, half_width)}) {
        const Eigen::Vector2d b = corner + Eigen::Vector2d(offset_x, offset_y);
        r.emplace_back(x + c * b.x() - s * b.y(), y + s * b.x() + c * b.y());
    }
    return r;
}

inline double segmentDistance(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    const Eigen::Vector2d e = b - a;
    const double t = std::clamp((p - a).dot(e) / e.squaredNorm(), 0.0, 1.0);
    return (p - a - t * e).norm();
}

// Independent clearance of two convex polygons: negative penetration depth if they overlap.
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

// A vehicle that drives the granted route exactly (no tracking error) at a fixed speed, stops at
// the end of the granted prefix, and reports the phase and progress the planner needs.
class IdealFollower {
   public:
    IdealFollower(const std::string& id, const rc::VehicleSpec& spec, const rc::Pose& start,
                  const rc::Pose& target, double ell_true, bool ell_known)
        : id_(id), spec_(spec), pose_(start), target_(target), ell_true_(ell_true),
          ell_known_(ell_known) {}

    void beginSession(uint32_t session) {
        session_ = session;
        ++generation_;
        acknowledged_ = 0;
        certified_ = false;
        phase_ = rc::Phase::Requesting;
        have_route_ = false;
        done_ = false;
    }

    void onDecision(const ugv_reset_safety::PlannerDecision& d) {
        if (d.session != session_ || d.request_generation != generation_) {
            return;
        }
        switch (d.state) {
            case rc::GrantState::Go:
                if (done_) return;
                acknowledged_ = d.grant_id; certified_ = false;
                if (!have_route_ || route_start_.x != d.start.x || route_start_.y != d.start.y ||
                    goal_.x != d.goal.x || goal_.y != d.goal.y || via_.size() != d.via.size()) {
                    startRoute(d);
                }
                s_limit_ = d.s_limit;
                break;
            case rc::GrantState::Brake:
                // This ideal geometry fixture has no inertia: receiving Brake is its
                // explicit zero/stop certificate, not evidence about a real plant.
                have_route_ = false; moving_ = false;
                acknowledged_ = d.grant_id; certified_ = true;
                if (!done_) phase_ = rc::Phase::Requesting;
                break;
            case rc::GrantState::Reject:
                done_ = true;
                rejected_ = d.reason;
                phase_ = rc::Phase::Done;
                break;
            case rc::GrantState::Hold:
                acknowledged_ = d.grant_id; certified_ = false;
                break;
        }
    }

    void onGrant(const rc::Grant& g) {
        ugv_reset_safety::PlannerDecision d;
        d.session = g.session;
        d.request_generation = g.request_generation;
        d.state = g.state;
        d.reason = g.reason;
        d.grant_id = g.grant_id;
        d.start = g.start;
        d.goal = g.goal;
        d.final_move = g.final_move;
        d.ell_plan = g.ell_plan;
        d.via = g.via;
        d.s_limit = g.s_limit;
        onDecision(d);
    }

    // Advance by dt seconds.
    void step(double dt) {
        if (done_ || !have_route_) {
            return;
        }
        double budget = dt;
        bool moved = false;
        while (budget > 1e-12 && index_ + 1 < path_.size()) {
            if (path_[index_ + 1].s > s_limit_ + 1e-9) {
                break;  // the next sample is beyond the granted prefix
            }
            const double need = (1.0 - fraction_) * time_[index_];
            if (budget >= need) {
                budget -= need;
                ++index_;
                fraction_ = 0.0;
                moved = true;
            } else {
                fraction_ += budget / time_[index_];
                budget = 0.0;
                moved = true;
            }
        }
        const auto& a = path_[index_];
        if (index_ + 1 < path_.size()) {
            const auto& b = path_[index_ + 1];
            pose_ = {a.pose.x + (b.pose.x - a.pose.x) * fraction_,
                     a.pose.y + (b.pose.y - a.pose.y) * fraction_,
                     wrap(a.pose.yaw + wrap(b.pose.yaw - a.pose.yaw) * fraction_)};
        } else {
            pose_ = a.pose;
        }
        const bool at_end = index_ + 1 >= path_.size();
        if (at_end) {
            if (final_) {
                phase_ = rc::Phase::Settling;
                settle_ += dt;
                if (settle_ > 0.5) {
                    phase_ = rc::Phase::Done;
                    done_ = true;
                    arrived_ = true;
                    ++generation_; acknowledged_ = 0; certified_ = false;
                    have_route_ = false;
                }
            } else if (have_route_) {
                ++generation_; acknowledged_ = 0; certified_ = false;
                have_route_ = false; phase_ = rc::Phase::Requesting;
            }
        } else {
            phase_ = moved ? rc::Phase::Moving : rc::Phase::Waiting;
        }
        moving_ = moved;
    }

    rc::Session session() const {
        rc::Session s;
        s.session = session_;
        s.request_generation = generation_;
        s.acknowledged_grant_id = acknowledged_;
        s.stop_certified = certified_;
        s.publication_seq = ++publication_;
        s.pose = pose_;
        s.target = target_;
        s.phase = phase_;
        s.spec = spec_;
        s.ell_hat = ell_known_ ? ell_true_ : 0.0;
        s.ell_spread = ell_known_ ? 0.02 : -1.0;
        s.healthy = true;
        return s;
    }
    const std::string& id() const {
        return id_;
    }
    const rc::Pose& pose() const {
        return pose_;
    }
    rc::Phase phase() const {
        return phase_;
    }
    bool done() const {
        return done_;
    }
    bool arrived() const {
        return arrived_;
    }
    bool moving() const {
        return moving_;
    }
    rc::Reason rejection() const {
        return rejected_;
    }
    bool hasMoved() const {
        return has_moved_;
    }

   private:
    void startRoute(const ugv_reset_safety::PlannerDecision& d) {
        route_start_ = d.start;
        goal_ = d.goal;
        via_ = d.via;
        final_ = d.final_move;
        // The vehicle's true rotation-centre offset, not the planned one.
        const rc::Route route(d.start, d.via, d.goal, ell_true_);
        const auto raw = route.sample(0.01, 1.0 * rc::kPi / 180.0, spec_.model == rc::Model::Unicycle);
        path_.assign(raw.begin(), raw.end());
        time_.clear();
        for (size_t i = 0; i + 1 < path_.size(); ++i) {
            const double distance = std::hypot(path_[i + 1].pose.x - path_[i].pose.x,
                                               path_[i + 1].pose.y - path_[i].pose.y);
            const double turn = std::abs(wrap(path_[i + 1].pose.yaw - path_[i].pose.yaw));
            time_.push_back(std::max({distance / 0.3, turn / 0.45, 1e-6}));
        }
        index_ = 0;
        fraction_ = 0.0;
        settle_ = 0.0;
        have_route_ = true;
        has_moved_ = true;
    }

    std::string id_;
    rc::VehicleSpec spec_;
    rc::Pose pose_, target_, route_start_, goal_;
    std::vector<rc::Point> via_;
    double ell_true_;
    bool ell_known_;
    uint32_t session_{0}, generation_{0}, acknowledged_{0};
    mutable uint32_t publication_{0};
    bool certified_{false};
    rc::Phase phase_{rc::Phase::Idle};
    bool have_route_{false}, done_{false}, arrived_{false}, moving_{false}, final_{true},
        has_moved_{false};
    rc::Reason rejected_{rc::Reason::None};
    std::vector<rc::Route::Sample> path_;
    std::vector<double> time_;
    size_t index_{0};
    double fraction_{0.0}, s_limit_{0.0}, settle_{0.0};
};

// Direct geometry cases use a separate, ideal stopped wire endpoint. Each Brake is
// returned with its exact certificate in a later update call, without simulating inertia.
// It is deliberately not used by the causal-retirement tests or the real Executor probe.
class StoppedPlannerFixture : public ugv_reset_safety::ResetPlanner {
    struct Wire { uint32_t session{0}, generation{0}, ack{0}; bool cert{false}; rc::Phase phase{rc::Phase::Idle}; };
    std::map<std::string, Wire> wires_;
   public:
    using ugv_reset_safety::ResetPlanner::ResetPlanner;
    std::vector<ugv_reset_safety::PlannerDecision> update(double now,
            std::vector<ugv_reset_safety::PlannerVehicle> vehicles) {
        std::map<std::string,bool> initial;
        for (auto& v : vehicles) {
            auto& w=wires_[v.id];
            const bool participating=v.phase!=rc::Phase::Idle && v.phase!=rc::Phase::Done;
            const bool was_participating=w.phase!=rc::Phase::Idle && w.phase!=rc::Phase::Done;
            initial[v.id]=w.generation==0;
            if ((participating && w.generation==0) || (w.generation && v.session!=w.session) || (!participating && was_participating))
                { ++w.generation; w.ack=0; w.cert=false; }
            w.session=v.session;w.phase=v.phase;
        }
        std::vector<ugv_reset_safety::PlannerDecision> result;
        for (unsigned turn=0;turn<4;++turn) {
            for(auto& v:vehicles) {const auto& w=wires_.at(v.id);v.request_generation=w.generation;v.acknowledged_grant_id=w.ack;v.stop_certified=w.cert;}
            auto ds=ugv_reset_safety::ResetPlanner::update(now,vehicles);bool barrier=false;
            for(const auto& d:ds) {
                auto& w=wires_.at(d.id);
                if(d.state==rc::GrantState::Brake) {w.ack=d.grant_id;w.cert=true;barrier=true;if(!initial[d.id]) result.push_back(d);}
                else {result.push_back(d);if(d.state==rc::GrantState::Go || d.state==rc::GrantState::Hold) {w.ack=d.grant_id;w.cert=false;}}
            }
            if(!barrier) break;
        }
        return result;
    }
};

// The planner driven by ideal followers, with an independent continuous audit.
struct DriveResult {
    bool all_arrived{false};
    double finished_at{-1.0};
    int rejected{0};
    double min_pair_clearance{1e9};    // planning footprints, pairs that started clear
    double min_static_clearance{1e9};  // planning footprints, pairs that started clear
    double min_chassis_clearance{1e9}; // physical chassis (smaller than the planning footprint)
    double plan_ms_max{0.0};
    std::vector<double> plan_ms;
    std::vector<double> work_ms;  // only the calls that planned (an admission or an extension)
    std::vector<rc::Reason> reasons;
    std::map<std::string, bool> arrived;
};

struct DriveOptions {
    double horizon{90.0};
    double coordinator_period{0.1};
    double plant_dt{0.002};
    double comms_delay{0.0};
    bool ell_known{true};
    double ell_true{-0.229};
    double chassis_half_length{0.31}, chassis_half_width{0.26};
    ugv_reset_client::VehicleSpec spec;
    ugv_reset_safety::PlannerConfig planner;
    bool stop_on_collision{true};
    std::function<void(double, const std::vector<ugv_reset_safety::PlannerDecision>&)> observer;
};

inline DriveResult drive(const ScenarioFile& world, const Scenario& scenario, const DriveOptions& opt,
                         ugv_reset_safety::Fence fence = ugv_reset_safety::Fence()) {
    using ugv_reset_safety::ConvexObstacle;
    ugv_reset_safety::ResetPlanner planner(opt.planner);
    std::vector<ConvexObstacle> statics;
    int id = 0;
    for (const auto& p : world.obstacles) {
        ConvexObstacle o;
        o.id = "obstacle" + std::to_string(id++);
        o.vertices = p;
        statics.push_back(o);
    }
    planner.setWorld(statics, fence);
    const size_t n = scenario.starts.size();
    std::vector<IdealFollower> robots;
    for (size_t i = 0; i < n; ++i) {
        robots.emplace_back("ugv" + std::to_string(i + 1), opt.spec,
                            rc::Pose{scenario.starts[i].x(), scenario.starts[i].y(), scenario.starts[i].z()},
                            rc::Pose{scenario.goals[i].x(), scenario.goals[i].y(), scenario.goals[i].z()},
                            opt.spec.model == rc::Model::Unicycle ? opt.ell_true : 0.0, opt.ell_known);
        robots.back().beginSession(1);
    }
    auto planning = [&](size_t i) {
        const auto& f = opt.spec.footprint;
        const auto& q = robots[i].pose();
        return rect(q.x, q.y, q.yaw, f.half_length, f.half_width, f.offset_x, f.offset_y);
    };
    auto chassis = [&](size_t i) {
        const auto& q = robots[i].pose();
        return rect(q.x, q.y, q.yaw, opt.chassis_half_length, opt.chassis_half_width);
    };
    // Pairs and statics that start inside the margin are excluded from the audit: the planner has
    // to let a vehicle leave such a pose, and a start violation is not the sweep's.
    std::vector<std::vector<bool>> pair_ok(n, std::vector<bool>(n, false));
    std::vector<std::vector<bool>> static_ok(n, std::vector<bool>(statics.size(), false));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            pair_ok[i][j] = clearance(planning(i), planning(j)) >= opt.planner.pair_margin;
        }
        for (size_t k = 0; k < statics.size(); ++k) {
            static_ok[i][k] = clearance(planning(i), statics[k].vertices) >= opt.planner.static_margin;
        }
    }
    DriveResult result;
    std::vector<std::pair<double, std::vector<ugv_reset_safety::PlannerDecision>>> in_flight;
    double next_coordinator = 0.0;
    for (double t = 0.0; t < opt.horizon; t += opt.plant_dt) {
        if (t + 1e-12 >= next_coordinator) {
            next_coordinator += opt.coordinator_period;
            std::vector<ugv_reset_safety::PlannerVehicle> vehicles;
            for (const auto& r : robots) {
                const rc::Session s = r.session();
                ugv_reset_safety::PlannerVehicle v;
                v.id = r.id();
                v.session = s.session;
                v.request_generation = s.request_generation;
                v.acknowledged_grant_id = s.acknowledged_grant_id;
                v.stop_certified = s.stop_certified;
                v.spec = s.spec;
                v.pose = s.pose;
                v.target = s.target;
                v.phase = s.phase;
                v.ell_hat = s.ell_hat;
                v.ell_spread = s.ell_spread;
                vehicles.push_back(v);
            }
            const auto t0 = std::chrono::steady_clock::now();
            const long before = planner.stats().events + planner.stats().extensions;
            auto decisions = planner.update(t, vehicles);
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            if (opt.observer) {
                opt.observer(t, decisions);
            }
            result.plan_ms.push_back(ms);
            if (planner.stats().events + planner.stats().extensions != before) {
                result.work_ms.push_back(ms);
            }
            result.plan_ms_max = std::max(result.plan_ms_max, ms);
            if (!decisions.empty()) {
                in_flight.emplace_back(t + opt.comms_delay, std::move(decisions));
            }
        }
        for (auto it = in_flight.begin(); it != in_flight.end();) {
            if (it->first <= t + 1e-12) {
                for (const auto& d : it->second) {
                    for (auto& r : robots) {
                        if (r.id() == d.id) {
                            r.onDecision(d);
                        }
                    }
                }
                it = in_flight.erase(it);
            } else {
                ++it;
            }
        }
        for (auto& r : robots) {
            r.step(opt.plant_dt);
        }
        // Audit every 10 ms of plant time.
        if (static_cast<long>(std::llround(t / opt.plant_dt)) % 5 == 0) {
            for (size_t i = 0; i < n; ++i) {
                for (size_t j = i + 1; j < n; ++j) {
                    if (pair_ok[i][j] && (robots[i].hasMoved() || robots[j].hasMoved())) {
                        result.min_pair_clearance =
                            std::min(result.min_pair_clearance, clearance(planning(i), planning(j)));
                    }
                    result.min_chassis_clearance =
                        std::min(result.min_chassis_clearance, clearance(chassis(i), chassis(j)));
                }
                for (size_t k = 0; k < statics.size(); ++k) {
                    if (static_ok[i][k] && robots[i].hasMoved()) {
                        result.min_static_clearance = std::min(
                            result.min_static_clearance, clearance(planning(i), statics[k].vertices));
                    }
                    result.min_chassis_clearance =
                        std::min(result.min_chassis_clearance, clearance(chassis(i), statics[k].vertices));
                }
            }
            if (opt.stop_on_collision && result.min_chassis_clearance < -1e-6) {
                break;
            }
        }
        bool all_done = true;
        for (const auto& r : robots) {
            all_done = all_done && r.done();
        }
        if (all_done) {
            result.finished_at = t;
            break;
        }
    }
    result.all_arrived = true;
    for (const auto& r : robots) {
        result.arrived[r.id()] = r.arrived();
        result.all_arrived = result.all_arrived && r.arrived();
        if (r.rejection() != rc::Reason::None) {
            ++result.rejected;
            result.reasons.push_back(r.rejection());
        }
    }
    return result;
}

}  // namespace reset_test
