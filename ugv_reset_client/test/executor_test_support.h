#pragma once

// A single vehicle in a plant model for the executor tests: first-order lag on both axes, the
// uncommanded lateral velocity of a skid-steer (vy = -ell * omega), pose delay and noise, and the
// command published at a fixed rate with zero-order hold. The executor is stepped at the
// controller's rate (500 Hz here) as the controller does.

#include <ugv_reset_client/reset_executor.h>

#include <cmath>
#include <functional>
#include <random>
#include <deque>

namespace reset_test {

namespace rc = ugv_reset_client;

struct PlantParams {
    double ell{-0.229};
    double lin_lag{0.12};
    double ang_lag{0.16};
    double pose_delay{0.05};
    double pos_noise{0.0};
    double yaw_noise{0.0};
    double publish_hz{30.0};
    bool unicycle{true};
};

class SoloRig {
   public:
    SoloRig(const rc::ExecutorConfig& config, const rc::Pose& start, const PlantParams& plant,
            unsigned seed = 7)
        : exec_(config), plant_(plant), rng_(seed), q_(start) {
        history_.push_back({0.0, q_});
    }

    rc::ResetExecutor& exec() {
        return exec_;
    }
    const rc::Pose& pose() const {
        return q_;
    }
    double time() const {
        return t_;
    }
    double speed() const {
        return std::hypot(actual_[0], plant_.unicycle ? -plant_.ell * actual_[2] : actual_[1]);
    }

    rc::Measurement measurement(bool healthy = true) {
        rc::Pose p = delayed();
        std::normal_distribution<double> gauss(0.0, 1.0);
        if (plant_.pos_noise > 0.0) {
            p.x += plant_.pos_noise * gauss(rng_);
            p.y += plant_.pos_noise * gauss(rng_);
            p.yaw += plant_.yaw_noise * gauss(rng_);
        }
        rc::Measurement m;
        m.ros_now = t_ + ros_offset_;
        m.wall_now = t_ + wall_offset_;
        m.pose = p;
        m.pose_stamp = t_ - plant_.pose_delay + ros_offset_;
        m.healthy = healthy && !unhealthy_;
        return m;
    }

    // Run for `seconds`; the hook is called every control cycle (2 ms) with the output of step().
    // Returns when the session ended or the time is up.
    bool run(double seconds, const std::function<void(SoloRig&, const rc::StepOutput&)>& hook = {}) {
        const double end = t_ + seconds;
        while (t_ < end) {
            const rc::Measurement m = measurement();
            exec_.observe(m);
            const rc::StepOutput out = exec_.step(m);
            if (out.ended) {
                cmd_ = rc::Command{};
                return true;
            }
            if (t_ + 1e-12 >= next_publish_) {
                next_publish_ = t_ + 1.0 / plant_.publish_hz;
                cmd_ = manual_set_ ? manual_ : out.command;
            }
            if (hook) {
                hook(*this, out);
            }
            integrate(0.002);
        }
        return false;
    }

    void integrate(double dt) {
        if (frozen_) {
            t_ += dt;
            history_.push_back({t_, q_});
            while (history_.size() > 2 && history_[1].first <= t_ - plant_.pose_delay) {
                history_.pop_front();
            }
            return;
        }
        const double cmd[3] = {cmd_.vx, cmd_.vy, cmd_.wz};
        for (int axis = 0; axis < 3; ++axis) {
            const double lag = axis == 2 ? plant_.ang_lag : plant_.lin_lag;
            const double f = lag > 0.0 ? 1.0 - std::exp(-dt / lag) : 1.0;
            actual_[axis] += f * (cmd[axis] - actual_[axis]);
        }
        const double vy = plant_.unicycle ? -plant_.ell * actual_[2] : actual_[1];
        const double c = std::cos(q_.yaw), s = std::sin(q_.yaw);
        q_.x += dt * (c * actual_[0] - s * vy);
        q_.y += dt * (s * actual_[0] + c * vy);
        q_.yaw = rc::wrapAngle(q_.yaw + dt * actual_[2]);
        t_ += dt;
        history_.push_back({t_, q_});
        while (history_.size() > 2 && history_[1].first <= t_ - plant_.pose_delay) {
            history_.pop_front();
        }
    }

    // Drive the plant directly (the turns of Custom1, outside any Reset session).
    void setManualCommand(const rc::Command& command) {
        manual_ = command;
        manual_set_ = true;
    }
    void clearManualCommand() {
        manual_set_ = false;
    }
    // Faults for the tests.
    void setUnhealthy(bool value) {
        unhealthy_ = value;
    }
    void setClockOffsets(double ros, double wall) {
        ros_offset_ = ros;
        wall_offset_ = wall;
    }
    void freezePlant(bool frozen) {
        // A plant that does not move: the command is ignored.
        frozen_ = frozen;
        if (frozen) {
            actual_[0] = actual_[1] = actual_[2] = 0.0;
        }
    }
    void push(double dx, double dy) {
        q_.x += dx;
        q_.y += dy;
    }

   private:
    rc::Pose delayed() const {
        return history_.front().second;
    }

    rc::ResetExecutor exec_;
    PlantParams plant_;
    std::mt19937 rng_;
    rc::Pose q_;
    double actual_[3]{0.0, 0.0, 0.0};
    rc::Command cmd_;
    rc::Command manual_;
    bool manual_set_{false};
    double t_{0.0};
    double next_publish_{0.0};
    double ros_offset_{100.0};
    double wall_offset_{5000.0};
    bool unhealthy_{false};
    bool frozen_{false};
    std::deque<std::pair<double, rc::Pose>> history_;
};

inline rc::Grant goGrant(uint32_t session, uint32_t id, const rc::Pose& start,
                         const std::vector<rc::Point>& via, const rc::Pose& goal, double ell,
                         double s_limit, bool final_move = true, double ttl = 1.0) {
    rc::Grant g;
    g.session = session;
    g.state = rc::GrantState::Go;
    g.grant_id = id;
    g.start = start;
    g.via = via;
    g.goal = goal;
    g.ell_plan = ell;
    g.s_limit = s_limit;
    g.final_move = final_move;
    g.ttl = ttl;
    return g;
}

inline rc::ExecutorConfig scoutConfig() {
    rc::ExecutorConfig cfg;
    cfg.spec.model = rc::Model::Unicycle;
    cfg.spec.footprint = {0.45, 0.40, 0.0, 0.0};
    cfg.spec.max_v = 0.35;
    cfg.spec.max_w = 0.5;
    cfg.spec.acc_v = 0.35;
    cfg.spec.acc_w = 0.6;
    cfg.spec.ell_bound = 0.25;
    return cfg;
}

// The Wheeltec driven as a unicycle: its 0.41 x 0.36 chassis plus the project's 0.14 m margin a side,
// and a small bound on the rotation-centre offset.
inline rc::ExecutorConfig wheeltecConfig() {
    rc::ExecutorConfig cfg = scoutConfig();
    cfg.spec.footprint = {0.345, 0.32, 0.0, 0.0};
    cfg.spec.ell_bound = 0.05;
    return cfg;
}

inline rc::ExecutorConfig omniConfig() {
    rc::ExecutorConfig cfg = scoutConfig();
    cfg.spec.model = rc::Model::Omni;
    cfg.spec.footprint = {0.345, 0.32, 0.0, 0.0};
    cfg.spec.max_vy = 0.35;
    cfg.spec.ell_bound = 0.0;
    return cfg;
}

}  // namespace reset_test
