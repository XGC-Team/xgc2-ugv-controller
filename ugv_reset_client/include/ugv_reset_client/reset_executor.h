#pragma once

// The vehicle's Reset session, without ROS: one object per vehicle that owns its motion, its 90 s
// deadline and its fail-stop. The coordinator only grants permission to move (a frozen route and
// the prefix of it that is free); this executor follows the grant with a RouteTracker and ends the
// session with exactly one typed Result, whatever the coordinator does:
//
//   no target             held for at most 2 s, then REJECTED_NO_TARGET
//   no coordinator        no answer for 2 s, then REJECTED_NO_COORDINATOR
//   heartbeat lost        a grant lapses (ttl, default 1 s) while it was needed: ABORTED_COORDINATOR_LOST
//   deviation / stall     the vehicle leaves its route tube, or may move and does not for 10 s
//   deadline              90 s on the wall clock and on the ROS clock, whichever comes first
//   unhealthy / Stop      the controller ends it (abort()): ABORTED_UNHEALTHY / STOPPED
//   arrival               within 5 cm and 5 deg of the target, measured stopped, held 0.4 s
//
// The controller keeps the CONTROL state machine (T11: Reset leaves only by arrival, timeout,
// rejection or Stop, always to Ready with an exact zero command) and the cmd_vel output rates
// (30 Hz drive, 5 Hz idle); this class decides the twist and the outcome. All times are passed in
// (ROS time and a monotonic wall clock), so the contract is testable and a paused simulation
// clock can not renew anything.

#include <ugv_reset_client/reset_protocol.h>
#include <ugv_reset_client/reset_route.h>
#include <ugv_reset_client/reset_tracker.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace ugv_reset_client {

struct ExecutorConfig {
    VehicleSpec spec;
    double timeout{kResetTimeout};
    double no_target_hold{2.0};
    double no_coordinator{kNoCoordinatorTimeout};
    double stall{10.0};
    double control_period{0.02};  // the tracker runs at 50 Hz
    // The route tube: the vehicle stops and reports ABORTED_DEVIATION when its rotation centre is
    // farther than this from the segment line (the planner's pair margin; the tracker stays within
    // a few centimetres), plus twice the uncertainty of its own rotation-centre estimate: a turn
    // in place swings the origin about a centre that is up to that far from where the tracker
    // believes it to be, and after the turn the rotation centre is off the next segment by up to
    // twice that (until the offset is learned the uncertainty is ell_bound).
    double tube_cross_track{0.10};
    // Arrival: stricter than the public gate (5 cm, 5 deg, 0.03 m/s, 0.05 rad/s) for margin.
    double position_tolerance{0.04};
    double yaw_tolerance{4.0 * kPi / 180.0};
    double stop_speed{0.02};
    double stop_yaw_rate{0.04};
    double settle_hold{0.4};
};

struct Measurement {
    double ros_now{0.0};
    double wall_now{0.0};
    Pose pose;
    double pose_stamp{0.0};
    bool healthy{false};
};

struct StepOutput {
    Command command;
    bool ended{false};
    Phase phase{Phase::Idle};
};

class ResetExecutor {
   public:
    explicit ResetExecutor(const ExecutorConfig& config = ExecutorConfig()) : cfg_(config) {
        ell_.setBound(cfg_.spec.ell_bound);
    }

    void setSpec(const VehicleSpec& spec) {
        cfg_.spec = spec;
        ell_.setBound(spec.ell_bound);
    }
    const ExecutorConfig& config() const {
        return cfg_;
    }

    // ---- always on: called every control cycle in every CONTROL state ----------------------------

    void observe(const Measurement& m) {
        if (m.pose_stamp > last_pose_stamp_ && std::isfinite(m.pose.x) && std::isfinite(m.pose.y) &&
            std::isfinite(m.pose.yaw)) {
            last_pose_stamp_ = m.pose_stamp;
            velocity_.add(m.pose_stamp, m.pose);
            if (cfg_.spec.model == Model::Unicycle) {
                ell_.add(m.pose);
            }
        }
    }

    // ---- the session ----------------------------------------------------------------------------

    bool active() const {
        return phase_ != Phase::Idle && phase_ != Phase::Done;
    }
    Phase phase() const {
        return phase_;
    }
    uint32_t sessionId() const {
        return session_;
    }
    const Pose& target() const {
        return target_;
    }

    // Entering Reset. A session with a missing target is held for no_target_hold seconds so that
    // the operator sees the typed reason, then ends REJECTED_NO_TARGET.
    void begin(uint32_t session, const Pose& target, bool target_valid, const Measurement& m) {
        session_ = session;
        target_ = target;
        target_valid_ = target_valid;
        phase_ = Phase::Requesting;
        // How far the rotation centre may be from where the tracker believes it to be during this
        // session: what is known when it starts (learning during the session does not undo a
        // displacement that already happened).
        const double spread = ell_.spread();
        allowance_ = cfg_.spec.model == Model::Unicycle ? (spread >= 0.0 ? spread : cfg_.spec.ell_bound) : 0.0;
        enter_ros_ = m.ros_now;
        enter_wall_ = m.wall_now;
        last_message_wall_ = m.wall_now;
        heard_ = false;
        have_route_ = false;
        final_ = true;
        s_limit_ = 0.0;
        grant_id_ = 0;
        heartbeat_deadline_ = 0.0;
        tracker_.clear();
        command_ = Command{};
        last_step_ros_ = -1.0;
        settle_ = 0.0;
        stall_since_ = m.ros_now;
        stall_pose_ = m.pose;
        result_ = Result{};
        result_.session = session;
        result_pending_ = false;
        hold_reason_ = Reason::None;
        hold_detail_.clear();
        hold_blockers_.clear();
    }

    // The controller leaves Reset (Stop, or the health monitor): the session ends with the outcome
    // it names, unless it already ended by itself.
    void abort(Outcome outcome, const std::string& detail, const Measurement& m) {
        if (active()) {
            end(outcome, Reason::None, detail, m);
        }
    }

    // After the controller left Reset.
    void finish() {
        tracker_.clear();
        command_ = Command{};
        have_route_ = false;
        if (phase_ != Phase::Idle) {
            phase_ = Phase::Idle;
        }
    }

    // ---- the coordinator -------------------------------------------------------------------------

    void onGrant(const Grant& g, const Measurement& m) {
        if (!active() || g.session != session_) {
            return;
        }
        last_message_wall_ = m.wall_now;
        heard_ = true;
        switch (g.state) {
            case GrantState::Hold:
                hold_reason_ = g.reason;
                hold_detail_ = g.detail;
                hold_blockers_ = g.blockers;
                if (have_route_ && !final_ && tracker_.atEnd()) {
                    // The parking move is done and the coordinator has no new route yet.
                    have_route_ = false;
                    tracker_.clear();
                    command_ = Command{};
                }
                if (!have_route_ && phase_ == Phase::Requesting) {
                    phase_ = Phase::Waiting;
                }
                break;
            case GrantState::Brake:
                have_route_ = false;
                tracker_.clear();
                command_ = Command{};
                hold_reason_ = g.reason;
                hold_detail_ = g.detail;
                hold_blockers_ = g.blockers;
                if (phase_ != Phase::Settling) {
                    phase_ = Phase::Waiting;
                }
                break;
            case GrantState::Reject:
                hold_blockers_ = g.blockers;
                end(g.reason == Reason::BadSession || g.reason == Reason::SceneChanged ||
                            g.reason == Reason::SceneUnavailable ||
                            g.reason == Reason::NonParticipantMoving || g.reason == Reason::None
                        ? Outcome::RejectedByCoordinator
                        : Outcome::Infeasible,
                    g.reason, g.detail, m);
                break;
            case GrantState::Go:
                acceptGo(g, m);
                break;
        }
    }

    // ---- one control step ------------------------------------------------------------------------

    StepOutput step(const Measurement& m) {
        StepOutput out;
        out.phase = phase_;
        if (!active()) {
            return out;
        }
        // The deadline first: 90 s on the ROS clock and on the wall clock.
        if (cfg_.timeout > 0.0 &&
            (m.ros_now - enter_ros_ >= cfg_.timeout || m.wall_now - enter_wall_ >= cfg_.timeout)) {
            end(Outcome::Timeout, Reason::None, "the vehicle's own deadline", m);
            return finished(m);
        }
        if (!m.healthy) {
            // The health monitor takes the controller to SelfCheck; until it does, stand still.
            command_ = Command{};
            tracker_.clear();
            return held(m);
        }
        if (!target_valid_) {
            if (m.wall_now - enter_wall_ >= cfg_.no_target_hold) {
                end(Outcome::RejectedNoTarget, Reason::None,
                    "no target: reset_pose cache and reset_initial_* missing", m);
                return finished(m);
            }
            return held(m);
        }
        // Silence: nothing from the coordinator for this session.
        if (m.wall_now - last_message_wall_ >= cfg_.no_coordinator) {
            end(heard_ ? Outcome::AbortedCoordinatorLost : Outcome::RejectedNoCoordinator, Reason::None,
                heard_ ? "the coordinator stopped answering" : "no coordinator answered", m);
            return finished(m);
        }
        if (have_route_ && m.wall_now > heartbeat_deadline_) {
            // The grant lapsed: whatever the vehicle was driving is no longer permitted.
            end(Outcome::AbortedCoordinatorLost, Reason::None, "the grant lapsed (no heartbeat)", m);
            return finished(m);
        }
        if (!have_route_) {
            command_ = Command{};
            tracker_.clear();
            return held(m);
        }
        // The tracker runs at its own rate; between steps the last command stands.
        const bool due = last_step_ros_ < 0.0 || m.ros_now - last_step_ros_ >= cfg_.control_period - 1e-9;
        if (due) {
            const double dt = last_step_ros_ < 0.0 ? cfg_.control_period
                                                   : std::clamp(m.ros_now - last_step_ros_, 1e-3, 0.1);
            last_step_ros_ = m.ros_now;
            const double v = velocity_.valid() ? velocity_.speed() : 0.0;
            command_ = tracker_.step(m.pose, ell_.ell(), v, dt, s_limit_);
            if (!checkTube(m)) {
                return finished(m);
            }
            if (!checkProgress(m)) {
                return finished(m);
            }
            if (checkArrival(m, dt)) {
                return finished(m);
            }
        }
        out.command = command_;
        out.phase = reportPhase(m);
        phase_ = out.phase;
        return out;
    }

    // ---- reports ----------------------------------------------------------------------------------

    // The Session message of the vehicle.
    Session makeSession(const Measurement& m) const {
        Session s;
        s.stamp = m.ros_now;
        s.session = session_;
        s.pose_stamp = m.pose_stamp;
        s.pose = m.pose;
        s.target = target_;
        s.phase = phase_;
        s.progress = tracker_.active() ? tracker_.progress() : 0.0;
        s.cross_track = tracker_.active() ? tracker_.crossTrack() : 0.0;
        s.spec = cfg_.spec;
        s.ell_hat = cfg_.spec.model == Model::Unicycle ? ell_.ell() : 0.0;
        s.ell_spread = cfg_.spec.model == Model::Unicycle ? ell_.spread() : 0.0;
        s.healthy = m.healthy;
        return s;
    }

    // True when a Session message is due: on every change of phase or session, else every 0.2 s.
    bool sessionDue(const Measurement& m) {
        const bool changed = phase_ != published_phase_ || session_ != published_session_;
        if (changed || m.wall_now - published_wall_ >= kSessionPeriod - 1e-9) {
            published_phase_ = phase_;
            published_session_ = session_;
            published_wall_ = m.wall_now;
            return true;
        }
        return false;
    }

    bool resultPending() const {
        return result_pending_;
    }
    // The outcome of the session that ended, once.
    bool takeResult(Result* out) {
        if (!result_pending_) {
            return false;
        }
        *out = result_;
        result_pending_ = false;
        return true;
    }
    const Result& result() const {
        return result_;
    }
    Reason holdReason() const {
        return hold_reason_;
    }
    const std::string& holdDetail() const {
        return hold_detail_;
    }
    const std::vector<std::string>& holdBlockers() const {
        return hold_blockers_;
    }
    const EllEstimator& ell() const {
        return ell_;
    }
    const VelocityEstimator& velocity() const {
        return velocity_;
    }
    const RouteTracker& tracker() const {
        return tracker_;
    }
    double sLimit() const {
        return s_limit_;
    }
    Command command() const {
        return command_;
    }

   private:
    StepOutput held(const Measurement&) {
        StepOutput out;
        out.phase = phase_ == Phase::Requesting ? Phase::Requesting : Phase::Waiting;
        phase_ = out.phase;
        return out;
    }

    StepOutput finished(const Measurement&) {
        StepOutput out;
        out.ended = true;
        out.phase = Phase::Done;
        return out;
    }

    static bool sameRoute(const Route& route, const Grant& g) {
        const auto& v = route.vertices();
        if (v.size() != g.via.size() + 2 || std::abs(route.start().x - g.start.x) > 1e-9 ||
            std::abs(route.start().y - g.start.y) > 1e-9 || std::abs(route.goal().x - g.goal.x) > 1e-9 ||
            std::abs(route.goal().y - g.goal.y) > 1e-9) {
            return false;
        }
        for (size_t i = 0; i < g.via.size(); ++i) {
            if (std::abs(v[i + 1].x - g.via[i].x) > 1e-9 || std::abs(v[i + 1].y - g.via[i].y) > 1e-9) {
                return false;
            }
        }
        return true;
    }

    void acceptGo(const Grant& g, const Measurement& m) {
        if (!std::isfinite(g.s_limit) || !std::isfinite(g.start.x) || !std::isfinite(g.goal.x) ||
            !std::isfinite(g.ell_plan)) {
            return;
        }
        if (have_route_ && sameRoute(tracker_.route(), g)) {
            if (g.grant_id >= grant_id_) {
                grant_id_ = g.grant_id;
                s_limit_ = std::max(s_limit_, g.s_limit);
            }
            heartbeat_deadline_ = m.wall_now + g.ttl;
            return;
        }
        // A different route is taken only from standstill: a stale or reordered message must not
        // redirect a moving vehicle (the coordinator brakes it first).
        const bool still = !(velocity_.valid() && velocity_.speed() > 0.05) && command_.zero();
        if (have_route_ && !still) {
            return;
        }
        const Route route(g.start, g.via, g.goal, cfg_.spec.model == Model::Unicycle ? g.ell_plan : 0.0);
        tracker_.start(route, cfg_.spec);
        have_route_ = tracker_.active();
        final_ = g.final_move;
        grant_id_ = g.grant_id;
        s_limit_ = g.s_limit;
        heartbeat_deadline_ = m.wall_now + g.ttl;
        stall_since_ = m.ros_now;
        stall_pose_ = m.pose;
        settle_ = 0.0;
        hold_reason_ = Reason::None;
        hold_detail_.clear();
        hold_blockers_.clear();
        if (phase_ == Phase::Requesting) {
            phase_ = Phase::Waiting;
        }
    }

    bool moving(const Measurement&) const {
        return !command_.zero() || (velocity_.valid() && (velocity_.speed() > cfg_.stop_speed * 1.5 ||
                                                          std::abs(velocity_.yawRate()) > cfg_.stop_yaw_rate * 1.5));
    }

    Phase reportPhase(const Measurement& m) const {
        if (!have_route_) {
            return phase_ == Phase::Requesting ? Phase::Requesting : Phase::Waiting;
        }
        if (final_ && tracker_.atEnd()) {
            return Phase::Settling;
        }
        return moving(m) ? Phase::Moving : Phase::Waiting;
    }

    bool checkTube(const Measurement& m) {
        if (std::abs(tracker_.crossTrack()) <= cfg_.tube_cross_track + 2.0 * allowance_) {
            return true;
        }
        char text[96];
        std::snprintf(text, sizeof(text), "left the route tube: cross-track %.2f m",
                      tracker_.crossTrack());
        end(Outcome::AbortedDeviation, Reason::None, text, m);
        return false;
    }

    // A vehicle that may move and does not, for `stall` seconds.
    bool checkProgress(const Measurement& m) {
        const bool may_move = s_limit_ - tracker_.progress() > 0.1;
        const double moved = std::hypot(m.pose.x - stall_pose_.x, m.pose.y - stall_pose_.y) +
                             0.3 * std::abs(wrapAngle(m.pose.yaw - stall_pose_.yaw));
        if (!may_move || moved > 0.03 || !tracker_.active()) {
            stall_since_ = m.ros_now;
            stall_pose_ = m.pose;
            return true;
        }
        if (m.ros_now - stall_since_ < cfg_.stall) {
            return true;
        }
        end(Outcome::AbortedStalled, Reason::None, "allowed to move but did not for 10 s", m);
        return false;
    }

    bool checkArrival(const Measurement& m, double dt) {
        if (!final_ || !tracker_.active()) {
            return false;
        }
        const double perr = std::hypot(m.pose.x - target_.x, m.pose.y - target_.y);
        const double yerr = std::abs(wrapAngle(target_.yaw - m.pose.yaw));
        const bool stopped = velocity_.valid() && velocity_.speed() <= cfg_.stop_speed &&
                             std::abs(velocity_.yawRate()) <= cfg_.stop_yaw_rate;
        if (perr <= cfg_.position_tolerance && yerr <= cfg_.yaw_tolerance && stopped) {
            settle_ += dt;
        } else {
            settle_ = 0.0;
        }
        if (settle_ >= cfg_.settle_hold) {
            end(Outcome::Arrived, Reason::None, "", m);
            return true;
        }
        return false;
    }

    void end(Outcome outcome, Reason reason, const std::string& detail, const Measurement& m) {
        result_.session = session_;
        result_.outcome = outcome;
        result_.reason = reason;
        result_.detail = detail;
        result_.final_planar_error = std::hypot(m.pose.x - target_.x, m.pose.y - target_.y);
        result_.final_yaw_error = std::abs(wrapAngle(target_.yaw - m.pose.yaw));
        result_.duration = m.ros_now - enter_ros_;
        result_pending_ = true;
        phase_ = Phase::Done;
        tracker_.clear();
        have_route_ = false;
        command_ = Command{};
    }

    ExecutorConfig cfg_;
    VelocityEstimator velocity_;
    EllEstimator ell_;
    RouteTracker tracker_;
    Command command_;

    uint32_t session_{0};
    Pose target_;
    bool target_valid_{false};
    Phase phase_{Phase::Idle};
    double enter_ros_{0.0}, enter_wall_{0.0};
    double last_message_wall_{0.0};
    bool heard_{false};
    bool have_route_{false};
    bool final_{true};
    uint32_t grant_id_{0};
    double s_limit_{0.0};
    double heartbeat_deadline_{0.0};
    double last_step_ros_{-1.0};
    double last_pose_stamp_{-1.0};
    double settle_{0.0};
    double stall_since_{0.0};
    double allowance_{0.0};
    Pose stall_pose_;
    Result result_;
    bool result_pending_{false};
    Reason hold_reason_{Reason::None};
    std::string hold_detail_;
    std::vector<std::string> hold_blockers_;
    Phase published_phase_{Phase::Idle};
    uint32_t published_session_{0};
    double published_wall_{-1e9};
};

}  // namespace ugv_reset_client
