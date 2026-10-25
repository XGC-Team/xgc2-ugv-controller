#pragma once

// The vehicle's Reset session, without ROS: one object per vehicle that owns its motion, its 90 s
// deadline and its fail-stop. The coordinator only grants permission to move (a frozen route and
// the prefix of it that is free); this executor follows the grant with a RouteTracker and ends the
// session with exactly one typed Result, whatever the coordinator does:
//
//   no target             held for at most 2 s, then REJECTED_NO_TARGET
//   no coordinator        no answer for 2 s, then REJECTED_NO_COORDINATOR
//   heartbeat lost        a grant lapses (ttl, default 1 s) while it was needed:
//   ABORTED_COORDINATOR_LOST deviation / stall     the vehicle leaves its route tube, or may move
//   and does not for 10 s deadline              90 s on the wall clock and on the ROS clock,
//   whichever comes first unhealthy / Stop      the controller ends it (abort()): ABORTED_UNHEALTHY
//   / STOPPED arrival               within 5 cm and 5 deg of the target, measured stopped, held 0.4
//   s
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
#include <limits>
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
    bool pose_stamp_trusted{true};
    bool external_command_zero{true};
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
        const auto& old = cfg_.spec;
        const bool changed =
            old.model != spec.model || old.footprint.half_length != spec.footprint.half_length ||
            old.footprint.half_width != spec.footprint.half_width ||
            old.footprint.offset_x != spec.footprint.offset_x ||
            old.footprint.offset_y != spec.footprint.offset_y || old.max_v != spec.max_v ||
            old.max_vy != spec.max_vy || old.max_w != spec.max_w || old.acc_v != spec.acc_v ||
            old.acc_w != spec.acc_w || old.ell_bound != spec.ell_bound;
        cfg_.spec = spec;
        ell_.setBound(spec.ell_bound);
        if (changed && stage_ != Stage::None && stage_ != Stage::Complete)
            requestAuthority(!active());
    }
    void setTimeout(double seconds) {
        cfg_.timeout = seconds;
    }
    const ExecutorConfig& config() const {
        return cfg_;
    }

    void observe(const Measurement& m) {
        if (have_ros_clock_ && m.ros_now < last_ros_now_ - 1e-9) {
            clock_invalid_ = true;
            stop_certified_ = false;
        }
        have_ros_clock_ = true;
        last_ros_now_ = m.ros_now;
        if (!std::isfinite(m.pose_stamp) || m.pose_stamp <= last_pose_stamp_ ||
            !std::isfinite(m.pose.x) || !std::isfinite(m.pose.y) || !std::isfinite(m.pose.yaw))
            return;
        last_pose_stamp_ = m.pose_stamp;
        // Learning remains always-on; the stop certificate uses a separate collection boundary.
        if (cfg_.spec.model == Model::Unicycle)
            ell_.add(m.pose);
        if (stage_ == Stage::AwaitZero || stage_ == Stage::AwaitBrake)
            return;
        if (zero_latched_ && (clock_invalid_ || !m.pose_stamp_trusted ||
                              m.pose_stamp <= zero_ros_ || m.pose_stamp > m.ros_now + 1e-9))
            return;
        velocity_.add(m.pose_stamp, m.pose);
        velocity_pose_stamp_ = m.pose_stamp;
    }

    bool active() const {
        return phase_ != Phase::Idle && phase_ != Phase::Done;
    }
    Phase phase() const {
        return phase_;
    }
    uint32_t sessionId() const {
        return session_;
    }
    uint32_t requestGeneration() const {
        return generation_;
    }
    const Pose& target() const {
        return target_;
    }
    bool protocolFault() const {
        return protocol_fault_;
    }
    bool permissionPending() const {
        return stage_ != Stage::None && stage_ != Stage::Executing && stage_ != Stage::Complete;
    }
    bool requiresZeroCommand() const {
        return protocol_fault_ || clock_invalid_ || permissionPending();
    }
    bool needsZeroPublication() const {
        return stage_ == Stage::AwaitZero;
    }

    // Two fixed outcome credits: the current result and one preceding uncommitted result.
    // Admission never waits for permission retirement, but it cannot overwrite an accepted S.
    bool reserveBegin() {
        if (begin_reserved_ || protocol_fault_ ||
            generation_ >= std::numeric_limits<uint32_t>::max() - 2 ||
            pendingResults() + (active() ? 1u : 0u) >= 2u)
            return false;
        begin_reserved_ = true;
        return true;
    }
    void cancelBeginReservation() {
        begin_reserved_ = false;
    }
    bool begin(uint32_t session, const Pose& target, bool target_valid, const Measurement& m) {
        if (!begin_reserved_ && !reserveBegin())
            return false;
        if (active())
            end(Outcome::Stopped, Reason::None, "replaced by a new Reset", m);
        begin_reserved_ = false;
        session_ = session;
        target_ = target;
        target_valid_ = target_valid;
        phase_ = Phase::Requesting;
        terminal_request_ = false;
        const double spread = ell_.spread();
        allowance_ =
            cfg_.spec.model == Model::Unicycle ? (spread >= 0 ? spread : cfg_.spec.ell_bound) : 0;
        enter_ros_ = m.ros_now;
        enter_wall_ = m.wall_now;
        last_message_wall_ = m.wall_now;
        heard_ = false;
        end_reported_ = false;
        hold_reason_ = Reason::None;
        hold_detail_.clear();
        hold_blockers_.clear();
        requestAuthority(false);
        return true;
    }
    void abort(Outcome outcome, const std::string& detail, const Measurement& m) {
        if (active())
            end(outcome, Reason::None, detail, m);
    }
    void finish() {
        tracker_.clear();
        command_ = {};
        have_route_ = false;
        terminal_request_ = true;
        if (stage_ == Stage::Executing)
            requestAuthority(true);
        phase_ = Phase::Idle;
    }

    // Called only AFTER the real output publisher returned with all commanded axes zero.
    // Native output is the same owner; the module receipt must carry the exact G/B pair.
    void onZeroPublished(double ros_now, uint32_t generation = 0, uint32_t brake = 0) {
        if (stage_ != Stage::AwaitZero || zero_latched_ || clock_invalid_ || protocol_fault_ ||
            !std::isfinite(ros_now) || !command_.zero() ||
            (generation != 0 && generation != generation_) || (brake != 0 && brake != brake_id_))
            return;
        zero_ros_ = ros_now;
        zero_latched_ = true;
        velocity_.reset();
        velocity_pose_stamp_ = -1;
        stage_ = Stage::AwaitStopped;
    }

    void onGrant(const Grant& g, const Measurement& m) {
        if (protocol_fault_ || stage_ == Stage::None || g.session != session_ ||
            g.request_generation != generation_ || generation_ == 0 || g.grant_id == 0 ||
            !std::isfinite(g.ttl) || g.ttl <= 0 ||
            (have_publication_ && g.publication_seq <= last_publication_))
            return;
        if (g.grant_id < last_decision_id_)
            return;
        if (g.grant_id == last_decision_id_ &&
            (!have_decision_ || !sameDecision(last_decision_, g)))
            return;
        if (g.state == GrantState::Go) {
            if (!validGo(g))
                return;
            if (stage_ == Stage::Executing) {
                if (!have_route_ || !sameRoute(tracker_.route(), g) || g.final_move != final_ ||
                    g.s_limit + 1e-9 < s_limit_)
                    return;
            } else if (stage_ != Stage::AwaitGo || terminal_request_ || !stop_certified_ ||
                       g.grant_id <= brake_id_ ||
                       (pending_go_ && g.grant_id != deferred_go_.grant_id))
                return;
        } else if (g.state == GrantState::Hold) {
            if (stage_ != Stage::Executing &&
                !(stage_ == Stage::AwaitGo && stop_certified_ && g.grant_id > brake_id_) &&
                !(stage_ == Stage::Complete && terminal_request_ && g.grant_id == acknowledged_id_))
                return;
        } else if (g.state != GrantState::Brake && g.state != GrantState::Reject)
            return;
        // All publication, generation, decision and geometry checks precede heartbeat mutation.
        have_publication_ = true;
        last_publication_ = g.publication_seq;
        have_decision_ = true;
        last_decision_id_ = g.grant_id;
        last_decision_ = g;
        last_message_wall_ = m.wall_now;
        heard_ = true;
        hold_reason_ = g.reason;
        hold_detail_ = g.detail;
        hold_blockers_ = g.blockers;
        switch (g.state) {
            case GrantState::Brake:
                if (stage_ == Stage::AwaitBrake) {
                    brake_id_ = g.grant_id;
                    acknowledged_id_ = brake_id_;
                    stage_ = Stage::AwaitZero;
                    command_ = {};
                    tracker_.clear();
                    have_route_ = false;
                } else if (g.grant_id != brake_id_) {
                    // An unsolicited revocation always opens a fresh local authority transaction.
                    requestAuthority(terminal_request_);
                }
                break;
            case GrantState::Go:
                heartbeat_deadline_ = m.wall_now + g.ttl;
                if (stage_ == Stage::Executing) {
                    grant_id_ = g.grant_id;
                    acknowledged_id_ = g.grant_id;
                    s_limit_ = g.s_limit;
                } else {
                    deferred_go_ = g;
                    pending_go_ = true;
                }
                break;
            case GrantState::Hold:
                if (terminal_request_ && stage_ == Stage::AwaitGo) {
                    acknowledged_id_ = g.grant_id;
                    stop_certified_ = false;
                    stage_ = Stage::Complete;
                    zero_latched_ = false;
                }
                break;
            case GrantState::Reject:
                if (active())
                    end(g.reason == Reason::BadSession || g.reason == Reason::SceneChanged ||
                                g.reason == Reason::SceneUnavailable ||
                                g.reason == Reason::NonParticipantMoving || g.reason == Reason::None
                            ? Outcome::RejectedByCoordinator
                            : Outcome::Infeasible,
                        g.reason, g.detail, m);
                break;
        }
    }

    // Non-motion transaction progress also runs in Ready/SelfCheck after the Result/finish.
    void poll(const Measurement& m) {
        if (protocol_fault_) {
            command_ = {};
            return;
        }
        if ((stage_ == Stage::AwaitStopped || stage_ == Stage::AwaitGo) && zero_latched_) {
            stop_certified_ = causalStopped(m);
            if (stop_certified_ && stage_ == Stage::AwaitStopped)
                stage_ = Stage::AwaitGo;
        }
        if (pending_go_ && stage_ == Stage::AwaitGo && stop_certified_ && !terminal_request_ &&
            !clock_invalid_ && causalStopped(m) && m.wall_now <= heartbeat_deadline_) {
            const Grant g = deferred_go_;
            pending_go_ = false;
            const Route route(g.start, g.via, g.goal,
                              cfg_.spec.model == Model::Unicycle ? g.ell_plan : 0);
            tracker_.start(route, cfg_.spec);
            have_route_ = tracker_.active();
            final_ = g.final_move;
            grant_id_ = g.grant_id;
            acknowledged_id_ = g.grant_id;
            s_limit_ = g.s_limit;
            stop_certified_ = false;
            zero_latched_ = false;
            stage_ = Stage::Executing;
            last_step_ros_ = -1;
            stall_since_ = m.ros_now;
            stall_pose_ = m.pose;
            settle_ = 0;
            phase_ = Phase::Waiting;
        }
    }

    StepOutput step(const Measurement& m) {
        poll(m);
        StepOutput out;
        out.phase = phase_;
        if (phase_ == Phase::Done && !end_reported_)
            return finished(m);
        if (!active())
            return out;
        if (protocol_fault_) {
            end(Outcome::RejectedByCoordinator, Reason::BadSession,
                "Reset authority counter exhausted", m);
            return finished(m);
        }
        if (cfg_.timeout > 0 &&
            (m.ros_now - enter_ros_ >= cfg_.timeout || m.wall_now - enter_wall_ >= cfg_.timeout)) {
            end(Outcome::Timeout, Reason::None, "the vehicle's own deadline", m);
            return finished(m);
        }
        if (!m.healthy) {
            command_ = {};
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
        if (m.wall_now - last_message_wall_ >= cfg_.no_coordinator) {
            end(heard_ ? Outcome::AbortedCoordinatorLost : Outcome::RejectedNoCoordinator,
                Reason::None,
                heard_ ? "the coordinator stopped answering" : "no coordinator answered", m);
            return finished(m);
        }
        if (clock_invalid_) {
            if (stage_ == Stage::Executing)
                requestAuthority(false);
            command_ = {};
            return held(m);
        }
        if (stage_ != Stage::Executing || !have_route_) {
            command_ = {};
            return held(m);
        }
        lapsed_ = m.wall_now > heartbeat_deadline_;
        if (lapsed_) {
            requestAuthority(false);
            return held(m);
        }
        const bool due =
            last_step_ros_ < 0 || m.ros_now - last_step_ros_ >= cfg_.control_period - 1e-9;
        if (due) {
            const double dt = last_step_ros_ < 0 ? cfg_.control_period
                                                 : std::clamp(m.ros_now - last_step_ros_, 1e-3, .1);
            last_step_ros_ = m.ros_now;
            command_ = tracker_.step(m.pose, ell_.ell(), velocity_.valid() ? velocity_.speed() : 0,
                                     dt, s_limit_);
            if (!checkTube(m) || !checkProgress(m))
                return finished(m);
            if (checkArrival(m, dt))
                return finished(m);
            const bool complete_prefix =
                RouteTracker::wholeRouteGranted(tracker_.route(), s_limit_);
            const double perr = std::hypot(m.pose.x - target_.x, m.pose.y - target_.y);
            const double yerr = std::abs(wrapAngle(target_.yaw - m.pose.yaw));
            if (complete_prefix && stopped(m) && command_.zero() &&
                ((!final_ && tracker_.atEnd()) ||
                 (final_ && tracker_.inFinalStage() &&
                  (perr > cfg_.position_tolerance || yerr > cfg_.yaw_tolerance)))) {
                requestAuthority(false);
                return held(m);
            }
        }
        out.command = command_;
        out.phase = reportPhase(m);
        phase_ = out.phase;
        return out;
    }

    Session makeSession(const Measurement& m) const {
        Session s;
        s.stamp = m.ros_now;
        s.session = session_;
        s.request_generation = generation_;
        s.acknowledged_grant_id = acknowledged_id_;
        s.stop_certified = stop_certified_;
        s.publication_seq = next_publication_;
        s.pose_stamp = m.pose_stamp;
        s.pose = m.pose;
        s.target = target_;
        s.phase = phase_;
        s.progress = tracker_.active() ? tracker_.progress() : 0;
        s.cross_track = tracker_.active() ? tracker_.crossTrack() : 0;
        s.spec = cfg_.spec;
        s.ell_hat = cfg_.spec.model == Model::Unicycle ? ell_.ell() : 0;
        s.ell_spread = cfg_.spec.model == Model::Unicycle ? ell_.spread() : 0;
        s.healthy = m.healthy;
        return s;
    }
    bool sessionDue(const Measurement& m) const {
        if (protocol_fault_)
            return false;  // no publication/authority counter reuse
        return !published_ || phase_ != published_phase_ || session_ != published_session_ ||
               generation_ != published_generation_ || acknowledged_id_ != published_ack_ ||
               stop_certified_ != published_cert_ ||
               m.wall_now - published_wall_ >= kSessionPeriod - 1e-9;
    }
    void markSessionPublished(const Measurement& m) {
        published_ = true;
        published_phase_ = phase_;
        published_session_ = session_;
        published_generation_ = generation_;
        published_ack_ = acknowledged_id_;
        published_cert_ = stop_certified_;
        published_wall_ = m.wall_now;
        if (next_publication_ == std::numeric_limits<uint32_t>::max())
            protocol_fault_ = true;
        else
            ++next_publication_;
    }
    bool resultPending() const {
        return result_pending_ || previous_pending_;
    }
    bool peekResult(Result* out) const {
        if (!out || !resultPending())
            return false;
        *out = previous_pending_ ? previous_result_ : result_;
        return true;
    }
    void commitResult() {
        if (previous_pending_)
            previous_pending_ = false;
        else
            result_pending_ = false;
    }
    bool takeResult(Result* out) {
        if (!peekResult(out))
            return false;
        commitResult();
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
    enum class Stage { None, AwaitBrake, AwaitZero, AwaitStopped, AwaitGo, Executing, Complete };
    size_t pendingResults() const {
        return (result_pending_ ? 1u : 0u) + (previous_pending_ ? 1u : 0u);
    }
    bool stopped(const Measurement& m) const {
        return m.healthy && m.external_command_zero && command_.zero() && velocity_.valid() &&
               velocity_.speed() <= cfg_.stop_speed &&
               std::abs(velocity_.yawRate()) <= cfg_.stop_yaw_rate;
    }
    bool causalStopped(const Measurement& m) const {
        return !clock_invalid_ && m.pose_stamp_trusted && m.pose_stamp > zero_ros_ &&
               m.pose_stamp <= m.ros_now && m.pose_stamp == velocity_pose_stamp_ && stopped(m);
    }
    void requestAuthority(bool terminal) {
        command_ = {};
        tracker_.clear();
        have_route_ = false;
        pending_go_ = false;
        terminal_request_ = terminal;
        stop_certified_ = false;
        zero_latched_ = false;
        brake_id_ = 0;
        acknowledged_id_ = 0;
        heartbeat_deadline_ = 0;
        stage_ = Stage::AwaitBrake;
        if (generation_ == std::numeric_limits<uint32_t>::max())
            protocol_fault_ = true;
        else
            ++generation_;
        if (!terminal)
            phase_ = Phase::Requesting;
        last_step_ros_ = -1;
        settle_ = 0;
    }
    StepOutput held(const Measurement&) {
        StepOutput out;
        out.phase = permissionPending()
                        ? Phase::Requesting
                        : (phase_ == Phase::Requesting ? Phase::Requesting : Phase::Waiting);
        phase_ = out.phase;
        return out;
    }
    StepOutput finished(const Measurement&) {
        end_reported_ = true;
        StepOutput out;
        out.ended = true;
        out.phase = Phase::Done;
        return out;
    }
    static bool samePose(const Pose& a, const Pose& b) {
        return std::abs(a.x - b.x) <= 1e-9 && std::abs(a.y - b.y) <= 1e-9 &&
               std::abs(wrapAngle(a.yaw - b.yaw)) <= 1e-9;
    }
    bool sameRoute(const Route& r, const Grant& g) const {
        if (!samePose(r.start(), g.start) || !samePose(r.goal(), g.goal) ||
            std::abs(r.ell() - g.ell_plan) > 1e-9 || final_ != g.final_move ||
            r.vertices().size() != g.via.size() + 2)
            return false;
        for (size_t i = 0; i < g.via.size(); ++i)
            if (std::abs(r.vertices()[i + 1].x - g.via[i].x) > 1e-9 ||
                std::abs(r.vertices()[i + 1].y - g.via[i].y) > 1e-9)
                return false;
        return true;
    }
    static bool sameDecision(const Grant& a, const Grant& b) {
        if (a.state != b.state || a.request_generation != b.request_generation ||
            a.session != b.session || !samePose(a.start, b.start) || !samePose(a.goal, b.goal) ||
            a.final_move != b.final_move || a.ell_plan != b.ell_plan || a.s_limit != b.s_limit ||
            a.reason != b.reason || a.ttl != b.ttl || a.detail != b.detail ||
            a.blockers != b.blockers || a.via.size() != b.via.size())
            return false;
        for (size_t i = 0; i < a.via.size(); ++i)
            if (a.via[i].x != b.via[i].x || a.via[i].y != b.via[i].y)
                return false;
        return true;
    }
    bool validGo(const Grant& g) const {
        auto finite = [](const Pose& p) {
            return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
        };
        if (!finite(g.start) || !finite(g.goal) || !std::isfinite(g.ell_plan) ||
            !std::isfinite(g.s_limit) || g.s_limit < 0 || g.via.size() > kMaxVia ||
            (g.final_move && !samePose(g.goal, target_)))
            return false;
        for (const Point& p : g.via)
            if (!std::isfinite(p.x) || !std::isfinite(p.y))
                return false;
        return true;
    }
    bool moving(const Measurement&) const {
        return std::abs(command_.vx) > .005 || std::abs(command_.vy) > .005 ||
               std::abs(command_.wz) > .005 ||
               (velocity_.valid() && (velocity_.speed() > cfg_.stop_speed * 1.5 ||
                                      std::abs(velocity_.yawRate()) > cfg_.stop_yaw_rate * 1.5));
    }
    Phase reportPhase(const Measurement& m) const {
        if (!have_route_)
            return Phase::Requesting;
        if (final_ && tracker_.atEnd())
            return Phase::Settling;
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
        const bool may_move = !lapsed_ && s_limit_ - tracker_.progress() > 0.1;
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
        if (!active())
            return;
        if (result_pending_) {
            previous_result_ = result_;
            previous_pending_ = true;
        }
        result_.session = session_;
        result_.outcome = outcome;
        result_.reason = reason;
        result_.detail = detail;
        result_.final_planar_error = std::hypot(m.pose.x - target_.x, m.pose.y - target_.y);
        result_.final_yaw_error = std::abs(wrapAngle(target_.yaw - m.pose.yaw));
        result_.duration = m.ros_now - enter_ros_;
        result_pending_ = true;
        end_reported_ = false;
        phase_ = Phase::Done;
        requestAuthority(true);
    }
    ExecutorConfig cfg_;
    VelocityEstimator velocity_;
    EllEstimator ell_;
    RouteTracker tracker_;
    Command command_;
    uint32_t session_{0}, generation_{0}, grant_id_{0}, brake_id_{0}, acknowledged_id_{0};
    Pose target_;
    bool target_valid_{false};
    Phase phase_{Phase::Idle};
    Stage stage_{Stage::None};
    bool terminal_request_{false}, stop_certified_{false}, zero_latched_{false},
        protocol_fault_{false};
    double velocity_pose_stamp_{-1};
    double zero_ros_{0}, allowance_{0}, enter_ros_{0}, enter_wall_{0}, last_message_wall_{0};
    bool heard_{false}, have_route_{false}, final_{true}, lapsed_{false};
    double s_limit_{0}, heartbeat_deadline_{0}, last_step_ros_{-1}, settle_{0}, stall_since_{0};
    Pose stall_pose_;
    uint32_t last_decision_id_{0}, last_publication_{0};
    bool have_decision_{false}, have_publication_{false};
    Grant last_decision_, deferred_go_;
    bool pending_go_{false};
    double last_pose_stamp_{-std::numeric_limits<double>::infinity()}, last_ros_now_{0};
    bool have_ros_clock_{false}, clock_invalid_{false};
    Result result_, previous_result_;
    bool result_pending_{false}, previous_pending_{false}, begin_reserved_{false},
        end_reported_{false};
    Reason hold_reason_{Reason::None};
    std::string hold_detail_;
    std::vector<std::string> hold_blockers_;
    bool published_{false}, published_cert_{false};
    Phase published_phase_{Phase::Idle};
    uint32_t published_session_{0}, published_generation_{0}, published_ack_{0},
        next_publication_{0};
    double published_wall_{0};
};

}  // namespace ugv_reset_client
