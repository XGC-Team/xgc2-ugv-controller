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
#include <variant>
#include <memory>
#include <optional>
#include <type_traits>
#include <ugv_reset_client/schedule.h>

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
    struct None {};
    struct AwaitBrake {};
    struct AwaitZero {};
    struct AwaitStopped { double zero_ros; };
    struct DeferredGo { Grant grant; double deadline; };
    struct AwaitGo {
        double zero_ros;
        bool certified;
        std::optional<DeferredGo> deferred;
    };
    struct Executing {
        RouteTracker tracker;
        Command command;
        bool final;
        uint32_t grant_id;
        double s_limit, heartbeat_deadline;
        double last_step_ros{-1}, settle{0}, stall_since;
        Pose stall_pose;
        Executing(const Grant& g, const VehicleSpec& spec, double deadline, const Measurement& m)
            : final(g.final_move), grant_id(g.grant_id), s_limit(g.s_limit),
              heartbeat_deadline(deadline), stall_since(m.ros_now), stall_pose(m.pose) {
            tracker.start(Route(g.start, g.via, g.goal, spec.model == Model::Unicycle ? g.ell_plan : 0), spec);
        }
    };
    struct Complete {};
    using Permission = std::variant<None, AwaitBrake, AwaitZero, AwaitStopped, AwaitGo, Executing, Complete>;
    struct SessionHeader { uint32_t session; Pose target; };
    struct Requesting {};
    struct Waiting {};
    struct Moving {};
    struct Settling {};
    struct Done { bool reported{false}; };
    using Progress = std::variant<Requesting, Waiting, Moving, Settling, Done>;

   public:
    // The CONTROL Reset alternative owns this value. The executor only borrows it on the owner.
    // Retirement moves its non-motion authority transaction into the persistent context.
    struct Run {
        ~Run();
        Run(const Run&) = delete;
        Run& operator=(const Run&) = delete;
        std::string last_hold_logged;
        double enteredDomain() const { return enter_ros; }

       private:
        friend class ResetExecutor;
        Run(SessionHeader h, bool valid, double bound, const Measurement& m)
            : header(std::move(h)), target_valid(valid), allowance(bound), enter_ros(m.ros_now),
              enter_wall(m.wall_now), last_message_wall(m.wall_now) {}
        ResetExecutor* owner{nullptr};
        SessionHeader header;
        Permission permission;
        Progress progress;
        bool target_valid;
        double allowance, enter_ros, enter_wall, last_message_wall;
        bool heard{false};
        Reason hold_reason{Reason::None};
        std::string hold_detail;
        std::vector<std::string> hold_blockers;
    };
    using RunPtr = std::unique_ptr<Run>;

    explicit ResetExecutor(const ExecutorConfig& config = ExecutorConfig()) : cfg_(config) {
        ell_.setBound(cfg_.spec.ell_bound);
    }
    ~ResetExecutor() { if (auto* r = run()) r->owner = nullptr; }
    ResetExecutor(const ResetExecutor&) = delete;
    ResetExecutor& operator=(const ResetExecutor&) = delete;

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
        if (changed && !is<None>() && !is<Complete>()) requestAuthority(!active());
    }
    void setTimeout(double seconds) { cfg_.timeout = seconds; }
    const ExecutorConfig& config() const { return cfg_; }

    void observe(const Measurement& m) {
        if (have_ros_clock_ && m.ros_now < last_ros_now_ - 1e-9) {
            clock_invalid_ = true;
            if (auto* go = std::get_if<AwaitGo>(&permission())) go->certified = false;
        }
        have_ros_clock_ = true;
        last_ros_now_ = m.ros_now;
        if (!std::isfinite(m.pose_stamp) || m.pose_stamp <= last_pose_stamp_ ||
            !std::isfinite(m.pose.x) || !std::isfinite(m.pose.y) || !std::isfinite(m.pose.yaw)) return;
        last_pose_stamp_ = m.pose_stamp;
        if (cfg_.spec.model == Model::Unicycle) ell_.add(m.pose);
        if (is<AwaitZero>() || is<AwaitBrake>()) return;
        const auto zero = zeroSince();
        if (zero && (clock_invalid_ || !m.pose_stamp_trusted || m.pose_stamp <= *zero ||
                     m.pose_stamp > m.ros_now + 1e-9)) return;
        velocity_.add(m.pose_stamp, m.pose);
        velocity_pose_stamp_ = m.pose_stamp;
    }

    bool active() const { return run() && !std::holds_alternative<Done>(run()->progress); }
    Phase phase() const {
        const auto* r = run();
        if (!r) return Phase::Idle;
        return std::visit([](const auto& progress) {
            using T = std::decay_t<decltype(progress)>;
            if constexpr (std::is_same_v<T, Requesting>) return Phase::Requesting;
            else if constexpr (std::is_same_v<T, Waiting>) return Phase::Waiting;
            else if constexpr (std::is_same_v<T, Moving>) return Phase::Moving;
            else if constexpr (std::is_same_v<T, Settling>) return Phase::Settling;
            else return Phase::Done;
        }, r->progress);
    }
    uint32_t sessionId() const { return header().session; }
    uint32_t requestGeneration() const { return generation_; }
    const Pose& target() const { return header().target; }
    bool protocolFault() const { return protocol_fault_; }
    bool permissionPending() const { return !is<None>() && !is<Executing>() && !is<Complete>(); }
    bool requiresZeroCommand() const { return protocol_fault_ || clock_invalid_ || permissionPending(); }
    bool needsZeroPublication() const { return is<AwaitZero>(); }

    // Admission and value construction happen on the single owner. No reserved flag, event post,
    // result-credit duplicate, or post-update inference. Install is part of the same command turn,
    // after the previous CONTROL alternative's existing exit behavior has completed.
    RunPtr prepare(uint32_t session, const Pose& target, bool valid, const Measurement& m) const {
        if (protocol_fault_ || generation_ >= std::numeric_limits<uint32_t>::max() - 2 ||
            pendingResults() + (active() ? 1u : 0u) >= 2u) return {};
        const double spread = ell_.spread();
        const double allowance = cfg_.spec.model == Model::Unicycle
                                     ? (spread >= 0 ? spread : cfg_.spec.ell_bound) : 0;
        return RunPtr(new Run({session, target}, valid, allowance, m));
    }
    void install(Run& value, const Measurement& m) {
        if (active()) end(Outcome::Stopped, Reason::None, "replaced by a new Reset", m);
        finish();
        value.owner = this;
        authority_.emplace<Run*>(&value);
        requestAuthority(false);
    }
    void abort(Outcome outcome, const std::string& detail, const Measurement& m) {
        if (active()) end(outcome, Reason::None, detail, m);
    }
    void finish() {
        if (auto* value = run()) {
            if (is<Executing>()) requestAuthority(true);
            Retirement retired{value->header, std::move(value->permission)};
            value->owner = nullptr;
            authority_.emplace<Retirement>(std::move(retired));
        }
    }

    void onZeroPublished(double ros_now, uint32_t generation = 0, uint32_t brake = 0) {
        if (!is<AwaitZero>() || clock_invalid_ || protocol_fault_ || !std::isfinite(ros_now) ||
            !command().zero() || (generation != 0 && generation != generation_) ||
            (brake != 0 && brake != brake_id_)) return;
        velocity_.reset();
        velocity_pose_stamp_ = -1;
        permission().emplace<AwaitStopped>(AwaitStopped{ros_now});
    }

    void onGrant(const Grant& g, const Measurement& m) {
        if (protocol_fault_ || is<None>() || g.session != sessionId() ||
            g.request_generation != generation_ || generation_ == 0 || g.grant_id == 0 ||
            !std::isfinite(g.ttl) || g.ttl <= 0 ||
            (have_publication_ && g.publication_seq <= last_publication_)) return;
        if (g.grant_id < last_decision_id_) return;
        if (g.grant_id == last_decision_id_ &&
            (!have_decision_ || !sameDecision(last_decision_, g))) return;
        if (g.state == GrantState::Go) {
            if (!validGo(g)) return;
            if (const auto* motion = executing()) {
                if (!motion->tracker.active() || !sameRoute(motion->tracker.route(), g) ||
                    g.final_move != motion->final || g.s_limit + 1e-9 < motion->s_limit) return;
            } else {
                const auto* go = std::get_if<AwaitGo>(&permission());
                if (!go || !active() || !go->certified || g.grant_id <= brake_id_ ||
                    (go->deferred && g.grant_id != go->deferred->grant.grant_id)) return;
            }
        } else if (g.state == GrantState::Hold) {
            if (!is<Executing>() &&
                !(is<AwaitGo>() && stopCertified() && g.grant_id > brake_id_) &&
                !(is<Complete>() && !active() && g.grant_id == acknowledged_id_)) return;
        } else if (g.state != GrantState::Brake && g.state != GrantState::Reject) return;
        have_publication_ = true;
        last_publication_ = g.publication_seq;
        have_decision_ = true;
        last_decision_id_ = g.grant_id;
        last_decision_ = g;
        if (auto* value = run()) {
            value->last_message_wall = m.wall_now;
            value->heard = true;
            value->hold_reason = g.reason;
            value->hold_detail = g.detail;
            value->hold_blockers = g.blockers;
        }
        switch (g.state) {
            case GrantState::Brake:
                if (is<AwaitBrake>()) {
                    brake_id_ = g.grant_id;
                    acknowledged_id_ = brake_id_;
                    permission().emplace<AwaitZero>();
                } else if (g.grant_id != brake_id_) requestAuthority(!active());
                break;
            case GrantState::Go:
                if (auto* motion = executing()) {
                    motion->heartbeat_deadline = m.wall_now + g.ttl;
                    motion->grant_id = g.grant_id;
                    acknowledged_id_ = g.grant_id;
                    motion->s_limit = g.s_limit;
                } else {
                    std::get<AwaitGo>(permission()).deferred = DeferredGo{g, m.wall_now + g.ttl};
                }
                break;
            case GrantState::Hold:
                if (!active() && is<AwaitGo>()) {
                    acknowledged_id_ = g.grant_id;
                    permission().emplace<Complete>();
                }
                break;
            case GrantState::Reject:
                if (active()) end(g.reason == Reason::BadSession || g.reason == Reason::SceneChanged ||
                                      g.reason == Reason::SceneUnavailable ||
                                      g.reason == Reason::NonParticipantMoving || g.reason == Reason::None
                                      ? Outcome::RejectedByCoordinator : Outcome::Infeasible,
                                  g.reason, g.detail, m);
                break;
        }
    }

    void poll(const Measurement& m) {
        if (protocol_fault_) {
            if (auto* motion = executing()) motion->command = {};
            return;
        }
        if (auto* stopped = std::get_if<AwaitStopped>(&permission())) {
            if (causalStopped(m, stopped->zero_ros)) {
                permission().emplace<AwaitGo>(AwaitGo{stopped->zero_ros, true, std::nullopt});
            }
        } else if (auto* go = std::get_if<AwaitGo>(&permission())) {
            go->certified = causalStopped(m, go->zero_ros);
        }
        if (auto* go = std::get_if<AwaitGo>(&permission())) {
            if (go->deferred && go->certified && active() && !clock_invalid_ &&
                causalStopped(m, go->zero_ros) && m.wall_now <= go->deferred->deadline) {
                const DeferredGo accepted = *go->deferred;
                acknowledged_id_ = accepted.grant.grant_id;
                permission().emplace<Executing>(accepted.grant, cfg_.spec, accepted.deadline, m);
                run()->progress.emplace<Waiting>();
            }
        }
    }

    StepOutput step(const Measurement& m) {
        poll(m);
        StepOutput out;
        out.phase = phase();
        if (auto* value = run()) {
            if (auto* done = std::get_if<Done>(&value->progress); done && !done->reported) return finished();
        }
        if (!active()) return out;
        Run& value = *run();
        if (protocol_fault_) {
            end(Outcome::RejectedByCoordinator, Reason::BadSession, "Reset authority counter exhausted", m);
            return finished();
        }
        if (cfg_.timeout > 0 && (m.ros_now - value.enter_ros >= cfg_.timeout ||
                                 m.wall_now - value.enter_wall >= cfg_.timeout)) {
            end(Outcome::Timeout, Reason::None, "the vehicle's own deadline", m);
            return finished();
        }
        if (!m.healthy) {
            if (auto* motion = executing()) motion->command = {};
            return held();
        }
        if (!value.target_valid) {
            if (m.wall_now - value.enter_wall >= cfg_.no_target_hold) {
                end(Outcome::RejectedNoTarget, Reason::None,
                    "no target: reset_pose cache and reset_initial_* missing", m);
                return finished();
            }
            return held();
        }
        if (m.wall_now - value.last_message_wall >= cfg_.no_coordinator) {
            end(value.heard ? Outcome::AbortedCoordinatorLost : Outcome::RejectedNoCoordinator,
                Reason::None, value.heard ? "the coordinator stopped answering" : "no coordinator answered", m);
            return finished();
        }
        if (clock_invalid_) {
            if (is<Executing>()) requestAuthority(false);
            return held();
        }
        auto* motion = executing();
        if (!motion || !motion->tracker.active()) {
            if (motion) motion->command = {};
            return held();
        }
        if (m.wall_now > motion->heartbeat_deadline) {
            requestAuthority(false);
            return held();
        }
        const bool due = motion->last_step_ros < 0 ||
                         m.ros_now - motion->last_step_ros >= cfg_.control_period - 1e-9;
        if (due) {
            const double dt = motion->last_step_ros < 0 ? cfg_.control_period
                : std::clamp(m.ros_now - motion->last_step_ros, 1e-3, .1);
            motion->last_step_ros = m.ros_now;
            motion->command = motion->tracker.step(m.pose, ell_.ell(), velocity_.valid() ? velocity_.speed() : 0,
                                                   dt, motion->s_limit);
            if (!checkTube(m) || !checkProgress(m)) return finished();
            if (checkArrival(m, dt)) return finished();
            const bool complete_prefix = RouteTracker::wholeRouteGranted(motion->tracker.route(), motion->s_limit);
            if (complete_prefix && !motion->final && motion->tracker.atEnd() && m.healthy &&
                velocity_.valid() && velocity_.speed() <= cfg_.stop_speed &&
                std::abs(velocity_.yawRate()) <= cfg_.stop_yaw_rate) motion->command = {};
            const double perr = std::hypot(m.pose.x - header().target.x, m.pose.y - header().target.y);
            const double yerr = std::abs(wrapAngle(header().target.yaw - m.pose.yaw));
            if (complete_prefix && stopped(m) && motion->command.zero() &&
                ((!motion->final && motion->tracker.atEnd()) ||
                 (motion->final && motion->tracker.inFinalStage() &&
                  (perr > cfg_.position_tolerance || yerr > cfg_.yaw_tolerance)))) {
                requestAuthority(false);
                return held();
            }
        }
        out.command = motion->command;
        if (motion->final && motion->tracker.atEnd()) value.progress.emplace<Settling>();
        else if (moving()) value.progress.emplace<Moving>();
        else value.progress.emplace<Waiting>();
        out.phase = phase();
        return out;
    }

    Session makeSession(const Measurement& m) const {
        Session s;
        s.stamp = m.ros_now;
        s.session = sessionId();
        s.request_generation = generation_;
        s.acknowledged_grant_id = acknowledged_id_;
        s.stop_certified = stopCertified();
        s.publication_seq = next_publication_;
        s.pose_stamp = m.pose_stamp;
        s.pose = m.pose;
        s.target = header().target;
        s.phase = phase();
        const auto* motion = executing();
        s.progress = motion && motion->tracker.active() ? motion->tracker.progress() : 0;
        s.cross_track = motion && motion->tracker.active() ? motion->tracker.crossTrack() : 0;
        s.spec = cfg_.spec;
        s.ell_hat = cfg_.spec.model == Model::Unicycle ? ell_.ell() : 0;
        s.ell_spread = cfg_.spec.model == Model::Unicycle ? ell_.spread() : 0;
        s.healthy = m.healthy;
        return s;
    }
    bool sessionDue(const Measurement& m) const {
        if (protocol_fault_) return false;
        return !published_ || phase() != published_phase_ || sessionId() != published_session_ ||
               generation_ != published_generation_ || acknowledged_id_ != published_ack_ ||
               stopCertified() != published_cert_ || m.wall_now - published_wall_ >= kSessionPeriod - 1e-9;
    }
    void markSessionPublished(const Measurement& m) {
        published_ = true;
        published_phase_ = phase();
        published_session_ = sessionId();
        published_generation_ = generation_;
        published_ack_ = acknowledged_id_;
        published_cert_ = stopCertified();
        published_wall_ = m.wall_now;
        if (next_publication_ == std::numeric_limits<uint32_t>::max()) protocol_fault_ = true;
        else ++next_publication_;
    }
    bool resultPending() const { return result_pending_ || previous_pending_; }
    bool peekResult(Result* out) const {
        if (!out || !resultPending()) return false;
        *out = previous_pending_ ? previous_result_ : result_;
        return true;
    }
    void commitResult() { if (previous_pending_) previous_pending_ = false; else result_pending_ = false; }
    bool takeResult(Result* out) { if (!peekResult(out)) return false; commitResult(); return true; }
    const Result& result() const { return result_; }
    Reason holdReason() const { return run() ? run()->hold_reason : Reason::None; }
    const std::string& holdDetail() const {
        static const std::string empty;
        return run() ? run()->hold_detail : empty;
    }
    const std::vector<std::string>& holdBlockers() const {
        static const std::vector<std::string> empty;
        return run() ? run()->hold_blockers : empty;
    }
    const EllEstimator& ell() const { return ell_; }
    const VelocityEstimator& velocity() const { return velocity_; }
    const RouteTracker* tracker() const { const auto* e = executing(); return e ? &e->tracker : nullptr; }
    double sLimit() const { const auto* e = executing(); return e ? e->s_limit : 0; }
    Command command() const { const auto* e = executing(); return e ? e->command : Command{}; }
    NextWake nextWake(const Measurement& m) const {
        NextWake next;
        if (!protocol_fault_) next.atWall(sessionDue(m) ? m.wall_now : std::nextafter(published_wall_ + kSessionPeriod - 1e-9, std::numeric_limits<double>::infinity()));
        if (!active()) return next;
        const Run& value = *run();
        if (cfg_.timeout > 0) {
            next.atDomain(std::nextafter(value.enter_ros + cfg_.timeout, std::numeric_limits<double>::infinity()));
            next.atWall(std::nextafter(value.enter_wall + cfg_.timeout, std::numeric_limits<double>::infinity()));
        }
        if (m.healthy) {
            if (!value.target_valid) next.atWall(std::nextafter(value.enter_wall + cfg_.no_target_hold, std::numeric_limits<double>::infinity()));
            else next.atWall(std::nextafter(value.last_message_wall + cfg_.no_coordinator, std::numeric_limits<double>::infinity()));
        }
        if (const auto* e = executing(); e && m.healthy && value.target_valid &&
            !clock_invalid_ && e->tracker.active()) {
            next.atWall(std::nextafter(e->heartbeat_deadline, std::numeric_limits<double>::infinity()));
            next.atDomain(e->last_step_ros < 0 ? m.ros_now :
                std::nextafter(e->last_step_ros + cfg_.control_period - 1e-9,
                               std::numeric_limits<double>::infinity()));
        }
        return next;
    }

   private:
    struct Retirement { SessionHeader header; Permission permission; };
    Run* run() {
        auto* ptr = std::get_if<Run*>(&authority_);
        return ptr ? *ptr : nullptr;
    }
    const Run* run() const {
        const auto* ptr = std::get_if<Run*>(&authority_);
        return ptr ? *ptr : nullptr;
    }
    Permission& permission() { return run() ? run()->permission : std::get<Retirement>(authority_).permission; }
    const Permission& permission() const { return run() ? run()->permission : std::get<Retirement>(authority_).permission; }
    const SessionHeader& header() const { return run() ? run()->header : std::get<Retirement>(authority_).header; }
    template<class T> bool is() const { return std::holds_alternative<T>(permission()); }
    Executing* executing() { return std::get_if<Executing>(&permission()); }
    const Executing* executing() const { return std::get_if<Executing>(&permission()); }
    std::optional<double> zeroSince() const {
        if (const auto* stopped = std::get_if<AwaitStopped>(&permission())) return stopped->zero_ros;
        if (const auto* go = std::get_if<AwaitGo>(&permission())) return go->zero_ros;
        return std::nullopt;
    }
    bool stopCertified() const {
        const auto* go = std::get_if<AwaitGo>(&permission());
        return go && go->certified;
    }
    size_t pendingResults() const { return (result_pending_ ? 1u : 0u) + (previous_pending_ ? 1u : 0u); }
    bool stopped(const Measurement& m) const {
        return m.healthy && m.external_command_zero && command().zero() && velocity_.valid() &&
               velocity_.speed() <= cfg_.stop_speed && std::abs(velocity_.yawRate()) <= cfg_.stop_yaw_rate;
    }
    bool causalStopped(const Measurement& m, double zero_ros) const {
        return !clock_invalid_ && m.pose_stamp_trusted && m.pose_stamp > zero_ros &&
               m.pose_stamp <= m.ros_now && m.pose_stamp == velocity_pose_stamp_ && stopped(m);
    }
    void requestAuthority(bool terminal) {
        permission().emplace<AwaitBrake>();
        brake_id_ = 0;
        acknowledged_id_ = 0;
        if (generation_ == std::numeric_limits<uint32_t>::max()) protocol_fault_ = true;
        else ++generation_;
        if (!terminal) run()->progress.emplace<Requesting>();
    }
    StepOutput held() {
        StepOutput out;
        if (permissionPending() || phase() == Phase::Requesting) run()->progress.emplace<Requesting>();
        else run()->progress.emplace<Waiting>();
        out.phase = phase();
        return out;
    }
    StepOutput finished() {
        std::get<Done>(run()->progress).reported = true;
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
            std::abs(r.ell() - g.ell_plan) > 1e-9 || executing()->final != g.final_move ||
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
            (g.final_move && !samePose(g.goal, header().target)))
            return false;
        for (const Point& p : g.via)
            if (!std::isfinite(p.x) || !std::isfinite(p.y))
                return false;
        return true;
    }
    bool moving() const {
        const auto cmd = command();
        return std::abs(cmd.vx) > .005 || std::abs(cmd.vy) > .005 || std::abs(cmd.wz) > .005 ||
               (velocity_.valid() && (velocity_.speed() > cfg_.stop_speed * 1.5 ||
                                      std::abs(velocity_.yawRate()) > cfg_.stop_yaw_rate * 1.5));
    }
    bool checkTube(const Measurement& m) {
        if (std::abs(executing()->tracker.crossTrack()) <= cfg_.tube_cross_track + 2.0 * run()->allowance) return true;
        char text[96];
        std::snprintf(text, sizeof(text), "left the route tube: cross-track %.2f m", executing()->tracker.crossTrack());
        end(Outcome::AbortedDeviation, Reason::None, text, m);
        return false;
    }
    bool checkProgress(const Measurement& m) {
        auto& motion = *executing();
        const bool may_move = motion.s_limit - motion.tracker.progress() > 0.1;
        const double moved = std::hypot(m.pose.x - motion.stall_pose.x, m.pose.y - motion.stall_pose.y) +
                             0.3 * std::abs(wrapAngle(m.pose.yaw - motion.stall_pose.yaw));
        if (!may_move || moved > 0.03 || !motion.tracker.active()) {
            motion.stall_since = m.ros_now;
            motion.stall_pose = m.pose;
            return true;
        }
        if (m.ros_now - motion.stall_since < cfg_.stall) return true;
        end(Outcome::AbortedStalled, Reason::None, "allowed to move but did not for 10 s", m);
        return false;
    }
    bool checkArrival(const Measurement& m, double dt) {
        auto& motion = *executing();
        if (!motion.final || !motion.tracker.active()) return false;
        const double perr = std::hypot(m.pose.x - header().target.x, m.pose.y - header().target.y);
        const double yerr = std::abs(wrapAngle(header().target.yaw - m.pose.yaw));
        const bool stopped = velocity_.valid() && velocity_.speed() <= cfg_.stop_speed &&
                             std::abs(velocity_.yawRate()) <= cfg_.stop_yaw_rate;
        if (perr <= cfg_.position_tolerance && yerr <= cfg_.yaw_tolerance && stopped) motion.settle += dt;
        else motion.settle = 0.0;
        if (motion.settle >= cfg_.settle_hold) {
            end(Outcome::Arrived, Reason::None, "", m);
            return true;
        }
        return false;
    }
    void end(Outcome outcome, Reason reason, const std::string& detail, const Measurement& m) {
        if (!active()) return;
        if (result_pending_) { previous_result_ = result_; previous_pending_ = true; }
        result_.session = sessionId();
        result_.outcome = outcome;
        result_.reason = reason;
        result_.detail = detail;
        result_.final_planar_error = std::hypot(m.pose.x - header().target.x, m.pose.y - header().target.y);
        result_.final_yaw_error = std::abs(wrapAngle(header().target.yaw - m.pose.yaw));
        result_.duration = m.ros_now - run()->enter_ros;
        result_pending_ = true;
        run()->progress.emplace<Done>();
        requestAuthority(true);
    }

    ExecutorConfig cfg_;
    VelocityEstimator velocity_;
    EllEstimator ell_;
    // Exactly one authoritative permission value: active Run or retained non-motion retirement.
    std::variant<Retirement, Run*> authority_{Retirement{{0, {}}, None{}}};
    uint32_t generation_{0}, brake_id_{0}, acknowledged_id_{0};
    bool protocol_fault_{false};
    double velocity_pose_stamp_{-1};
    uint32_t last_decision_id_{0}, last_publication_{0};
    bool have_decision_{false}, have_publication_{false};
    Grant last_decision_;
    double last_pose_stamp_{-std::numeric_limits<double>::infinity()}, last_ros_now_{0};
    bool have_ros_clock_{false}, clock_invalid_{false};
    Result result_, previous_result_;
    bool result_pending_{false}, previous_pending_{false};
    bool published_{false}, published_cert_{false};
    Phase published_phase_{Phase::Idle};
    uint32_t published_session_{0}, published_generation_{0}, published_ack_{0}, next_publication_{0};
    double published_wall_{0};
};
inline ResetExecutor::Run::~Run() { if (owner) owner->finish(); }

}  // namespace ugv_reset_client
