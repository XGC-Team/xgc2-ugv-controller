#include <ugv_reset_safety/reset_coordinator.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace ugv_reset_safety {

namespace rc = ugv_reset_client;

ResetCoordinator::ResetCoordinator(const CoordinatorConfig& config)
    : config_(config), planner_(config.planner) {}

void ResetCoordinator::setWorld(const std::vector<ConvexObstacle>& statics, const Fence& fence) {
    planner_.setWorld(statics, fence);
}

void ResetCoordinator::setScene(bool authored, bool available) {
    scene_authored_ = authored;
    scene_available_ = available;
}

bool ResetCoordinator::participantPhase(rc::Phase phase) {
    return phase == rc::Phase::Requesting || phase == rc::Phase::Waiting ||
           phase == rc::Phase::Moving || phase == rc::Phase::Settling;
}

bool ResetCoordinator::validSpec(const rc::VehicleSpec& spec) {
    const rc::Footprint& f = spec.footprint;
    auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
    return positive(f.half_length) && positive(f.half_width) && f.half_length < 5.0 &&
           f.half_width < 5.0 && std::isfinite(f.offset_x) && std::isfinite(f.offset_y) &&
           std::abs(f.offset_x) < 5.0 && std::abs(f.offset_y) < 5.0 && positive(spec.max_v) &&
           positive(spec.max_w) && positive(spec.acc_v) && positive(spec.acc_w) &&
           std::isfinite(spec.ell_bound) && spec.ell_bound >= 0.0 && spec.ell_bound < 2.0;
}

void ResetCoordinator::onSession(const std::string& vehicle, const rc::Session& s, double now) {
    Entry& e = entries_[vehicle];
    const bool participant = participantPhase(s.phase);
    std::string why;
    if (vehicle.empty()) {
        why = "empty vehicle name";
    } else if (!validSpec(s.spec)) {
        why = "the declared footprint or limits are invalid";
    } else if (!std::isfinite(s.pose.x) || !std::isfinite(s.pose.y) || !std::isfinite(s.pose.yaw)) {
        why = "the pose is not finite";
    } else if (participant && (!std::isfinite(s.target.x) || !std::isfinite(s.target.y) ||
                               !std::isfinite(s.target.yaw))) {
        why = "the target is not finite";
    }
    e.bad = !why.empty();
    e.bad_reason = why;
    if (e.have && (s.session != e.session.session || !participant)) {
        e.has_last = false;  // a new session, or none: the earlier answer does not carry over
        e.first_reject = -1.0;
    }
    if (!e.bad) {
        if (!e.have) {
            e.reference = s.pose;
            e.reference_stamp = s.pose_stamp;
        } else if (s.pose_stamp - e.reference_stamp >= 0.2) {
            const double dt = s.pose_stamp - e.reference_stamp;
            e.speed = std::hypot(s.pose.x - e.reference.x, s.pose.y - e.reference.y) / dt;
            e.reference = s.pose;
            e.reference_stamp = s.pose_stamp;
        }
    }
    e.session = s;
    e.have = true;
    e.received = now;
    e.lost = false;
    e.participant = participant;
}

rc::Grant ResetCoordinator::grantOf(const PlannerDecision& d, double now) const {
    rc::Grant g;
    g.stamp = now;
    g.ttl = config_.ttl;
    g.session = d.session;
    g.state = d.state;
    g.grant_id = d.grant_id;
    g.start = d.start;
    g.goal = d.goal;
    g.final_move = d.final_move;
    g.ell_plan = d.ell_plan;
    g.via = d.via;
    g.s_limit = d.s_limit;
    g.reason = d.reason;
    g.detail = d.detail;
    g.blockers = d.blockers;
    return g;
}

std::vector<OutgoingGrant> ResetCoordinator::step(double now) {
    std::vector<OutgoingGrant> out;
    std::vector<PlannerVehicle> vehicles;
    for (auto& entry : entries_) {
        Entry& e = entry.second;
        if (!e.have) {
            continue;
        }
        e.lost = now - e.received > config_.lost_after;
        e.participant = participantPhase(e.session.phase) && !e.lost;
        if (e.bad) {
            continue;
        }
        PlannerVehicle v;
        v.id = entry.first;
        v.session = e.session.session;
        v.spec = e.session.spec;
        v.pose = e.session.pose;
        v.target = e.session.target;
        v.phase = e.lost ? rc::Phase::Idle : e.session.phase;
        v.ell_hat = e.session.ell_hat;
        v.ell_spread = e.session.ell_spread;
        v.uncontrolled_moving = !participantPhase(v.phase) && !e.lost && e.speed > config_.moving_speed;
        vehicles.push_back(std::move(v));
    }
    const bool admit = !scene_authored_ || scene_available_;
    std::map<std::string, bool> sent;
    for (const PlannerDecision& d : planner_.update(now, vehicles, admit)) {
        auto found = entries_.find(d.id);
        if (found == entries_.end() || d.session != found->second.session.session) {
            continue;
        }
        Entry& e = found->second;
        e.last = grantOf(d, now);
        e.has_last = true;
        e.last_sent = now;
        if (d.state == rc::GrantState::Reject && e.first_reject < 0.0) {
            e.first_reject = now;
        }
        sent[d.id] = true;
        out.push_back({d.id, e.last});
    }
    // Held for a missing Scene: told why, so that the wait is visible instead of silent.
    if (!admit) {
        for (const PlannerBodyReport& r : planner_.report()) {
            Entry& e = entries_[r.id];
            if (!e.participant || r.has_route || sent.count(r.id) != 0 ||
                (e.session.phase != rc::Phase::Requesting && e.session.phase != rc::Phase::Waiting)) {
                continue;
            }
            const bool already = e.has_last && e.last.state == rc::GrantState::Hold &&
                                 e.last.reason == rc::Reason::SceneUnavailable;
            if (already && now - e.last_sent < config_.refresh_period) {
                continue;
            }
            rc::Grant g;
            g.stamp = now;
            g.ttl = config_.ttl;
            g.session = e.session.session;
            g.state = rc::GrantState::Hold;
            g.reason = rc::Reason::SceneUnavailable;
            g.detail = "a Scene is authored for this Experiment but its snapshot or heartbeat is missing";
            e.last = g;
            e.has_last = true;
            e.last_sent = now;
            sent[r.id] = true;
            out.push_back({r.id, g});
        }
    }
    // The heartbeat: every participant hears the coordinator at least every refresh period, and a
    // vehicle the coordinator refuses hears that too, until it has ended its session.
    for (auto& entry : entries_) {
        Entry& e = entry.second;
        if (!e.have || !e.participant || sent.count(entry.first) != 0) {
            continue;
        }
        if (e.bad) {
            if (e.has_last && now - e.last_sent < config_.refresh_period) {
                continue;
            }
            rc::Grant g;
            g.stamp = now;
            g.ttl = config_.ttl;
            g.session = e.session.session;
            g.state = rc::GrantState::Reject;
            g.reason = rc::Reason::BadSession;
            g.detail = e.bad_reason;
            e.last = g;
            e.has_last = true;
            e.last_sent = now;
            out.push_back({entry.first, g});
            continue;
        }
        if (!e.has_last) {
            rc::Grant g;
            g.stamp = now;
            g.ttl = config_.ttl;
            g.session = e.session.session;
            g.state = rc::GrantState::Hold;
            g.reason = rc::Reason::WaitingForPeer;
            g.detail = "planning";
            e.last = g;
            e.has_last = true;
            e.last_sent = now;
            out.push_back({entry.first, g});
            continue;
        }
        if (now - e.last_sent < config_.refresh_period) {
            continue;
        }
        if (e.last.state == rc::GrantState::Reject && e.first_reject >= 0.0 &&
            now - e.first_reject > config_.reject_resend) {
            continue;
        }
        e.last.stamp = now;
        e.last_sent = now;
        out.push_back({entry.first, e.last});
    }
    return out;
}

bool ResetCoordinator::busy() const {
    for (const auto& entry : entries_) {
        if (entry.second.have && participantPhase(entry.second.session.phase)) {
            return true;
        }
    }
    for (const PlannerBodyReport& r : planner_.report()) {
        if (r.has_route) {
            return true;
        }
    }
    return false;
}

FleetSummary ResetCoordinator::fleet(double now) const {
    FleetSummary summary;
    summary.scene_available = !scene_authored_ || scene_available_;
    std::map<std::string, PlannerBodyReport> reports;
    for (const PlannerBodyReport& r : planner_.report()) {
        reports[r.id] = r;
    }
    for (const auto& entry : entries_) {
        const Entry& e = entry.second;
        if (!e.have) {
            continue;
        }
        FleetMember m;
        m.vehicle = entry.first;
        m.session = e.session.session;
        m.phase = e.session.phase;
        m.lost = now - e.received > config_.lost_after;
        if (e.has_last) {
            m.grant_state = e.last.state;
            m.reason = e.last.reason;
            m.blockers = e.last.blockers;
        }
        auto found = reports.find(entry.first);
        if (found != reports.end() && found->second.has_route) {
            m.progress = found->second.progress;
            m.remaining = found->second.remaining;
        }
        summary.members.push_back(std::move(m));
    }
    return summary;
}

}  // namespace ugv_reset_safety
