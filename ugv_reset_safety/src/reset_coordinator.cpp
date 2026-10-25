#include <ugv_reset_safety/reset_coordinator.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace ugv_reset_safety {

namespace rc = ugv_reset_client;

ResetCoordinator::ResetCoordinator(const CoordinatorConfig& config)
    : config_(config), planner_(config.planner) {
    std::sort(config_.expected_namespaces.begin(), config_.expected_namespaces.end());
    if (std::adjacent_find(config_.expected_namespaces.begin(),
                           config_.expected_namespaces.end()) !=
        config_.expected_namespaces.end()) {
        throw std::invalid_argument("Reset expected namespaces must be unique");
    }
    for (const auto& name : config_.expected_namespaces) {
        if (name.empty() || name.front() == '/' || name.back() == '/' ||
            name.find("//") != std::string::npos) {
            throw std::invalid_argument(
                "Reset expected namespaces must be normalized nonempty names");
        }
    }
}

bool ResetCoordinator::scopeReady() const {
    if (config_.expected_namespaces.empty()) {
        return false;
    }
    for (const auto& name : config_.expected_namespaces) {
        auto found = entries_.find(name);
        if (found == entries_.end() || !found->second.scope_seen) {
            return false;
        }
    }
    return true;
}

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
    // Source publication freshness is checked before health, phase, pose, receipt time or cache.
    if (e.have && (s.publication_seq <= e.session.publication_seq ||
                   s.request_generation < e.session.request_generation ||
                   (s.request_generation == e.session.request_generation &&
                    s.session != e.session.session))) {
        return;
    }
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
    if (e.have &&
        (s.session != e.session.session || s.request_generation != e.session.request_generation)) {
        e.has_last = false;  // a new session, or none: the earlier answer does not carry over
        e.first_reject = -1.0;
    }
    const bool body_valid = validSpec(s.spec) && std::isfinite(s.pose.x) &&
                            std::isfinite(s.pose.y) && std::isfinite(s.pose.yaw) &&
                            std::isfinite(s.pose_stamp);
    if (body_valid && s.healthy) {
        e.scope_seen = true;
        e.body_pose = s.pose;
        e.body_spec = s.spec;
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
    g.request_generation = d.request_generation;
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
        if (e.bad && !e.scope_seen) {
            continue;
        }
        PlannerVehicle v;
        v.id = entry.first;
        v.session = e.session.session;
        v.request_generation = e.session.request_generation;
        v.acknowledged_grant_id = e.session.acknowledged_grant_id;
        v.available = !e.lost && !e.bad && e.session.healthy;
        v.stop_certified = e.session.stop_certified && e.session.healthy && v.available;
        v.spec = e.body_spec;
        v.pose = e.body_pose;
        v.target = e.session.target;
        v.phase = e.session.phase;
        v.ell_hat = e.session.ell_hat;
        v.ell_spread = e.session.ell_spread;
        const bool outside = !config_.expected_namespaces.empty() &&
                             !std::binary_search(config_.expected_namespaces.begin(),
                                                 config_.expected_namespaces.end(), entry.first);
        if (outside || e.bad) {
            v.phase = rc::Phase::Idle;
        }
        v.uncontrolled_moving =
            !participantPhase(v.phase) && !e.lost && e.speed > config_.moving_speed;
        vehicles.push_back(std::move(v));
    }
    const bool scene = !scene_authored_ || scene_available_;
    const bool complete = scopeReady();
    const bool admit = scene && complete;
    std::map<std::string, bool> sent;
    auto send = [&](const std::string& id, rc::Grant g) {
        Entry& e = entries_.at(id);
        if (e.next_publication_seq == std::numeric_limits<uint32_t>::max()) {
            throw std::overflow_error("Reset grant publication sequence exhausted");
        }
        g.publication_seq = e.next_publication_seq++;
        g.stamp = now;
        g.ttl = config_.ttl;
        e.last = g;
        e.has_last = true;
        e.last_sent = now;
        sent[id] = true;
        out.push_back({id, std::move(g)});
    };
    for (const PlannerDecision& d : planner_.update(now, vehicles, admit)) {
        auto found = entries_.find(d.id);
        if (found == entries_.end() || d.session != found->second.session.session ||
            d.request_generation != found->second.session.request_generation) {
            continue;
        }
        if (d.state == rc::GrantState::Reject && found->second.first_reject < 0.0) {
            found->second.first_reject = now;
        }
        send(d.id, grantOf(d, now));
    }
    // No scene/scope observation can synthesize a Go, clear an outstanding reservation, or
    // substitute for the correlated Brake. Once no route remains, report the local hold.
    if (!admit) {
        for (const PlannerBodyReport& r : planner_.report()) {
            Entry& e = entries_[r.id];
            if (!e.participant || r.has_route || sent.count(r.id) || !e.session.stop_certified ||
                e.session.request_generation == 0) {
                continue;
            }
            const rc::Reason reason =
                scene ? rc::Reason::WaitingForPeer : rc::Reason::SceneUnavailable;
            if (e.has_last && e.last.state == rc::GrantState::Hold && e.last.reason == reason) {
                continue;
            }
            rc::Grant g;
            g.session = e.session.session;
            g.request_generation = e.session.request_generation;
            g.state = rc::GrantState::Hold;
            g.grant_id = planner_.allocateGrantId();
            g.reason = reason;
            g.detail = scene ? "expected namespace scope or first valid body observations missing"
                             : "a Scene is authored but its live snapshot is unavailable";
            send(r.id, g);
        }
    }
    for (auto& entry : entries_) {
        Entry& e = entry.second;
        if (!e.have || e.lost || sent.count(entry.first) || e.session.request_generation == 0) {
            continue;
        }
        const bool outside = !config_.expected_namespaces.empty() &&
                             !std::binary_search(config_.expected_namespaces.begin(),
                                                 config_.expected_namespaces.end(), entry.first);
        if (e.bad || outside) {
            const std::string detail =
                e.bad ? e.bad_reason : "namespace is outside the declared Reset scope";
            if (e.has_last && e.last.state == rc::GrantState::Reject && e.last.detail == detail) {
                if (now - e.last_sent >= config_.refresh_period) {
                    send(entry.first, e.last);
                }
                continue;
            }
            rc::Grant g;
            g.session = e.session.session;
            g.request_generation = e.session.request_generation;
            g.state = rc::GrantState::Reject;
            g.grant_id = planner_.allocateGrantId();
            g.reason = rc::Reason::BadSession;
            g.detail = detail;
            send(entry.first, g);
            continue;
        }
        if (!e.has_last || now - e.last_sent < config_.refresh_period) {
            continue;
        }
        // Terminal Hold receipts are sent until the vehicle echoes them. They do not depend on
        // an Idle/Done heartbeat being classified as an active Reset participant.
        const bool pending = e.last.state == rc::GrantState::Brake ||
                             (e.last.state == rc::GrantState::Hold &&
                              e.session.acknowledged_grant_id != e.last.grant_id);
        if (!e.participant && !pending) {
            continue;
        }
        send(entry.first, e.last);
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
