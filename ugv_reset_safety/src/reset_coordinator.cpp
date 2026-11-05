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
        if (found == entries_.end() || !found->second.body) {
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
    auto inserted = entries_.try_emplace(vehicle);
    Entry& e = inserted.first->second;
    // Source publication freshness is checked before health, phase, pose, receipt time or cache.
    if (!inserted.second && (s.publication_seq <= e.session.publication_seq ||
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
    e.bad_reason = why.empty() ? std::nullopt : std::optional<std::string>{std::move(why)};
    if (!inserted.second &&
        (s.session != e.session.session || s.request_generation != e.session.request_generation)) {
        e.last.reset();  // a new session, or none: the earlier answer does not carry over
    }
    const bool body_valid = validSpec(s.spec) && std::isfinite(s.pose.x) &&
                            std::isfinite(s.pose.y) && std::isfinite(s.pose.yaw) &&
                            std::isfinite(s.pose_stamp);
    if (body_valid && s.healthy) {
        e.body = Entry::Body{s.pose, s.spec};
    }
    if (!e.bad_reason) {
        if (inserted.second) {
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
    e.received = now;
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
        if (e.bad_reason && !e.body) {
            continue;
        }
        PlannerVehicle v;
        v.id = entry.first;
        v.session = e.session.session;
        v.request_generation = e.session.request_generation;
        v.acknowledged_grant_id = e.session.acknowledged_grant_id;
        v.available = !lost(e, now) && !e.bad_reason && e.session.healthy;
        v.stop_certified = e.session.stop_certified && e.session.healthy && v.available;
        const auto body = e.body.value_or(Entry::Body{});
        v.spec = body.spec;
        v.pose = body.pose;
        v.target = e.session.target;
        v.phase = e.session.phase;
        v.ell_hat = e.session.ell_hat;
        v.ell_spread = e.session.ell_spread;
        const bool outside = !config_.expected_namespaces.empty() &&
                             !std::binary_search(config_.expected_namespaces.begin(),
                                                 config_.expected_namespaces.end(), entry.first);
        if (outside || e.bad_reason) {
            v.phase = rc::Phase::Idle;
        }
        v.uncontrolled_moving =
            !participantPhase(v.phase) && !lost(e, now) && e.speed > config_.moving_speed;
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
        e.last = Entry::Publication{g, now};
        sent[id] = true;
        out.push_back({id, std::move(g)});
    };
    for (const PlannerDecision& d : planner_.update(now, vehicles, admit)) {
        auto found = entries_.find(d.id);
        if (found == entries_.end() || d.session != found->second.session.session ||
            d.request_generation != found->second.session.request_generation) {
            continue;
        }
        send(d.id, grantOf(d, now));
    }
    // No scene/scope observation can synthesize a Go, clear an outstanding reservation, or
    // substitute for the correlated Brake. Once no route remains, report the local hold.
    if (!admit) {
        for (const PlannerBodyReport& r : planner_.report()) {
            Entry& e = entries_.find(r.id)->second;
            if (!participant(e, now) || r.has_route || sent.count(r.id) ||
                !e.session.stop_certified || e.session.request_generation == 0) {
                continue;
            }
            const rc::Reason reason =
                scene ? rc::Reason::WaitingForPeer : rc::Reason::SceneUnavailable;
            if (e.last.has_value() && e.last->grant.state == rc::GrantState::Hold &&
                e.last->grant.reason == reason) {
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
        if (lost(e, now) || sent.count(entry.first) || e.session.request_generation == 0) {
            continue;
        }
        const bool outside = !config_.expected_namespaces.empty() &&
                             !std::binary_search(config_.expected_namespaces.begin(),
                                                 config_.expected_namespaces.end(), entry.first);
        if (e.bad_reason || outside) {
            const std::string detail =
                e.bad_reason ? *e.bad_reason : "namespace is outside the declared Reset scope";
            if (e.last.has_value() && e.last->grant.state == rc::GrantState::Reject &&
                e.last->grant.detail == detail) {
                if (now - e.last->sent >= config_.refresh_period) {
                    send(entry.first, e.last->grant);
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
        if (!e.last.has_value() || now - e.last->sent < config_.refresh_period) {
            continue;
        }
        // Terminal Hold receipts are sent until the vehicle echoes them. They do not depend on
        // an Idle/Done heartbeat being classified as an active Reset participant.
        const bool pending = e.last->grant.state == rc::GrantState::Brake ||
                             (e.last->grant.state == rc::GrantState::Hold &&
                              e.session.acknowledged_grant_id != e.last->grant.grant_id);
        if (!participant(e, now) && !pending) {
            continue;
        }
        send(entry.first, e.last->grant);
    }
    return out;
}

bool ResetCoordinator::lost(const Entry& e, double now) const {
    return now - e.received > config_.lost_after;
}
bool ResetCoordinator::participant(const Entry& e, double now) const {
    return participantPhase(e.session.phase) && !lost(e, now);
}
double ResetCoordinator::nextDeadline(double now) const {
    const double never = std::numeric_limits<double>::infinity();
    double next = planner_.nextDeadline((!scene_authored_ || scene_available_) && scopeReady());
    for (const auto& item : entries_) {
        const Entry& e = item.second;
        const double expiry = e.received + config_.lost_after;
        if (now <= expiry)
            next = std::min(next, std::nextafter(expiry, never));
        if (lost(e, now) || e.session.request_generation == 0 || !e.last)
            continue;
        const bool outside = !config_.expected_namespaces.empty() &&
                             !std::binary_search(config_.expected_namespaces.begin(),
                                                 config_.expected_namespaces.end(), item.first);
        const bool pending = e.last->grant.state == rc::GrantState::Brake ||
                             (e.last->grant.state == rc::GrantState::Hold &&
                              e.session.acknowledged_grant_id != e.last->grant.grant_id);
        if (e.bad_reason || outside || participant(e, now) || pending)
            next = std::min(next, std::nextafter(e.last->sent + config_.refresh_period, never));
    }
    return next;
}

bool ResetCoordinator::busy() const {
    for (const auto& entry : entries_) {
        if (participantPhase(entry.second.session.phase)) {
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
        FleetMember m;
        m.vehicle = entry.first;
        m.session = e.session.session;
        m.phase = e.session.phase;
        m.lost = now - e.received > config_.lost_after;
        if (e.last.has_value()) {
            m.grant_state = e.last->grant.state;
            m.reason = e.last->grant.reason;
            m.blockers = e.last->grant.blockers;
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
