#pragma once

// The vehicle's side of a granted route: pose estimators and the route follower.
//
//  * VelocityEstimator: planar speed and yaw rate from the pose stream alone, over a window long
//    enough that millimetre pose noise does not read as motion (the arrival gate needs it).
//  * EllEstimator: the rotation-centre offset of a unicycle (uncommanded body-lateral velocity
//    = -ell * omega, the Scout's lateral coupling). It compares the pose with an anchor taken at
//    least 0.06 rad of heading change ago, so white pose noise is not differentiated sample by
//    sample (the per-tick difference of the old controllers is destroyed by 2 mm / 6.7 mrad).
//    It must run in every CONTROL state: the turns of Custom1 teach it before Reset starts, and the
//    planner's sweeps are far smaller when the spread is known than when it is the declared bound.
//  * RouteTracker: follows the route of a grant. A unicycle turns in place about its rotation
//    centre when the heading error to the segment direction exceeds 0.45 rad, otherwise drives
//    along the segment direction (never the bearing to a vertex a few centimetres away: chasing it
//    makes the heading swing) with a Stanley cross-track term and a speed profile
//    sqrt(2 a d) to the next stop vertex or to the end of the granted prefix. An omnidirectional
//    vehicle translates in the world frame with the same profile and turns concurrently.

#include <ugv_reset_client/reset_protocol.h>
#include <ugv_reset_client/reset_route.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

namespace ugv_reset_client {

struct Command {
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
    bool zero() const {
        return vx == 0.0 && vy == 0.0 && wz == 0.0;
    }
};

class VelocityEstimator {
   public:
    void reset() {
        history_.clear();
        speed_ = 0.0;
        yaw_rate_ = 0.0;
        valid_ = false;
    }
    // Feed a pose with its stamp [s]; repeated stamps are ignored.
    void add(double stamp, const Pose& pose) {
        if (!history_.empty() && stamp <= history_.back().t) {
            return;
        }
        history_.push_back({stamp, pose});
        while (history_.size() > 2 && stamp - history_[1].t >= kWindow) {
            history_.pop_front();
        }
        const Entry& first = history_.front();
        const double span = stamp - first.t;
        if (span >= kMinSpan) {
            speed_ = std::hypot(pose.x - first.pose.x, pose.y - first.pose.y) / span;
            yaw_rate_ = wrapAngle(pose.yaw - first.pose.yaw) / span;
            valid_ = true;
        }
    }
    bool valid() const {
        return valid_;
    }
    double speed() const {
        return speed_;
    }
    double yawRate() const {
        return yaw_rate_;
    }

   private:
    struct Entry {
        double t;
        Pose pose;
    };
    static constexpr double kWindow = 0.4;
    static constexpr double kMinSpan = 0.2;
    std::deque<Entry> history_;
    double speed_{0.0};
    double yaw_rate_{0.0};
    bool valid_{false};
};

class EllEstimator {
   public:
    explicit EllEstimator(double bound = 0.25) : bound_(bound) {}
    void setBound(double bound) {
        bound_ = bound;
    }
    void reset() {
        have_anchor_ = false;
        ell_ = 0.0;
        turn_total_ = 0.0;
        samples_.clear();
    }
    // A fresh pose (stamps must increase; the caller filters repeats).
    void add(const Pose& q) {
        if (!have_anchor_) {
            anchor_ = q;
            have_anchor_ = true;
            return;
        }
        const double turn = wrapAngle(q.yaw - anchor_.yaw);
        if (std::abs(turn) >= 0.06) {
            const double mid = anchor_.yaw + 0.5 * turn;
            // The origin's displacement in the frame of the mean heading. The offset only explains
            // the lateral part of a turn in place; an interval that also contains a drive (the
            // anchor lies before a stop) says nothing about it and is dropped.
            const double along = (q.x - anchor_.x) * std::cos(mid) + (q.y - anchor_.y) * std::sin(mid);
            const double lat = -(q.x - anchor_.x) * std::sin(mid) + (q.y - anchor_.y) * std::cos(mid);
            if (std::abs(along) > 2.0 * bound_ * std::abs(turn) + 0.02) {
                anchor_ = q;
                return;
            }
            const double sample = std::clamp(-lat / (2.0 * std::sin(0.5 * turn)), -bound_, bound_);
            ell_ += 0.5 * (sample - ell_);
            turn_total_ += std::abs(turn);
            samples_.push_back(sample);
            if (samples_.size() > kKeep) {
                samples_.pop_front();
            }
            anchor_ = q;
        } else if (std::abs(turn) < 0.002 && std::hypot(q.x - anchor_.x, q.y - anchor_.y) > 0.5) {
            anchor_ = q;  // a long straight run: refresh so the anchor does not go stale
        }
    }
    double ell() const {
        return ell_;
    }
    // The error the planner must still cover around ell(); negative while it is not known, then
    // the declared bound applies. It is known after three samples and half a radian of turning:
    // the filtered estimate converges within that, and before it the error of the estimate can be
    // the whole offset (a first turn about a centre believed to be elsewhere).
    double spread() const {
        if (samples_.size() < 3 || turn_total_ < 0.5) {
            return -1.0;
        }
        double worst = 0.0;
        for (double s : samples_) {
            worst = std::max(worst, std::abs(s - ell_));
        }
        return std::min(bound_, std::max(0.02, worst + 0.02));
    }

   private:
    static constexpr size_t kKeep = 6;
    double bound_;
    Pose anchor_;
    bool have_anchor_{false};
    double ell_{0.0};
    double turn_total_{0.0};
    std::deque<double> samples_;
};

class RouteTracker {
   public:
    void clear() {
        active_ = false;
        rotating_ = false;
        final_stage_ = false;
        vx_ = w_ = 0.0;
        cmd_ = Command{};
    }
    bool active() const {
        return active_;
    }
    void start(const Route& route, const VehicleSpec& spec) {
        route_ = route;
        spec_ = spec;
        active_ = route.segments() > 0;
        rotating_ = false;
        final_stage_ = false;
        segment_ = 0;
        progress_ = 0.0;
        cross_track_ = 0.0;
        heading_error_ = 0.0;
    }
    const Route& route() const {
        return route_;
    }
    // Arc length travelled and the signed distance of the rotation centre from the segment line
    // (positive: left of the path).
    double progress() const {
        return progress_;
    }
    double crossTrack() const {
        return cross_track_;
    }
    double headingError() const {
        return heading_error_;
    }
    // The vehicle has reached the end of the route (the rotation centre is within 3 cm of it).
    bool atEnd() const {
        return at_end_;
    }
    // The prefix covers the whole route, including the final turn.
    static bool wholeRouteGranted(const Route& route, double s_limit) {
        return route.length() <= s_limit + 1e-6;
    }

    // One step of dt seconds. `q` is the measured pose, `ell` the rotation-centre offset estimate,
    // `v_meas` the measured speed, `s_limit` the end of the granted prefix.
    Command step(const Pose& q, double ell_estimate, double v_meas, double dt, double s_limit) {
        if (!active_) {
            return {};
        }
        const bool unicycle = spec_.model == Model::Unicycle;
        const double ell = unicycle ? ell_estimate : 0.0;
        const double th = q.yaw;
        const Point c{q.x + ell * std::cos(th), q.y + ell * std::sin(th)};
        const auto& planned = route_.vertices();
        const auto& cum = route_.cumulative();
        const size_t nseg = route_.segments();
        // The end of the route is the target pose of the body origin: its rotation centre follows
        // the live offset estimate (the route's own end was planned with the estimate of that
        // time), so that turning in place at the end leaves the origin at the target.
        auto vertex = [&](size_t i) -> Point {
            if (i == nseg) {
                return {route_.goal().x + ell * std::cos(route_.goal().yaw),
                        route_.goal().y + ell * std::sin(route_.goal().yaw)};
            }
            return planned[i];
        };
        auto passedVertex = [&](size_t j) {
            if (j >= nseg) {
                return false;
            }
            const Point a = vertex(j), b = vertex(j + 1);
            const double sx = b.x - a.x, sy = b.y - a.y;
            const double sl = std::hypot(sx, sy);
            return sl > 0.05 && ((c.x - a.x) * sx + (c.y - a.y) * sy) / sl >= sl - 0.03;
        };
        while (segment_ + 1 < nseg &&
               (Route::distance(vertex(segment_ + 1), c) < 0.06 || passedVertex(segment_))) {
            ++segment_;
        }
        const bool last = segment_ + 1 == nseg;
        const Point prevp = vertex(segment_);
        const Point target = vertex(segment_ + 1);
        const double d = Route::distance(target, c);
        at_end_ = Route::distance(vertex(nseg), c) < 0.03;
        const double vstep = spec_.acc_v * 0.9 * dt;
        const double wstep = spec_.acc_w * 0.9 * dt;
        {
            const double sx = target.x - prevp.x, sy = target.y - prevp.y;
            const double sl = std::hypot(sx, sy);
            progress_ = cum[segment_] +
                        (sl > 1e-9 ? std::clamp(((c.x - prevp.x) * sx + (c.y - prevp.y) * sy) / sl, 0.0, sl)
                                   : 0.0);
        }
        const double allowed = s_limit - progress_;
        // In-place rotation: proportional with a square-root profile, no creep offset (a minimum
        // command makes the yaw chatter around zero with the lag and the pose noise) and a dead
        // band well inside the arrival gate.
        auto rotateTo = [&](double to) {
            const double e = wrapAngle(to - th);
            if (std::abs(e) < 0.012) {
                return 0.0;
            }
            const double wmag = std::min({spec_.max_w, std::sqrt(2.0 * spec_.acc_w * 0.5 * std::abs(e)),
                                          1.5 * std::abs(e)});
            return std::copysign(wmag, e);
        };
        if (unicycle) {
            double vdes = 0.0, wdes = 0.0;
            cross_track_ = 0.0;
            heading_error_ = 0.0;
            // The final stage: turn in place to the goal heading about the rotation centre, then
            // nudge along the heading to cancel the remaining error of the body origin. It latches
            // (entered within 3 cm of the end, left beyond 15 cm), so that the jitter of the offset
            // estimate can not throw the vehicle back into route following with a half turn to do.
            if (last && d < 0.03) {
                final_stage_ = true;
            } else if (final_stage_ && d > 0.15) {
                final_stage_ = false;
            }
            if (last && final_stage_) {
                const bool granted = wholeRouteGranted(route_, s_limit);
                const double e = wrapAngle(route_.goal().yaw - th);
                wdes = granted ? rotateTo(route_.goal().yaw) : 0.0;
                if (granted && std::abs(e) < 0.1) {
                    const double along = (route_.goal().x - q.x) * std::cos(th) + (route_.goal().y - q.y) * std::sin(th);
                    vdes = std::abs(along) > 0.012 ? std::clamp(0.8 * along, -0.06, 0.06) : 0.0;
                }
            } else {
                const int dir = route_.direction(segment_);
                const double bx = target.x - c.x, by = target.y - c.y;
                const double psi = std::atan2(by, bx);
                const double sx = target.x - prevp.x, sy = target.y - prevp.y;
                const double segl = std::hypot(sx, sy);
                double ect = 0.0, psi_seg = psi;
                // Follow the segment direction everywhere except the last 25 cm of the final
                // segment (position servo).
                if (segl > 0.05 && (!last || (segl > 0.2 && d >= 0.25))) {
                    const double ux = sx / segl, uy = sy / segl;
                    ect = ux * (c.y - prevp.y) - uy * (c.x - prevp.x);
                    psi_seg = std::atan2(uy, ux);
                }
                cross_track_ = ect;
                const double body_target = dir > 0 ? psi_seg : psi_seg + kPi;
                const double e_use = wrapAngle(body_target - th);
                heading_error_ = e_use;
                // Cross-track correction (Stanley): the velocity direction is steered toward the
                // path for both travel directions.
                const double stanley = std::atan2(1.5 * ect, 0.15 + std::abs(v_meas));
                const double ang = wrapAngle(e_use - stanley);
                bool must_stop = last;
                if (!last) {
                    const Point next = vertex(segment_ + 2);
                    must_stop = std::abs(wrapAngle(std::atan2(next.y - target.y, next.x - target.x) - psi)) > 0.10;
                }
                double vcap = must_stop ? stopSpeed(d - 0.015) : spec_.max_v;
                vcap = std::min(vcap, spec_.max_v);
                if (must_stop) {
                    vcap = std::max(vcap, std::min(0.04, 4.0 * d));
                }
                vcap = std::min(vcap, stopSpeed(allowed - 0.05));
                const bool near_vertex = progress_ - cum[segment_] < 0.12;
                const bool vertex_ungranted =
                    segment_ > 0 && segment_ < cum.size() && cum[segment_] > s_limit + 1e-6;
                if (vertex_ungranted && progress_ >= cum[segment_] - 0.08) {
                    vdes = 0.0;  // at a vertex whose turn is not reserved yet: hold
                    wdes = 0.0;
                } else if (std::abs(e_use) > 0.45 || (near_vertex && std::abs(e_use) > 0.10) ||
                           (rotating_ && std::abs(e_use) > (near_vertex ? 0.04 : 0.15))) {
                    // Large heading error, or any turn at a vertex: stop and rotate in place about c
                    // until the heading is on the segment (the sweep the planner verified turns to
                    // the segment heading before it translates; steering through a corner while
                    // driving would cut it).
                    rotating_ = true;
                    vdes = 0.0;
                    wdes = rotateTo(body_target);
                } else {
                    rotating_ = false;
                    const double sc = std::max(0.0, std::cos(e_use));
                    vdes = dir * vcap * sc * sc;
                    wdes = std::clamp(2.2 * ang, -spec_.max_w, spec_.max_w);
                }
            }
            vx_ = slew(vx_, vdes, vstep);
            w_ = slew(w_, wdes, wstep);
            cmd_ = {vx_, 0.0, w_};
            return cmd_;
        }
        // Omnidirectional: translate in the world frame along the segment (pure cross-track
        // correction), with the same stopping profile at turning vertices and at the end of the
        // prefix; the heading converges concurrently (the planner sweeps a disc, so any heading
        // schedule is covered).
        const double sx = target.x - prevp.x, sy = target.y - prevp.y;
        const double segl = std::hypot(sx, sy);
        const double ux = segl > 0.05 ? sx / segl : 0.0, uy = segl > 0.05 ? sy / segl : 0.0;
        const double ect = segl > 0.05 ? ux * (q.y - prevp.y) - uy * (q.x - prevp.x) : 0.0;
        cross_track_ = ect;
        const Point goal{route_.goal().x, route_.goal().y};
        const double gx = (last ? goal.x : target.x) - q.x;
        const double gy = (last ? goal.y : target.y) - q.y;
        const double dist = std::hypot(gx, gy);
        bool must_stop = last;
        if (!last) {
            const Point next = vertex(segment_ + 2);
            const double psi = std::atan2(sy, sx);
            must_stop = std::abs(wrapAngle(std::atan2(next.y - target.y, next.x - target.x) - psi)) > 0.10;
        }
        double sp = must_stop ? stopSpeed(dist - 0.01) + (dist > 0.02 ? 0.03 : 0.0) : spec_.max_v;
        sp = std::min({sp, spec_.max_v, stopSpeed(allowed - 0.05)});
        if (dist < 0.02) {
            sp = 0.0;
        }
        // Along the segment, except for the last 25 cm of the final segment (position servo).
        const bool servo = segl <= 0.05 || (last && dist < 0.25);
        double wx = servo ? (dist > 1e-9 ? gx / dist * sp : 0.0) : ux * sp;
        double wy = servo ? (dist > 1e-9 ? gy / dist * sp : 0.0) : uy * sp;
        if (!servo && sp > 0.0) {
            // Cross-track correction while it may move: a held vehicle commands exact zero.
            const double k = std::clamp(1.5 * std::abs(ect), 0.0, 0.15);
            wx += (ect > 0.0 ? 1.0 : -1.0) * k * uy;  // toward the line: against the left normal
            wy -= (ect > 0.0 ? 1.0 : -1.0) * k * ux;
        }
        const double w = rotateTo(route_.goal().yaw);
        const double bxv = std::cos(th) * wx + std::sin(th) * wy;
        const double byv = -std::sin(th) * wx + std::cos(th) * wy;
        cmd_.vx = slew(cmd_.vx, bxv, vstep);
        cmd_.vy = slew(cmd_.vy, byv, vstep);
        cmd_.wz = slew(cmd_.wz, w, wstep);
        return cmd_;
    }

   private:
    // The largest speed from which the vehicle still stops within `distance` when its velocity
    // follows the command through a first-order lag and a pose delay (kLag in total): the stopping
    // distance is v * lag + v^2 / (2 a). A profile that ignored the lag overshot the vertices of
    // a route by 10-20 cm on a slow chassis, outside the swept footprint the planner verified.
    double stopSpeed(double distance) const {
        const double a = spec_.acc_v * 0.8;
        return a * (-kLag + std::sqrt(kLag * kLag + 2.0 * std::max(0.0, distance) / a));
    }

    static constexpr double kLag = 0.3;

    static double slew(double current, double desired, double step) {
        return current + std::clamp(desired - current, -step, step);
    }

    Route route_;
    VehicleSpec spec_;
    bool active_{false};
    bool rotating_{false};
    bool final_stage_{false};
    bool at_end_{false};
    size_t segment_{0};
    double progress_{0.0}, cross_track_{0.0}, heading_error_{0.0};
    double vx_{0.0}, w_{0.0};
    Command cmd_;
};

}  // namespace ugv_reset_client
