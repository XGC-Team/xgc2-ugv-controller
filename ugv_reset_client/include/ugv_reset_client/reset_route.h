#pragma once

// What a granted route means, defined once for the planner that verifies it and the tracker that
// follows it. Both build a Route from the same Grant (start pose, via points, goal, ell), so the
// sweep the coordinator proved clear is the motion the vehicle performs.
//
// A route is a polyline of the vehicle's rotation centre c = p + ell * (cos yaw, sin yaw), p the
// body origin: c_0 = c(start), the via points, c_n = c(goal). A unicycle turns in place about c at
// every vertex and drives each segment forward or in reverse, whichever needs the smaller rotation
// (a Scout may drive either way, vy = 0); an omnidirectional vehicle translates along the polyline.
// The arc length s counts the translation only: the in-place turn at a vertex belongs to the
// vertex's s, and a prefix limit that is below a vertex excludes the turn at it.

#include <ugv_reset_client/reset_protocol.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace ugv_reset_client {

class Route {
   public:
    Route() = default;
    Route(const Pose& start, const std::vector<Point>& via, const Pose& goal, double ell)
        : start_(start), goal_(goal), ell_(ell) {
        vertices_.push_back(centre(start));
        for (const Point& p : via) {
            vertices_.push_back(p);
        }
        vertices_.push_back(centre(goal));
        cumulative_.assign(1, 0.0);
        for (size_t j = 0; j + 1 < vertices_.size(); ++j) {
            cumulative_.push_back(cumulative_.back() + distance(vertices_[j], vertices_[j + 1]));
        }
        // Per segment: the body heading and the direction, chosen by the smaller rotation from the
        // previous heading. A zero-length segment keeps the previous heading.
        double previous = start.yaw;
        for (size_t j = 0; j + 1 < vertices_.size(); ++j) {
            const double dx = vertices_[j + 1].x - vertices_[j].x;
            const double dy = vertices_[j + 1].y - vertices_[j].y;
            if (std::hypot(dx, dy) < 1e-9) {
                headings_.push_back(previous);
                directions_.push_back(1);
                continue;
            }
            const double bearing = std::atan2(dy, dx);
            const double reverse = wrapAngle(bearing + kPi);
            const bool forward =
                std::abs(wrapAngle(bearing - previous)) <= std::abs(wrapAngle(reverse - previous));
            previous = forward ? bearing : reverse;
            headings_.push_back(previous);
            directions_.push_back(forward ? 1 : -1);
        }
    }

    bool empty() const {
        return vertices_.empty();
    }
    const Pose& start() const {
        return start_;
    }
    const Pose& goal() const {
        return goal_;
    }
    double ell() const {
        return ell_;
    }
    // c-space vertices: start, via..., goal.
    const std::vector<Point>& vertices() const {
        return vertices_;
    }
    // Arc length at each vertex.
    const std::vector<double>& cumulative() const {
        return cumulative_;
    }
    double length() const {
        return cumulative_.empty() ? 0.0 : cumulative_.back();
    }
    size_t segments() const {
        return vertices_.size() < 2 ? 0 : vertices_.size() - 1;
    }
    double heading(size_t segment) const {
        return headings_[segment];
    }
    // +1 drives the body forward along the segment, -1 in reverse.
    int direction(size_t segment) const {
        return directions_[segment];
    }

    Point centre(const Pose& p) const {
        return {p.x + ell_ * std::cos(p.yaw), p.y + ell_ * std::sin(p.yaw)};
    }
    // The body origin when the rotation centre is at c and the heading is yaw.
    Pose origin(const Point& c, double yaw) const {
        return {c.x - ell_ * std::cos(yaw), c.y - ell_ * std::sin(yaw), yaw};
    }

    // Origin poses along the route at most `step` metres and `step_yaw` radians apart, with the arc
    // length of each. Turns in place are sampled at the vertex's s.
    struct Sample {
        Pose pose;
        double s;
    };
    std::vector<Sample> sample(double step, double step_yaw, bool unicycle) const {
        std::vector<Sample> out;
        if (vertices_.empty()) {
            return out;
        }
        double yaw = start_.yaw;
        double s = 0.0;
        out.push_back({origin(vertices_[0], yaw), 0.0});
        auto rotate = [&](const Point& c, double to) {
            const double delta = wrapAngle(to - yaw);
            const int n = std::max(1, static_cast<int>(std::ceil(std::abs(delta) / step_yaw)));
            for (int k = 1; k <= n; ++k) {
                out.push_back({origin(c, wrapAngle(yaw + delta * k / n)), s});
            }
            yaw = wrapAngle(yaw + delta);
        };
        for (size_t j = 0; j + 1 < vertices_.size(); ++j) {
            const double length = distance(vertices_[j], vertices_[j + 1]);
            if (length < 1e-9) {
                continue;
            }
            if (unicycle) {
                rotate(vertices_[j], headings_[j]);
            }
            const int n = std::max(1, static_cast<int>(std::ceil(length / step)));
            for (int k = 1; k <= n; ++k) {
                s += length / n;
                const Point c{vertices_[j].x + (vertices_[j + 1].x - vertices_[j].x) * k / n,
                              vertices_[j].y + (vertices_[j + 1].y - vertices_[j].y) * k / n};
                out.push_back({origin(c, yaw), s});
            }
        }
        if (unicycle) {
            rotate(vertices_.back(), goal_.yaw);
        } else {
            // An omnidirectional vehicle turns concurrently; its footprint is a disc, so the
            // heading of the samples does not matter. Keep the goal heading for the last sample.
            out.push_back({origin(vertices_.back(), goal_.yaw), s});
        }
        return out;
    }

    static double distance(const Point& a, const Point& b) {
        return std::hypot(b.x - a.x, b.y - a.y);
    }

   private:
    Pose start_, goal_;
    double ell_{0.0};
    std::vector<Point> vertices_;
    std::vector<double> cumulative_;
    std::vector<double> headings_;
    std::vector<int> directions_;
};

}  // namespace ugv_reset_client
