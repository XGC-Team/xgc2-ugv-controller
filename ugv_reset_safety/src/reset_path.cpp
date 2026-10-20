#include <ugv_reset_safety/reset_path.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace ugv_reset_safety {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kGeometryEpsilon = 1e-9;
using Polygon = std::vector<Eigen::Vector2d>;

double cross(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return a.x() * b.y() - a.y() * b.x();
}

// Validate cyclic convex input before removing redundant collinear vertices.
// Taking an arbitrary convex hull here would silently repair malformed scene
// geometry; invalid maps instead produce an explicit rejected plan.
bool normalizePolygon(const Polygon& input, Polygon* output) {
    if (input.size() < 3) {
        return false;
    }
    *output = input;
    double area = 0.0;
    for (std::size_t i = 0; i < output->size(); ++i) {
        const auto& a = (*output)[i];
        const auto& b = (*output)[(i + 1) % output->size()];
        if (!a.allFinite() || (b - a).norm() < kGeometryEpsilon) {
            return false;
        }
        area += cross(a, b);
    }
    if (!std::isfinite(area) || std::abs(area) < kGeometryEpsilon) {
        return false;
    }
    if (area < 0.0) {
        std::reverse(output->begin(), output->end());
    }
    for (std::size_t i = 0; i < output->size(); ++i) {
        const Eigen::Vector2d edge = (*output)[(i + 1) % output->size()] - (*output)[i];
        for (const auto& vertex : *output) {
            if (cross(edge, vertex - (*output)[i]) / edge.norm() < -kGeometryEpsilon) {
                return false;
            }
        }
    }
    bool changed = true;
    while (changed && output->size() > 3) {
        changed = false;
        for (std::size_t i = 0; i < output->size(); ++i) {
            const Eigen::Vector2d previous =
                (*output)[i] - (*output)[(i + output->size() - 1) % output->size()];
            const Eigen::Vector2d next = (*output)[(i + 1) % output->size()] - (*output)[i];
            if (std::abs(cross(previous, next)) <=
                kGeometryEpsilon * previous.norm() * next.norm()) {
                output->erase(output->begin() + static_cast<std::ptrdiff_t>(i));
                changed = true;
                break;
            }
        }
    }
    return true;
}

// Intersect outward-offset support lines. This miter polygon contains the
// Euclidean disk Minkowski sum (it can be more conservative at acute corners).
bool inflate(const Polygon& input, double distance, Polygon* output) {
    Polygon polygon;
    if (!normalizePolygon(input, &polygon)) {
        return false;
    }
    output->clear();
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const Eigen::Vector2d incoming =
            polygon[i] - polygon[(i + polygon.size() - 1) % polygon.size()];
        const Eigen::Vector2d outgoing = polygon[(i + 1) % polygon.size()] - polygon[i];
        const Eigen::Vector2d n1(incoming.y() / incoming.norm(), -incoming.x() / incoming.norm());
        const Eigen::Vector2d n2(outgoing.y() / outgoing.norm(), -outgoing.x() / outgoing.norm());
        const double determinant = cross(n1, n2);
        if (std::abs(determinant) < 1e-12) {
            return false;
        }
        const double b1 = n1.dot(polygon[i]) + distance;
        const double b2 = n2.dot(polygon[i]) + distance;
        Eigen::Vector2d vertex((b1 * n2.y() - n1.y() * b2) / determinant,
                               (n1.x() * b2 - b1 * n2.x()) / determinant);
        if (!vertex.allFinite()) {
            return false;
        }
        output->push_back(vertex);
    }
    return true;
}

bool insideInterior(const Eigen::Vector2d& point, const Polygon& polygon) {
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const Eigen::Vector2d edge = polygon[(i + 1) % polygon.size()] - polygon[i];
        if (cross(edge, point - polygon[i]) / edge.norm() <= kGeometryEpsilon) {
            return false;
        }
    }
    return true;
}

bool inFence(const Eigen::Vector2d& point, const Fence& fence, double erosion) {
    return !fence.enabled || (point.x() >= fence.xmin + erosion - kGeometryEpsilon &&
                              point.x() <= fence.xmax - erosion + kGeometryEpsilon &&
                              point.y() >= fence.ymin + erosion - kGeometryEpsilon &&
                              point.y() <= fence.ymax - erosion + kGeometryEpsilon);
}

// Clip the segment against the strict interior halfspaces. Boundary segments
// of the expanded obstacle are permitted; the underlying body remains clear.
bool entersInterior(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Polygon& polygon) {
    double lower = 0.0;
    double upper = 1.0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const Eigen::Vector2d edge = polygon[(i + 1) % polygon.size()] - polygon[i];
        const double initial = cross(edge, a - polygon[i]) / edge.norm();
        const double slope = cross(edge, b - a) / edge.norm();
        if (std::abs(slope) < 1e-14) {
            if (initial <= kGeometryEpsilon) {
                return false;
            }
        } else {
            const double boundary = (kGeometryEpsilon - initial) / slope;
            if (slope > 0.0) {
                lower = std::max(lower, boundary);
            } else {
                upper = std::min(upper, boundary);
            }
        }
        if (lower >= upper - 1e-12) {
            return false;
        }
    }
    return lower < upper - 1e-12;
}

}  // namespace

bool validConvexObstacle(const ConvexObstacle& obstacle) {
    Polygon normalized;
    return normalizePolygon(obstacle.vertices, &normalized);
}

VisibilityPath planVisibilityPath(const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
                                  const std::vector<ConvexObstacle>& obstacles, const Fence& fence,
                                  double radius, double clearance) {
    VisibilityPath result;
    if (!start.allFinite() || !goal.allFinite() || !std::isfinite(radius) || radius < 0.0 ||
        !std::isfinite(clearance) || clearance < 0.0 ||
        (fence.enabled &&
         (!std::isfinite(fence.xmin) || !std::isfinite(fence.xmax) || !std::isfinite(fence.ymin) ||
          !std::isfinite(fence.ymax) || fence.xmin >= fence.xmax || fence.ymin >= fence.ymax))) {
        result.invalid_input = true;
        result.message = "Invalid pose, footprint, clearance, or fence";
        return result;
    }
    const double expansion = radius + clearance;
    std::vector<Polygon> expanded;
    std::size_t vertex_count = 2;
    for (const auto& obstacle : obstacles) {
        Polygon polygon;
        vertex_count += obstacle.vertices.size();
        if (vertex_count > 1024 || !inflate(obstacle.vertices, expansion, &polygon)) {
            result.invalid_input = true;
            result.message =
                "Invalid convex obstacle or visibility graph capacity exceeded: " + obstacle.id;
            return result;
        }
        expanded.push_back(std::move(polygon));
    }
    auto free_point = [&](const Eigen::Vector2d& point) {
        if (!inFence(point, fence, expansion)) {
            return false;
        }
        for (const auto& polygon : expanded) {
            if (insideInterior(point, polygon)) {
                return false;
            }
        }
        return true;
    };
    if (!free_point(start) || !free_point(goal)) {
        result.message = "Start or target lacks required footprint clearance";
        return result;
    }
    std::vector<Eigen::Vector2d> nodes{start, goal};
    for (const auto& polygon : expanded) {
        for (const auto& point : polygon) {
            if (free_point(point)) {
                nodes.push_back(point);
            }
        }
    }

    const double infinity = std::numeric_limits<double>::infinity();
    const std::size_t count = nodes.size();
    std::vector<double> distances(count, infinity);
    std::vector<std::size_t> previous(count, count);
    std::vector<bool> visited(count, false);
    distances[0] = 0.0;
    for (std::size_t iteration = 0; iteration < count; ++iteration) {
        std::size_t current = count;
        for (std::size_t i = 0; i < count; ++i) {
            if (!visited[i] && (current == count || distances[i] < distances[current])) {
                current = i;
            }
        }
        if (current == count || !std::isfinite(distances[current])) {
            break;
        }
        if (current == 1) {
            break;
        }
        visited[current] = true;
        for (std::size_t next = 0; next < count; ++next) {
            if (visited[next] || next == current) {
                continue;
            }
            // Break equal-length detours to the robot's right, independent of
            // polygon winding and the peer's opposite direction of travel.
            const double tie =
                current == 0 && cross(goal - start, nodes[next] - start) > 0.0 ? 1.0e-6 : 0.0;
            const double candidate =
                distances[current] + (nodes[next] - nodes[current]).norm() + tie;
            if (candidate >= distances[next]) {
                continue;
            }
            bool visible = true;
            for (const auto& polygon : expanded) {
                if (entersInterior(nodes[current], nodes[next], polygon)) {
                    visible = false;
                    break;
                }
            }
            if (visible) {
                distances[next] = candidate;
                previous[next] = current;
            }
        }
    }
    if (!std::isfinite(distances[1])) {
        result.message = "No route within footprint clearance and fence";
        return result;
    }
    for (std::size_t vertex = 1; vertex != count; vertex = previous[vertex]) {
        result.points.push_back(nodes[vertex]);
        if (vertex == 0) {
            break;
        }
    }
    std::reverse(result.points.begin(), result.points.end());
    result.valid = true;
    return result;
}

}  // namespace ugv_reset_safety
