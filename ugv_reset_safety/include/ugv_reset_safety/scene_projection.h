#pragma once
#include <ugv_reset_safety/reset_geometry.h>
#include <xgc2_geometry_msgs/SceneSnapshot.h>
#include <xgc2_geometry_msgs/SceneState.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <xgc2_math/geometry/prepared_geometry.hpp>
namespace ugv_reset_safety {
namespace scene_projection {
inline constexpr double occupancyHorizon() {
    return 2.0;
}
inline bool supportedMotion(const std::string& type) {
    return type == "hold" || type == "constant_twist" || type == "ping_pong" || type == "circle";
}
inline std::string motionType(const xgc2_geometry_msgs::SceneObstacle& obstacle) {
    if (obstacle.motion_type.empty()) {
        return obstacle.dynamic ? std::string() : std::string("hold");
    }
    return obstacle.motion_type;
}
inline Eigen::Quaterniond rotation(const geometry_msgs::Quaternion& q) {
    Eigen::Quaterniond r(q.w, q.x, q.y, q.z);
    if (!r.coeffs().allFinite() || r.norm() < 1e-9)
        throw std::invalid_argument("invalid scene rotation");
    return r.normalized();
}
inline Eigen::Vector3d point(const geometry_msgs::Point& p) {
    Eigen::Vector3d v(p.x, p.y, p.z);
    if (!v.allFinite())
        throw std::invalid_argument("nonfinite scene position");
    return v;
}
inline Eigen::Vector3d vector(const geometry_msgs::Vector3& v) {
    Eigen::Vector3d value(v.x, v.y, v.z);
    if (!value.allFinite())
        throw std::invalid_argument("nonfinite scene twist");
    return value;
}
struct BodyState {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d linear = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular = Eigen::Vector3d::Zero();
};
struct PreparedPart {
    std::string id;
    xgc2_math::Pose3 pose;
    std::shared_ptr<const xgc2_math::geometry::PreparedGeometry> geometry;
};
struct PreparedObstacle {
    std::string id;
    bool dynamic = false;
    BodyState initial;
    std::vector<PreparedPart> parts;
    std::vector<ConvexObstacle> fixed;
};
struct PreparedScene {
    std::string epoch, frame;
    std::uint64_t revision = 0;
    std::vector<PreparedObstacle> obstacles;
};
inline ConvexObstacle projectPart(const std::string& obstacle_id, const PreparedPart& part,
                                  std::size_t index, const BodyState& body, double extra) {
    if (part.id.empty())
        throw std::invalid_argument("duplicate/empty scene part id");
    const Eigen::Quaterniond orientation = body.orientation * part.pose.orientation;
    const Eigen::Vector3d center = body.position + body.orientation * part.pose.position;
    constexpr int count = 32;
    constexpr double pi = 3.14159265358979323846;
    std::vector<Eigen::Vector2d> normals;
    std::vector<double> offsets;
    xgc2_math::Pose3 instance;
    instance.position = center;
    instance.orientation = orientation;
    xgc2_math::geometry::visitConvexPiece(
        part.geometry->compiledParts.at(index), instance, [&](const auto& shape) {
            for (int i = 0; i < count; ++i) {
                const double angle = 2 * pi * i / count;
                const Eigen::Vector3d direction(std::cos(angle), std::sin(angle), 0);
                normals.push_back(direction.head<2>());
                offsets.push_back(shape.support(direction).support_value + extra);
            }
        });
    ConvexObstacle projected;
    projected.id = std::to_string(obstacle_id.size()) + ":" + obstacle_id + part.id;
    if (part.geometry->compiledParts.size() > 1)
        projected.id += "/" + std::to_string(index);
    projected.origin = body.position.head<2>();
    projected.velocity = body.linear.head<2>();
    projected.omega = body.angular.z();
    for (int i = 0; i < count; ++i) {
        const int j = (i + 1) % count;
        Eigen::Matrix2d matrix;
        matrix.row(0) = normals[i];
        matrix.row(1) = normals[j];
        const Eigen::Vector2d vertex = matrix.inverse() * Eigen::Vector2d(offsets[i], offsets[j]);
        if (!vertex.allFinite())
            throw std::invalid_argument("scene projection overflow");
        if (projected.vertices.empty() || (projected.vertices.back() - vertex).norm() > 1e-9)
            projected.vertices.push_back(vertex);
    }
    if (projected.vertices.size() > 1 &&
        (projected.vertices.front() - projected.vertices.back()).norm() < 1e-9)
        projected.vertices.pop_back();
    if (projected.vertices.size() < 3)
        throw std::invalid_argument("degenerate scene projection");
    return projected;
}
inline void validateObstacle(const xgc2_geometry_msgs::SceneObstacle& obstacle,
                             std::set<std::string>* obstacle_ids) {
    if (obstacle.id.empty() || !obstacle_ids->insert(obstacle.id).second || obstacle.parts.empty())
        throw std::invalid_argument("invalid obstacle identity or empty parts");
    const std::string type = motionType(obstacle);
    if (!supportedMotion(type))
        throw std::invalid_argument("unsupported motion type: " +
                                    (type.empty() ? std::string("<empty>") : type));
    if (obstacle.dynamic != (type != "hold"))
        throw std::invalid_argument("dynamic flag disagrees with declared motion type");
    std::set<std::string> part_ids;
    for (const auto& part : obstacle.parts) {
        if (part.id.empty() || !part_ids.insert(part.id).second)
            throw std::invalid_argument("duplicate/empty scene part id");
        rotation(part.pose.orientation);
        point(part.pose.position);
    }
}
inline BodyState snapshotBody(const xgc2_geometry_msgs::SceneObstacle& obstacle) {
    BodyState body;
    body.orientation = rotation(obstacle.pose.orientation);
    body.position = point(obstacle.pose.position);
    return body;
}
inline BodyState liveBody(const xgc2_geometry_msgs::SceneObstacleState& state) {
    BodyState body;
    body.orientation = rotation(state.pose.orientation);
    body.position = point(state.pose.position);
    body.linear = vector(state.twist.linear);
    body.angular = vector(state.twist.angular);
    return body;
}
inline double radiusBound(const ConvexObstacle& projected) {
    double rho = 0.0;
    for (const auto& vertex : projected.vertices)
        rho = std::max(rho, (vertex - projected.origin).norm());
    return rho;
}
inline PreparedScene prepare(const xgc2_geometry_msgs::SceneSnapshot& scene,
                             const std::string& frame,
                             xgc2_math::geometry::PreparedGeometryCache& cache) {
    if (scene.epoch.empty() || scene.header.frame_id != frame)
        throw std::invalid_argument("scene epoch/frame mismatch");
    PreparedScene result;
    result.epoch = scene.epoch;
    result.revision = scene.revision;
    result.frame = frame;
    std::set<std::string> ids;
    for (const auto& obstacle : scene.obstacles) {
        validateObstacle(obstacle, &ids);
        PreparedObstacle body;
        body.id = obstacle.id;
        body.dynamic = obstacle.dynamic;
        body.initial = snapshotBody(obstacle);
        for (const auto& part : obstacle.parts) {
            PreparedPart prepared;
            prepared.id = part.id;
            prepared.pose.position = point(part.pose.position);
            prepared.pose.orientation = rotation(part.pose.orientation);
            try {
                prepared.geometry = cache.prepare({part.geometry.type, part.geometry.parameters});
            } catch (const std::exception& error) {
                throw std::invalid_argument("obstacle " + obstacle.id + " part " + part.id + ": " +
                                            error.what());
            }
            body.parts.push_back(std::move(prepared));
        }
        if (!body.dynamic) {
            for (const auto& part : body.parts)
                for (std::size_t i = 0; i < part.geometry->compiledParts.size(); ++i)
                    body.fixed.push_back(projectPart(body.id, part, i, body.initial, 0.0));
        }
        result.obstacles.push_back(std::move(body));
    }
    return result;
}
inline std::vector<ConvexObstacle> projectBodies(const PreparedScene& scene,
                                                 const std::map<std::string, BodyState>& bodies,
                                                 const std::string& frame, bool occupancy) {
    if (scene.epoch.empty() || scene.frame != frame)
        throw std::invalid_argument("scene epoch/frame mismatch");
    std::vector<ConvexObstacle> result;
    for (const auto& obstacle : scene.obstacles) {
        if (!obstacle.dynamic) {
            result.insert(result.end(), obstacle.fixed.begin(), obstacle.fixed.end());
            continue;
        }
        const auto found = bodies.find(obstacle.id);
        if (found == bodies.end())
            throw std::invalid_argument("scene state missing obstacle " + obstacle.id);
        const BodyState& body = found->second;
        const double speed = body.linear.head<2>().norm();
        const double yaw_rate = std::abs(body.angular.z());
        for (const auto& part : obstacle.parts) {
            for (std::size_t i = 0; i < part.geometry->compiledParts.size(); ++i) {
                auto live = projectPart(obstacle.id, part, i, body, 0.0);
                const double extra =
                    occupancy ? (speed + yaw_rate * radiusBound(live)) * occupancyHorizon() : 0.0;
                result.push_back(extra > 0.0 ? projectPart(obstacle.id, part, i, body, extra)
                                             : std::move(live));
            }
        }
    }
    if (bodies.size() != scene.obstacles.size())
        throw std::invalid_argument("scene state has extra obstacle identities");
    std::sort(result.begin(), result.end(),
              [](const auto& a, const auto& b) { return a.id < b.id; });
    return result;
}
inline std::map<std::string, BodyState> snapshotBodies(const PreparedScene& scene) {
    std::map<std::string, BodyState> bodies;
    for (const auto& obstacle : scene.obstacles)
        bodies.emplace(obstacle.id, obstacle.initial);
    return bodies;
}
inline std::map<std::string, BodyState> liveBodies(const PreparedScene& scene,
                                                   const xgc2_geometry_msgs::SceneState& state) {
    if (state.epoch != scene.epoch || state.revision != scene.revision)
        throw std::invalid_argument("scene state epoch/revision mismatch");
    if (state.header.frame_id != scene.frame)
        throw std::invalid_argument("scene state frame mismatch");
    if (!std::isfinite(state.scene_time) || state.scene_time < 0.0)
        throw std::invalid_argument("invalid scene time");
    std::map<std::string, BodyState> bodies;
    std::set<std::string> dynamic;
    for (const auto& obstacle : scene.obstacles) {
        if (obstacle.dynamic)
            dynamic.insert(obstacle.id);
        else
            bodies.emplace(obstacle.id, obstacle.initial);
    }
    for (const auto& obstacle : state.obstacles) {
        if (!dynamic.count(obstacle.id) || bodies.count(obstacle.id))
            throw std::invalid_argument(
                "scene state has extra/duplicate dynamic obstacle identities");
        bodies.emplace(obstacle.id, liveBody(obstacle));
    }
    for (const auto& id : dynamic)
        if (!bodies.count(id))
            throw std::invalid_argument("scene state missing dynamic obstacle " + id);
    return bodies;
}
inline std::vector<ConvexObstacle> project(const PreparedScene& scene, const std::string& frame) {
    for (const auto& obstacle : scene.obstacles)
        if (obstacle.dynamic)
            throw std::invalid_argument("live SceneState required for dynamic motion");
    return projectBodies(scene, snapshotBodies(scene), frame, false);
}
inline std::vector<ConvexObstacle> project(const xgc2_geometry_msgs::SceneSnapshot& scene,
                                           const std::string& frame) {
    xgc2_math::geometry::PreparedGeometryCache cache;
    return project(prepare(scene, frame, cache), frame);
}
struct SceneGeometry {
    std::vector<ConvexObstacle> live;
    std::vector<ConvexObstacle> occupancy;
};
inline SceneGeometry live(const PreparedScene& scene, const xgc2_geometry_msgs::SceneState& state,
                          const std::string& frame) {
    const auto bodies = liveBodies(scene, state);
    SceneGeometry result;
    result.live = projectBodies(scene, bodies, frame, false);
    result.occupancy = projectBodies(scene, bodies, frame, true);
    return result;
}
inline SceneGeometry live(const xgc2_geometry_msgs::SceneSnapshot& scene,
                          const xgc2_geometry_msgs::SceneState& state, const std::string& frame) {
    xgc2_math::geometry::PreparedGeometryCache cache;
    return live(prepare(scene, frame, cache), state, frame);
}
}  // namespace scene_projection
}  // namespace ugv_reset_safety
