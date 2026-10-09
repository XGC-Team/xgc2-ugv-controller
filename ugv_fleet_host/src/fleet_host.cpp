#include <mecanum_ugv_controller/mecanum_ugv_ros_node.h>
#include <ros/ros.h>
#include <ugv_fleet_host/fleet_runtime.h>
#include <ugv_fleet_host/json.h>
#include <ugv_fleet_host/reset_http_boundary.h>
#include <ugv_fleet_host/scene_cache.h>
#include <ugv_reset_safety/fixed_executor.h>
#include <unicycle_ugv_controller/ros_log_sink.h>
#include <unicycle_ugv_controller/unicycle_fleet_edge.h>

#include <cmath>
#include <csignal>
#include <iostream>
#include <set>
#include <xgc2/xrpc/runtime_policy.hpp>
extern char** environ;
namespace {
volatile std::sig_atomic_t shutdown_requested = 0;
void requestShutdown(int) {
    shutdown_requested = 1;
}
double number(const XmlRpc::XmlRpcValue& value, const std::string& field) {
    const auto& v = value[field];
    double n;
    if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        n = static_cast<double>(v);
    else if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
        n = static_cast<int>(v);
    else
        throw std::invalid_argument("numeric robot field required: " + field);
    if (!std::isfinite(n))
        throw std::invalid_argument("finite robot field required: " + field);
    return n;
}
ugv_reset_safety::NativeClock clockNow() {
    return {{ros::Time::now().toNSec()}, {ugv_reset_safety::monotonicSeconds()}};
}
xgc2::xrpc::ServiceRef sceneReference(const Json::Value& value) {
    const auto& ref = ugv_fleet_host::wire::object(value);
    const auto& endpoint =
        ugv_fleet_host::wire::object(ugv_fleet_host::wire::member(ref, "endpoint"));
    if (ref.size() != 6 || endpoint.size() != 2)
        throw std::invalid_argument("complete frozen scene ServiceRef required");
    const auto field = [](const Json::Value& object, const char* name) {
        return std::string(
            ugv_fleet_host::wire::string(ugv_fleet_host::wire::member(object, name)));
    };
    return {field(ref, "target_id"),   field(ref, "service"),
            field(ref, "api_version"), field(ref, "instance_id"),
            field(ref, "profile"),     {field(endpoint, "kind"), field(endpoint, "address")}};
}
}  // namespace
int main(int argc, char** argv) {
    try {
        std::string bootstrap_path;
        std::vector<char*> ros_arguments{argv[0]};
        for (int i = 1; i < argc; ++i) {
            const std::string argument(argv[i]);
            if (argument == "--bootstrap-input") {
                if (!bootstrap_path.empty() || ++i == argc)
                    throw std::invalid_argument("one --bootstrap-input value required");
                bootstrap_path = argv[i];
            } else if (argument.find(":=") != std::string::npos &&
                       (argument[0] != '_' || argument.starts_with("__"))) {
                ros_arguments.push_back(argv[i]);
            } else {
                throw std::invalid_argument(
                    "only --bootstrap-input and standard ROS remappings are accepted");
            }
        }
        if (bootstrap_path.empty())
            throw std::invalid_argument("--bootstrap-input is required");
        auto bootstrap = xgc2::xrpc::loadBootstrapInput(bootstrap_path);
        const auto application = bootstrap.application_json();
        if (!application)
            throw std::invalid_argument("frozen native application input required");
        const auto native = ugv_fleet_host::wire::parse(std::string(*application));
        if (!native.isObject() || native.size() != 2)
            throw std::invalid_argument("sceneServiceRef and worldBoundary are required");
        auto scene_reference =
            sceneReference(ugv_fleet_host::wire::member(native, "sceneServiceRef"));
        const auto& boundary = ugv_fleet_host::wire::member(native, "worldBoundary");
        int ros_argc = static_cast<int>(ros_arguments.size());
        ros::init(ros_argc, ros_arguments.data(), "ugv_fleet_host",
                  ros::init_options::NoSigintHandler);
        std::signal(SIGINT, requestShutdown);
        std::signal(SIGTERM, requestShutdown);
        ros::NodeHandle nh, private_nh("~");
        xgc2::xrpc::RuntimePolicyOptions policy_input;
        for (char** item = environ; *item; ++item) {
            const std::string entry(*item);
            const auto equals = entry.find('=');
            if (equals != std::string::npos)
                policy_input.environment.emplace_back(entry.substr(0, equals),
                                                      entry.substr(equals + 1));
        }
        const auto policy = xgc2::xrpc::resolve_runtime_policy(policy_input);
        ugv_reset_safety::FleetCoordinator::Configuration config;
        private_nh.param("frequency", config.frequency, 50.0);
        private_nh.param("input_timeout", config.input_timeout, .15);
        private_nh.param("state_timeout", config.state_timeout, 1.0);
        private_nh.param("world_frame", config.world_frame, std::string("world"));
        private_nh.param("obstacle_avoidance", config.obstacle_avoidance, true);
        private_nh.param("clearance", config.dwa.clearance, .08);
        private_nh.param("uncertainty_margin", config.dwa.uncertainty_margin, .03);
        config.fence.enabled = true;
        if (!private_nh.getParam("fence/x_min", config.fence.xmin) ||
            !private_nh.getParam("fence/x_max", config.fence.xmax) ||
            !private_nh.getParam("fence/y_min", config.fence.ymin) ||
            !private_nh.getParam("fence/y_max", config.fence.ymax))
            throw std::invalid_argument("explicit fence required");
        if (!boundary.isNull()) {
            const auto& object = boundary;
            if (object.size() != 5 ||
                ugv_fleet_host::wire::member(object, "schemaVersion").type() != Json::intValue ||
                ugv_fleet_host::wire::member(object, "schemaVersion") != 1 ||
                ugv_fleet_host::wire::member(object, "frameId").asString() != "world" ||
                ugv_fleet_host::wire::member(object, "unit").asString() != "m")
                throw std::invalid_argument("current Session worldBoundary required");
            const auto finite = [](const Json::Value& v) {
                double n = v.isDouble()
                               ? v.asDouble()
                               : v.isInt64()
                                     ? static_cast<double>(v.asInt64())
                                     : v.isUInt64() ? static_cast<double>(v.asUInt64()) : NAN;
                if (!std::isfinite(n))
                    throw std::invalid_argument("finite boundary required");
                return n;
            };
            if (!ugv_fleet_host::wire::member(object, "groundZ").isNull())
                finite(ugv_fleet_host::wire::member(object, "groundZ"));
            if (!ugv_fleet_host::wire::member(object, "controlBounds").isNull()) {
                const auto& bounds = ugv_fleet_host::wire::member(object, "controlBounds");
                if (bounds.size() != 6)
                    throw std::invalid_argument("six world boundary endpoints required");
                for (const auto* axis : {"x", "y", "z"})
                    if (finite(ugv_fleet_host::wire::member(bounds, std::string(axis) + "Min")) >=
                        finite(ugv_fleet_host::wire::member(bounds, std::string(axis) + "Max")))
                        throw std::invalid_argument("ordered world boundary required");
                config.fence.xmin = finite(ugv_fleet_host::wire::member(bounds, "xMin"));
                config.fence.xmax = finite(ugv_fleet_host::wire::member(bounds, "xMax"));
                config.fence.ymin = finite(ugv_fleet_host::wire::member(bounds, "yMin"));
                config.fence.ymax = finite(ugv_fleet_host::wire::member(bounds, "yMax"));
            }
        }
        XmlRpc::XmlRpcValue values;
        if (!private_nh.getParam("robots", values) ||
            values.getType() != XmlRpc::XmlRpcValue::TypeArray || values.size() < 1 ||
            values.size() > 128)
            throw std::invalid_argument("bounded complete robots roster required");
        int workers = 2;
        private_nh.param("nmpc_workers", workers, 2);
        ugv_reset_safety::FixedExecutor executor(workers);
        std::vector<ugv_reset_safety::Robot> robots;
        std::vector<std::string> ids;
        std::vector<std::unique_ptr<ugv_reset_safety::FleetEdge>> edges;
        std::set<std::string> unique;
        unicycle_ugv_controller::installRosLogSink();
        for (int i = 0; i < values.size(); ++i) {
            const auto& value = values[i];
            ugv_reset_safety::Robot robot;
            robot.id = static_cast<std::string>(value["namespace"]);
            if (robot.id.empty() || robot.id.size() > 127 ||
                robot.id.find('/') != std::string::npos ||
                robot.id.find("..") != std::string::npos || !unique.insert(robot.id).second)
                throw std::invalid_argument("unique robot namespace required");
            const auto type = static_cast<std::string>(value["type"]);
            if (type == "scout")
                robot.type = ugv_reset_safety::RobotType::Unicycle;
            else if (type == "mecanum")
                robot.type = ugv_reset_safety::RobotType::Mecanum;
            else
                throw std::invalid_argument("unknown robot type");
            robot.half_length = number(value, "length") / 2;
            robot.half_width = number(value, "width") / 2;
            robot.body_center_offset = {number(value, "body_offset_x"),
                                        number(value, "body_offset_y")};
            robot.lateral_velocity_per_yaw_bound = number(value, "lateral_velocity_per_yaw_bound");
            robot.limits.max_vx = number(value, "max_vx");
            robot.limits.max_vy = number(value, "max_vy");
            robot.limits.max_omega = number(value, "max_omega");
            robot.limits.accel_vx = number(value, "accel_vx");
            robot.limits.accel_vy = number(value, "accel_vy");
            robot.limits.accel_omega = number(value, "accel_omega");
            ros::NodeHandle robot_nh("/" + robot.id);
            if (robot.type == ugv_reset_safety::RobotType::Mecanum)
                edges.push_back(mecanum_ugv_controller::createFleetEdge(
                    robot_nh, ros::NodeHandle(robot_nh, "mecanum_ugv_controller")));
            else
                edges.push_back(unicycle_ugv_controller::createFleetEdge(
                    robot_nh, ros::NodeHandle(robot_nh, "unicycle_ugv_controller"), executor, i));
            ids.push_back(robot.id);
            robots.push_back(std::move(robot));
        }
        double host_frequency = 500;
        for (const auto& edge : edges)
            host_frequency = std::max(host_frequency, edge->controlRate());
        ugv_fleet_host::FleetRuntime runtime(std::move(edges), std::move(robots), config,
                                             clockNow());
        ugv_fleet_host::SceneCache scene(std::move(scene_reference), policy);
        ugv_fleet_host::ResetHttpBoundary host(bootstrap, ids, policy);
        // All controller update rates were 500 Hz. Coordinator math stays at its
        // existing configured 50 Hz; SDK management and scene IO have fixed owners.
        ros::WallRate turn_rate(host_frequency);
        ugv_fleet_host::SceneSnapshot snapshot, incoming;
        bool pending_scene = false;
        while (ros::ok() && !shutdown_requested) {
            ros::spinOnce();
            const auto clock = clockNow();
            if (scene.take(incoming)) {
                const bool identity_changed =
                    pending_scene &&
                    (incoming.definition.epoch != snapshot.definition.epoch ||
                     incoming.definition.revision != snapshot.definition.revision ||
                     incoming.state.simulation_time.epoch != snapshot.state.simulation_time.epoch);
                if (!incoming.applied) {
                    runtime.invalidateScene(clock);
                    pending_scene = false;
                } else if (!pending_scene || identity_changed) {
                    if (identity_changed)
                        runtime.invalidateScene(clock);
                    snapshot = std::move(incoming);
                    pending_scene = true;
                }
            }
            // Retain one exact observation until the ordinary world clock catches
            // its stamp. Continually replacing it with a newer future stamp would
            // starve admission when native SDK and ROS input arrivals interleave.
            if (pending_scene && snapshot.state.header.stamp <= clock.world) {
                runtime.scene(snapshot.definition, snapshot.state, clock);
                pending_scene = false;
            }
            if (scene.failed())
                runtime.invalidateScene(clock);
            host.ownerTurn(runtime.control(), clock.world.toNSec(), clock.monotonic.seconds);
            runtime.turn(clock);
            host.publish(runtime.control().snapshot());
            turn_rate.sleep();
        }
        runtime.stopAll();
        // Execute the original Stop transition and sole zero publishers before the
        // owner and solver edges are destroyed. No fabricated completion is emitted.
        for (int i = 0; i < 3; ++i)
            runtime.turn(clockNow());
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[UgvFleetHost] " << error.what() << '\n';
        return 1;
    }
}
