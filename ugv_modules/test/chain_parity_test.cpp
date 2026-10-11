// The module chain against the chain of the two ROS nodes, without ROS, on the replay scenario of
// the reference generator and a kinematic unicycle.
//
// Reference node path: ReferenceTrajectoryDriver, driven as unicycle_reference_trajectory_node
// drives it at 100 Hz, with the requests of reference_replay_scenario.h (the scenario that gates
// the core's refactoring), publishing what the node's output consumer publishes. Controller node
// path: UnicycleUgvController, driven as unicycle_ugv_controller_node drives it at 500 Hz, with the
// input producers' work done as they do it (state, references, commands) and the NMPC solved the
// way NmpcExecution solves it, one request at a time. Module chain: the two shared libraries in
// in-test hosts, the outputs of the generator moved to the controller's inputs, the controller's
// solver on its worker thread.
//
// What the node path publishes is what the modules must write, bit for bit: every output of the
// generator on every step, the twist of the controller on every control period, and the control
// state whenever the controller wrote its status. The solver's worker is waited for after each
// request, so the result is taken in the next step, as in the node path.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "reference_payloads.h"
#include "reference_replay_scenario.h"
#include "test_host.h"
#include "unicycle_reference_trajectory/reference_driver.h"
#include "unicycle_ugv_controller/common/reference_types.h"
#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/config_loader.h"
#include "unicycle_ugv_controller/nmpc/nmpc_execution.h"
#include "unicycle_ugv_controller/nmpc/nmpc_tracking_backend.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"
#include "xgc2_ugv/payloads.h"

namespace {

using ugv_modules_test::ModuleLibrary;
using ugv_modules_test::TestHost;
namespace rp = ugv_modules::reference_payloads;
namespace urt = unicycle_reference_trajectory;
namespace ugv = unicycle_ugv_controller;
namespace replay = reference_replay;

constexpr int64_t kSecond = 1000000000LL;
constexpr int64_t kControlStep = 2000000LL;  // 500 Hz
constexpr int64_t kReferenceEvery = 5;       // the generator runs every 5th control step
constexpr int64_t kT0 = 1000 * kSecond;      // replay::kT0
constexpr int64_t kStateEvery = 5;           // the vehicle's state arrives at 100 Hz
constexpr double kEnd = 26.0;                // seconds, the replay scenario's length
constexpr int64_t kSteps = static_cast<int64_t>(kEnd * 500.0);

struct Plant {
    double x{0.0}, y{0.0}, yaw{0.0}, v{0.0}, omega{0.0};

    void step(double linear, double angular, double dt) {
        v = linear;
        omega = angular;
        x += v * std::cos(yaw) * dt;
        y += v * std::sin(yaw) * dt;
        yaw = ugv::wrapAngle(yaw + omega * dt);
    }
};

// The operator's commands of the run.
struct Command {
    double at;
    uint32_t kind;
};
const Command kCommands[] = {{1.0, XGC2_UGV_COMMAND_CUSTOM1},
                             {12.0, XGC2_UGV_COMMAND_STOP},
                             {12.6, XGC2_UGV_COMMAND_CUSTOM1}};

// What the requests of the replay scenario are made of (the types of the plain messages).
struct ScriptGlue {
    using Analytic = urt::reference::AnalyticReference;
    using Sampled = urt::reference::SampledReference;
    using Waypoint = urt::reference::WaypointReferenceRequest;
    using Pose = urt::reference::Pose;
    urt::Time time(double sec) const {
        return sec <= 0.0 ? urt::Time() : urt::Time(sec);
    }
};

using Script = replay::Script<ScriptGlue>;

Script makeScript(ScriptGlue& glue) {
    Script script(glue);
    replay::buildScript(script, replay::Config{});
    std::stable_sort(script.items.begin(), script.items.end(),
                     [](const auto& a, const auto& b) { return a.at < b.at; });
    return script;
}

// A configuration source with nothing in it: every parameter keeps its default.
struct NoParameters {
    template <typename T>
    bool get(const std::string&, T&) const {
        return false;
    }
    bool has(const std::string&) const {
        return false;
    }
    bool isMapping(const std::string&) const {
        return false;
    }
    std::vector<std::string> keys(const std::string&) const {
        return {};
    }
};

// One output of the generator: the port, its commit stamp and the payload's bytes.
struct RefOutput {
    std::string port;
    int64_t stamp_ns;
    std::vector<uint8_t> bytes;

    bool operator==(const RefOutput& o) const {
        return port == o.port && stamp_ns == o.stamp_ns && bytes == o.bytes;
    }
};

template <typename T>
std::vector<uint8_t> bytesOf(const T& value) {
    std::vector<uint8_t> bytes(sizeof(T));
    std::memcpy(bytes.data(), &value, sizeof(T));
    return bytes;
}

// One control period of the controller as the chassis sees it.
struct Control {
    bool published{false};
    uint32_t kind{0};
    double linear{0.0};
    double angular{0.0};
    uint32_t control_state{0};  // the CONTROL region after the update
    bool solve_requested{false};
    bool worker_solve{
        false};  // the request went to the solver (the reference horizon was available)

    bool sameTwist(const Control& o) const {
        return published == o.published &&
               (!published || (kind == o.kind && linear == o.linear && angular == o.angular));
    }
};

struct Trace {
    std::vector<RefOutput> reference;
    std::vector<Control> control;
    std::vector<double> plant_x, plant_y;
    // control state as the status port reported it: step index -> name
    std::vector<std::pair<int64_t, std::string>> statuses;
};

// ---- the node path
// -------------------------------------------------------------------------------------

ugv::reference::AnalyticReference toController(const urt::reference::AnalyticReference& m) {
    ugv::reference::AnalyticReference r;
    r.request_id = m.request_id;
    r.trajectory_id = m.trajectory_id;
    r.revision = m.revision;
    r.analytic_type = m.analytic_type;
    r.flags = m.flags;
    r.start_time = ugv::Time(m.start_time.sec, m.start_time.nsec);
    r.duration = m.duration;
    r.origin.position = {m.origin.position.x, m.origin.position.y, m.origin.position.z};
    r.origin.orientation = {m.origin.orientation.x, m.origin.orientation.y, m.origin.orientation.z,
                            m.origin.orientation.w};
    r.params = m.params;
    return r;
}

ugv::reference::SampledReference toController(const urt::reference::SampledReference& m) {
    ugv::reference::SampledReference r;
    r.trajectory_id = m.trajectory_id;
    r.revision = m.revision;
    r.flags = m.flags;
    r.start_time = ugv::Time(m.start_time.sec, m.start_time.nsec);
    r.sample_dt = m.sample_dt;
    for (const auto& p : m.points) {
        r.points.push_back({p.t_from_start, p.x, p.y, p.yaw, p.speed, p.linear_acceleration,
                            p.yaw_rate, p.yaw_acceleration, p.curvature, p.vx, p.vy, p.ax, p.ay,
                            p.jx, p.jy});
    }
    return r;
}

ugv::reference::ActivePolynomialReference toController(
    const urt::reference::ActivePolynomialReference& m) {
    ugv::reference::ActivePolynomialReference r;
    r.trajectory_id = m.trajectory_id;
    r.revision = m.revision;
    r.flags = m.flags;
    r.start_time = ugv::Time(m.start_time.sec, m.start_time.nsec);
    r.duration = m.duration;
    r.order = m.order;
    r.segment_durations = m.segment_durations;
    r.coeff_x = m.coeff_x;
    r.coeff_y = m.coeff_y;
    r.coeff_yaw = m.coeff_yaw;
    return r;
}

Trace nodePath() {
    Trace run;
    // generator
    urt::ReferenceTrajectoryDriver driver;
    driver.configure(urt::ReferenceTrajectoryConfig{}, urt::DefaultAnalyticReferenceConfig{});
    ScriptGlue glue;
    Script script = makeScript(glue);
    size_t next_request = 0;
    // controller
    ugv::UgvState state;
    ugv::UnicycleUgvController controller(state);
    ugv::ControllerConfig config;
    ugv::loadControllerConfig(NoParameters(), config);  // the node's defaults, as it loads them
    controller.setConfig(config);
    ugv::NmpcTrackingBackend backend;
    backend.configure(controller.config());
    bool entered = false;
    Plant plant;
    double linear = 0.0, angular = 0.0;
    size_t next_command = 0;

    auto post = [&](uint32_t id, double stamp, const char* source) {
        ::state_machine::Event event(id, ::state_machine::EventTimestamp{stamp});
        event.source = source;
        event.category = ::state_machine::EventCategory::kInput;
        (void)controller.postEvent(std::move(event));
    };

    for (int64_t j = 0; j < kSteps; ++j) {
        const int64_t ns = kT0 + j * kControlStep;
        ugv::Time stamp;
        stamp.fromNSec(static_cast<uint64_t>(ns));
        const double now = stamp.toSec();

        // the vehicle's state (StateInputProducer::stateCallback)
        if (j % kStateEvery == 0) {
            state.stamp = stamp;
            state.x = plant.x;
            state.y = plant.y;
            state.yaw = plant.yaw;
            state.vx = plant.v * std::cos(plant.yaw);
            state.vy = plant.v * std::sin(plant.yaw);
            state.speed = plant.v;
            state.yaw_rate = plant.omega;
            state.estimator_state = 3;
            state.estimator_flags = 0;
            state.received = true;
            state.velocity_valid = true;
            post(ugv::event_type::INPUT_STATE_UPDATED, now, "state_estimate");
        }

        // the generator, every 5th period (unicycle_reference_trajectory_node's main loop)
        std::vector<std::pair<std::string, std::vector<uint8_t>>> published;
        if (j % kReferenceEvery == 0) {
            const double reference_now = urt::Time().fromNSec(static_cast<uint64_t>(ns)).toSec();
            while (next_request < script.items.size() &&
                   replay::kT0 + script.items[next_request].at <= reference_now) {
                auto& item = script.items[next_request++];
                switch (item.kind) {
                    case replay::Kind::kAnalytic:
                        driver.acceptAnalytic(item.analytic, reference_now);
                        break;
                    case replay::Kind::kSampled:
                        driver.acceptSampled(item.sampled, reference_now);
                        break;
                    case replay::Kind::kWaypoint:
                        driver.acceptWaypoint(item.waypoint, reference_now);
                        break;
                    case replay::Kind::kReset:
                        (void)driver.reset(reference_now);
                        break;
                }
            }
            const auto update = driver.update(reference_now);
            const urt::ReferenceTrajectoryRuntime& runtime = driver.runtime();
            for (const auto& event : update.events) {
                if (event.id == urt::output_event_type::PUBLISH_STATUS) {
                    xgc2_ugv_reference_status p;
                    std::memset(&p, 0, sizeof p);
                    rp::toPayload(p, runtime.makeStatus(event.timestamp > 0.0 ? event.timestamp
                                                                              : reference_now));
                    run.reference.push_back({"status", ns, bytesOf(p)});
                } else if (event.id == urt::output_event_type::PUBLISH_ACTIVE_ANALYTIC) {
                    const auto& m = runtime.activeAnalyticMessage();
                    auto p = std::make_unique<xgc2_ugv_analytic_reference>();
                    std::memset(p.get(), 0, sizeof *p);
                    rp::toPayload(*p, m);
                    run.reference.push_back({"active_analytic", ns, bytesOf(*p)});
                    if (controller.referenceCache().updateAnalytic(toController(m))) {
                        post(ugv::event_type::INPUT_REFERENCE_UPDATED, now, "active_analytic");
                    }
                } else if (event.id == urt::output_event_type::PUBLISH_ACTIVE_POLYNOMIAL) {
                    const auto& m = runtime.activePolynomialMessage();
                    auto p = std::make_unique<xgc2_ugv_polynomial_reference>();
                    std::memset(p.get(), 0, sizeof *p);
                    rp::toPayload(*p, m);
                    run.reference.push_back({"active_polynomial", ns, bytesOf(*p)});
                    if (controller.referenceCache().updatePolynomial(toController(m))) {
                        post(ugv::event_type::INPUT_REFERENCE_UPDATED, now, "active_polynomial");
                    }
                } else if (event.id == urt::output_event_type::PUBLISH_ACTIVE_SAMPLED) {
                    const auto& m = runtime.activeSampledMessage();
                    auto p = std::make_unique<xgc2_ugv_sampled_reference>();
                    std::memset(p.get(), 0, sizeof *p);
                    rp::toPayload(*p, m);
                    run.reference.push_back({"active_sampled", ns, bytesOf(*p)});
                    if (controller.referenceCache().updateSampled(toController(m))) {
                        post(ugv::event_type::INPUT_REFERENCE_UPDATED, now, "active_sampled");
                    }
                }
            }
        }

        // the operator
        while (next_command < sizeof(kCommands) / sizeof(kCommands[0]) &&
               kT0 + static_cast<int64_t>(kCommands[next_command].at * 1e9) <= ns) {
            const uint32_t kind = kCommands[next_command++].kind;
            post(kind == XGC2_UGV_COMMAND_CUSTOM1
                     ? ugv::event_type::CUSTOM1_REQUESTED
                     : kind == XGC2_UGV_COMMAND_STOP ? ugv::event_type::STOP_REQUESTED
                                                     : ugv::event_type::RESET_REQUESTED,
                 now, "command");
        }

        // the controller's loop (UnicycleUgvRosNode::updateOnce)
        controller.update(now);
        Control control;
        for (const auto& event : controller.stateMachine().currentOutputEvents()) {
            if (event.id == ugv::output_event_type::REQUEST_NMPC_SOLVE) {
                control.solve_requested = true;
                ugv::NmpcRequest request;
                ugv::NmpcOutcome outcome;
                if (ugv::makeNmpcRequest(controller, event, now, request)) {
                    control.worker_solve = true;
                    outcome = ugv::solveNmpcRequest(backend, entered, request);
                }
                (void)controller.postEvent(
                    ugv::makeNmpcResultEvent(event.correlation_id, outcome, now));
            } else if (event.id == ugv::output_event_type::PUBLISH_CMD_VEL) {
                const ugv::CmdVel twist = controller.cmdVel();
                control.published = true;
                control.kind = XGC2_UGV_CMD_VEL_COMMAND;
                control.linear = twist.linear_x;
                control.angular = twist.angular_z;
            } else if (event.id == ugv::output_event_type::PUBLISH_ZERO_CMD_VEL) {
                control.published = true;
                control.kind = XGC2_UGV_CMD_VEL_ZERO;
            }
        }
        control.control_state = controller.stateMachine().currentState(ugv::region_type::CONTROL);
        if (control.published) {
            linear = control.linear;
            angular = control.angular;
        }
        run.control.push_back(control);
        run.plant_x.push_back(plant.x);
        run.plant_y.push_back(plant.y);
        plant.step(linear, angular, 0.002);
    }
    return run;
}

// ---- the module chain
// ---------------------------------------------------------------------------------

Trace moduleChain(const Trace& expected) {
    Trace run;
    ModuleLibrary reference_library(REFERENCE_MODULE_PATH);
    ModuleLibrary controller_library(CONTROLLER_MODULE_PATH);
    TestHost reference(reference_library.desc());
    TestHost controller(controller_library.desc());
    reference.setNow(kT0);
    controller.setNow(kT0);
    EXPECT_EQ(reference.create("{}"), XGC2_OK);
    EXPECT_EQ(controller.create("{}"), XGC2_OK);
    EXPECT_EQ(reference.start(), XGC2_OK);
    EXPECT_EQ(controller.start(), XGC2_OK);

    ScriptGlue glue;
    Script script = makeScript(glue);
    size_t next_request = 0;
    size_t next_command = 0;
    Plant plant;
    double linear = 0.0, angular = 0.0;
    uint64_t wakes = 0;

    for (int64_t j = 0; j < kSteps; ++j) {
        const int64_t ns = kT0 + j * kControlStep;
        reference.setNow(ns);
        controller.setNow(ns);

        // the vehicle's state
        if (j % kStateEvery == 0) {
            xgc2_ugv_planar_state s;
            std::memset(&s, 0, sizeof s);
            s.stamp_ns = ns;
            s.x = plant.x;
            s.y = plant.y;
            s.yaw = plant.yaw;
            s.vx = plant.v * std::cos(plant.yaw);
            s.vy = plant.v * std::sin(plant.yaw);
            s.speed = plant.v;
            s.yaw_rate = plant.omega;
            s.estimator_state = 3;
            s.source = XGC2_UGV_STATE_SOURCE_ESTIMATE;
            s.velocity_valid = 1;
            controller.push("state", s, ns);
        }

        // the generator
        if (j % kReferenceEvery == 0) {
            const double reference_now = urt::Time().fromNSec(static_cast<uint64_t>(ns)).toSec();
            while (next_request < script.items.size() &&
                   replay::kT0 + script.items[next_request].at <= reference_now) {
                auto& item = script.items[next_request++];
                auto push = [&](const char* port, const auto& plain, auto payload) {
                    EXPECT_TRUE(rp::fits(plain));
                    std::memset(payload.get(), 0, sizeof *payload);
                    rp::toPayload(*payload, plain);
                    EXPECT_TRUE(reference.push(port, *payload, ns));
                };
                switch (item.kind) {
                    case replay::Kind::kAnalytic:
                        push("analytic_request", item.analytic,
                             std::make_unique<xgc2_ugv_analytic_reference>());
                        break;
                    case replay::Kind::kSampled:
                        push("sampled_request", item.sampled,
                             std::make_unique<xgc2_ugv_sampled_reference>());
                        break;
                    case replay::Kind::kWaypoint:
                        push("waypoint_request", item.waypoint,
                             std::make_unique<xgc2_ugv_waypoint_request>());
                        break;
                    case replay::Kind::kReset: {
                        xgc2_ugv_reference_reset reset;
                        std::memset(&reset, 0, sizeof reset);
                        reset.stamp_ns = ns;
                        EXPECT_TRUE(reference.push("reset", reset, ns));
                        break;
                    }
                }
            }
            EXPECT_EQ(reference.step(), XGC2_OK);
            // The outputs of this step, in the order the node path publishes them, go to the
            // controller. (Per port: the generator publishes at most one of each per step.)
            for (const char* port :
                 {"status", "active_analytic", "active_polynomial", "active_sampled"}) {
                for (const auto& sample : reference.take(port)) {
                    run.reference.push_back({port, sample.stamp_ns, sample.bytes});
                    if (std::string(port) != "status") {
                        EXPECT_TRUE(controller.push(controller.port(port), sample.bytes.data(),
                                                    static_cast<uint32_t>(sample.bytes.size()),
                                                    sample.stamp_ns));
                    }
                }
            }
        }

        // the operator
        while (next_command < sizeof(kCommands) / sizeof(kCommands[0]) &&
               kT0 + static_cast<int64_t>(kCommands[next_command].at * 1e9) <= ns) {
            xgc2_ugv_command command;
            std::memset(&command, 0, sizeof command);
            command.stamp_ns = ns;
            command.kind = kCommands[next_command++].kind;
            command.source = XGC2_UGV_COMMAND_SOURCE_NAMESPACED;
            EXPECT_TRUE(controller.push("command", command, ns));
        }

        EXPECT_EQ(controller.step(), XGC2_OK);
        Control control;
        for (const auto& twist : controller.takeAs<xgc2_ugv_cmd_vel>("cmd_vel")) {
            control.published = true;
            control.kind = twist.kind;
            control.linear = twist.linear_x;
            control.angular = twist.angular_z;
        }
        for (const auto& status : controller.takeAs<xgc2_ugv_controller_status>("status")) {
            run.statuses.emplace_back(j, status.control_state_name);
        }
        // The solver works on its own thread: wait for the result of a request, as the node path
        // takes the result of a request in the next period.
        control.solve_requested = expected.control[static_cast<size_t>(j)].solve_requested;
        control.worker_solve = expected.control[static_cast<size_t>(j)].worker_solve;
        if (control.worker_solve) {
            EXPECT_TRUE(controller.waitWake(wakes, std::chrono::seconds(5)))
                << "no NMPC result at step " << j;
            wakes = controller.wakeCount();
        }
        if (control.published) {
            linear = control.linear;
            angular = control.angular;
        }
        run.control.push_back(control);
        run.plant_x.push_back(plant.x);
        run.plant_y.push_back(plant.y);
        plant.step(linear, angular, 0.002);
    }
    controller.stop();
    reference.stop();
    return run;
}

TEST(ChainParity, TheModuleChainWritesWhatTheTwoNodesPublish) {
    const Trace node = nodePath();
    ASSERT_EQ(node.control.size(), static_cast<size_t>(kSteps));

    // The scenario is worth the comparison: the vehicle tracks, stops and tracks again, the
    // generator serves analytic, sampled and waypoint references, and the solver is asked for the
    // whole run.
    size_t solves = 0, twists = 0, commands = 0;
    for (const Control& c : node.control) {
        solves += c.solve_requested ? 1U : 0U;
        twists += c.published ? 1U : 0U;
        commands += c.published && c.kind == XGC2_UGV_CMD_VEL_COMMAND && c.linear != 0.0 ? 1U : 0U;
    }
    EXPECT_GT(solves, 1000u);
    EXPECT_GT(twists, 500u);
    EXPECT_GT(commands, 300u);
    size_t analytic = 0, polynomial = 0, sampled = 0, statuses = 0;
    for (const RefOutput& o : node.reference) {
        analytic += o.port == "active_analytic" ? 1U : 0U;
        polynomial += o.port == "active_polynomial" ? 1U : 0U;
        sampled += o.port == "active_sampled" ? 1U : 0U;
        statuses += o.port == "status" ? 1U : 0U;
    }
    EXPECT_GT(analytic, 20u);
    EXPECT_GT(polynomial, 5u);
    EXPECT_GT(sampled, 5u);
    EXPECT_GT(statuses, 200u);
    double travelled = 0.0;
    for (size_t i = 1; i < node.plant_x.size(); ++i) {
        travelled += std::hypot(node.plant_x[i] - node.plant_x[i - 1],
                                node.plant_y[i] - node.plant_y[i - 1]);
    }
    EXPECT_GT(travelled, 8.0) << "the vehicle drove";

    const Trace module = moduleChain(node);
    ASSERT_EQ(module.control.size(), node.control.size());

    // the generator: every output of every step, bit for bit (the outputs of one step in port
    // order)
    auto by_step = [](std::vector<RefOutput> outputs) {
        std::stable_sort(
            outputs.begin(), outputs.end(), [](const RefOutput& a, const RefOutput& b) {
                return a.stamp_ns != b.stamp_ns ? a.stamp_ns < b.stamp_ns : a.port < b.port;
            });
        return outputs;
    };
    const std::vector<RefOutput> node_reference = by_step(node.reference);
    const std::vector<RefOutput> module_reference = by_step(module.reference);
    ASSERT_EQ(module_reference.size(), node_reference.size());
    for (size_t i = 0; i < node_reference.size(); ++i) {
        ASSERT_TRUE(module_reference[i] == node_reference[i])
            << "generator output " << i << " on " << node_reference[i].port << " at "
            << (node_reference[i].stamp_ns - kT0) / 1000000 << " ms: the port or the stamp or the "
            << "payload differs (" << module_reference[i].port << ")";
    }

    // the controller: the twist of every control period, bit for bit, and with it the vehicle
    for (size_t j = 0; j < node.control.size(); ++j) {
        ASSERT_TRUE(module.control[j].sameTwist(node.control[j]))
            << "control step " << j << " (" << j * 2 << " ms): node " << node.control[j].published
            << " kind " << node.control[j].kind << " " << node.control[j].linear << " "
            << node.control[j].angular << ", module " << module.control[j].published << " kind "
            << module.control[j].kind << " " << module.control[j].linear << " "
            << module.control[j].angular;
        ASSERT_EQ(module.plant_x[j], node.plant_x[j]) << j;
        ASSERT_EQ(module.plant_y[j], node.plant_y[j]) << j;
    }

    // the control state wherever the module wrote its status
    ASSERT_GT(module.statuses.size(), 100u);
    const char* names[] = {"", "SelfCheck", "Ready", "Custom1", "", "Reset"};
    for (const auto& status : module.statuses) {
        const uint32_t state = node.control[static_cast<size_t>(status.first)].control_state;
        ASSERT_LT(state, 6u);
        EXPECT_EQ(status.second, names[state]) << "step " << status.first;
    }
}

}  // namespace
