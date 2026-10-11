// The controller module as a shared library, loaded the way the host loads it, in an in-test host,
// against a kinematic unicycle.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>

#include "test_host.h"
#include "xgc2_ugv/payloads.h"

namespace {

using ugv_modules_test::ModuleLibrary;
using ugv_modules_test::TestHost;

constexpr int64_t kMs = 1000000LL;
constexpr int64_t kSecond = 1000 * kMs;
constexpr int64_t kStep = 2 * kMs;  // the 500 Hz period of the node's main loop
constexpr int64_t kT0 = 10 * kSecond;
constexpr uint32_t kEstimatorRunning = 3;  // RigidStateEstimate::STATE_RUNNING

double wallSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

double wrap(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

struct Plant {
    double x{0.0}, y{0.0}, yaw{0.0}, v{0.0}, omega{0.0};

    void step(double linear, double angular, double dt) {
        v = linear;
        omega = angular;
        x += v * std::cos(yaw) * dt;
        y += v * std::sin(yaw) * dt;
        yaw = wrap(yaw + omega * dt);
    }
};

xgc2_ugv_planar_state estimate(int64_t stamp_ns, const Plant& p) {
    xgc2_ugv_planar_state s;
    std::memset(&s, 0, sizeof s);
    s.stamp_ns = stamp_ns;
    s.x = p.x;
    s.y = p.y;
    s.yaw = p.yaw;
    s.vx = p.v * std::cos(p.yaw);
    s.vy = p.v * std::sin(p.yaw);
    s.speed = p.v;
    s.yaw_rate = p.omega;
    s.estimator_state = kEstimatorRunning;
    s.source = XGC2_UGV_STATE_SOURCE_ESTIMATE;
    s.velocity_valid = 1;
    return s;
}

xgc2_ugv_planar_state poseOf(int64_t stamp_ns, const Plant& p) {
    xgc2_ugv_planar_state s;
    std::memset(&s, 0, sizeof s);
    s.stamp_ns = stamp_ns;
    s.x = p.x;
    s.y = p.y;
    s.yaw = p.yaw;
    s.source = XGC2_UGV_STATE_SOURCE_POSE;
    return s;
}

xgc2_ugv_command commandOf(uint32_t kind, int64_t stamp_ns,
                           uint32_t source = XGC2_UGV_COMMAND_SOURCE_NAMESPACED) {
    xgc2_ugv_command c;
    std::memset(&c, 0, sizeof c);
    c.stamp_ns = stamp_ns;
    c.kind = kind;
    c.source = source;
    return c;
}

// A circle of radius 1.5 m with its centre at (-1.5, 0): it starts at the origin heading +y and
// runs counter-clockwise at 0.6 m/s.
xgc2_ugv_analytic_reference circle(int64_t stamp_ns, int64_t start_ns) {
    xgc2_ugv_analytic_reference r;
    std::memset(&r, 0, sizeof r);
    r.stamp_ns = stamp_ns;
    r.start_time_ns = start_ns;
    r.trajectory_id = 7;
    r.revision = 1;
    r.analytic_type = XGC2_UGV_ANALYTIC_CIRCLE;
    r.duration = 60.0;
    r.origin_qw = 1.0;
    r.param_count = 5;
    const double params[5] = {1.5, 0.6, 3.0, -1.5, 0.0};  // radius, line speed, entry, centre x, y
    std::memcpy(r.params, params, sizeof params);
    return r;
}

class Rig {
   public:
    Rig(const std::string& config, bool pose_source = false)
        : library_(CONTROLLER_MODULE_PATH), host_(library_.desc()), pose_source_(pose_source) {
        host_.setNow(now_);
        EXPECT_EQ(host_.create(config), XGC2_OK);
        EXPECT_EQ(host_.start(), XGC2_OK);
    }
    ~Rig() {
        host_.stop();
        // The host keeps an instance on one thread; so does this test.
        EXPECT_EQ(host_.callingThreads(), 1u);
    }

    TestHost& host() {
        return host_;
    }
    Plant& plant() {
        return plant_;
    }
    int64_t now() const {
        return now_;
    }
    // Real time: the next tick starts one control period after the previous one began, so that
    // what runs on other threads (the solver) and the wall clock the Reset lease measures see
    // the time the host would give them.
    void pace(bool on) {
        paced_ = on;
        pace_origin_ = std::chrono::steady_clock::now();
        pace_ticks_ = 0;
    }

    // One control period: the vehicle state is measured, the module steps, the chassis takes the
    // twist of the newest cmd_vel, the plant moves.
    void tick(bool measure = true) {
        if (paced_) {
            std::this_thread::sleep_until(pace_origin_ +
                                          std::chrono::nanoseconds(pace_ticks_++ * kStep));
        }
        if (measure) {
            const auto sample = pose_source_ ? poseOf(now_, plant_) : estimate(now_, plant_);
            host_.push("state", sample, now_);
        }
        host_.setNow(now_);
        ASSERT_EQ(host_.step(XGC2_STEP_TIMER), XGC2_OK);
        for (const auto& twist : host_.takeAs<xgc2_ugv_cmd_vel>("cmd_vel")) {
            linear_ = twist.linear_x;
            angular_ = twist.angular_z;
            twists_.push_back(twist);
        }
        plant_.step(linear_, angular_, 2e-3);
        now_ += kStep;
    }
    void ticks(int n, bool measure = true) {
        for (int i = 0; i < n; ++i) {
            tick(measure);
        }
    }

    // Ticks until the published control state is `name` (the status is written at 5 Hz).
    bool reach(const std::string& name, int max_ticks = 300) {
        for (int k = 0; k < max_ticks; ++k) {
            if (controlState() == name) {
                return true;
            }
            tick();
        }
        return controlState() == name;
    }

    xgc2_ugv_controller_status status() {
        const auto samples = host_.outputsAs<xgc2_ugv_controller_status>("status");
        return samples.empty() ? xgc2_ugv_controller_status{} : samples.back();
    }
    std::string controlState() {
        return status().control_state_name;
    }
    const std::vector<xgc2_ugv_cmd_vel>& twists() const {
        return twists_;
    }
    xgc2_ugv_reset_session session() {
        const auto samples = host_.outputsAs<xgc2_ugv_reset_session>("reset_session");
        return samples.empty() ? xgc2_ugv_reset_session{} : samples.back();
    }

   private:
    ModuleLibrary library_;
    TestHost host_;
    bool pose_source_;
    bool paced_{false};
    std::chrono::steady_clock::time_point pace_origin_;
    int64_t pace_ticks_{0};
    Plant plant_;
    int64_t now_{kT0};
    double linear_{0.0};
    double angular_{0.0};
    std::vector<xgc2_ugv_cmd_vel> twists_;
};

const char* kScout = "{\"fence\": {\"x_min\": -12, \"x_max\": 12, \"y_min\": -7, \"y_max\": 7}}";

TEST(ControllerModule, DescribesItsPortsForTheHost) {
    ModuleLibrary library(CONTROLLER_MODULE_PATH);
    const xgc2_module_desc* d = library.desc();
    EXPECT_EQ(d->abi_major, XGC2_MODULE_ABI_MAJOR);
    EXPECT_STREQ(d->name, "ugv_unicycle_controller");
    ASSERT_EQ(d->port_count, 11u);
    struct Expect {
        const char* name;
        xgc2_port_direction direction;
        xgc2_port_kind kind;
        const char* schema;
        uint32_t size;
        uint32_t depth;
        uint32_t flags;
    };
    const Expect expected[] = {
        {"state", XGC2_PORT_IN, XGC2_PORT_STATE, "xgc2.ugv.planar_state.v1",
         sizeof(xgc2_ugv_planar_state), 0, XGC2_PORT_REQUIRED},
        {"command", XGC2_PORT_IN, XGC2_PORT_EVENT, "xgc2.ugv.command.v1", sizeof(xgc2_ugv_command),
         8, 0},
        {"reset_target", XGC2_PORT_IN, XGC2_PORT_STATE, "xgc2.ugv.reset_target.v1",
         sizeof(xgc2_ugv_reset_target), 0, 0},
        {"reset_clearance", XGC2_PORT_IN, XGC2_PORT_STATE, "xgc2.ugv.reset_clearance.v1",
         sizeof(xgc2_ugv_reset_clearance), 0, 0},
        {"active_analytic", XGC2_PORT_IN, XGC2_PORT_STATE,
         "xgc2.ugv.unicycle_reference.analytic.v1", sizeof(xgc2_ugv_analytic_reference), 0, 0},
        {"active_polynomial", XGC2_PORT_IN, XGC2_PORT_STATE,
         "xgc2.ugv.unicycle_reference.polynomial.v1", sizeof(xgc2_ugv_polynomial_reference), 0, 0},
        {"active_sampled", XGC2_PORT_IN, XGC2_PORT_STATE, "xgc2.ugv.unicycle_reference.sampled.v1",
         sizeof(xgc2_ugv_sampled_reference), 0, 0},
        {"pva", XGC2_PORT_IN, XGC2_PORT_STATE, "xgc2.ugv.planar_pva.v1",
         sizeof(xgc2_ugv_planar_pva), 0, 0},
        {"cmd_vel", XGC2_PORT_OUT, XGC2_PORT_STATE, "xgc2.ugv.cmd_vel.v1", sizeof(xgc2_ugv_cmd_vel),
         0, 0},
        {"status", XGC2_PORT_OUT, XGC2_PORT_STATE, "xgc2.ugv.controller_status.v1",
         sizeof(xgc2_ugv_controller_status), 0, 0},
        {"reset_session", XGC2_PORT_OUT, XGC2_PORT_STATE, "xgc2.ugv.reset_session.v1",
         sizeof(xgc2_ugv_reset_session), 0, 0},
    };
    for (uint32_t i = 0; i < d->port_count; ++i) {
        const xgc2_port_desc& p = d->ports[i];
        EXPECT_STREQ(p.name, expected[i].name);
        EXPECT_EQ(p.direction, static_cast<uint32_t>(expected[i].direction)) << p.name;
        EXPECT_EQ(p.kind, static_cast<uint32_t>(expected[i].kind)) << p.name;
        EXPECT_STREQ(p.schema_id, expected[i].schema) << p.name;
        EXPECT_EQ(p.size, expected[i].size) << p.name;
        EXPECT_EQ(p.align, 8u) << p.name;
        EXPECT_EQ(p.queue_depth, expected[i].depth) << p.name;
        EXPECT_EQ(p.flags, expected[i].flags) << p.name;
        // The inputs come first: the index of an input is its index in the port table.
        EXPECT_EQ(p.direction == XGC2_PORT_IN, i < 8u) << p.name;
    }
}

TEST(ControllerModule, TheConfigurationIsStrict) {
    ModuleLibrary library(CONTROLLER_MODULE_PATH);
    {
        TestHost host(library.desc());
        EXPECT_EQ(host.create("{\"state_timout\": 0.3}"), XGC2_ERR_INVALID);
        EXPECT_TRUE(host.logged(3, "unknown configuration key 'state_timout'"));
    }
    {
        TestHost host(library.desc());
        EXPECT_EQ(host.create("{\"control_rate_hz\": 500.0}"), XGC2_ERR_INVALID);
        EXPECT_TRUE(host.logged(3, "control_rate_hz does not apply to the module"));
    }
    {
        TestHost host(library.desc());
        EXPECT_EQ(host.create("{\"state_source\": \"gps\"}"), XGC2_ERR_INVALID);
        EXPECT_TRUE(host.logged(3, "Unknown state_source: gps"));
    }
    {
        TestHost host(library.desc());
        EXPECT_EQ(host.create("{\"limits\": {\"max_linear_speed\": \"fast\"}}"), XGC2_ERR_INVALID);
        EXPECT_TRUE(host.logged(3, "'limits/max_linear_speed' must be a number"));
    }
    {
        TestHost host(library.desc());
        EXPECT_EQ(host.create("{\"flatness\": {\"heading_recovery\": {\"gain\": 1.0}}}"),
                  XGC2_ERR_INVALID);  // heading recovery needs the flatness strategy
    }
    {
        TestHost host(library.desc());
        EXPECT_EQ(host.create("{\"auto_start_tracking\": 1}"), XGC2_ERR_INVALID);
    }
    TestHost good(library.desc());
    EXPECT_EQ(good.create(kScout), XGC2_OK);
    EXPECT_EQ(good.configure("{\"state_timeout\": 0.3, \"status_publish_rate_hz\": 10}"), XGC2_OK);
    EXPECT_EQ(good.configure("{\"state_timeout\": \"0.3\"}"), XGC2_ERR_INVALID);
}

TEST(ControllerModule, StaysInSelfCheckWithoutAVehicleStateAndIdlesZero) {
    Rig rig(kScout);
    rig.ticks(1000, false);  // two seconds, no state input
    EXPECT_EQ(rig.controlState(), "SelfCheck");
    ASSERT_FALSE(rig.twists().empty());
    // Idle zeros at idle_cmd_rate_hz = 5 Hz.
    EXPECT_GE(rig.twists().size(), 9u);
    EXPECT_LE(rig.twists().size(), 11u);
    for (const auto& twist : rig.twists()) {
        EXPECT_EQ(twist.kind, XGC2_UGV_CMD_VEL_ZERO);
        EXPECT_EQ(twist.linear_x, 0.0);
        EXPECT_EQ(twist.angular_z, 0.0);
    }
    const auto session = rig.session();
    EXPECT_EQ(session.flags, 0u);
}

TEST(ControllerModule, ABadVehicleStateKeepsTheControllerOutOfReady) {
    {
        // A pose for a controller that takes a state estimate is refused and said so.
        Rig rig(kScout, /*pose_source=*/true);
        rig.ticks(500);
        EXPECT_EQ(rig.controlState(), "SelfCheck");
        EXPECT_TRUE(rig.host().logged(2, "The vehicle state is a pose"));
    }
    {
        // An estimator that is not running is not healthy.
        Rig rig(kScout);
        rig.plant();
        for (int k = 0; k < 500; ++k) {
            auto s = estimate(rig.now(), rig.plant());
            s.estimator_state = 2;
            rig.host().push("state", s, rig.now());
            rig.tick(false);
        }
        EXPECT_EQ(rig.controlState(), "SelfCheck");
    }
    {
        // A state that stops arriving is stale after state_timeout.
        Rig rig(kScout);
        rig.ticks(500);
        EXPECT_EQ(rig.controlState(), "Ready");
        rig.ticks(400, false);
        EXPECT_EQ(rig.controlState(), "SelfCheck");
    }
    {
        // Outside the fence is not healthy either.
        Rig rig(kScout);
        rig.plant().x = 13.0;
        rig.ticks(500);
        EXPECT_EQ(rig.controlState(), "SelfCheck");
    }
}

TEST(ControllerModule, BecomesReadyOnAFreshStateAndPublishesItsStatus) {
    Rig rig(kScout);
    rig.ticks(500);
    EXPECT_EQ(rig.controlState(), "Ready");
    const auto status = rig.status();
    EXPECT_EQ(status.control_state, XGC2_UGV_CONTROL_READY);
    EXPECT_EQ(status.stamp_ns % kStep, 0);
    // At status_publish_rate_hz = 5 Hz, stamped with the host clock of the step.
    const auto samples = rig.host().outputs("status");
    ASSERT_GE(samples.size(), 4u);
    EXPECT_NEAR(static_cast<double>(samples[3].stamp_ns - samples[2].stamp_ns), 0.2e9, 3e6);
    EXPECT_EQ(rig.session().flags, XGC2_UGV_RESET_SESSION_HEALTHY);
    EXPECT_EQ(rig.session().generation, 0u);
    // The idle zero is a command of kind zero.
    ASSERT_FALSE(rig.twists().empty());
    EXPECT_EQ(rig.twists().back().kind, XGC2_UGV_CMD_VEL_ZERO);
    EXPECT_EQ(rig.host().openSlots(), 0u);
}

TEST(ControllerModule, OperatorCommandsStartAndStopTracking) {
    Rig rig(kScout);
    rig.ticks(500);
    ASSERT_EQ(rig.controlState(), "Ready");
    rig.host().push("command", commandOf(XGC2_UGV_COMMAND_CUSTOM1, rig.now()), rig.now());
    ASSERT_TRUE(rig.reach("Custom1"));
    EXPECT_EQ(rig.status().control_state, XGC2_UGV_CONTROL_CUSTOM1);
    rig.host().push("command",
                    commandOf(XGC2_UGV_COMMAND_STOP, rig.now(), XGC2_UGV_COMMAND_SOURCE_PUBLIC),
                    rig.now());
    ASSERT_TRUE(rig.reach("Ready"));
    // An unknown kind is refused and does not wedge the port.
    rig.host().push("command", commandOf(99, rig.now()), rig.now());
    rig.host().push("command", commandOf(XGC2_UGV_COMMAND_CUSTOM1, rig.now()), rig.now());
    ASSERT_TRUE(rig.reach("Custom1"));
    EXPECT_TRUE(rig.host().logged(2, "Unknown command kind 99"));
}

TEST(ControllerModule, TracksAnAnalyticReferenceWithTheNmpcSolverOnItsWorker) {
    Rig rig(kScout);
    rig.plant().yaw = M_PI / 2.0;
    rig.ticks(500);  // Ready after one second
    ASSERT_EQ(rig.controlState(), "Ready");
    const int64_t start_ns = rig.now() + 4 * kMs;
    rig.host().push("active_analytic", circle(rig.now(), start_ns), rig.now());
    rig.host().push("command", commandOf(XGC2_UGV_COMMAND_CUSTOM1, rig.now()), rig.now());

    // The solver runs on its own thread and wakes the instance when it has a result. The ticks
    // run in real time, as under the host, so the solves and the controller's timeouts see the
    // time they are designed for.
    rig.pace(true);
    double worst = 0.0;
    for (int k = 0; k < 2000; ++k) {  // four seconds
        rig.tick();
        if (k > 1250) {
            const double t = static_cast<double>(rig.now() - start_ns) * 1e-9;
            const double w = 0.6 / 1.5;
            const double rx = -1.5 + 1.5 * std::cos(w * t);
            const double ry = 1.5 * std::sin(w * t);
            worst = std::max(worst, std::hypot(rig.plant().x - rx, rig.plant().y - ry));
        }
    }
    EXPECT_EQ(rig.controlState(), "Custom1");
    EXPECT_LT(worst, 0.15) << "tracking error after the first 2.5 s";
    EXPECT_GT(rig.host().wakeCount(), 100u) << "the solver wakes the instance for each result";
    EXPECT_NE(rig.host().lastWakeThread(), std::this_thread::get_id());

    // The vehicle drives the circle: on average 0.6 m/s forward and a turn of 0.4 rad/s, at the
    // 30 Hz of command_publish_rate_hz.
    ASSERT_GT(rig.twists().size(), 100u);
    double linear = 0.0;
    double angular = 0.0;
    const size_t window = 30;
    for (size_t i = rig.twists().size() - window; i < rig.twists().size(); ++i) {
        const auto& twist = rig.twists()[i];
        EXPECT_EQ(twist.kind, XGC2_UGV_CMD_VEL_COMMAND);
        EXPECT_EQ(twist.linear_y, 0.0);
        linear += twist.linear_x / window;
        angular += twist.angular_z / window;
    }
    EXPECT_NEAR(linear, 0.6, 0.06);
    EXPECT_NEAR(angular, 0.4, 0.06);
    EXPECT_NEAR(static_cast<double>(rig.twists().back().stamp_ns -
                                    rig.twists()[rig.twists().size() - window].stamp_ns),
                (window - 1) / 30.0 * 1e9, 0.1e9);

    // stop() joins the worker: nothing wakes the host after it.
    EXPECT_EQ(rig.host().stop(), XGC2_OK);
    const uint64_t after_stop = rig.host().wakeCount();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(rig.host().wakeCount(), after_stop);
    // and a restart brings the solver back
    EXPECT_EQ(rig.host().start(), XGC2_OK);
    rig.ticks(250);
    EXPECT_GT(rig.host().wakeCount(), after_stop);
}

// The state machine of the controller belongs to the thread that built it. The host keeps an
// instance on one thread (affinity "sticky"); a host that did not would leave the controller half
// alive without a sound, so the module checks and fails the instance with the reason.
TEST(ControllerModule, AStepOnAnotherThreadThanTheBuilderFailsTheInstanceLoudly) {
    ModuleLibrary library(CONTROLLER_MODULE_PATH);
    TestHost host(library.desc());
    ugv_modules_test::RotatingThreads threads(2);  // the builder's and another one
    int64_t now = kT0;
    host.setNow(now);
    xgc2_status status = XGC2_OK;
    threads.run([&] { status = host.create(kScout); });  // thread 0 builds the controller
    ASSERT_EQ(status, XGC2_OK);
    threads.run(
        [&] { status = host.start(); });  // thread 1; start does not touch the state machine
    ASSERT_EQ(status, XGC2_OK);
    auto step = [&] {
        now += kStep;
        host.setNow(now);
        threads.run([&] { status = host.step(); });
    };
    step();  // thread 0
    EXPECT_EQ(status, XGC2_OK);
    EXPECT_TRUE(host.reports().empty());
    step();  // thread 1
    EXPECT_EQ(status, XGC2_ERR_INTERNAL);
    ASSERT_FALSE(host.reports().empty());
    EXPECT_EQ(host.reports().back().first, 2);  // failed
    EXPECT_NE(host.reports().back().second.find("another thread"), std::string::npos)
        << host.reports().back().second;
    EXPECT_NE(host.reports().back().second.find("affinity"), std::string::npos);
    EXPECT_EQ(host.callingThreads(), 2u);
    threads.run([&] { host.stop(); });
    threads.run([&] { host.destroy(); });
}

TEST(ControllerModule, TracksAWorldPvaReferenceWithFlatness) {
    const std::string config =
        "{\"state_source\": \"platform_pose\", \"tracking_strategy\": \"flatness\", "
        "\"fence\": {\"x_max\": 12, \"x_min\": -12, \"y_max\": 7, \"y_min\": -7}, "
        "\"chassis\": {\"max_linear_speed\": 1.5}}";
    Rig rig(config, /*pose_source=*/true);
    rig.ticks(500);
    ASSERT_EQ(rig.controlState(), "Ready");
    rig.host().push("command", commandOf(XGC2_UGV_COMMAND_CUSTOM1, rig.now()), rig.now());
    const int64_t t0 = rig.now();
    for (int k = 0; k < 2500; ++k) {
        const double s = static_cast<double>(rig.now() - t0) * 1e-9;
        xgc2_ugv_planar_pva pva;
        std::memset(&pva, 0, sizeof pva);
        pva.stamp_ns = rig.now();
        pva.x = 0.4 * s;
        pva.y = 0.1 * std::sin(0.5 * s);
        pva.yaw = std::atan2(0.05 * std::cos(0.5 * s), 0.4);
        pva.vx = 0.4;
        pva.vy = 0.05 * std::cos(0.5 * s);
        pva.ay = -0.025 * std::sin(0.5 * s);
        rig.host().push("pva", pva, rig.now());
        rig.tick();
    }
    EXPECT_EQ(rig.controlState(), "Custom1");
    const double s = static_cast<double>(rig.now() - t0) * 1e-9;
    EXPECT_NEAR(rig.plant().x, 0.4 * s, 0.08);
    EXPECT_NEAR(rig.plant().y, 0.1 * std::sin(0.5 * s), 0.08);
    EXPECT_EQ(rig.twists().back().kind, XGC2_UGV_CMD_VEL_COMMAND);
    // A non-finite reference is refused and the previous one stays.
    xgc2_ugv_planar_pva bad;
    std::memset(&bad, 0, sizeof bad);
    bad.stamp_ns = rig.now();
    bad.x = std::nan("");
    rig.host().push("pva", bad, rig.now());
    rig.tick();
    EXPECT_TRUE(rig.host().logged(2, "Rejecting non-finite PVA"));
}

class ResetTest : public ::testing::Test {
   protected:
    ResetTest()
        : rig_(
              "{\"reset_initial_x\": 2.0, \"reset_initial_y\": 0.5, \"reset_initial_yaw\": 0.0, "
              "\"fence\": {\"x_min\": -12, \"x_max\": 12, \"y_min\": -7, \"y_max\": 7}, "
              "\"reset\": {\"timeout\": 20.0}}") {}

    void SetUp() override {
        rig_.ticks(500);
        ASSERT_EQ(rig_.controlState(), "Ready");
        // The Reset state paces its commands on the wall clock, so these tests run in real time.
        rig_.pace(true);
    }

    void requestReset() {
        rig_.host().push("command", commandOf(XGC2_UGV_COMMAND_RESET, rig_.now()), rig_.now());
        ASSERT_TRUE(rig_.reach("Reset"));
    }

    xgc2_ugv_reset_clearance clearance(uint32_t generation, uint32_t status, double lx, double wz,
                                       int64_t stamp_ns) {
        xgc2_ugv_reset_clearance c;
        std::memset(&c, 0, sizeof c);
        c.stamp_ns = stamp_ns;
        c.issue_wall = wallSeconds();
        c.linear_x = lx;
        c.yaw_rate = wz;
        c.lease_seconds = 0.15;
        c.generation = generation;
        c.status = status;
        return c;
    }

    Rig rig_;
};

TEST_F(ResetTest, AResetCommandBeginsASessionTowardsTheSeededTarget) {
    requestReset();
    const auto session = rig_.session();
    EXPECT_EQ(session.flags, XGC2_UGV_RESET_SESSION_ACTIVE | XGC2_UGV_RESET_SESSION_HEALTHY);
    EXPECT_NE(session.generation, 0u);
    EXPECT_DOUBLE_EQ(session.target_x, 2.0);
    EXPECT_DOUBLE_EQ(session.target_y, 0.5);
    EXPECT_DOUBLE_EQ(session.target_yaw, 0.0);
    // Without a clearance the vehicle is held.
    rig_.ticks(100);
    EXPECT_EQ(rig_.twists().back().linear_x, 0.0);
    EXPECT_EQ(rig_.twists().back().angular_z, 0.0);
    // The session is written when it changes, not at the control rate.
    EXPECT_LE(rig_.host().outputs("reset_session").size(), 3u);
}

TEST_F(ResetTest, ATargetFromThePortReplacesTheSeededOne) {
    xgc2_ugv_reset_target target{-1.0, 2.0, 3.5};
    rig_.host().push("reset_target", target, rig_.now());
    rig_.ticks(5);
    requestReset();
    const auto session = rig_.session();
    EXPECT_DOUBLE_EQ(session.target_x, -1.0);
    EXPECT_DOUBLE_EQ(session.target_y, 2.0);
    EXPECT_NEAR(session.target_yaw, 3.5 - 2.0 * M_PI, 1e-12);  // wrapped to (-pi, pi]
}

TEST_F(ResetTest, ExecutesTheClearanceOfItsSessionWhileTheLeaseHolds) {
    requestReset();
    const uint32_t generation = rig_.session().generation;
    // An answer to another session is ignored.
    rig_.host().push("reset_clearance",
                     clearance(generation + 1, XGC2_UGV_RESET_RUNNING, 0.5, 0.2, rig_.now()),
                     rig_.now());
    rig_.ticks(100);
    EXPECT_EQ(rig_.twists().back().linear_x, 0.0);
    // The answer to this one is executed while the lease holds ...
    rig_.host().push("reset_clearance",
                     clearance(generation, XGC2_UGV_RESET_RUNNING, 0.3, 0.1, rig_.now()),
                     rig_.now());
    rig_.ticks(20);
    EXPECT_EQ(rig_.twists().back().kind, XGC2_UGV_CMD_VEL_COMMAND);
    EXPECT_DOUBLE_EQ(rig_.twists().back().linear_x, 0.3);
    EXPECT_DOUBLE_EQ(rig_.twists().back().angular_z, 0.1);
    // ... and the vehicle stops when it does not, although no one cancelled it.
    rig_.ticks(120);  // 0.24 s: the lease is 0.15 s
    EXPECT_EQ(rig_.twists().back().linear_x, 0.0);
    EXPECT_EQ(rig_.twists().back().angular_z, 0.0);
    EXPECT_EQ(rig_.controlState(), "Reset");
}

TEST_F(ResetTest, AnOldClearanceIsNotRenewedByTheClock) {
    requestReset();
    const uint32_t generation = rig_.session().generation;
    const int64_t old_stamp = rig_.now() - 400 * kMs;
    rig_.host().push("reset_clearance",
                     clearance(generation, XGC2_UGV_RESET_RUNNING, 0.3, 0.1, old_stamp),
                     rig_.now());
    rig_.ticks(20);
    EXPECT_EQ(rig_.twists().back().linear_x, 0.0);
}

TEST_F(ResetTest, ArrivalEndsTheSessionAndRejectionEndsItToo) {
    requestReset();
    uint32_t generation = rig_.session().generation;
    rig_.host().push("reset_clearance",
                     clearance(generation, XGC2_UGV_RESET_ARRIVED, 0.0, 0.0, rig_.now()),
                     rig_.now());
    ASSERT_TRUE(rig_.reach("Ready"));
    EXPECT_EQ(rig_.session().flags, XGC2_UGV_RESET_SESSION_HEALTHY);

    requestReset();
    const auto second = rig_.session();
    EXPECT_NE(second.generation, generation);  // a new session is told apart from the old one
    generation = second.generation;
    rig_.host().push("reset_clearance",
                     clearance(generation, XGC2_UGV_RESET_REJECTED, 0.0, 0.0, rig_.now()),
                     rig_.now());
    ASSERT_TRUE(rig_.reach("Ready"));
}

TEST_F(ResetTest, AStopEndsTheSessionAndAMalformedClearanceChangesNothing) {
    requestReset();
    const uint32_t generation = rig_.session().generation;
    rig_.host().push("reset_clearance", clearance(generation, 7, 0.3, 0.1, rig_.now()), rig_.now());
    rig_.ticks(120);
    EXPECT_TRUE(rig_.host().logged(2, "Ignoring a malformed reset clearance"));
    EXPECT_EQ(rig_.controlState(), "Reset");
    rig_.host().push("command", commandOf(XGC2_UGV_COMMAND_STOP, rig_.now()), rig_.now());
    ASSERT_TRUE(rig_.reach("Ready"));
    EXPECT_EQ(rig_.session().flags, XGC2_UGV_RESET_SESSION_HEALTHY);
}

TEST_F(ResetTest, ALostVehicleStateWithdrawsHealthFromTheSession) {
    requestReset();
    ASSERT_EQ(rig_.session().flags & XGC2_UGV_RESET_SESSION_HEALTHY,
              XGC2_UGV_RESET_SESSION_HEALTHY);
    rig_.ticks(400, false);  // no state for 0.8 s
    EXPECT_EQ(rig_.session().flags & XGC2_UGV_RESET_SESSION_HEALTHY, 0u);
    EXPECT_EQ(rig_.controlState(), "SelfCheck");
    EXPECT_EQ(rig_.session().flags & XGC2_UGV_RESET_SESSION_ACTIVE, 0u);
}

TEST(ControllerModule, WritesItsOutputsWholeAndNeverLeaksASlot) {
    Rig rig(kScout);
    rig.ticks(800);
    // The test host fills a slot with 0xA5 before the module gets it: whatever the module did not
    // write must have been zeroed, so each payload equals its fields over zeros.
    const auto status = rig.status();
    xgc2_ugv_controller_status expected_status;
    std::memset(&expected_status, 0, sizeof expected_status);
    expected_status.stamp_ns = status.stamp_ns;
    expected_status.control_state = status.control_state;
    expected_status.health_state = status.health_state;
    std::strcpy(expected_status.control_state_name, "Ready");
    EXPECT_EQ(std::memcmp(&expected_status, &status, sizeof status), 0);

    const auto session = rig.session();
    xgc2_ugv_reset_session expected_session;
    std::memset(&expected_session, 0, sizeof expected_session);
    expected_session.flags = XGC2_UGV_RESET_SESSION_HEALTHY;
    EXPECT_EQ(std::memcmp(&expected_session, &session, sizeof session), 0);

    ASSERT_FALSE(rig.twists().empty());
    for (const auto& twist : rig.twists()) {
        xgc2_ugv_cmd_vel expected;
        std::memset(&expected, 0, sizeof expected);
        expected.stamp_ns = twist.stamp_ns;
        expected.kind = XGC2_UGV_CMD_VEL_ZERO;
        EXPECT_EQ(std::memcmp(&expected, &twist, sizeof twist), 0);
    }
    EXPECT_EQ(rig.host().openSlots(), 0u);

    // Outputs that refuse writes cost samples, not the instance.
    rig.host().limitOutput("cmd_vel", 0);
    rig.host().limitOutput("status", 0);
    rig.ticks(500);
    EXPECT_GT(rig.host().refusedWrites("cmd_vel"), 0u);
    EXPECT_GT(rig.host().refusedWrites("status"), 0u);
    EXPECT_EQ(rig.host().openSlots(), 0u);
    EXPECT_TRUE(rig.host().reports().empty());
}

}  // namespace
