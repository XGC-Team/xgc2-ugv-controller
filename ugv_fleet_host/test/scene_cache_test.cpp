#include <gtest/gtest.h>
#include <ugv_fleet_host/json.h>
#include <ugv_fleet_host/scene_cache.h>
#include <unistd.h>

#include <atomic>
#include <filesystem>
#include <future>
#include <thread>
#include <xgc2/xrpc/http.hpp>
using namespace ugv_fleet_host;
Json::Value scene() {
    return wire::parse(
        R"({"epoch":"world-boot","revision":3,"frame":"world","stamp_ns":"18446744073709551615","scene_time":1.0,"playing":true,"applied":true,"serial":7,"simulation_time":{"epoch":9,"nanoseconds":"18446744073709551615"},"definition":{"obstacles":[]},"state":{"obstacles":[]}})");
}
TEST(SceneCache, ScientificTimeRetainsExactU64AndExplicitEpoch) {
    auto parsed = parseSceneSnapshot(wire::serialize(scene()));
    EXPECT_EQ(parsed.state.header.stamp.toNSec(), UINT64_MAX);
    EXPECT_EQ(parsed.state.simulation_time.nanoseconds.toNSec(), UINT64_MAX);
    EXPECT_EQ(parsed.state.simulation_time.epoch, 9U);
    EXPECT_EQ(parsed.definition.revision, 3U);
}
TEST(SceneCache, RejectsLossyNoncanonicalTimeAndMismatchedGeometry) {
    for (auto value :
         {Json::Value(Json::UInt64(9007199254740992ULL)), Json::Value("01"),
          Json::Value("18446744073709551616"), Json::Value("-1"), Json::Value("1e9")}) {
        auto s = scene();
        s["stamp_ns"] = value;
        EXPECT_THROW(parseSceneSnapshot(wire::serialize(s)), std::exception);
    }
    auto noepoch = scene();
    noepoch.removeMember("simulation_time");
    EXPECT_THROW(parseSceneSnapshot(wire::serialize(noepoch)), std::exception);
    auto invalid = scene();
    Json::Value body;
    body["id"] = "foreign";
    invalid["state"]["obstacles"].append(body);
    EXPECT_THROW(parseSceneSnapshot(wire::serialize(invalid)), std::exception);
    auto duplicate = wire::serialize(scene());
    duplicate.insert(1, "\"revision\":8,");
    EXPECT_THROW(parseSceneSnapshot(duplicate), std::exception);
}
TEST(SceneCache, FrozenProviderCoordinateArraysProjectActualCompoundGeometry) {
    auto s = scene();
    const auto definition = wire::parse(
        R"({"id":"wall","dynamic":false,"motion_type":"hold","pose":{"position":[1,2,0],"orientation":[0,0,0,1]},"parts":[{"id":"box","pose":{"position":[0.2,0,0],"orientation":[0,0,0,1]},"geometry":{"type":"box","size":[1,2,3]}}]})");
    const auto state = wire::parse(
        R"({"id":"wall","pose":{"position":[1,2,0],"orientation":[0,0,0,1]},"twist":{"linear":[0,0,0],"angular":[0,0,0]}})");
    s["definition"]["obstacles"].append(definition);
    s["state"]["obstacles"].append(state);
    const auto parsed = parseSceneSnapshot(wire::serialize(s));
    ASSERT_EQ(parsed.definition.obstacles.size(), 1U);
    EXPECT_EQ(parsed.definition.obstacles[0].pose.position.x, 1);
    EXPECT_EQ(parsed.definition.obstacles[0].parts[0].geometry.size.y, 2);
    EXPECT_EQ(parsed.state.obstacles[0].twist.linear.x, 0);
    for (const auto& invalid : {wire::parse(R"({"x":1,"y":2,"z":3})"), wire::parse("[1,2]"),
                                wire::parse("[1,2,3,4]"), wire::parse("[1,2,\"3\"]")}) {
        auto bad = s;
        bad["definition"]["obstacles"][0]["pose"]["position"] = invalid;
        EXPECT_THROW(parseSceneSnapshot(wire::serialize(bad)), std::exception);
    }
    auto bad = s;
    bad["state"]["obstacles"][0]["pose"]["orientation"] = wire::parse("[0,0,1]");
    EXPECT_THROW(parseSceneSnapshot(wire::serialize(bad)), std::exception);
}
namespace {
using namespace xgc2::xrpc;
class WorldFixture {
   public:
    explicit WorldFixture(bool wrong_target = false, bool wrong_instance = false) {
        char name[] = "/tmp/ugv-scene-http-XXXXXX";
        const auto* made = mkdtemp(name);
        if (!made)
            throw std::runtime_error("private fixture root creation failed");
        root = made;
        endpoint = (root / "world.sock").string();
        const auto instance = new_instance_id();
        reference = {"fixture-world",
                     wrong_target ? "foreign.service" : "xgc2.simulation",
                     "v1",
                     wrong_instance ? "foreign-incarnation" : instance,
                     "http.v1",
                     {"unix", endpoint}};
        auto ready = std::make_shared<std::promise<void>>();
        auto started = ready->get_future();
        worker = std::thread([this, instance, wrong_target, wrong_instance, ready] {
            bool announced = false;
            try {
                HttpReply held;
                Clock::time_point expires;
                HttpServer server(
                    {endpoint},
                    [&](HttpRequest request, HttpReply reply) {
                        if (request.target == "/v1/describe") {
                            ++descriptions;
                            Json::Value value;
                            auto& ref = value["service_ref"];
                            ref["service"] = "xgc2.simulation";
                            ref["target_id"] = wrong_target ? "foreign-world" : "fixture-world";
                            ref["api_version"] = "v1";
                            ref["profile"] = "http.v1";
                            ref["instance_id"] = wrong_instance ? "foreign-incarnation" : instance;
                            ref["endpoint"]["kind"] = "unix";
                            ref["endpoint"]["address"] = endpoint;
                            reply.complete({200, {}, wire::serialize(value), true});
                        } else if (request.target == "/v1/extensions/scene") {
                            ++snapshots;
                            reply.complete({200, {}, wire::serialize(scene()), true});
                        } else {
                            ++observations;
                            held = std::move(reply);
                            // A test provider emits the explicit deadline fault
                            // just before SDK expiry so the response path is covered.
                            expires = request.deadline - std::chrono::milliseconds(10);
                        }
                    },
                    {}, {instance, {"/v1/describe"}});
                ready->set_value();
                announced = true;
                while (!stopping) {
                    server.poll(std::chrono::milliseconds(2));
                    if (expires != Clock::time_point{} && Clock::now() >= expires) {
                        held.complete(
                            http_error(504, "deadline_exceeded", "quiet observation deadline"));
                        held = {};
                        expires = {};
                    }
                }
                held = {};
                server.drain_until(Clock::now() + std::chrono::milliseconds(100));
            } catch (...) {
                if (!announced)
                    ready->set_exception(std::current_exception());
            }
        });
        try {
            started.get();
        } catch (...) {
            worker.join();
            throw;
        }
    }
    ~WorldFixture() {
        stopping = true;
        worker.join();
        std::filesystem::remove_all(root);
    }
    std::filesystem::path root;
    std::string endpoint;
    ServiceRef reference;
    std::atomic<int> descriptions{0}, snapshots{0}, observations{0};

   private:
    std::atomic<bool> stopping{false};
    std::thread worker;
};
template <class Predicate>
bool boundedWait(Predicate predicate) {
    const auto until = Clock::now() + std::chrono::seconds(2);
    while (Clock::now() < until) {
        if (predicate())
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}
}  // namespace
TEST(SceneCache, QuietNativeDeadlineKeepsInstanceAndDoesNotRenewScientificEvidence) {
    WorldFixture world;
    SceneCache cache(world.reference, resolve_runtime_policy({}));
    SceneSnapshot snapshot;
    ASSERT_TRUE(boundedWait([&] { return cache.take(snapshot); }));
    EXPECT_EQ(snapshot.state.header.stamp.toNSec(), UINT64_MAX);
    ASSERT_TRUE(boundedWait([&] { return world.observations >= 2; }));
    EXPECT_FALSE(cache.failed());
    EXPECT_FALSE(cache.take(snapshot));
    EXPECT_EQ(world.descriptions, 0);
    EXPECT_GE(world.snapshots, 2);
}
TEST(SceneCache, ForeignServiceAndIncarnationFailClosedWithoutDiscovery) {
    {
        WorldFixture world(true, false);
        EXPECT_THROW(SceneCache(world.reference, resolve_runtime_policy({})),
                     std::invalid_argument);
        EXPECT_EQ(world.descriptions, 0);
        EXPECT_EQ(world.snapshots, 0);
    }
    WorldFixture world(false, true);
    SceneCache cache(world.reference, resolve_runtime_policy({}));
    ASSERT_TRUE(boundedWait([&] { return cache.failed(); }));
    SceneSnapshot snapshot;
    EXPECT_FALSE(cache.take(snapshot));
    EXPECT_EQ(world.descriptions, 0);
    EXPECT_EQ(world.snapshots, 0);
    EXPECT_EQ(world.observations, 0);
}
