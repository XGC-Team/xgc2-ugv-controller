#include <gtest/gtest.h>
#include <sys/stat.h>
#include <ugv_fleet_host/json.h>
#include <ugv_fleet_host/reset_http_boundary.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <thread>
using namespace ugv_fleet_host;
using namespace xgc2::xrpc;

namespace {
struct Fixture {
    std::filesystem::path dir;
    std::unique_ptr<BootstrapInput> bootstrap;
    std::unique_ptr<ResetHttpBoundary> host;
    std::unique_ptr<HttpClient> client;
    int starts = 0, stops = 0;
    ResetControl control{1,
                         [this](auto) {
                             ++starts;
                             return true;
                         },
                         [this](auto) {
                             ++stops;
                             return true;
                         }};
    ResetControl::SceneFence fence{};
    Fixture() {
        char path[] = "/tmp/ugv-reset-http-XXXXXX";
        dir = mkdtemp(path);
        chmod(dir.c_str(), 0700);
        fence.epoch[0] = 'w';
        fence.revision = 1;
        fence.simulation_time_epoch = 2;
        control.scene(fence, true, 1'000'000'000, 1);
        Json::Value input, binding;
        input["schema_version"] = binding["schema_version"] = 1;
        binding["target_id"] = "fixture";
        binding["service"] = "ugv-reset";
        binding["api_version"] = "v1";
        binding["profile"] = "http.v1";
        binding["endpoint"]["kind"] = "unix";
        binding["endpoint"]["address"] = (dir / "reset.sock").string();
        binding["runtime_grant"] = "fixture.runtime";
        binding["authentication"] = "local_private";
        binding["secret_handles"] = Json::Value(Json::objectValue);
        binding["storage_grants"] = Json::Value(Json::arrayValue);
        input["binding"] = std::move(binding);
        input["grants"] = Json::Value(Json::objectValue);
        const auto input_path = dir / "bootstrap.json";
        {
            std::ofstream file(input_path);
            file << wire::serialize(input);
        }
        chmod(input_path.c_str(), 0600);
        bootstrap = std::make_unique<BootstrapInput>(loadBootstrapInput(input_path.string()));
        host = std::make_unique<ResetHttpBoundary>(*bootstrap, std::vector<std::string>{"ugv0"},
                                                   resolve_runtime_policy({}));
        client = std::make_unique<HttpClient>((dir / "reset.sock").string(), HttpLimits{},
                                              host->instance());
    }
    ~Fixture() {
        client.reset();
        host.reset();
        std::filesystem::remove_all(dir);
    }
    HttpResponse call(std::string target, std::string body = "", int ms = 1000) {
        return client->call(
            {body.empty() ? "GET" : "POST", std::move(target), std::move(body), "", {}, {}},
            Clock::now() + std::chrono::milliseconds(ms));
    }
    void turn() {
        host->ownerTurn(control, 1'000'000'000, 1);
    }
    void await(std::future<HttpResponse>& result) {
        for (int i = 0;
             i < 200 && result.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready;
             ++i)
            turn();
    }
};
const std::string start =
    "{\"expected_revision\":0,\"robots\":[\"ugv0\"],\"expected_scene\":{\"epoch\":\"w\","
    "\"revision\":1,\"simulation_time_epoch\":2}}";
}  // namespace
TEST(ResetHttpBoundary, ActualSdkCallsDescribeBindAndCompleteAtOwnerTurn) {
    Fixture f;
    HttpClient discovery((f.dir / "reset.sock").string());
    auto describe = discovery.call({"GET", "/v1/describe", "", "", {}, {}},
                                   Clock::now() + std::chrono::seconds(1));
    ASSERT_EQ(describe.status, 200);
    auto ref = wire::parse(describe.body)["service_ref"];
    EXPECT_EQ(ref["service"].asString(), "ugv-reset");
    EXPECT_EQ(ref["instance_id"].asString(), f.host->instance());
    auto mutation =
        std::async(std::launch::async, [&] { return f.call("/v1/extensions/reset/start", start); });
    f.await(mutation);
    ASSERT_EQ(mutation.get().status, 202);
    EXPECT_EQ(f.starts, 1);
    EXPECT_EQ(f.control.snapshot().state, ResetControl::State::Accepted);
    auto snapshot = f.call("/v1/extensions/reset");
    EXPECT_EQ(wire::parse(snapshot.body)["state"].asString(), "accepted");
    auto conflict =
        std::async(std::launch::async, [&] { return f.call("/v1/extensions/reset/start", start); });
    f.await(conflict);
    EXPECT_EQ(conflict.get().status, 409);
    EXPECT_EQ(f.starts, 1);
}
TEST(ResetHttpBoundary, HeldObserveReturnsActualNativeResultAndNeverReadySuccess) {
    Fixture f;
    auto mutation =
        std::async(std::launch::async, [&] { return f.call("/v1/extensions/reset/start", start); });
    f.await(mutation);
    mutation.get();
    const auto before = f.control.snapshot();
    Json::Value observe;
    observe["operation"] = Json::UInt64(before.operation);
    observe["after_revision"] = Json::UInt64(before.revision);
    const auto body = wire::serialize(observe);
    auto held = std::async(std::launch::async,
                           [&] { return f.call("/v1/extensions/reset/observe", body); });
    ugv_reset_safety::ResetSession session;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    f.control.sample(0, session, false, false, true, 1'000'000'000, 1'020'000'000, 1.02);
    f.host->publish(f.control.snapshot());
    EXPECT_EQ(held.wait_for(std::chrono::milliseconds(20)), std::future_status::timeout);
    session.begin({});
    f.control.sample(0, session, true, false, true, 1'000'000'000, 1'040'000'000, 1.04);
    session.complete(ugv_reset_safety::ResetSession::Completion::Rejected);
    session.cancel();
    session.noteApplied({}, 1'060'000'000);
    f.control.sample(0, session, false, false, true, 1'060'000'000, 1'060'000'000, 1.06);
    f.host->publish(f.control.snapshot());
    ASSERT_EQ(held.wait_for(std::chrono::seconds(1)), std::future_status::ready);
    EXPECT_EQ(wire::parse(held.get().body)["state"].asString(), "rejected");
}
TEST(ResetHttpBoundary, ExpiredQueuedMutationIsNotExecutedAndClientKeepsUnknownDisposition) {
    Fixture f;
    auto expired = std::async(std::launch::async, [&] {
        try {
            f.call("/v1/extensions/reset/start", start, 40);
        } catch (const HttpCallError& error) {
            return error.delivery;
        }
        return Delivery::NotSent;
    });
    ASSERT_EQ(expired.wait_for(std::chrono::seconds(1)), std::future_status::ready);
    EXPECT_EQ(expired.get(), Delivery::OutcomeUnknown);
    f.turn();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_EQ(f.starts, 0);
}
TEST(ResetHttpBoundary, WrongInstanceMissingFenceAndDuplicateRosterCannotMutate) {
    Fixture f;
    HttpClient wrong((f.dir / "reset.sock").string(), {}, "foreign-incarnation");
    EXPECT_THROW(wrong.call({"POST", "/v1/extensions/reset/start", start, "", {}, {}},
                            Clock::now() + std::chrono::seconds(1)),
                 HttpCallError);
    EXPECT_EQ(f.starts, 0);
    EXPECT_EQ(
        f.call("/v1/extensions/reset/start", "{\"expected_revision\":0,\"robots\":[\"ugv0\"]}")
            .status,
        400);
    EXPECT_EQ(
        f.call("/v1/extensions/reset/start",
               "{\"expected_revision\":0,\"expected_revision\":0,\"robots\":[\"ugv0\"],\"expected_"
               "scene\":{\"epoch\":\"w\",\"revision\":1,\"simulation_time_epoch\":2}}")
            .status,
        400);
    EXPECT_EQ(f.starts, 0);
}
