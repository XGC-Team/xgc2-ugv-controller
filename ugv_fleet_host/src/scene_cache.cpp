#include <ugv_fleet_host/json.h>
#include <ugv_fleet_host/scene_cache.h>
#include <ugv_reset_safety/scene_projection.h>

#include <atomic>
#include <charconv>
#include <cmath>
#include <mutex>
#include <set>
#include <thread>
#include <xgc2/xrpc/http.hpp>
namespace ugv_fleet_host {
namespace {
using namespace ugv_reset_safety;
using namespace xgc2::xrpc;
uint64_t u53(const Json::Value& value) {
    uint64_t n = value.type() == Json::uintValue
                     ? value.asUInt64()
                     : value.type() == Json::intValue && value.asInt64() >= 0
                           ? static_cast<uint64_t>(value.asInt64())
                           : UINT64_MAX;
    if (n > 9'007'199'254'740'991ULL)
        throw std::invalid_argument("u53 required");
    return n;
}
uint64_t decimal(const Json::Value& value) {
    const auto& s = wire::string(value);
    if (s.empty() || (s.size() > 1 && s.front() == '0'))
        throw std::invalid_argument("canonical decimal timestamp required");
    uint64_t n = 0;
    auto result = std::from_chars(s.data(), s.data() + s.size(), n);
    if (result.ec != std::errc{} || result.ptr != s.data() + s.size())
        throw std::invalid_argument("u64 timestamp required");
    return n;
}
double real(const Json::Value& value) {
    const double n =
        value.isDouble()
            ? value.asDouble()
            : value.type() == Json::intValue
                  ? static_cast<double>(value.asInt64())
                  : value.type() == Json::uintValue ? static_cast<double>(value.asUInt64()) : NAN;
    if (!std::isfinite(n))
        throw std::invalid_argument("finite geometry required");
    return n;
}
std::string id(const Json::Value& value) {
    const auto& s = wire::string(value);
    if (s.empty() || s.size() > 128 || s.find('\0') != s.npos)
        throw std::invalid_argument("bounded identity required");
    return std::string(s);
}
scene_model::Vector vector(const Json::Value& value) {
    const auto& v = wire::array(value);
    if (v.size() != 3)
        throw std::invalid_argument("three coordinate vector required");
    return {real(v[0]), real(v[1]), real(v[2])};
}
scene_model::Pose pose(const Json::Value& value) {
    const auto& o = value;
    const auto& q = wire::array(wire::member(o, "orientation"));
    if (q.size() != 4)
        throw std::invalid_argument("four coordinate quaternion required");
    return {vector(wire::member(o, "position")), {real(q[0]), real(q[1]), real(q[2]), real(q[3])}};
}
}  // namespace
SceneSnapshot parseSceneSnapshot(const std::string& body) {
    if (body.size() > 1024 * 1024)
        throw std::invalid_argument("scene response budget exceeded");
    const auto value = wire::parse(body);
    const auto& o = value;
    SceneSnapshot result;
    const auto epoch = id(wire::member(o, "epoch"));
    const auto revision = u53(wire::member(o, "revision"));
    const auto stamp = decimal(wire::member(o, "stamp_ns"));
    if (wire::string(wire::member(o, "frame")) != "world")
        throw std::invalid_argument("world frame required");
    const auto& simulation = wire::member(o, "simulation_time");
    const scene_model::SimulationTime time{u53(wire::member(simulation, "epoch")),
                                           {decimal(wire::member(simulation, "nanoseconds"))}};
    result.definition.epoch = result.state.epoch = epoch;
    result.definition.revision = result.state.revision = revision;
    result.definition.header.stamp = result.state.header.stamp = {stamp};
    result.definition.simulation_time = result.state.simulation_time = time;
    result.state.scene_time = real(wire::member(o, "scene_time"));
    if (result.state.scene_time < 0)
        throw std::invalid_argument("negative scene time");
    result.serial = u53(wire::member(o, "serial"));
    result.applied = wire::boolean(wire::member(o, "applied"));
    result.playing = wire::boolean(wire::member(o, "playing"));
    const auto& obstacles = wire::array(wire::member(wire::member(o, "definition"), "obstacles"));
    if (obstacles.size() > 256)
        throw std::invalid_argument("scene roster budget exceeded");
    std::size_t part_count = 0;
    for (const auto& value : obstacles) {
        const auto& obstacle = value;
        scene_model::Obstacle native;
        native.id = id(wire::member(obstacle, "id"));
        native.motion_type = id(wire::member(obstacle, "motion_type"));
        native.dynamic = wire::boolean(wire::member(obstacle, "dynamic"));
        native.pose = pose(wire::member(obstacle, "pose"));
        const auto& parts = wire::array(wire::member(obstacle, "parts"));
        if (parts.empty() || parts.size() > 128 || part_count + parts.size() > 4096)
            throw std::invalid_argument("scene parts budget exceeded");
        part_count += parts.size();
        for (const auto& value : parts) {
            const auto& part = value;
            scene_model::Part p;
            p.id = id(wire::member(part, "id"));
            p.pose = pose(wire::member(part, "pose"));
            const auto& g = wire::member(part, "geometry");
            p.geometry.type = id(wire::member(g, "type"));
            if (g.isMember("size"))
                p.geometry.size = vector(g["size"]);
            if (g.isMember("radius"))
                p.geometry.radius = real(g["radius"]);
            if (g.isMember("height"))
                p.geometry.height = real(g["height"]);
            if (g.isMember("vertices")) {
                if (wire::array(g["vertices"]).size() > 4096)
                    throw std::invalid_argument("convex vertex budget exceeded");
                for (const auto& vertex : wire::array(g["vertices"]))
                    p.geometry.vertices.push_back(vector(vertex));
            }
            native.parts.push_back(std::move(p));
        }
        result.definition.obstacles.push_back(std::move(native));
    }
    const auto& states = wire::array(wire::member(wire::member(o, "state"), "obstacles"));
    if (states.size() != obstacles.size())
        throw std::invalid_argument("scene state roster mismatch");
    for (const auto& value : states) {
        const auto& o = value;
        scene_model::ObstacleState state;
        state.id = id(wire::member(o, "id"));
        state.pose = pose(wire::member(o, "pose"));
        const auto& twist = wire::member(o, "twist");
        state.twist = {vector(wire::member(twist, "linear")),
                       vector(wire::member(twist, "angular"))};
        result.state.obstacles.push_back(std::move(state));
    }
    scene_projection::live(result.definition, result.state, "world");
    return result;
}
class SceneCache::Impl {
   public:
    Impl(ServiceRef reference, const RuntimePolicy& policy)
        : reference_(std::move(reference)), limits_(http_limits(policy)) {
        if (reference_.target_id.empty() || reference_.service != "xgc2.simulation" ||
            reference_.api_version != "v1" || reference_.profile != "http.v1" ||
            reference_.endpoint.kind != "unix" || reference_.endpoint.address.empty() ||
            reference_.instance_id.empty())
            throw std::invalid_argument("actual native simulation ServiceRef required");
        limits_.response_bytes = std::min<std::size_t>(limits_.response_bytes, 1024 * 1024);
        worker_ = std::thread([this] { run(); });
    }
    ~Impl() {
        stopping_ = true;
        if (worker_.joinable())
            worker_.join();
    }
    bool take(SceneSnapshot& snapshot) {
        if (!mutex_.try_lock())
            return false;
        std::unique_lock lock(mutex_, std::adopt_lock);
        if (!pending_)
            return false;
        snapshot = std::move(snapshot_);
        pending_ = false;
        return true;
    }
    bool failed() const {
        return failed_;
    }

   private:
    void run() {
        try {
            HttpClient client(reference_.endpoint.address, limits_, reference_.instance_id);
            uint64_t serial = 0, stamp = 0, clock_epoch = 0, clock_ns = 0;
            bool first = true, read_snapshot = true;
            while (!stopping_) {
                HttpRequest request;
                request.method = read_snapshot ? "GET" : "POST";
                request.target =
                    read_snapshot ? "/v1/extensions/scene" : "/v1/extensions/scene/observe";
                if (!read_snapshot)
                    request.body = wire::serialize([&] {
                        Json::Value value;
                        value["after_serial"] = Json::UInt64(serial);
                        return value;
                    }());
                HttpResponse response;
                try {
                    response =
                        client.call(std::move(request),
                                    Clock::now() + std::chrono::milliseconds(first ? 1000 : 400));
                } catch (const HttpCallError& error) {
                    if (error.code == "deadline_exceeded" && !read_snapshot) {
                        read_snapshot = true;
                        continue;
                    }
                    throw;
                }
                if (!read_snapshot && response.status == 504) {
                    const auto error = wire::parse(response.body);
                    if (wire::string(wire::member(wire::member(error, "error"), "code")) ==
                        "deadline_exceeded") {
                        read_snapshot = true;
                        continue;
                    }
                }
                if (response.status != 200)
                    throw std::runtime_error("scene observation rejected");
                auto snapshot = parseSceneSnapshot(response.body);
                // Serial is a change cursor, not the scientific clock. A quiet
                // held observation ends with one read from the same bound Ref;
                // only the provider's actual epoch/stamp can renew freshness.
                if (!first &&
                    (snapshot.serial < serial || (!read_snapshot && snapshot.serial == serial)))
                    throw std::runtime_error("scene observation cursor did not advance");
                const auto next_stamp = snapshot.state.header.stamp.toNSec();
                const auto next_epoch = snapshot.state.simulation_time.epoch;
                const auto next_ns = snapshot.state.simulation_time.nanoseconds.toNSec();
                if (!first && snapshot.serial == serial && next_stamp == stamp &&
                    next_epoch == clock_epoch && next_ns == clock_ns) {
                    read_snapshot = false;
                    continue;
                }
                serial = snapshot.serial;
                stamp = next_stamp;
                clock_epoch = next_epoch;
                clock_ns = next_ns;
                first = false;
                read_snapshot = false;
                {
                    std::lock_guard lock(mutex_);
                    snapshot_ = std::move(snapshot);
                    pending_ = true;
                }
            }
        } catch (...) {
            failed_ = true;
        }
    }
    ServiceRef reference_;
    HttpLimits limits_;
    std::atomic<bool> stopping_{false}, failed_{false};
    std::thread worker_;
    std::mutex mutex_;
    SceneSnapshot snapshot_;
    bool pending_{false};
};
SceneCache::SceneCache(ServiceRef reference, const RuntimePolicy& policy)
    : impl_(std::make_unique<Impl>(std::move(reference), policy)) {}
SceneCache::~SceneCache() = default;
bool SceneCache::take(SceneSnapshot& snapshot) {
    return impl_->take(snapshot);
}
bool SceneCache::failed() const {
    return impl_->failed();
}
}  // namespace ugv_fleet_host
