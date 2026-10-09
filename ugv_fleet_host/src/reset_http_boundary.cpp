#include <fcntl.h>
#include <ugv_fleet_host/json.h>
#include <ugv_fleet_host/reset_http_boundary.h>
#include <unistd.h>

#include <array>
#include <cctype>
#include <condition_variable>
#include <mutex>
#include <set>
#include <sstream>
#include <thread>
namespace ugv_fleet_host {
namespace {
using namespace xgc2::xrpc;

struct OwnedFd {
    int value;
    explicit OwnedFd(int fd) : value(fd) {
        if (fd < 0)
            throw std::invalid_argument("explicit runtime allocation required");
    }
    ~OwnedFd() {
        ::close(value);
    }
    OwnedFd(const OwnedFd&) = delete;
    OwnedFd& operator=(const OwnedFd&) = delete;
};
int retainedRuntime(const BootstrapInput& bootstrap) {
    const auto grant = bootstrap.resolve_runtime([](const auto&, const auto& binding) {
        const auto& path = binding.endpoint().address;
        const auto parent = path.substr(0, path.rfind('/'));
        OwnedFd fd(::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW));
        return DirectoryGrant::from_owned_directory(fd.value, GrantPurpose::Runtime);
    });
    return grant.duplicate_fd();
}
bool authorizationName(std::string_view name) {
    constexpr std::string_view expected = "authorization";
    return name.size() == expected.size() &&
           std::equal(name.begin(), name.end(), expected.begin(),
                      [](unsigned char actual, unsigned char value) {
                          return std::tolower(actual) == value;
                      });
}

uint64_t number(const Json::Value& object, const char* key) {
    const auto& value = wire::member(object, key);
    if (value.type() != Json::intValue && value.type() != Json::uintValue)
        throw std::invalid_argument("canonical u53 required");
    if (!value.isUInt64() || value.asUInt64() > 9'007'199'254'740'991ULL)
        throw std::invalid_argument("canonical u53 required");
    return value.asUInt64();
}
Json::Value parse(const std::string& body, std::initializer_list<const char*> fields) {
    const auto value = wire::parse(body);
    wire::object(value);
    std::set<std::string> allowed;
    for (const auto* field : fields)
        allowed.insert(field);
    const auto keys = value.getMemberNames();
    if (std::set<std::string>(keys.begin(), keys.end()) != allowed)
        throw std::invalid_argument("required exact fields missing");
    return value;
}
HttpResponse response(int status, const Json::Value& body) {
    return {status, {{"content-type", "application/json"}}, wire::serialize(body), true};
}

}  // namespace
class ResetHttpBoundary::Impl {
    enum Stage { Free, Queued, Processing, Complete };
    struct Command {
        bool cancel{false};
        uint64_t expected{0}, operation{0};
        std::array<bool, ResetControl::capacity> robots{};
        ResetControl::SceneFence scene;
        Clock::time_point deadline;
    };
    struct Slot {
        std::atomic<Stage> stage{Free};
        Command command;
        HttpReply reply;
        int status{200};
        ResetControl::Snapshot snapshot;
    };
    struct Observer {
        bool used{false};
        uint64_t operation{0}, after{0};
        Clock::time_point deadline;
        HttpReply reply;
    };

   public:
    Impl(const BootstrapInput& bootstrap, std::vector<std::string> robots,
         const RuntimePolicy& policy)
        : bootstrap_(bootstrap),
          runtime_(retainedRuntime(bootstrap)),
          socket_(bootstrap.binding().endpoint().address),
          target_(bootstrap.binding().target_id()),
          robots_(std::move(robots)),
          instance_(new_instance_id()),
          limits_(http_limits(policy)) {
        const auto& binding = bootstrap_.binding();
        if (binding.service() != "ugv-reset" || binding.api_version() != "v1" ||
            binding.profile() != "http.v1" || binding.endpoint().kind != "unix" ||
            binding.authentication() != "local_private" || !binding.storage_grants().empty())
            throw std::invalid_argument("local-private UGV Reset HTTP v1 binding required");
        if (robots_.empty() || robots_.size() > ResetControl::capacity || target_.empty())
            throw std::invalid_argument("bounded roster and target identity required");
        std::set<std::string> ids;
        for (const auto& id : robots_)
            if (id.empty() || !ids.insert(id).second)
                throw std::invalid_argument("unique robot IDs required");
        limits_.request_bytes = std::min<std::size_t>(limits_.request_bytes, 32 * 1024);
        limits_.response_bytes = std::min<std::size_t>(limits_.response_bytes, 64 * 1024);
        limits_.inflight =
            std::min<std::size_t>(limits_.inflight, command_capacity + observer_capacity);
        snapshot_.count = robots_.size();
        worker_ = std::thread([this] { serve(); });
        std::unique_lock lock(startup_mutex_);
        startup_changed_.wait(lock, [this] { return ready_; });
        if (startup_error_) {
            lock.unlock();
            worker_.join();
            std::rethrow_exception(startup_error_);
        }
    }
    ~Impl() {
        stopping_.store(true);
        if (worker_.joinable())
            worker_.join();
    }
    void ownerTurn(ResetControl& control, uint64_t world, double steady) {
        for (auto& slot : slots_) {
            Stage expected = Queued;
            if (!slot.stage.compare_exchange_strong(expected, Processing,
                                                    std::memory_order_acquire))
                continue;
            bool ok = false;
            const bool expired = Clock::now() >= slot.command.deadline;
            if (!expired)
                ok = slot.command.cancel ? control.cancel(slot.command.expected,
                                                          slot.command.operation, world, steady)
                                         : control.start(slot.command.expected, slot.command.robots,
                                                         world, steady, slot.command.scene);
            slot.status = expired ? 408 : ok ? 202 : 409;
            slot.snapshot = control.snapshot();
            slot.stage.store(Complete, std::memory_order_release);
        }
        publish(control.snapshot());
    }
    void publish(const ResetControl::Snapshot& snapshot) noexcept {
        if (snapshot_mutex_.try_lock()) {
            snapshot_ = snapshot;
            snapshot_mutex_.unlock();
        }
        if (auto* server = wake_.load(std::memory_order_acquire))
            server->wake();
    }
    const std::string& instance() const {
        return instance_;
    }

   private:
    Json::Value serviceRef() const {
        Json::Value ref;
        ref["target_id"] = target_;
        ref["service"] = "ugv-reset";
        ref["api_version"] = "v1";
        ref["profile"] = "http.v1";
        ref["instance_id"] = instance_;
        ref["endpoint"]["kind"] = "unix";
        ref["endpoint"]["address"] = socket_;
        return ref;
    }
    Json::Value view(const ResetControl::Snapshot& snapshot) const {
        Json::Value value;
        value["revision"] = Json::UInt64(snapshot.revision);
        value["operation"] = Json::UInt64(snapshot.operation);
        value["state"] = ResetControl::name(snapshot.state);
        value["scene"]["epoch"] =
            std::string(snapshot.scene.epoch.begin(),
                        std::find(snapshot.scene.epoch.begin(), snapshot.scene.epoch.end(), '\0'));
        value["scene"]["revision"] = Json::UInt64(snapshot.scene.revision);
        value["scene"]["simulation_time_epoch"] =
            Json::UInt64(snapshot.scene.simulation_time_epoch);
        value["scene"]["operational"] = snapshot.scene_operational;
        value["robots"] = Json::Value(Json::arrayValue);
        for (std::size_t i = 0; i < snapshot.count; ++i) {
            Json::Value robot;
            robot["id"] = robots_[i];
            robot["generation"] = snapshot.robots[i].generation;
            robot["state"] = ResetControl::name(snapshot.robots[i].state);
            value["robots"].append(std::move(robot));
        }
        return value;
    }
    ResetControl::Snapshot latest() {
        std::lock_guard lock(snapshot_mutex_);
        return snapshot_;
    }
    void handle(HttpRequest request, HttpReply reply) {
        try {
            std::array<std::string_view, 2> authorization{};
            std::size_t count = 0;
            for (const auto& header : request.headers)
                if (authorizationName(header.first)) {
                    if (count == authorization.size()) {
                        reply.complete(
                            http_error(403, "permission_denied", "owner authorization required"));
                        return;
                    }
                    authorization[count++] = header.second;
                }
            if (!bootstrap_.authorize({authorization.data(), count}, request.deadline)) {
                reply.complete(
                    http_error(403, "permission_denied", "owner authorization required"));
                return;
            }
            if (request.method == "GET" && request.target == "/v1/describe") {
                Json::Value value;
                value["service_ref"] = serviceRef();
                reply.complete(response(200, value));
                return;
            }
            if (request.method == "GET" && request.target == "/v1/extensions/reset") {
                reply.complete(response(200, view(latest())));
                return;
            }
            if (request.method != "POST") {
                reply.complete(http_error(404, "not_found", "unknown reset method"));
                return;
            }
            if (request.target == "/v1/extensions/reset/observe") {
                const auto body = parse(request.body, {"operation", "after_revision"});
                const auto operation = number(body, "operation"),
                           after = number(body, "after_revision");
                const auto snapshot = latest();
                if (operation != snapshot.operation) {
                    reply.complete(
                        http_error(409, "operation_changed", "reset operation no longer current"));
                    return;
                }
                if (after > snapshot.revision) {
                    reply.complete(
                        http_error(409, "future_revision", "observe revision is in the future"));
                    return;
                }
                if (after < snapshot.revision) {
                    reply.complete(response(200, view(snapshot)));
                    return;
                }
                for (auto& observer : observers_)
                    if (!observer.used) {
                        observer = {true, operation, after, request.deadline, std::move(reply)};
                        return;
                    }
                reply.complete(
                    http_error(503, "observe_capacity", "held reset observers are full"));
                return;
            }
            Command command;
            command.deadline = request.deadline;
            if (request.target == "/v1/extensions/reset/start") {
                const auto body =
                    parse(request.body, {"expected_revision", "robots", "expected_scene"});
                command.expected = number(body, "expected_revision");
                const auto& scene = wire::object(wire::member(body, "expected_scene"));
                if (scene.size() != 3)
                    throw std::invalid_argument("exact scientific scene fence required");
                const auto& epoch = wire::string(wire::member(scene, "epoch"));
                if (epoch.empty() || epoch.size() > command.scene.epoch.size() ||
                    epoch.find('\0') != epoch.npos)
                    throw std::invalid_argument("bounded scene epoch required");
                std::copy(epoch.begin(), epoch.end(), command.scene.epoch.begin());
                command.scene.revision = number(scene, "revision");
                command.scene.simulation_time_epoch = number(scene, "simulation_time_epoch");
                const auto& selected = wire::array(wire::member(body, "robots"));
                if (selected.empty() || selected.size() > robots_.size())
                    throw std::invalid_argument("nonempty bounded robot subset required");
                for (const auto& value : selected) {
                    const auto id = wire::string(value);
                    const auto found = std::find(robots_.begin(), robots_.end(), id);
                    if (found == robots_.end())
                        throw std::invalid_argument("robot outside frozen roster");
                    auto& bit = command.robots[std::distance(robots_.begin(), found)];
                    if (bit)
                        throw std::invalid_argument("duplicate robot");
                    bit = true;
                }
            } else if (request.target == "/v1/extensions/reset/cancel") {
                const auto body = parse(request.body, {"expected_revision", "operation"});
                command.cancel = true;
                command.expected = number(body, "expected_revision");
                command.operation = number(body, "operation");
            } else {
                reply.complete(http_error(404, "not_found", "unknown reset route"));
                return;
            }
            for (auto& slot : slots_)
                if (slot.stage.load(std::memory_order_acquire) == Free) {
                    slot.command = command;
                    slot.reply = std::move(reply);
                    slot.stage.store(Queued, std::memory_order_release);
                    return;
                }
            reply.complete(
                http_error(503, "command_capacity", "reset owner command slots are full"));
        } catch (const std::exception&) {
            reply.complete(http_error(400, "invalid_request", "invalid bounded reset request"));
        }
    }
    void complete() {
        for (auto& slot : slots_)
            if (slot.stage.load(std::memory_order_acquire) == Complete) {
                slot.reply.complete(
                    slot.status == 202
                        ? response(slot.status, view(slot.snapshot))
                        : http_error(slot.status,
                                     slot.status == 408 ? "deadline_exceeded" : "conflict",
                                     slot.status == 408
                                         ? "reset command expired before admission"
                                         : "reset revision, scene or native state conflicts"));
                slot.reply = {};
                slot.stage.store(Free, std::memory_order_release);
            }
        const auto snapshot = latest();
        for (auto& observer : observers_)
            if (observer.used) {
                if (observer.reply.cancelled() || Clock::now() >= observer.deadline) {
                    observer.reply = {};
                    observer.used = false;
                } else if (observer.operation != snapshot.operation) {
                    observer.reply.complete(
                        http_error(409, "operation_changed", "reset operation changed"));
                    observer.reply = {};
                    observer.used = false;
                } else if (observer.after < snapshot.revision) {
                    observer.reply.complete(response(200, view(snapshot)));
                    observer.reply = {};
                    observer.used = false;
                }
            }
    }
    void serve() {
        try {
            HttpServer server(
                {socket_},
                [this](auto request, auto reply) { handle(std::move(request), std::move(reply)); },
                limits_, {instance_, {"/v1/describe"}}, runtime_.value);
            server.set_wakeup_handler([this] { complete(); });
            wake_.store(&server, std::memory_order_release);
            {
                std::lock_guard lock(startup_mutex_);
                ready_ = true;
            }
            startup_changed_.notify_all();
            while (!stopping_.load()) {
                server.poll(std::chrono::milliseconds{2});
                complete();
            }
            // Producers have stopped before destruction. Finish/cancel every
            // retained business slot before the SDK releases its endpoint lease.
            for (auto& slot : slots_) {
                if (slot.stage.load() != Free) {
                    slot.reply.complete(http_error(503, "stopping", "reset owner stopped"));
                    slot.reply = {};
                    slot.stage.store(Free);
                }
            }
            for (auto& observer : observers_) {
                observer.reply = {};
                observer.used = false;
            }
            wake_.store(nullptr, std::memory_order_release);
            server.drain_until(Clock::now() + limits_.shutdown_timeout);
        } catch (...) {
            std::lock_guard lock(startup_mutex_);
            startup_error_ = std::current_exception();
            ready_ = true;
            startup_changed_.notify_all();
        }
    }
    const BootstrapInput& bootstrap_;
    OwnedFd runtime_;
    std::string socket_, target_;
    std::vector<std::string> robots_;
    std::string instance_;
    HttpLimits limits_;
    std::array<Slot, command_capacity> slots_{};
    std::array<Observer, observer_capacity> observers_{};
    std::mutex snapshot_mutex_;
    ResetControl::Snapshot snapshot_;
    std::atomic<HttpServer*> wake_{nullptr};
    std::atomic<bool> stopping_{false};
    std::thread worker_;
    std::mutex startup_mutex_;
    std::condition_variable startup_changed_;
    bool ready_{false};
    std::exception_ptr startup_error_;
};
ResetHttpBoundary::ResetHttpBoundary(const BootstrapInput& bootstrap,
                                     std::vector<std::string> robots, const RuntimePolicy& policy)
    : impl_(std::make_unique<Impl>(bootstrap, std::move(robots), policy)) {}
ResetHttpBoundary::~ResetHttpBoundary() = default;
void ResetHttpBoundary::ownerTurn(ResetControl& control, uint64_t world, double steady) {
    impl_->ownerTurn(control, world, steady);
}
void ResetHttpBoundary::publish(const ResetControl::Snapshot& snapshot) noexcept {
    impl_->publish(snapshot);
}
const std::string& ResetHttpBoundary::instance() const {
    return impl_->instance();
}
}  // namespace ugv_fleet_host
