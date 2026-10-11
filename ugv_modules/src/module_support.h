#pragma once

// What the three modules of this package share: the typed view of the host's function table, the
// descriptor helpers, the JSON configuration as a source for the cores' config loaders and the
// trampolines that turn an instance class into the module's lifecycle functions.

#include <jsoncpp/json/json.h>
#include <xgc2/module.h>

#include <array>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "xgc2_ugv/payloads.h"

#define UGV_MODULE_EXPORT extern "C" __attribute__((visibility("default")))

namespace ugv_modules {

constexpr int kLogDebug = 0;
constexpr int kLogInfo = 1;
constexpr int kLogWarn = 2;
constexpr int kLogError = 3;
constexpr int kHealthOk = 0;
constexpr int kHealthDegraded = 1;
constexpr int kHealthFailed = 2;

// A port descriptor of payload type T. The alignment of every payload of this package is 8.
template <typename T>
constexpr xgc2_port_desc port(const char* name, xgc2_port_direction direction, xgc2_port_kind kind,
                              const char* schema_id, uint32_t queue_depth = 0U,
                              uint32_t flags = 0U) {
    return xgc2_port_desc{name,
                          static_cast<uint32_t>(direction),
                          static_cast<uint32_t>(kind),
                          schema_id,
                          static_cast<uint32_t>(sizeof(T)),
                          static_cast<uint32_t>(alignof(T)),
                          queue_depth,
                          flags};
}

// A sample read from an input: the payload (borrowed until the step returns), the producer's stamp
// and the channel's sequence number.
template <typename T>
struct Sample {
    const T* data{nullptr};
    int64_t stamp_ns{0};
    uint64_t seq{0};

    explicit operator bool() const {
        return data != nullptr;
    }
    const T* operator->() const {
        return data;
    }
    const T& operator*() const {
        return *data;
    }
};

class Host;

// An output slot to fill in place, zeroed first (a slot holds the data of an older sample). It is
// published by commit(); a slot that goes out of scope without a commit is handed back.
template <typename T>
class Slot {
   public:
    Slot(const Host& host, uint32_t port);
    ~Slot();
    Slot(const Slot&) = delete;
    Slot& operator=(const Slot&) = delete;

    // False when the port is being written or its event queue is full.
    explicit operator bool() const {
        return data_ != nullptr;
    }
    T* operator->() {
        return data_;
    }
    T& operator*() {
        return *data_;
    }
    bool commit(int64_t stamp_ns);

   private:
    const Host* host_;
    uint32_t port_;
    T* data_;
};

// The host's function table of one instance. Reads are valid inside step only and the pointers
// they return are borrowed until the step returns.
class Host {
   public:
    Host() = default;
    Host(const xgc2_host_api* api, void* context) : api_(api), context_(context) {}

    int64_t nowNs() const {
        return api_->now_ns(context_);
    }
    void wake() const {
        api_->wake(context_);
    }
    void log(int level, const std::string& message) const {
        api_->log(context_, level, message.c_str());
    }
    void report(int health, const std::string& detail) const {
        api_->report(context_, health, detail.c_str());
    }

    // The newest sample of a state input; empty without data (or of another size).
    template <typename T>
    Sample<T> latest(uint32_t port_index) const {
        xgc2_sample_view view{};
        if (api_->read_latest(context_, port_index, &view) != XGC2_OK || view.size != sizeof(T)) {
            return {};
        }
        return {static_cast<const T*>(view.data), view.stamp_ns, view.seq};
    }
    // The next event of an event input; empty when there is none.
    template <typename T>
    Sample<T> next(uint32_t port_index) const {
        xgc2_sample_view view{};
        if (api_->read_next(context_, port_index, &view) != XGC2_OK || view.size != sizeof(T)) {
            return {};
        }
        return {static_cast<const T*>(view.data), view.stamp_ns, view.seq};
    }
    template <typename T>
    Slot<T> write(uint32_t port_index) const {
        return Slot<T>(*this, port_index);
    }

    void* begin(uint32_t port_index) const {
        return api_->write_begin(context_, port_index);
    }
    bool commit(uint32_t port_index, int64_t stamp_ns) const {
        return api_->write_commit(context_, port_index, stamp_ns) == XGC2_OK;
    }
    void abort(uint32_t port_index) const {
        api_->write_abort(context_, port_index);
    }

   private:
    const xgc2_host_api* api_{nullptr};
    void* context_{nullptr};
};

template <typename T>
Slot<T>::Slot(const Host& host, uint32_t port)
    : host_(&host), port_(port), data_(static_cast<T*>(host.begin(port))) {
    static_assert(std::is_trivially_copyable<T>::value, "payloads are plain data");
    if (data_ != nullptr) {
        std::memset(static_cast<void*>(data_), 0, sizeof(T));
    }
}

template <typename T>
Slot<T>::~Slot() {
    if (data_ != nullptr) {
        host_->abort(port_);
    }
}

template <typename T>
bool Slot<T>::commit(int64_t stamp_ns) {
    if (data_ == nullptr) {
        return false;
    }
    data_ = nullptr;
    return host_->commit(port_, stamp_ns);
}

// The newest sequence number taken from each state input, to tell a new sample from the one the
// previous step already took.
class SeenSamples {
   public:
    template <typename T>
    bool fresh(uint32_t port, const Sample<T>& sample) {
        if (!sample) {
            return false;
        }
        Entry& entry = entries_.at(port);
        if (entry.seen && entry.seq == sample.seq) {
            return false;
        }
        entry.seen = true;
        entry.seq = sample.seq;
        return true;
    }
    void forget() {
        entries_ = {};
    }

   private:
    struct Entry {
        bool seen{false};
        uint64_t seq{0};
    };
    std::array<Entry, XGC2_MODULE_MAX_PORTS> entries_{};
};

// A host of the same major version and at least the minor version this module was built against.
template <uint32_t Minor>
constexpr bool minorAtLeast(uint32_t host_minor) {
    return host_minor >= Minor;
}

inline bool compatibleHost(const xgc2_host_api* host) {
    return host != nullptr && host->abi_major == XGC2_MODULE_ABI_MAJOR &&
           minorAtLeast<XGC2_MODULE_ABI_MINOR>(host->abi_minor);
}

// The configuration object of an instance as the source of the cores' config loaders: keys are
// '/'-separated paths into nested objects (see config_loader.h of the cores). A value of another
// type than asked for is an error, and every key that was never read is reported by
// rejectUnused(), so a typo or a key of another module never leaves a default in place silently.
class JsonSource {
   public:
    // Throws std::invalid_argument unless `config` is a JSON object (or empty). The text is
    // strict JSON: no comments, no duplicate keys, nothing after the object.
    explicit JsonSource(const xgc2_config* config) {
        if (config == nullptr || config->json == nullptr || config->length == 0U) {
            root_ = Json::Value(Json::objectValue);
            return;
        }
        Json::CharReaderBuilder builder;
        Json::CharReaderBuilder::strictMode(&builder.settings_);
        const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
        std::string errors;
        if (!reader->parse(config->json, config->json + config->length, &root_, &errors)) {
            throw std::invalid_argument("the configuration is not JSON: " + errors);
        }
        if (!root_.isObject()) {
            throw std::invalid_argument("the configuration is not a JSON object");
        }
    }

    template <typename T>
    typename std::enable_if<std::is_floating_point<T>::value, bool>::type get(
        const std::string& key, T& value) const {
        const Json::Value* node = find(key);
        if (node == nullptr) {
            return false;
        }
        if (!node->isNumeric()) {
            throw typeError(key, "a number");
        }
        value = static_cast<T>(node->asDouble());
        return true;
    }
    template <typename T>
    typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, bool>::value, bool>::type
    get(const std::string& key, T& value) const {
        const Json::Value* node = find(key);
        if (node == nullptr) {
            return false;
        }
        if (!node->isIntegral() || !node->isInt64()) {
            throw typeError(key, "an integer");
        }
        const Json::Int64 wide = node->asInt64();
        const bool in_range =
            wide < 0 ? (std::is_signed<T>::value &&
                        wide >= static_cast<Json::Int64>(std::numeric_limits<T>::lowest()))
                     : static_cast<uint64_t>(wide) <=
                           static_cast<uint64_t>(std::numeric_limits<T>::max());
        if (!in_range) {
            throw typeError(key, "an integer in range");
        }
        value = static_cast<T>(wide);
        return true;
    }
    bool get(const std::string& key, bool& value) const {
        const Json::Value* node = find(key);
        if (node == nullptr) {
            return false;
        }
        if (!node->isBool()) {
            throw typeError(key, "a boolean");
        }
        value = node->asBool();
        return true;
    }
    bool get(const std::string& key, std::string& value) const {
        const Json::Value* node = find(key);
        if (node == nullptr) {
            return false;
        }
        if (!node->isString()) {
            throw typeError(key, "a string");
        }
        value = node->asString();
        return true;
    }
    bool has(const std::string& key) const {
        return peek(key) != nullptr;
    }
    bool isMapping(const std::string& key) const {
        const Json::Value* node = peek(key);
        return node != nullptr && node->isObject();
    }
    std::vector<std::string> keys(const std::string& key) const {
        const Json::Value* node = peek(key);
        return node != nullptr && node->isObject() ? node->getMemberNames()
                                                   : std::vector<std::string>();
    }

    // Throws std::invalid_argument naming the first key (sorted) that nothing read.
    void rejectUnused() const {
        std::set<std::string> leaves;
        collectLeaves(root_, "", leaves);
        for (const std::string& leaf : leaves) {
            if (used_.count(leaf) == 0U) {
                throw std::invalid_argument("unknown configuration key '" + leaf + "'");
            }
        }
    }

   private:
    static std::invalid_argument typeError(const std::string& key, const char* expected) {
        return std::invalid_argument("configuration key '" + key + "' must be " + expected);
    }

    const Json::Value* peek(const std::string& key) const {
        const Json::Value* node = &root_;
        size_t begin = 0;
        for (;;) {
            const size_t end = key.find('/', begin);
            const std::string part =
                key.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
            if (!node->isObject() || !node->isMember(part)) {
                return nullptr;
            }
            node = &(*node)[part];
            if (end == std::string::npos) {
                return node;
            }
            begin = end + 1;
        }
    }
    const Json::Value* find(const std::string& key) const {
        const Json::Value* node = peek(key);
        if (node != nullptr) {
            used_.insert(key);
        }
        return node;
    }
    static void collectLeaves(const Json::Value& node, const std::string& prefix,
                              std::set<std::string>& leaves) {
        for (const std::string& name : node.getMemberNames()) {
            std::string path = prefix;
            if (!path.empty()) {
                path += '/';
            }
            path += name;
            if (node[name].isObject()) {
                collectLeaves(node[name], path, leaves);
            } else {
                leaves.insert(path);
            }
        }
    }

    Json::Value root_;
    mutable std::set<std::string> used_;
};

// At most one message per `period` seconds of the caller's clock, for warnings that can repeat at
// the rate of the inputs.
class LogThrottle {
   public:
    explicit LogThrottle(double period = 1.0) : period_(period) {}
    bool due(double now_sec) {
        if (now_sec >= last_ && now_sec - last_ < period_) {
            return false;
        }
        last_ = now_sec;
        return true;
    }

   private:
    double period_;
    double last_{-1.0e300};
};

// "text %d" into a std::string.
inline std::string format(const char* pattern, ...) __attribute__((format(printf, 1, 2)));
inline std::string format(const char* pattern, ...) {
    va_list args;
    va_start(args, pattern);
    va_list measure;
    va_copy(measure, args);
    const int length = std::vsnprintf(nullptr, 0, pattern, measure);
    va_end(measure);
    std::string text(length > 0 ? static_cast<size_t>(length) : 0U, '\0');
    if (length > 0) {
        std::vsnprintf(&text[0], text.size() + 1U, pattern, args);
    }
    va_end(args);
    return text;
}

// The seconds of a host time in ns, computed the way ros::Time::toSec does (whole seconds plus
// 1e-9 times the nanoseconds), so the cores see the same doubles they saw from ROS time.
template <typename Time>
double secondsOf(int64_t ns) {
    Time time;
    time.fromNSec(ns > 0 ? static_cast<uint64_t>(ns) : 0U);
    return time.toSec();
}

// A time in ns as the core's Time; a non-positive value is the zero time.
template <typename Time>
Time timeOf(int64_t ns) {
    Time time;
    time.fromNSec(ns > 0 ? static_cast<uint64_t>(ns) : 0U);
    return time;
}

// The thread that built the cores of an instance.
//
// The state machine of a core belongs to the thread that built it and refuses every other one:
// a core that is built on one thread and stepped on another stops working without a sound, the
// invalid-input flag its only sign. The host keeps an instance on one thread for its life
// (manifest `affinity = "sticky"`, the default of the host),
// and an instance checks that it has been: bind() where the cores are built, check() where they are
// used. A host that moved the instance would stop it with a message that says why, not leave it
// half alive.
class ThreadGuard {
   public:
    // The calling thread built the cores (again).
    void bind() {
        owner_ = std::this_thread::get_id();
    }

    // Throws unless the calling thread is the one that built the cores.
    void check(const char* call) const {
        if (std::this_thread::get_id() != owner_) {
            throw std::runtime_error(
                std::string(call) +
                " came from another thread than the one that built the cores; the instance needs "
                "the host's thread affinity (affinity = \"sticky\" in the manifest)");
        }
    }

   private:
    std::thread::id owner_;
};

// The lifecycle functions of a module whose instances are objects of class Instance:
//   Instance(const Host&)
//   void configure(const xgc2_config*)    first and live; throws std::invalid_argument for a
//                                          configuration it refuses, and then keeps the old one
//   void start();  void step(const xgc2_step_ctx&);  void stop();
// An exception never crosses into the host: a refused configuration is XGC2_ERR_INVALID, any other
// failure XGC2_ERR_INTERNAL (and a failed step is reported, which stops the instance).
template <typename Instance>
struct Lifecycle {
    static Instance& self(xgc2_instance* handle) {
        return *reinterpret_cast<Instance*>(handle);
    }

    static xgc2_status create(const xgc2_host_api* api, void* context, const xgc2_config* config,
                              xgc2_instance** out) {
        if (!compatibleHost(api) || out == nullptr) {
            return XGC2_ERR_INVALID;
        }
        const Host host(api, context);
        std::unique_ptr<Instance> instance;
        try {
            instance.reset(new Instance(host));
            instance->configure(config);
        } catch (const std::invalid_argument& error) {
            host.log(kLogError, std::string("Invalid configuration: ") + error.what());
            return XGC2_ERR_INVALID;
        } catch (const std::exception& error) {
            host.log(kLogError, std::string("Cannot create the instance: ") + error.what());
            return XGC2_ERR_INTERNAL;
        }
        *out = reinterpret_cast<xgc2_instance*>(instance.release());
        return XGC2_OK;
    }

    static xgc2_status configure(xgc2_instance* handle, const xgc2_config* config) {
        Instance& instance = self(handle);
        try {
            instance.configure(config);
        } catch (const std::invalid_argument& error) {
            instance.host().log(kLogError, std::string("Invalid configuration: ") + error.what());
            return XGC2_ERR_INVALID;
        } catch (const std::exception& error) {
            instance.host().log(kLogError, std::string("Configuration failed: ") + error.what());
            return XGC2_ERR_INTERNAL;
        }
        return XGC2_OK;
    }

    static xgc2_status start(xgc2_instance* handle) {
        Instance& instance = self(handle);
        try {
            instance.start();
        } catch (const std::exception& error) {
            instance.host().log(kLogError, std::string("Cannot start: ") + error.what());
            return XGC2_ERR_INTERNAL;
        }
        return XGC2_OK;
    }

    static xgc2_status step(xgc2_instance* handle, const xgc2_step_ctx* ctx) {
        Instance& instance = self(handle);
        try {
            instance.step(*ctx);
        } catch (const std::exception& error) {
            instance.host().report(kHealthFailed, std::string("step failed: ") + error.what());
            return XGC2_ERR_INTERNAL;
        }
        return XGC2_OK;
    }

    static xgc2_status stop(xgc2_instance* handle) {
        Instance& instance = self(handle);
        try {
            instance.stop();
        } catch (const std::exception& error) {
            instance.host().log(kLogError, std::string("Cannot stop: ") + error.what());
            return XGC2_ERR_INTERNAL;
        }
        return XGC2_OK;
    }

    static void destroy(xgc2_instance* handle) {
        delete &self(handle);
    }
};

}  // namespace ugv_modules
