// ugv_unicycle_reference: the planar reference trajectory generator of a unicycle vehicle as a
// module.
//
// The generator is unicycle_reference_trajectory_core, the code of the ROS node
// unicycle_reference_trajectory_node: the requests arrive on event ports instead of topics, and the
// status and the active references leave on state ports. One instance is stepped by its inputs and
// by its period (the node's main loop: period_ms = 10 for its 100 Hz).
//
//   in  analytic_request   event  xgc2.ugv.unicycle_reference.analytic.v1
//   in  sampled_request    event  xgc2.ugv.unicycle_reference.sampled.v1
//   in  waypoint_request   event  xgc2.ugv.unicycle_reference.waypoint_request.v1
//   in  reset              event  xgc2.ugv.unicycle_reference.reset.v1
//   out status             state  xgc2.ugv.unicycle_reference.status.v1
//   out active_analytic    state  xgc2.ugv.unicycle_reference.analytic.v1
//   out active_polynomial  state  xgc2.ugv.unicycle_reference.polynomial.v1
//   out active_sampled     state  xgc2.ugv.unicycle_reference.sampled.v1
//
// The configuration is a JSON object with the keys of config/unicycle_reference_trajectory.yaml
// that concern the generator: status_rate, active_publish_rate, validation_sample_dt,
// trajectory_timeout, min_lead_time, max_velocity, max_acceleration, max_yaw_rate and the
// default_analytic table. A key that is absent keeps the generator's default; a key the module does
// not know is an error. A live configure applies the values as the node does at start: it restarts
// the generator, and the active reference is dropped.
//
// Requests are served in the order they were committed. The times of the payloads are host clock
// times, and the generator's time is the now_ns of the step.

#include <xgc2/module.h>

#include <algorithm>
#include <string>
#include <vector>

#include "module_support.h"
#include "reference_payloads.h"
#include "unicycle_reference_trajectory/config_loader.h"
#include "unicycle_reference_trajectory/reference_driver.h"

#ifndef UGV_MODULES_VERSION
#define UGV_MODULES_VERSION "0.0.0"
#endif

namespace {

using namespace ugv_modules;
namespace urt = unicycle_reference_trajectory;
namespace rp = ugv_modules::reference_payloads;
using Request = urt::ReferenceTrajectoryDriver::Request;

enum Port : uint32_t {
    kAnalyticRequest = 0,
    kSampledRequest,
    kWaypointRequest,
    kResetRequest,
    kStatus,
    kActiveAnalytic,
    kActivePolynomial,
    kActiveSampled,
    kPortCount
};
constexpr uint32_t kInputCount = kStatus;
constexpr uint32_t kDefaultAnalytic = kPortCount;  // the throttle of the default request's warnings

const xgc2_port_desc kPorts[kPortCount] = {
    port<xgc2_ugv_analytic_reference>("analytic_request", XGC2_PORT_IN, XGC2_PORT_EVENT,
                                      XGC2_UGV_SCHEMA_ANALYTIC_REFERENCE, 8U),
    port<xgc2_ugv_sampled_reference>("sampled_request", XGC2_PORT_IN, XGC2_PORT_EVENT,
                                     XGC2_UGV_SCHEMA_SAMPLED_REFERENCE, 4U),
    port<xgc2_ugv_waypoint_request>("waypoint_request", XGC2_PORT_IN, XGC2_PORT_EVENT,
                                    XGC2_UGV_SCHEMA_WAYPOINT_REQUEST, 4U),
    port<xgc2_ugv_reference_reset>("reset", XGC2_PORT_IN, XGC2_PORT_EVENT,
                                   XGC2_UGV_SCHEMA_REFERENCE_RESET, 2U),
    port<xgc2_ugv_reference_status>("status", XGC2_PORT_OUT, XGC2_PORT_STATE,
                                    XGC2_UGV_SCHEMA_REFERENCE_STATUS),
    port<xgc2_ugv_analytic_reference>("active_analytic", XGC2_PORT_OUT, XGC2_PORT_STATE,
                                      XGC2_UGV_SCHEMA_ANALYTIC_REFERENCE),
    port<xgc2_ugv_polynomial_reference>("active_polynomial", XGC2_PORT_OUT, XGC2_PORT_STATE,
                                        XGC2_UGV_SCHEMA_POLYNOMIAL_REFERENCE),
    port<xgc2_ugv_sampled_reference>("active_sampled", XGC2_PORT_OUT, XGC2_PORT_STATE,
                                     XGC2_UGV_SCHEMA_SAMPLED_REFERENCE),
};

// One request taken from an input, in the order of the commit stamps.
struct Pending {
    int64_t stamp_ns;
    uint32_t port;
    uint32_t order;  // the order it was read in, to keep equal stamps in port order
    const void* payload;
};

class Instance {
   public:
    explicit Instance(const Host& host) : host_(host) {}

    const Host& host() const {
        return host_;
    }

    void configure(const xgc2_config* config) {
        JsonSource source(config);
        urt::ReferenceTrajectoryConfig runtime;
        urt::DefaultAnalyticReferenceConfig default_analytic;
        urt::loadReferenceConfig(source, runtime, default_analytic);
        source.rejectUnused();
        driver_.configure(runtime, default_analytic);
    }

    void start() {}
    void stop() {}

    void step(const xgc2_step_ctx& ctx) {
        const double now = secondsOf<urt::Time>(ctx.now_ns);
        collect();
        for (const Pending& request : pending_) {
            serve(request, now);
        }
        const auto update = driver_.update(now);
        if (update.default_analytic.has_value()) {
            warn(*update.default_analytic, kDefaultAnalytic, "default analytic reference", now);
        }
        // The outputs are committed last: a consumer runs right after this step.
        for (const auto& event : update.events) {
            publish(event, now, ctx.now_ns);
        }
    }

   private:
    // Reads every request waiting on the four event inputs.
    void collect() {
        pending_.clear();
        uint32_t order = 0;
        for (uint32_t index = 0; index < kInputCount; ++index) {
            for (;;) {
                const void* payload = nullptr;
                int64_t stamp = 0;
                switch (index) {
                    case kAnalyticRequest: {
                        const auto sample = host_.next<xgc2_ugv_analytic_reference>(index);
                        payload = sample.data;
                        stamp = sample.stamp_ns;
                        break;
                    }
                    case kSampledRequest: {
                        const auto sample = host_.next<xgc2_ugv_sampled_reference>(index);
                        payload = sample.data;
                        stamp = sample.stamp_ns;
                        break;
                    }
                    case kWaypointRequest: {
                        const auto sample = host_.next<xgc2_ugv_waypoint_request>(index);
                        payload = sample.data;
                        stamp = sample.stamp_ns;
                        break;
                    }
                    default: {
                        const auto sample = host_.next<xgc2_ugv_reference_reset>(index);
                        payload = sample.data;
                        stamp = sample.stamp_ns;
                        break;
                    }
                }
                if (payload == nullptr) {
                    break;
                }
                pending_.push_back({stamp, index, order++, payload});
            }
        }
        std::sort(pending_.begin(), pending_.end(), [](const Pending& a, const Pending& b) {
            return a.stamp_ns != b.stamp_ns ? a.stamp_ns < b.stamp_ns : a.order < b.order;
        });
    }

    // Warnings repeat at the rate of the inputs: once a second for each kind of request.
    void warn(Request result, uint32_t kind, const char* what, double now) {
        if (result == Request::kPosted || !throttle_[kind].due(now)) {
            return;
        }
        if (result == Request::kRejected) {
            host_.log(kLogWarn, std::string("Rejected ") + what);
        } else {
            host_.log(kLogWarn, std::string("Failed to post the event of ") + what + ": " +
                                    driver_.postError());
        }
    }

    void refuse(uint32_t kind, const char* what, double now) {
        if (throttle_[kind].due(now)) {
            host_.log(kLogWarn, std::string("Refused ") + what + ": a count exceeds the payload");
        }
    }

    void serve(const Pending& request, double now) {
        const uint32_t kind = request.port;
        switch (kind) {
            case kAnalyticRequest: {
                urt::reference::AnalyticReference plain;
                if (rp::toPlain(*static_cast<const xgc2_ugv_analytic_reference*>(request.payload),
                                plain)) {
                    warn(driver_.acceptAnalytic(plain, now), kind, "analytic reference", now);
                } else {
                    refuse(kind, "an analytic reference", now);
                }
                break;
            }
            case kSampledRequest: {
                urt::reference::SampledReference plain;
                if (rp::toPlain(*static_cast<const xgc2_ugv_sampled_reference*>(request.payload),
                                plain)) {
                    warn(driver_.acceptSampled(plain, now), kind, "sampled reference", now);
                } else {
                    refuse(kind, "a sampled reference", now);
                }
                break;
            }
            case kWaypointRequest: {
                urt::reference::WaypointReferenceRequest plain;
                if (rp::toPlain(*static_cast<const xgc2_ugv_waypoint_request*>(request.payload),
                                plain)) {
                    warn(driver_.acceptWaypoint(plain, now), kind, "waypoint request", now);
                } else {
                    refuse(kind, "a waypoint request", now);
                }
                break;
            }
            default: {
                const auto status = driver_.reset(now);
                if (!status.ok() && throttle_[kind].due(now)) {
                    host_.log(kLogWarn, "Failed to post the event of reset: " + status.message);
                }
                break;
            }
        }
    }

    // What the node's output consumer published for an output event.
    void publish(const ::state_machine::Event& event, double now, int64_t stamp_ns) {
        const urt::ReferenceTrajectoryRuntime& runtime = driver_.runtime();
        if (event.id == urt::output_event_type::PUBLISH_STATUS) {
            auto slot = host_.write<xgc2_ugv_reference_status>(kStatus);
            if (slot) {
                rp::toPayload(*slot,
                              runtime.makeStatus(event.timestamp > 0.0 ? event.timestamp : now));
                slot.commit(stamp_ns);
            }
        } else if (event.id == urt::output_event_type::PUBLISH_ACTIVE_ANALYTIC) {
            publishActive<xgc2_ugv_analytic_reference>(
                kActiveAnalytic, runtime.activeAnalyticMessage(), now, stamp_ns);
        } else if (event.id == urt::output_event_type::PUBLISH_ACTIVE_POLYNOMIAL) {
            publishActive<xgc2_ugv_polynomial_reference>(
                kActivePolynomial, runtime.activePolynomialMessage(), now, stamp_ns);
        } else if (event.id == urt::output_event_type::PUBLISH_ACTIVE_SAMPLED) {
            publishActive<xgc2_ugv_sampled_reference>(
                kActiveSampled, runtime.activeSampledMessage(), now, stamp_ns);
        }
    }

    template <typename Payload, typename Plain>
    void publishActive(uint32_t port_index, const Plain& plain, double now, int64_t stamp_ns) {
        if (!rp::fits(plain)) {
            if (throttle_[port_index].due(now)) {
                host_.log(kLogError, "The active reference does not fit its payload");
            }
            return;
        }
        auto slot = host_.write<Payload>(port_index);
        if (slot) {
            rp::toPayload(*slot, plain);
            slot.commit(stamp_ns);
        }
    }

    Host host_;
    urt::ReferenceTrajectoryDriver driver_;
    LogThrottle throttle_[kPortCount + 1];
    std::vector<Pending> pending_;
};

const xgc2_module_desc kDescriptor = {XGC2_MODULE_ABI_MAJOR,
                                      XGC2_MODULE_ABI_MINOR,
                                      "ugv_unicycle_reference",
                                      UGV_MODULES_VERSION,
                                      kPorts,
                                      kPortCount,
                                      Lifecycle<Instance>::create,
                                      Lifecycle<Instance>::configure,
                                      Lifecycle<Instance>::start,
                                      Lifecycle<Instance>::step,
                                      Lifecycle<Instance>::stop,
                                      Lifecycle<Instance>::destroy};

}  // namespace

UGV_MODULE_EXPORT const xgc2_module_desc* xgc2_module_entry(void) {
    return &kDescriptor;
}
