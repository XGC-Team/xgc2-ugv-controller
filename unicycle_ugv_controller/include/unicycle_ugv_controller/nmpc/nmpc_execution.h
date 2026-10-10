#pragma once

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <state_machine/state_machine.hpp>
#include <thread>
#include <vector>

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/nmpc/nmpc_tracking_backend.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

// What one NMPC solve needs, snapshot on the control thread when Custom1 asks
// for it: the vehicle state, the configuration and the sampled reference horizon.
struct NmpcRequest {
    uint64_t sequence{0U};
    Time now;
    UgvState state;
    std::vector<Se2Reference> references;
    ControllerConfig config;
};

struct NmpcOutcome {
    bool success{false};
    ControlCommand command;
};

// Snapshots the controller for a REQUEST_NMPC_SOLVE event. `fallback_now_sec` stands in
// for an event without a time. False when the reference horizon is unavailable.
bool makeNmpcRequest(const UnicycleUgvController& controller, const ::state_machine::Event& event,
                     double fallback_now_sec, NmpcRequest& request);

// Solves the request on `backend`, entering it on first use (`entered`).
NmpcOutcome solveNmpcRequest(NmpcTrackingBackend& backend, bool& entered,
                             const NmpcRequest& request);

// The input event with which the Custom1 state takes the result of request `sequence`.
::state_machine::Event makeNmpcResultEvent(uint64_t sequence, const NmpcOutcome& outcome,
                                           double timestamp_sec);

// The NMPC solve off the control thread: a snapshot on the caller's thread, one
// worker, one pending request, a busy worker rejects (the controller then keeps
// its previous command until it expires). The result goes back as an input
// event through `event_sink`; `notify` runs after it on the worker, so the
// owner can wake its control loop. The ROS node and the aggregator module both
// run this class.
class NmpcExecution {
   public:
    using EventSink = std::function<::state_machine::Status(::state_machine::Event)>;
    using Clock = std::function<double()>;
    using Notify = std::function<void()>;
    // Runs on the worker after a successful solve, before the result is posted.
    using PredictionSink = std::function<void(const Time& stamp, const NmpcTrackingBackend&)>;

    NmpcExecution(UnicycleUgvController& controller, EventSink event_sink, Clock clock,
                  Notify notify = {}, PredictionSink prediction = {});
    ~NmpcExecution();
    NmpcExecution(const NmpcExecution&) = delete;
    NmpcExecution& operator=(const NmpcExecution&) = delete;

    // Starts and joins the worker. The worker uses nothing of the owner after stop() returns.
    void start();
    void stop();

    // Takes a REQUEST_NMPC_SOLVE event; false for any other event.
    bool handle(const ::state_machine::Event& event);

   private:
    void workerLoop();
    void postResult(uint64_t sequence, const NmpcOutcome& outcome);

    UnicycleUgvController& controller_;
    EventSink event_sink_;
    Clock clock_;
    Notify notify_;
    PredictionSink prediction_;
    NmpcTrackingBackend backend_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::thread worker_;
    bool stop_{false};
    bool busy_{false};
    bool has_pending_{false};
    NmpcRequest pending_;
};

}  // namespace unicycle_ugv_controller
