#pragma once

#include <cstdint>
#include <functional>
#include <state_machine/async_worker.hpp>
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

// Snapshots the controller for a typed solve request. `fallback_now_sec` stands in
// for a request without a time. False when the reference horizon is unavailable.
bool makeNmpcRequest(const UnicycleUgvController& controller, const SolveNmpc& event,
                     double fallback_now_sec, NmpcRequest& request);

// Solves the request on `backend`, entering it on first use (`entered`).
NmpcOutcome solveNmpcRequest(NmpcTrackingBackend& backend, bool& entered,
                             const NmpcRequest& request);

// Typed completion consumed by the active Custom1 value, with its original sequence/deadline.
NmpcCompletion makeNmpcCompletion(uint64_t sequence, const NmpcOutcome& outcome);

// The NMPC solve off the control thread: a snapshot on the caller's thread, one
// worker, one pending request, a busy worker rejects (the controller then keeps
// its previous command until it expires). The result goes back as an input
// value through `completion_sink`; `notify` runs after it on the worker, so the
// owner can wake its control loop. The ROS node and the aggregator module both
// run this class.
class NmpcExecution {
   public:
    using CompletionSink = std::function<void(NmpcCompletion)>;
    using Clock = std::function<double()>;
    using Notify = std::function<void()>;
    // Runs on the worker after a successful solve, before the result is published.
    using PredictionSink = std::function<void(const Time& stamp, const NmpcTrackingBackend&)>;

    NmpcExecution(UnicycleUgvController& controller, CompletionSink completion_sink, Clock clock,
                  Notify notify = {}, PredictionSink prediction = {});
    ~NmpcExecution();
    NmpcExecution(const NmpcExecution&) = delete;
    NmpcExecution& operator=(const NmpcExecution&) = delete;

    // Starts and joins the worker. The worker uses nothing of the owner after stop() returns.
    void start();
    void stop();

    // Consumes the solve request on the control owner.
    void handle(const SolveNmpc& request);

   private:
    NmpcCompletion compute(const NmpcRequest& request, bool& entered);
    void postResult(uint64_t sequence, const NmpcOutcome& outcome);

    UnicycleUgvController& controller_;
    CompletionSink completion_sink_;
    Clock clock_;
    Notify notify_;
    PredictionSink prediction_;
    NmpcTrackingBackend backend_;
    state_machine::AsyncWorker<NmpcRequest, NmpcCompletion> worker_;
};

}  // namespace unicycle_ugv_controller
