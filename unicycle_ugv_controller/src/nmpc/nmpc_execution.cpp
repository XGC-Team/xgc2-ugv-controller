#include "unicycle_ugv_controller/nmpc/nmpc_execution.h"

#include <cmath>
#include <utility>

#include "unicycle_ugv_controller/common/core_log.h"

namespace unicycle_ugv_controller {
namespace {

void unwrapReferenceYaw(std::vector<Se2Reference>& refs, double anchor_yaw) {
    if (!std::isfinite(anchor_yaw)) {
        return;
    }
    double previous_yaw = anchor_yaw;
    for (auto& ref : refs) {
        if (!std::isfinite(ref.state.yaw)) {
            continue;
        }
        ref.state.yaw = previous_yaw + wrapAngle(ref.state.yaw - previous_yaw);
        previous_yaw = ref.state.yaw;
    }
}

}  // namespace

bool makeNmpcRequest(const UnicycleUgvController& controller, const ::state_machine::Event& event,
                     double fallback_now_sec, NmpcRequest& request) {
    const ControllerConfig config = controller.config();
    const Time now(event.timestamp > 0.0 ? event.timestamp : fallback_now_sec);
    request = NmpcRequest{};
    request.sequence = event.correlation_id;
    request.now = now;
    request.state = controller.state();
    request.config = config;
    const double stage_dt =
        config.prediction_horizon / static_cast<double>(UnicycleNmpcSolver::horizonSteps());
    if (!controller.referenceCache().sampleHorizon(
            now, stage_dt, UnicycleNmpcSolver::horizonSteps(), request.references)) {
        UGV_LOG_WARN_THROTTLE(1.0,
                              "[UgvNmpcExecution] Reject solve seq=%lu: reference horizon "
                              "unavailable now=%.3f stage_dt=%.3f",
                              static_cast<unsigned long>(request.sequence), now.toSec(), stage_dt);
        return false;
    }
    unwrapReferenceYaw(request.references, request.state.yaw);
    return true;
}

NmpcOutcome solveNmpcRequest(NmpcTrackingBackend& backend, bool& entered,
                             const NmpcRequest& request) {
    NmpcOutcome outcome;
    backend.configure(request.config);
    if (!entered) {
        entered = backend.enter();
        if (!entered) {
            UGV_LOG_WARN("[UgvNmpcExecution] Failed to enter NMPC backend");
        }
    }
    outcome.success =
        entered && backend.compute(request.state, request.references, request.now, outcome.command);
    return outcome;
}

::state_machine::Event makeNmpcResultEvent(uint64_t sequence, const NmpcOutcome& outcome,
                                           double timestamp_sec) {
    const bool success = outcome.success && outcome.command.valid;
    ::state_machine::Event event(
        success ? event_type::INPUT_NMPC_SOLVE_SUCCEEDED : event_type::INPUT_NMPC_SOLVE_FAILED,
        ::state_machine::EventTimestamp{timestamp_sec});
    event.source = "nmpc_output_consumer";
    event.category = ::state_machine::EventCategory::kInput;
    event.correlation_id = sequence;
    if (success) {
        event.payload["command_stamp"] = outcome.command.stamp.toSec();
        event.payload["linear_speed"] = outcome.command.linear_speed;
        event.payload["angular_speed"] = outcome.command.angular_speed;
    }
    return event;
}

NmpcExecution::NmpcExecution(UnicycleUgvController& controller, EventSink event_sink, Clock clock,
                             Notify notify, PredictionSink prediction)
    : controller_(controller),
      event_sink_(std::move(event_sink)),
      clock_(std::move(clock)),
      notify_(std::move(notify)),
      prediction_(std::move(prediction)) {
    backend_.configure(controller_.config());
}

NmpcExecution::~NmpcExecution() {
    stop();
}

void NmpcExecution::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (worker_.joinable()) {
        return;
    }
    stop_ = false;
    busy_ = false;
    has_pending_ = false;
    worker_ = std::thread(&NmpcExecution::workerLoop, this);
}

void NmpcExecution::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
        has_pending_ = false;
    }
    condition_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
        backend_.exit();
    }
}

bool NmpcExecution::handle(const ::state_machine::Event& event) {
    if (event.id != output_event_type::REQUEST_NMPC_SOLVE) {
        return false;
    }
    NmpcRequest request;
    if (!makeNmpcRequest(controller_, event, clock_(), request)) {
        postResult(event.correlation_id, NmpcOutcome{});
        return true;
    }
    bool rejected = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_ || busy_ || has_pending_) {
            rejected = true;
        } else {
            pending_ = std::move(request);
            has_pending_ = true;
        }
    }
    if (rejected) {
        UGV_LOG_WARN_THROTTLE(1.0, "[UgvNmpcExecution] Reject solve seq=%lu: backend busy",
                              static_cast<unsigned long>(event.correlation_id));
        postResult(event.correlation_id, NmpcOutcome{});
        return true;
    }
    condition_.notify_one();
    return true;
}

void NmpcExecution::workerLoop() {
    bool entered = false;
    while (true) {
        NmpcRequest request;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this] { return stop_ || has_pending_; });
            if (stop_) {
                return;
            }
            request = std::move(pending_);
            has_pending_ = false;
            busy_ = true;
        }

        const NmpcOutcome outcome = solveNmpcRequest(backend_, entered, request);
        if (outcome.success) {
            if (prediction_) {
                prediction_(request.now, backend_);
            }
            UGV_LOG_INFO_THROTTLE(1.0,
                                  "[UgvNmpcExecution] Solve ok seq=%lu linear=%.3f angular=%.3f "
                                  "solve=%.3f ms",
                                  static_cast<unsigned long>(request.sequence),
                                  outcome.command.linear_speed, outcome.command.angular_speed,
                                  backend_.solveTimeMs());
        } else {
            UGV_LOG_WARN_THROTTLE(1.0,
                                  "[UgvNmpcExecution] Solve failed seq=%lu status=%d solve=%.3f ms "
                                  "refs=%zu",
                                  static_cast<unsigned long>(request.sequence), backend_.status(),
                                  backend_.solveTimeMs(), request.references.size());
        }
        postResult(request.sequence, outcome);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            busy_ = false;
        }
        if (notify_) {
            notify_();
        }
    }
}

void NmpcExecution::postResult(uint64_t sequence, const NmpcOutcome& outcome) {
    if (!event_sink_) {
        return;
    }
    (void)event_sink_(makeNmpcResultEvent(sequence, outcome, clock_()));
}

}  // namespace unicycle_ugv_controller
