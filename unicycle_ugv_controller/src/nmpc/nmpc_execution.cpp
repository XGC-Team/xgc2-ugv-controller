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

bool makeNmpcRequest(const UnicycleUgvController& controller, const SolveNmpc& event,
                     double fallback_now_sec, NmpcRequest& request) {
    const ControllerConfig config = controller.config();
    const Time now(event.timestamp > 0.0 ? event.timestamp : fallback_now_sec);
    request = NmpcRequest{};
    request.sequence = event.sequence;
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

NmpcCompletion makeNmpcCompletion(uint64_t sequence, const NmpcOutcome& outcome) {
    return {sequence, outcome.success && outcome.command.valid
                          ? std::optional<ControlCommand>{outcome.command} : std::nullopt};
}

NmpcExecution::NmpcExecution(UnicycleUgvController& controller, CompletionSink completion_sink, Clock clock,
                             Notify notify, PredictionSink prediction)
    : controller_(controller),
      completion_sink_(std::move(completion_sink)),
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
    pending_.reset();
    worker_ = std::thread(&NmpcExecution::workerLoop, this);
}

void NmpcExecution::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
        pending_.reset();
    }
    condition_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
        backend_.exit();
    }
}

void NmpcExecution::handle(const SolveNmpc& event) {
    NmpcRequest request;
    if (!makeNmpcRequest(controller_, event, clock_(), request)) {
        postResult(event.sequence, NmpcOutcome{});
        if (notify_) notify_();
        return;
    }
    bool rejected = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_ || busy_ || pending_.has_value()) {
            rejected = true;
        } else {
            pending_ = std::move(request);

        }
    }
    if (rejected) {
        UGV_LOG_WARN_THROTTLE(1.0, "[UgvNmpcExecution] Reject solve seq=%lu: backend busy",
                              static_cast<unsigned long>(event.sequence));
        postResult(event.sequence, NmpcOutcome{});
        if (notify_) notify_();
        return;
    }
    condition_.notify_one();
    return;
}

void NmpcExecution::workerLoop() {
    bool entered = false;
    while (true) {
        NmpcRequest request;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this] { return stop_ || pending_.has_value(); });
            if (stop_) {
                return;
            }
            request = std::move(*pending_);
            pending_.reset();
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
    if (!completion_sink_) {
        return;
    }
    completion_sink_(makeNmpcCompletion(sequence, outcome));
}

}  // namespace unicycle_ugv_controller
