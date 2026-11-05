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
                          ? std::optional<ControlCommand>{outcome.command}
                          : std::nullopt};
}

NmpcExecution::NmpcExecution(UnicycleUgvController& controller, CompletionSink completion_sink,
                             Clock clock, Notify notify, PredictionSink prediction)
    : controller_(controller),
      completion_sink_(std::move(completion_sink)),
      clock_(std::move(clock)),
      notify_(std::move(notify)),
      prediction_(std::move(prediction)),
      worker_([this, entered = false](
                  const NmpcRequest& request) mutable { return compute(request, entered); },
              [this](const NmpcCompletion& completion) {
                  if (completion_sink_) {
                      completion_sink_(completion);
                  }
                  return true;
              },
              [this](const NmpcCompletion&) {
                  if (notify_) {
                      notify_();
                  }
              }) {
    backend_.configure(controller_.config());
}

NmpcExecution::~NmpcExecution() {
    stop();
}

void NmpcExecution::start() {
    worker_.start();
}

void NmpcExecution::stop() {
    if (worker_.stop()) {
        backend_.exit();
    }
}

void NmpcExecution::handle(const SolveNmpc& event) {
    NmpcRequest request;
    if (!makeNmpcRequest(controller_, event, clock_(), request)) {
        postResult(event.sequence, NmpcOutcome{});
        if (notify_) {
            notify_();
        }
        return;
    }
    if (!worker_.trySubmit(std::move(request))) {
        UGV_LOG_WARN_THROTTLE(1.0, "[UgvNmpcExecution] Reject solve seq=%lu: backend busy",
                              static_cast<unsigned long>(event.sequence));
        postResult(event.sequence, NmpcOutcome{});
        if (notify_) {
            notify_();
        }
        return;
    }
}

NmpcCompletion NmpcExecution::compute(const NmpcRequest& request, bool& entered) {
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
    return makeNmpcCompletion(request.sequence, outcome);
}

void NmpcExecution::postResult(uint64_t sequence, const NmpcOutcome& outcome) {
    if (!completion_sink_) {
        return;
    }
    completion_sink_(makeNmpcCompletion(sequence, outcome));
}

}  // namespace unicycle_ugv_controller
