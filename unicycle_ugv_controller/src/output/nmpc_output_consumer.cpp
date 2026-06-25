#include "unicycle_ugv_controller/output/nmpc_output_consumer.h"

#include <utility>

namespace unicycle_ugv_controller {

NmpcOutputConsumer::NmpcOutputConsumer(UnicycleUgvController& controller, EventSink event_sink)
    : controller_(controller), event_sink_(std::move(event_sink)) {
    backend_.configure(controller_.config());
    worker_ = std::thread(&NmpcOutputConsumer::workerLoop, this);
}

NmpcOutputConsumer::~NmpcOutputConsumer() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    condition_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    backend_.exit();
}

bool NmpcOutputConsumer::handle(const ::state_machine::Event& event) {
    if (event.id != output_event_type::REQUEST_NMPC_SOLVE) {
        return false;
    }

    const ControllerConfig config = controller_.config();
    const ros::Time now(event.timestamp > 0.0 ? event.timestamp : ros::Time::now().toSec());
    Request request;
    request.sequence = event.correlation_id;
    request.now = now;
    request.state = controller_.state();
    request.config = config;
    const double stage_dt =
        config.prediction_horizon / static_cast<double>(UnicycleNmpcSolver::horizonSteps());
    if (!controller_.referenceCache().sampleHorizon(now, stage_dt,
                                                    UnicycleNmpcSolver::horizonSteps(),
                                                    config.reference_timeout, request.references)) {
        reject(request.sequence, -10);
        return true;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (busy_ || has_pending_) {
            reject(request.sequence, -11);
            return true;
        }
        pending_ = std::move(request);
        has_pending_ = true;
    }
    condition_.notify_one();
    return true;
}

void NmpcOutputConsumer::workerLoop() {
    bool entered = false;
    while (true) {
        Request request;
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

        backend_.configure(request.config);
        if (!entered) {
            entered = backend_.enter();
        }
        ControlCommand command;
        const bool ok =
            entered && backend_.compute(request.state, request.references, request.now, command);
        if (ok) {
            controller_.setCommand(command);
        }
        postResultEvent(request.sequence, ok);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            busy_ = false;
        }
    }
}

void NmpcOutputConsumer::reject(uint64_t sequence, int solver_status) {
    (void)solver_status;
    postResultEvent(sequence, false);
}

void NmpcOutputConsumer::postResultEvent(uint64_t sequence, bool success) {
    if (!event_sink_) {
        return;
    }
    ::state_machine::Event event(
        success ? event_type::INPUT_NMPC_SOLVE_SUCCEEDED : event_type::INPUT_NMPC_SOLVE_FAILED,
        ::state_machine::EventTimestamp{ros::Time::now().toSec()});
    event.source = "nmpc_output_consumer";
    event.category = ::state_machine::EventCategory::kInput;
    event.correlation_id = sequence;
    (void)event_sink_(std::move(event));
}

}  // namespace unicycle_ugv_controller
