#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <string>
#include <thread>

#include "unicycle_ugv_controller/nmpc/nmpc_tracking_backend.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

class NmpcOutputConsumer final : public ::state_machine::runtime::EventConsumer {
   public:
    using EventSink = std::function<::state_machine::Status(::state_machine::Event)>;

    NmpcOutputConsumer(UnicycleUgvController& controller, EventSink event_sink);
    ~NmpcOutputConsumer() override;

    std::string name() const override {
        return "NmpcOutputConsumer";
    }
    bool handle(const ::state_machine::Event& event) override;

   private:
    struct Request {
        uint64_t sequence{0U};
        ros::Time now;
        UgvState state;
        std::vector<Se2Reference> references;
        ControllerConfig config;
    };

    void workerLoop();
    void reject(uint64_t sequence, int solver_status);
    void postResultEvent(uint64_t sequence, bool success);

    UnicycleUgvController& controller_;
    EventSink event_sink_;
    NmpcTrackingBackend backend_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::thread worker_;
    bool stop_{false};
    bool busy_{false};
    bool has_pending_{false};
    Request pending_;
};

}  // namespace unicycle_ugv_controller
