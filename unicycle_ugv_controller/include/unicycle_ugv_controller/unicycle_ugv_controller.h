#pragma once

#include <memory>
#include <mutex>
#include <state_machine/state_machine.hpp>

#include "unicycle_ugv_controller/common/reference_cache.h"
#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {

class UnicycleUgvController {
   public:
    explicit UnicycleUgvController(const UgvState& state);
    void update(double now_sec);
    ::state_machine::Status postEvent(::state_machine::Event event);

    const UgvState& state() const {
        return state_;
    }
    double currentTime() const {
        return current_time_sec_;
    }
    ControllerConfig config() const;
    void setConfig(const ControllerConfig& config);
    ReferenceCache& referenceCache() {
        return reference_cache_;
    }
    const ReferenceCache& referenceCache() const {
        return reference_cache_;
    }
    ::state_machine::StateMachine& stateMachine() {
        return *machine_;
    }
    bool healthReady() const;
    bool referenceReady() const;
    bool commandReady() const;
    void setCommand(ControlCommand command);
    ControlCommand command() const;
    void clearCommand();

   private:
    void setupMachine();
    void maybeAutoStartTracking();

    const UgvState& state_;
    mutable std::mutex config_mutex_;
    ControllerConfig config_;
    ReferenceCache reference_cache_;
    mutable std::mutex command_mutex_;
    ControlCommand command_;
    std::unique_ptr<::state_machine::StateMachine> machine_;
    double current_time_sec_{0.0};
};

}  // namespace unicycle_ugv_controller
