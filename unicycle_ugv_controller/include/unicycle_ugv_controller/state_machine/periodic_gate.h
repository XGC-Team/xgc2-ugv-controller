#pragma once

namespace unicycle_ugv_controller {

class PeriodicGate {
   public:
    bool due(double now, double period) {
        if (period <= 0.0) {
            return false;
        }
        if (!initialized_ || now - last_time_ >= period) {
            initialized_ = true;
            last_time_ = now;
            return true;
        }
        return false;
    }
    void reset() {
        initialized_ = false;
        last_time_ = 0.0;
    }

   private:
    bool initialized_{false};
    double last_time_{0.0};
};

}  // namespace unicycle_ugv_controller
