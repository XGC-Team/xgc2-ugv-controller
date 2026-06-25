#pragma once

namespace unicycle_reference_trajectory {

class PeriodicGate {
   public:
    bool due(double now_sec, double period_sec) {
        if (period_sec <= 0.0) {
            return false;
        }
        if (!initialized_ || now_sec - last_sec_ >= period_sec) {
            last_sec_ = now_sec;
            initialized_ = true;
            return true;
        }
        return false;
    }
    void reset() {
        initialized_ = false;
        last_sec_ = 0.0;
    }

   private:
    bool initialized_{false};
    double last_sec_{0.0};
};

}  // namespace unicycle_reference_trajectory
