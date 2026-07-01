#pragma once

#include <cmath>

namespace unicycle_reference_trajectory {

class PeriodicGate {
   public:
    bool due(double now_sec, double period_sec) {
        if (!std::isfinite(now_sec) || !std::isfinite(period_sec) || period_sec <= 0.0) {
            return false;
        }
        if (!initialized_ || now_sec < last_sec_) {
            initialized_ = true;
            last_sec_ = now_sec;
            return true;
        }
        if (now_sec - last_sec_ < period_sec) {
            return false;
        }
        if (now_sec - last_sec_ > 10.0 * period_sec) {
            last_sec_ = now_sec;
        } else {
            while (now_sec - last_sec_ >= period_sec) {
                last_sec_ += period_sec;
            }
        }
        return true;
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
