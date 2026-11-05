#pragma once

#include <cmath>
#include <limits>

namespace unicycle_ugv_controller {

class PeriodicGate {
   public:
    bool due(double now, double period) {
        if (!std::isfinite(now) || !std::isfinite(period) || period <= 0.0) {
            return false;
        }
        if (!initialized_ || now < last_time_) {
            initialized_ = true;
            last_time_ = now;
            return true;
        }
        if (now - last_time_ < period) {
            return false;
        }
        if (now - last_time_ > 10.0 * period) {
            last_time_ = now;
        } else {
            while (now - last_time_ >= period) {
                last_time_ += period;
            }
        }
        return true;
    }

    double next(double now, double period) const {
        if (!std::isfinite(period) || period <= 0) return std::numeric_limits<double>::infinity();
        if (!initialized_ || now < last_time_) return now;
        return std::nextafter(last_time_ + period, std::numeric_limits<double>::infinity());
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
