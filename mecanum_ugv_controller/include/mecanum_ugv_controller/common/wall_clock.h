#pragma once

#include <chrono>

namespace mecanum_ugv_controller {

// Wall clock seconds (steady), for the leases that must expire also while the
// controller's own clock is paused.
inline double monotonicSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace mecanum_ugv_controller
