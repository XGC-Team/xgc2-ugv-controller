#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace ugv_reset_client {
// Absolute deadlines in the two existing clocks. Infinity means no obligation.
struct NextWake {
    double domain{std::numeric_limits<double>::infinity()};
    double wall{std::numeric_limits<double>::infinity()};
    void atDomain(double at) {
        domain = std::min(domain, at);
    }
    void atWall(double at) {
        wall = std::min(wall, at);
    }
    void include(NextWake other) {
        atDomain(other.domain);
        atWall(other.wall);
    }
};
// The standalone controllers already sampled on WallRate. This clock binding keeps that
// phase without sampling again for each input callback. It owns no business predicate.
struct WallControlCadence {
    int64_t phase_ns;
    double rate;
    bool external_domain;
    double next(double wall, bool include_now = false) const {
        const int64_t period = static_cast<int64_t>(std::llround(1.0e9 / rate));
        const int64_t now = static_cast<int64_t>(std::llround(wall * 1.0e9));
        const int64_t elapsed = now - phase_ns;
        const int64_t frame = elapsed >= 0 ? elapsed / period : (elapsed - period + 1) / period;
        const int64_t floor = phase_ns + frame * period;
        return static_cast<double>(include_now && now == floor ? floor : floor + period) / 1.0e9;
    }
    bool take(std::optional<double>& pending, double wall) const {
        if (!pending)
            pending = next(wall, true);
        if (std::llround(wall * 1.0e9) < std::llround(*pending * 1.0e9))
            return false;
        pending.reset();
        return true;
    }
};
}  // namespace ugv_reset_client
