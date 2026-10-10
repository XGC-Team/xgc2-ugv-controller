#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace unicycle_reference_trajectory {

// Time for the reference trajectory runtime, without ROS. It reproduces
// ros::Time's representation (unsigned 32-bit seconds plus nanoseconds), the
// rounding in fromSec and the range errors exactly, so replacing ros::Time
// with it changes no result bit. It is the Time of
// multirotor_reference_trajectory/time.h, reduced to what this runtime uses;
// keep the two in step.
class Time {
   public:
    uint32_t sec{0};
    uint32_t nsec{0};

    Time() = default;
    Time(uint32_t s, uint32_t ns) : sec(s), nsec(ns) {
        normalize(sec, nsec);
    }
    explicit Time(double t) {
        fromSec(t);
    }

    Time& fromSec(double t) {
        if (t < 0)
            throw std::runtime_error("Time cannot be negative.");
        if (!std::isfinite(t))
            throw std::runtime_error("Time has to be finite.");
        constexpr double kMax = static_cast<double>(std::numeric_limits<int64_t>::max());
        if (t >= kMax)
            throw std::runtime_error("Time is out of 64-bit integer range");
        const int64_t sec64 = static_cast<int64_t>(std::floor(t));
        if (sec64 > std::numeric_limits<uint32_t>::max())
            throw std::runtime_error("Time is out of dual 32-bit range");
        sec = static_cast<uint32_t>(sec64);
        nsec = static_cast<uint32_t>(std::round((t - sec) * 1e9));
        sec += (nsec / 1000000000ul);
        nsec %= 1000000000ul;
        return *this;
    }

    Time& fromNSec(uint64_t t) {
        uint64_t sec64 = 0;
        uint64_t nsec64 = t;
        normalize64(sec64, nsec64);
        sec = static_cast<uint32_t>(sec64);
        nsec = static_cast<uint32_t>(nsec64);
        return *this;
    }

    double toSec() const {
        return static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec);
    }
    uint64_t toNSec() const {
        return static_cast<uint64_t>(sec) * 1000000000ull + static_cast<uint64_t>(nsec);
    }
    bool isZero() const {
        return sec == 0 && nsec == 0;
    }

    bool operator==(const Time& r) const {
        return sec == r.sec && nsec == r.nsec;
    }
    bool operator!=(const Time& r) const {
        return !(*this == r);
    }

   private:
    // rostime's normalizeSecNSec (unsigned).
    static void normalize64(uint64_t& s, uint64_t& ns) {
        const uint64_t nsec_part = ns % 1000000000UL;
        const uint64_t sec_part = ns / 1000000000UL;
        if (s + sec_part > std::numeric_limits<uint32_t>::max())
            throw std::runtime_error("Time is out of dual 32-bit range");
        s += sec_part;
        ns = nsec_part;
    }
    static void normalize(uint32_t& s, uint32_t& ns) {
        uint64_t sec64 = s;
        uint64_t nsec64 = ns;
        normalize64(sec64, nsec64);
        s = static_cast<uint32_t>(sec64);
        ns = static_cast<uint32_t>(nsec64);
    }
};

}  // namespace unicycle_reference_trajectory
