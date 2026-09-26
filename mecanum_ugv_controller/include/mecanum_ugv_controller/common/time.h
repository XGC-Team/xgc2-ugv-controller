#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace mecanum_ugv_controller {

// Time and Duration for the controller core, without ROS.
//
// They reproduce rostime's representation and arithmetic exactly (unsigned
// or signed 32-bit seconds plus nanoseconds, the same rounding in fromSec,
// the same normalization and range errors), so replacing ros::Time with
// them changes no result bit. They are the multirotor controller core's
// Time and Duration (px4_multirotor_controller/common/time.h), whose
// time_equivalence test checks them against ros::Time; keep the copies in
// step.
class Duration {
   public:
    int32_t sec{0};
    int32_t nsec{0};

    Duration() = default;
    Duration(int32_t s, int32_t ns) : sec(s), nsec(ns) {
        normalize(sec, nsec);
    }
    explicit Duration(double d) {
        fromSec(d);
    }

    Duration& fromSec(double d) {
        if (!std::isfinite(d))
            throw std::runtime_error("Duration has to be finite.");
        constexpr double kMin = static_cast<double>(std::numeric_limits<int64_t>::min());
        constexpr double kMax = static_cast<double>(std::numeric_limits<int64_t>::max());
        if (d <= kMin || d >= kMax)
            throw std::runtime_error("Duration is out of 64-bit integer range");
        const int64_t sec64 = static_cast<int64_t>(std::floor(d));
        if (sec64 < std::numeric_limits<int32_t>::min() ||
            sec64 > std::numeric_limits<int32_t>::max()) {
            throw std::runtime_error("Duration is out of dual 32-bit range");
        }
        sec = static_cast<int32_t>(sec64);
        nsec = static_cast<int32_t>(std::round((d - sec) * 1e9));
        const int32_t rollover = nsec / 1000000000ul;
        sec += rollover;
        nsec %= 1000000000ul;
        return *this;
    }

    Duration& fromNSec(int64_t t) {
        const int64_t sec64 = t / 1000000000LL;
        if (sec64 < std::numeric_limits<int32_t>::min() ||
            sec64 > std::numeric_limits<int32_t>::max()) {
            throw std::runtime_error("Duration is out of dual 32-bit range");
        }
        sec = static_cast<int32_t>(sec64);
        nsec = static_cast<int32_t>(t % 1000000000LL);
        normalize(sec, nsec);
        return *this;
    }

    double toSec() const {
        return static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec);
    }
    int64_t toNSec() const {
        return static_cast<int64_t>(sec) * 1000000000ll + static_cast<int64_t>(nsec);
    }
    bool isZero() const {
        return sec == 0 && nsec == 0;
    }

    Duration operator+(const Duration& rhs) const {
        return Duration().fromNSec(toNSec() + rhs.toNSec());
    }
    Duration operator-(const Duration& rhs) const {
        return Duration().fromNSec(toNSec() - rhs.toNSec());
    }
    Duration operator-() const {
        return Duration().fromNSec(-toNSec());
    }
    Duration operator*(double scale) const {
        return Duration(toSec() * scale);
    }
    bool operator==(const Duration& r) const {
        return sec == r.sec && nsec == r.nsec;
    }
    bool operator!=(const Duration& r) const {
        return !(*this == r);
    }
    bool operator<(const Duration& r) const {
        return sec < r.sec || (sec == r.sec && nsec < r.nsec);
    }
    bool operator>(const Duration& r) const {
        return sec > r.sec || (sec == r.sec && nsec > r.nsec);
    }
    bool operator<=(const Duration& r) const {
        return sec < r.sec || (sec == r.sec && nsec <= r.nsec);
    }
    bool operator>=(const Duration& r) const {
        return sec > r.sec || (sec == r.sec && nsec >= r.nsec);
    }

    // rostime's normalizeSecNSecSigned.
    static void normalize(int32_t& s, int32_t& ns) {
        int64_t sec64 = s;
        int64_t nsec64 = ns;
        int64_t nsec_part = nsec64 % 1000000000L;
        int64_t sec_part = sec64 + nsec64 / 1000000000L;
        if (nsec_part < 0) {
            nsec_part += 1000000000L;
            --sec_part;
        }
        if (sec_part < std::numeric_limits<int32_t>::min() ||
            sec_part > std::numeric_limits<int32_t>::max()) {
            throw std::runtime_error("Duration is out of dual 32-bit range");
        }
        s = static_cast<int32_t>(sec_part);
        ns = static_cast<int32_t>(nsec_part);
    }
};

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

    Duration operator-(const Time& rhs) const {
        return Duration().fromNSec(static_cast<int64_t>(toNSec() - rhs.toNSec()));
    }
    Time operator+(const Duration& rhs) const {
        int64_t sec_sum = static_cast<uint64_t>(sec) + static_cast<uint64_t>(rhs.sec);
        int64_t nsec_sum = static_cast<uint64_t>(nsec) + static_cast<uint64_t>(rhs.nsec);
        normalizeUnsigned(sec_sum, nsec_sum);
        return Time(static_cast<uint32_t>(sec_sum), static_cast<uint32_t>(nsec_sum));
    }
    Time operator-(const Duration& rhs) const {
        return *this + (-rhs);
    }
    Time& operator+=(const Duration& rhs) {
        return *this = *this + rhs;
    }
    Time& operator-=(const Duration& rhs) {
        return *this = *this + (-rhs);
    }
    bool operator==(const Time& r) const {
        return sec == r.sec && nsec == r.nsec;
    }
    bool operator!=(const Time& r) const {
        return !(*this == r);
    }
    bool operator<(const Time& r) const {
        return sec < r.sec || (sec == r.sec && nsec < r.nsec);
    }
    bool operator>(const Time& r) const {
        return sec > r.sec || (sec == r.sec && nsec > r.nsec);
    }
    bool operator<=(const Time& r) const {
        return sec < r.sec || (sec == r.sec && nsec <= r.nsec);
    }
    bool operator>=(const Time& r) const {
        return sec > r.sec || (sec == r.sec && nsec >= r.nsec);
    }

   private:
    // rostime's normalizeSecNSec (unsigned) and normalizeSecNSecUnsigned.
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
    static void normalizeUnsigned(int64_t& s, int64_t& ns) {
        int64_t nsec_part = ns % 1000000000L;
        int64_t sec_part = s + ns / 1000000000L;
        if (nsec_part < 0) {
            nsec_part += 1000000000L;
            --sec_part;
        }
        if (sec_part < 0 || sec_part > std::numeric_limits<uint32_t>::max()) {
            throw std::runtime_error("Time is out of dual 32-bit range");
        }
        s = sec_part;
        ns = nsec_part;
    }
};

}  // namespace mecanum_ugv_controller
