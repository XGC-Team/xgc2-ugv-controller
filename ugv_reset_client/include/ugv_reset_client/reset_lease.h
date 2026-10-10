#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>

namespace ugv_reset_client {

inline double monotonicSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// The vehicle side of the Reset protocol with the station's coordinator, for
// one vehicle: a request/response lease. The vehicle's controller owns the
// Reset session (its generation and frozen target); the lease follows it. It
// issues a request per fresh pose, with the command the vehicle really
// applied, and accepts only responses that echo an issued request of the
// current generation, once, and while both the ROS clock and the wall clock
// say the request is fresh. The controller still checks the freshness of the
// clearance it is given at every control step.
//
// There is no other vehicle here and no ROS: the caller supplies the stamps
// (ROS time in nanoseconds) and the wall clock seconds, so the contract is
// testable. The lease is thread safe, because the response arrives on a
// subscriber thread while requests leave from the control loop.
class ResetLease {
   public:
    struct Pose {
        double x{0}, y{0}, yaw{0};
    };
    struct Command {
        double x{0}, y{0}, yaw{0};
    };
    enum Status : uint8_t { RUNNING = 0, ARRIVED = 1, REJECTED = 2 };
    struct Request {
        bool valid{false};
        uint32_t generation{0};
        uint64_t stamp{0};
        Pose pose, target;
        Command applied_command;
        uint64_t applied_stamp{0};
    };
    // An accepted response, as the controller executes it. The controller keeps
    // executing it only while `lease_seconds` have not passed on the ROS clock
    // since `stamp` and on the wall clock since `issue_wall`.
    struct Clearance {
        uint32_t generation{0};
        uint64_t stamp{0};     // stamp of the request it answers
        double issue_wall{0};  // wall clock seconds when that request was issued
        Status status{RUNNING};
        Command command;
        double lease_seconds{0};
    };

    // Starts the session `generation` towards `target`. A session with a non-finite
    // target is not started.
    void begin(uint32_t generation, Pose target) {
        std::lock_guard<std::mutex> lock(mutex_);
        cancelLocked();
        if (!finite(target)) {
            return;
        }
        generation_ = generation;
        target_ = target;
        active_ = true;
    }
    void cancel() {
        std::lock_guard<std::mutex> lock(mutex_);
        cancelLocked();
    }
    bool active() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return active_;
    }
    uint32_t generation() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return generation_;
    }

    // Called by the sole native publisher, after publish(), including zero.
    // Receiving a safe proposal must never advance the executed slew state.
    void noteApplied(Command command, uint64_t stamp) {
        if (!finite(command) || stamp == 0) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        applied_command_ = command;
        applied_stamp_ = stamp;
    }

    Request issue(Pose pose, uint64_t stamp, double wall) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_ || !finite(pose) || stamp == 0 || !std::isfinite(wall)) {
            return {};
        }
        // A rewound or paused ROS clock must not renew a moving command lease.
        if (stamp <= last_issued_) {
            return {};
        }
        if (!issued_.empty() && wall - issued_.back().wall < 0.02) {
            return {};
        }
        while (!issued_.empty() && wall - issued_.front().wall > kLeaseSeconds) {
            issued_.pop_front();
        }
        last_issued_ = stamp;
        issued_.push_back({stamp, wall});
        return {true, generation_, stamp, pose, target_, applied_command_, applied_stamp_};
    }

    // Validates a response. On success `clearance` is what the controller executes.
    bool accept(uint32_t generation, uint64_t stamp, uint8_t status, Command command, uint64_t now,
                double wall, Clearance& clearance) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_ || generation != generation_ || stamp <= last_accepted_ || status > REJECTED ||
            !finite(command) || !freshRos(stamp, now)) {
            return false;
        }
        for (const auto& issued : issued_) {
            if (issued.stamp != stamp) {
                continue;
            }
            if (!freshWall(issued.wall, wall)) {
                return false;
            }
            if (status != RUNNING && (command.x != 0 || command.y != 0 || command.yaw != 0)) {
                return false;
            }
            last_accepted_ = stamp;
            // The issue time, never the delayed receive time, starts the wall lease.
            clearance = {generation, stamp,        issued.wall, static_cast<Status>(status),
                         command,    kLeaseSeconds};
            return true;
        }
        return false;
    }

    static constexpr double kLeaseSeconds = 0.15;

   private:
    struct Issued {
        uint64_t stamp;
        double wall;
    };
    static bool freshRos(uint64_t stamp, uint64_t now) {
        return stamp > 0 && now >= stamp && now - stamp <= 150000000ULL;
    }
    static bool freshWall(double sent, double now) {
        return std::isfinite(now) && now >= sent && now - sent <= kLeaseSeconds;
    }
    static bool finite(Pose value) {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.yaw);
    }
    static bool finite(Command value) {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.yaw);
    }
    void cancelLocked() {
        active_ = false;
        issued_.clear();
        last_issued_ = 0;
        last_accepted_ = 0;
    }

    mutable std::mutex mutex_;
    bool active_{false};
    uint32_t generation_{0};
    uint64_t last_issued_{0}, last_accepted_{0};
    Pose target_;
    Command applied_command_;
    uint64_t applied_stamp_{0};
    std::deque<Issued> issued_;
};

}  // namespace ugv_reset_client
