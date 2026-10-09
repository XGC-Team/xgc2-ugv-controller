#pragma once
#include <ugv_reset_safety/reset_session.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <vector>

namespace ugv_fleet_host {
// One native owner, one current bounded operation. Only the owner turn calls
// these methods; HTTP hands fixed commands to that owner, never ROS or motion.
class ResetControl {
   public:
    static constexpr std::size_t capacity = 128;
    enum class State : uint8_t {
        Idle,
        Accepted,
        Running,
        Cancelling,
        Arrived,
        Rejected,
        Expired,
        Cancelled
    };
    struct RobotResult {
        State state{State::Idle};
        uint32_t generation{0};
    };
    struct SceneFence {
        std::array<char, 128> epoch{};
        uint64_t revision{0}, simulation_time_epoch{0};
        friend bool operator==(const SceneFence&, const SceneFence&) = default;
    };
    struct Snapshot {
        uint64_t revision{0}, operation{0};
        State state{State::Idle};
        std::size_t count{0};
        std::array<RobotResult, capacity> robots{};
        SceneFence scene;
        bool scene_operational{false};
    };
    void scene(SceneFence fence, bool operational, uint64_t world_ns, double steady) {
        if (active(snapshot_.state) && (fence != scene_ || !operational)) {
            rejecting_ = true;
            if (!cancelling_)
                cancel(snapshot_.revision, snapshot_.operation, world_ns, steady);
        }
        scene_ = fence;
        scene_operational_ = operational;
        snapshot_.scene = fence;
        snapshot_.scene_operational = operational;
    }
    using Submit = std::function<bool(std::size_t)>;
    ResetControl(std::size_t robots, Submit start, Submit stop)
        : start_(std::move(start)), stop_(std::move(stop)) {
        if (!robots || robots > capacity || !start_ || !stop_)
            throw std::invalid_argument("bounded roster and native hooks required");
        snapshot_.count = robots;
    }
    bool start(uint64_t expected, const std::array<bool, capacity>& requested, uint64_t world_ns,
               double steady, const SceneFence& expected_scene) {
        if (!scene_operational_ || expected_scene != scene_)
            return false;
        if (expected != snapshot_.revision || active(snapshot_.state) || !world_ns ||
            !std::isfinite(steady))
            return false;
        bool any = false;
        for (std::size_t i = 0; i < snapshot_.count; ++i)
            any |= requested[i];
        for (std::size_t i = snapshot_.count; i < capacity; ++i)
            if (requested[i])
                return false;
        if (!any || snapshot_.revision == UINT64_MAX)
            return false;
        snapshot_.operation = ++snapshot_.revision;
        snapshot_.state = State::Accepted;
        world_ = world_ns;
        steady_ = steady;
        cancelling_ = false;
        rejecting_ = false;
        expiring_ = false;
        for (std::size_t i = 0; i < snapshot_.count; ++i) {
            snapshot_.robots[i] = {requested[i] ? State::Accepted : State::Idle, 0};
            if (requested[i] && !start_(i))
                snapshot_.robots[i].state = State::Rejected;
        }
        aggregate();
        return true;
    }
    bool cancel(uint64_t expected, uint64_t operation, uint64_t world_ns, double steady) {
        if (expected != snapshot_.revision || operation != snapshot_.operation ||
            !active(snapshot_.state) || !std::isfinite(steady))
            return false;
        cancelling_ = true;
        cancel_world_ = world_ns;
        cancel_steady_ = steady;
        for (std::size_t i = 0; i < snapshot_.count; ++i)
            if (active(snapshot_.robots[i].state)) {
                cancel_publication_floor_[i] = last_publication_[i] + 1;
                stop_(i);
                snapshot_.robots[i].state = State::Cancelling;
            }
        change(State::Cancelling);
        aggregate();
        return true;
    }
    // Actual native terminal event and the sole publisher's zero acknowledgement
    // finish work. A Ready state, accepted command or lost lease cannot do so.
    void sample(std::size_t robot, const ugv_reset_safety::ResetSession& session, bool in_reset,
                bool admission_rejected, bool actual_zero, uint64_t applied_stamp,
                uint64_t world_ns, double steady) {
        auto& result = snapshot_.robots.at(robot);
        last_publication_[robot] = session.appliedSerial();
        if (!active(result.state))
            return;
        const auto before = result;
        using Completion = ugv_reset_safety::ResetSession::Completion;
        if (!cancelling_ && (!std::isfinite(steady) || world_ns < world_ || steady < steady_ ||
                             world_ns - world_ >= 90'000'000'000ULL || steady - steady_ >= 90)) {
            expiring_ = true;
            cancel(snapshot_.revision, snapshot_.operation, world_ns, steady);
        }
        if (!cancelling_ && result.generation && result.generation != session.generation()) {
            rejecting_ = true;
            cancel(snapshot_.revision, snapshot_.operation, world_ns, steady);
        }
        if (!cancelling_ && admission_rejected) {
            rejecting_ = true;
            cancel(snapshot_.revision, snapshot_.operation, world_ns, steady);
        }
        if (cancelling_) {
            if (!session.active() && !in_reset && actual_zero && applied_stamp >= cancel_world_ &&
                steady >= cancel_steady_ &&
                session.appliedSerial() >= cancel_publication_floor_[robot])
                result.state =
                    rejecting_ ? State::Rejected : expiring_ ? State::Expired : State::Cancelled;
        } else if (result.generation == 0 && session.active() && in_reset) {
            result.generation = session.generation();
            result.state = State::Running;
        } else if (result.generation && result.generation == session.generation() && actual_zero &&
                   session.appliedSerial() >= session.completionPublicationFloor()) {
            switch (session.completion()) {
                case Completion::Arrived:
                    result.state = State::Arrived;
                    break;
                case Completion::Rejected:
                    result.state = State::Rejected;
                    break;
                case Completion::Expired:
                    result.state = State::Expired;
                    break;
                case Completion::Cancelled:
                    result.state = State::Cancelled;
                    break;
                case Completion::None:
                    break;
            }
        }
        if (before.state != result.state || before.generation != result.generation)
            ++snapshot_.revision;
        aggregate();
    }
    const Snapshot& snapshot() const {
        return snapshot_;
    }
    static bool active(State state) {
        return state == State::Accepted || state == State::Running || state == State::Cancelling;
    }
    static const char* name(State state) {
        switch (state) {
            case State::Idle:
                return "idle";
            case State::Accepted:
                return "accepted";
            case State::Running:
                return "running";
            case State::Cancelling:
                return "cancelling";
            case State::Arrived:
                return "arrived";
            case State::Rejected:
                return "rejected";
            case State::Expired:
                return "expired";
            case State::Cancelled:
                return "cancelled";
        }
        return "rejected";
    }

   private:
    void change(State state) {
        if (snapshot_.state != state) {
            snapshot_.state = state;
            ++snapshot_.revision;
        }
    }
    void aggregate() {
        bool ongoing = false, running = false, arrived = false, rejected = false, expired = false,
             cancelled = false;
        for (std::size_t i = 0; i < snapshot_.count; ++i) {
            auto s = snapshot_.robots[i].state;
            ongoing |= active(s);
            running |= s == State::Running;
            arrived |= s == State::Arrived;
            rejected |= s == State::Rejected;
            expired |= s == State::Expired;
            cancelled |= s == State::Cancelled;
        }
        if (ongoing)
            change(cancelling_ ? State::Cancelling : running ? State::Running : State::Accepted);
        else
            change(expired ? State::Expired
                           : rejected ? State::Rejected
                                      : cancelled ? State::Cancelled
                                                  : arrived ? State::Arrived : State::Idle);
    }
    Submit start_, stop_;
    Snapshot snapshot_;
    uint64_t world_{0}, cancel_world_{0};
    double steady_{0}, cancel_steady_{0};
    bool cancelling_{false}, rejecting_{false}, expiring_{false}, scene_operational_{false};
    std::array<uint64_t, capacity> last_publication_{}, cancel_publication_floor_{};
    SceneFence scene_;
};
}  // namespace ugv_fleet_host
