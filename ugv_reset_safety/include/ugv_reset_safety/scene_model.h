#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace ugv_reset_safety {

// Device/world timestamps retain nanosecond identity; monotonic input time is
// separate and supplied by the one native execution owner.
struct NativeDuration {
    double seconds{0};
    double toSec() const {
        return seconds;
    }
};
struct NativeStamp {
    uint64_t nanoseconds{0};
    bool isZero() const {
        return nanoseconds == 0;
    }
    double toSec() const {
        return static_cast<double>(nanoseconds) * 1e-9;
    }
    uint64_t toNSec() const {
        return nanoseconds;
    }
    friend NativeDuration operator-(NativeStamp a, NativeStamp b) {
        return {a.nanoseconds >= b.nanoseconds
                    ? static_cast<double>(a.nanoseconds - b.nanoseconds) * 1e-9
                    : -static_cast<double>(b.nanoseconds - a.nanoseconds) * 1e-9};
    }
    friend bool operator<(NativeStamp a, NativeStamp b) {
        return a.nanoseconds < b.nanoseconds;
    }
    friend bool operator<=(NativeStamp a, NativeStamp b) {
        return a.nanoseconds <= b.nanoseconds;
    }
    friend bool operator>(NativeStamp a, NativeStamp b) {
        return b < a;
    }
    friend bool operator>=(NativeStamp a, NativeStamp b) {
        return b <= a;
    }
};
struct NativeWallTime {
    double seconds{0};
    bool isZero() const {
        return seconds == 0;
    }
    friend NativeDuration operator-(NativeWallTime a, NativeWallTime b) {
        return {a.seconds - b.seconds};
    }
};
struct NativeClock {
    NativeStamp world;
    NativeWallTime monotonic;
};

namespace scene_model {
struct Vector {
    double x{0}, y{0}, z{0};
};
struct Quaternion {
    double x{0}, y{0}, z{0}, w{1};
};
struct Pose {
    Vector position;
    Quaternion orientation;
};
struct Twist {
    Vector linear, angular;
};
struct Header {
    NativeStamp stamp;
    std::string frame_id{"world"};
};
struct Geometry {
    std::string type;
    Vector size;
    double radius{0}, height{0};
    std::vector<Vector> vertices;
};
struct Part {
    std::string id;
    Pose pose;
    Geometry geometry;
};
struct Obstacle {
    std::string id, motion_type;
    bool dynamic{false};
    Pose pose;
    std::vector<Part> parts;
};
struct SimulationTime {
    uint64_t epoch{0};
    NativeStamp nanoseconds;
};
struct Snapshot {
    SimulationTime simulation_time;
    Header header;
    std::string epoch;
    uint64_t revision{0};
    std::vector<Obstacle> obstacles;
};
struct ObstacleState {
    std::string id;
    Pose pose;
    Twist twist;
};
struct State {
    SimulationTime simulation_time;
    Header header;
    std::string epoch;
    uint64_t revision{0};
    double scene_time{0};
    std::vector<ObstacleState> obstacles;
};
struct Pose2 {
    double x{0}, y{0}, theta{0};
};
struct PoseSample {
    Header header;
    Pose pose;
};
}  // namespace scene_model
}  // namespace ugv_reset_safety
