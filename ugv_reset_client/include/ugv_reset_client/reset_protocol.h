#pragma once

// The vocabulary of the Reset protocol as plain C++ types, without ROS. The ROS messages of
// ugv_reset_msgs (ResetSession, ResetGrant, ResetResult) are converted to and from these at the
// edges (reset_ros.h on the vehicle, the coordinator node on the station), so the vehicle's
// executor, the station's planner and the scenario harness exercise one definition.
//
// Who decides what: the vehicle owns its motion, its deadline and its fail-stop; the coordinator
// only grants permission to move (a frozen route and the prefix of it that is free). A vehicle
// declares body/spec/target truth in its Session. A separate finite namespace scope gates initial
// discovery; it never supplies shapes, kinematics or targets.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ugv_reset_client {

constexpr double kPi = 3.14159265358979323846;

inline double wrapAngle(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

struct Pose {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

struct Point {
    double x{0.0};
    double y{0.0};
};

enum class Model : uint8_t { Unicycle = 0, Omni = 1 };

enum class Phase : uint8_t {
    Idle = 0,        // not resetting
    Requesting = 1,  // a route request; exact stop certification is separate
    Waiting = 2,     // stopped: between moves, or held at the end of the granted prefix
    Moving = 3,
    Settling = 4,  // at the end of the final route, closing the arrival gate
    Done = 5       // the session ended; the Result says how
};

enum class GrantState : uint8_t { Hold = 0, Go = 1, Brake = 2, Reject = 3 };

// Why a vehicle is held, braked or rejected (ResetGrant.REASON_*).
enum class Reason : uint8_t {
    None = 0,
    WaitingForPeer = 1,
    Precedence = 2,
    Replan = 3,
    SceneUnavailable = 4,
    NonParticipantMoving = 5,
    GoalOccupied = 20,
    NoRoute = 21,
    Deadlock = 22,
    TooSlow = 23,
    BadSession = 24,
    Stalled = 25,
    SceneChanged = 26
};

// The one outcome of a session (ResetResult.OUTCOME_*).
enum class Outcome : uint8_t {
    None = 0,
    Arrived = 1,
    Timeout = 2,
    Stopped = 3,
    RejectedNoTarget = 10,
    RejectedNoCoordinator = 11,
    RejectedByCoordinator = 12,
    Infeasible = 20,
    AbortedDeviation = 30,
    AbortedStalled = 31,
    AbortedCoordinatorLost = 32,
    AbortedUnhealthy = 33
};

inline const char* name(Outcome o) {
    switch (o) {
        case Outcome::Arrived:
            return "ARRIVED";
        case Outcome::Timeout:
            return "TIMEOUT";
        case Outcome::Stopped:
            return "STOPPED";
        case Outcome::RejectedNoTarget:
            return "REJECTED_NO_TARGET";
        case Outcome::RejectedNoCoordinator:
            return "REJECTED_NO_COORDINATOR";
        case Outcome::RejectedByCoordinator:
            return "REJECTED_BY_COORDINATOR";
        case Outcome::Infeasible:
            return "INFEASIBLE";
        case Outcome::AbortedDeviation:
            return "ABORTED_DEVIATION";
        case Outcome::AbortedStalled:
            return "ABORTED_STALLED";
        case Outcome::AbortedCoordinatorLost:
            return "ABORTED_COORDINATOR_LOST";
        case Outcome::AbortedUnhealthy:
            return "ABORTED_UNHEALTHY";
        default:
            return "NONE";
    }
}

inline const char* name(Reason r) {
    switch (r) {
        case Reason::WaitingForPeer:
            return "waiting for a peer";
        case Reason::Precedence:
            return "another vehicle passes first";
        case Reason::Replan:
            return "route revoked, re-planning";
        case Reason::SceneUnavailable:
            return "scene unavailable";
        case Reason::NonParticipantMoving:
            return "a body outside Reset is moving";
        case Reason::GoalOccupied:
            return "target occupied";
        case Reason::NoRoute:
            return "no route";
        case Reason::Deadlock:
            return "deadlock";
        case Reason::TooSlow:
            return "predicted duration exceeds the limit";
        case Reason::BadSession:
            return "invalid session";
        case Reason::Stalled:
            return "stalled";
        case Reason::SceneChanged:
            return "scene changed";
        default:
            return "";
    }
}

// What a vehicle declares about itself (the planner has no other source).
struct Footprint {
    double half_length{0.45};
    double half_width{0.40};
    double offset_x{0.0};  // rectangle centre in the body frame
    double offset_y{0.0};
};

struct VehicleSpec {
    Model model{Model::Unicycle};
    Footprint footprint;
    double max_v{0.35};
    double max_vy{0.0};
    double max_w{0.5};
    double acc_v{0.35};
    double acc_w{0.6};
    double ell_bound{
        0.25};  // |rotation-centre offset| bound of a unicycle (Scout lateral coupling)
};

// ResetSession. G and publication sequence continue across S within one live owner;
// zero G is observational only. No wrap or cross-restart resume is supported.
// stop_certified refers only to the exact accepted Brake (S,G,ack), after actual
// zero publication and fresh stopped acquisition. Go/Hold receipts have cert=false.
struct Session {
    uint32_t request_generation{0};
    uint32_t acknowledged_grant_id{0};
    bool stop_certified{false};
    uint32_t publication_seq{0};  // preserve ROS Header.seq; not an additional wire field
    double stamp{0.0};            // the vehicle's clock when it was made [s]
    uint32_t session{0};
    double pose_stamp{0.0};
    Pose pose;
    Pose target;
    Phase phase{Phase::Idle};
    double progress{0.0};
    double cross_track{0.0};
    VehicleSpec spec;
    double ell_hat{0.0};
    double ell_spread{-1.0};  // < 0: unknown, the planner covers +-ell_bound
    bool healthy{false};
};

// ResetGrant. A GO carries the whole frozen route and the prefix of it that may be driven.
struct Grant {
    uint32_t request_generation{0};
    uint32_t publication_seq{0};  // preserve ROS Header.seq before heartbeat validation
    double stamp{0.0};            // the coordinator's clock when it was published [s]
    double ttl{1.0};  // heartbeat: the grant holds for this long after it is received [s]
    uint32_t session{0};
    GrantState state{GrantState::Hold};
    uint32_t grant_id{0};
    Pose start;  // the pose the route was planned from
    Pose goal;   // where the route ends (the target for a final move, a parking pose otherwise)
    bool final_move{true};
    double ell_plan{0.0};
    std::vector<Point> via;  // interior waypoints of the rotation-centre polyline
    double s_limit{0.0};     // arc length of the route the vehicle may reach
    Reason reason{Reason::None};
    std::string detail;
    std::vector<std::string> blockers;
};

// ResetResult.
struct Result {
    uint32_t session{0};
    Outcome outcome{Outcome::None};
    Reason reason{Reason::None};
    double final_planar_error{0.0};
    double final_yaw_error{0.0};
    double duration{0.0};
    std::string detail;
};

// The numbers of the decided requirements (T14): arrival is the same for every vehicle.
constexpr double kResetTimeout = 90.0;            // s, total
constexpr double kArrivePosition = 0.05;          // m
constexpr double kArriveYaw = 5.0 * kPi / 180.0;  // rad
constexpr double kNoCoordinatorTimeout = 2.0;     // s without any answer
constexpr double kHeartbeatLoss = 1.0;            // s moving without a refresh
constexpr double kSessionPeriod = 0.2;            // s between Session messages (5 Hz)
// A route has at most this many interior vertices (the module payload holds that many).
constexpr size_t kMaxVia = 24;

}  // namespace ugv_reset_client
