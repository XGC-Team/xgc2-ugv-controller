#pragma once

// Shared by the ROS-level tests of the ROS edge: they run as rostests, on the master that rostest
// starts for them, with the edge module loaded as a shared library into the test process.

#include <ros/ros.h>

#include <chrono>
#include <thread>

namespace edge_test {

template <typename Predicate>
bool waitUntil(Predicate done, int timeout_ms = 3000) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (!done()) {
        if (std::chrono::steady_clock::now() > end) {
            return done();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return true;
}

// A ROS publisher is connected when its subscriber (here the edge's) has registered with it.
inline bool connected(const ros::Publisher& publisher, int timeout_ms = 5000) {
    return waitUntil([&] { return publisher.getNumSubscribers() > 0; }, timeout_ms);
}
inline bool connected(const ros::Subscriber& subscriber, int timeout_ms = 5000) {
    return waitUntil([&] { return subscriber.getNumPublishers() > 0; }, timeout_ms);
}

}  // namespace edge_test
