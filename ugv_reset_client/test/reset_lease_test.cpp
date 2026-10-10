#include "ugv_reset_client/reset_lease.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

using ugv_reset_client::ResetLease;

namespace {

constexpr uint64_t t = 1000000000ULL;

TEST(ResetLease, OwnershipReplayAndDualClockLease) {
    ResetLease lease;
    ResetLease::Clearance cleared;
    ASSERT_TRUE(!lease.issue({}, t, 1.0).valid);  // no session, no request
    lease.begin(41U, {2.0, 3.0, 0.5});
    lease.noteApplied({}, t);
    auto request = lease.issue({}, t, 1.0);
    ASSERT_TRUE(request.valid && request.target.x == 2.0 && request.target.y == 3.0);
    ASSERT_EQ(request.generation, 41U);
    ASSERT_TRUE(request.applied_stamp == t && request.applied_command.x == 0.0);
    // Wrong generation, an unissued stamp, a bad status, a bad command, and a
    // terminal status that still carries motion are all refused.
    ASSERT_TRUE(!lease.accept(request.generation + 1, t, 0, {0.1, 0, 0}, t, 1.0, cleared));
    ASSERT_TRUE(!lease.accept(request.generation, t + 1, 0, {0.1, 0, 0}, t + 1, 1.0, cleared));
    ASSERT_TRUE(!lease.accept(request.generation, t, 3, {}, t, 1.0, cleared));
    ASSERT_TRUE(!lease.accept(request.generation, t, 0, {NAN, 0, 0}, t, 1.0, cleared));
    ASSERT_TRUE(!lease.accept(request.generation, t, 1, {0.1, 0, 0}, t, 1.0, cleared));
    ASSERT_TRUE(lease.accept(request.generation, t, 0, {0.1, 0, 0.2}, t, 1.01, cleared));
    EXPECT_EQ(cleared.generation, 41U);
    EXPECT_EQ(cleared.stamp, t);
    EXPECT_DOUBLE_EQ(cleared.issue_wall, 1.0);  // the issue time, not the receive time
    EXPECT_EQ(cleared.status, ResetLease::RUNNING);
    EXPECT_DOUBLE_EQ(cleared.command.x, 0.1);
    EXPECT_DOUBLE_EQ(cleared.command.yaw, 0.2);
    EXPECT_DOUBLE_EQ(cleared.lease_seconds, 0.15);
    // Replays cannot renew the lease.
    ASSERT_TRUE(!lease.accept(request.generation, t, 0, {0.1, 0, 0}, t, 1.02, cleared));
    ASSERT_TRUE(!lease.issue({}, t, 1.2).valid);  // a paused clock cannot renew
    ASSERT_TRUE(!lease.issue({}, t - 1, 1.2).valid);

    auto newer = lease.issue({}, t + 20000000, 1.03);
    ASSERT_TRUE(newer.valid);
    // A received proposal that has not been published is never acknowledged.
    ASSERT_TRUE(newer.applied_command.x == 0.0 && newer.applied_stamp == t);
    lease.noteApplied({0.1, 0, 0.2}, t + 20000000);
    // Both clocks independently expire the request: this one is 151 ms old on the wall clock.
    ASSERT_TRUE(!lease.accept(newer.generation, newer.stamp, 0, {}, newer.stamp, 1.181, cleared));
    ASSERT_TRUE(!lease.accept(newer.generation, newer.stamp, 0, {}, newer.stamp + 150000001ULL,
                              1.04, cleared));
    ASSERT_TRUE(lease.accept(newer.generation, newer.stamp, 1, {}, newer.stamp, 1.04, cleared));
    EXPECT_EQ(cleared.status, ResetLease::ARRIVED);

    lease.cancel();
    ASSERT_TRUE(!lease.active());
    ASSERT_TRUE(!lease.accept(newer.generation, newer.stamp, 0, {}, newer.stamp, 1.04, cleared));
    lease.begin(42U, {4.0, 5.0, 0.0});
    ASSERT_TRUE(lease.active());
    ASSERT_TRUE(!lease.accept(newer.generation, newer.stamp, 0, {}, newer.stamp, 1.04, cleared));
    const auto next = lease.issue({}, t + 40000000, 1.05);
    ASSERT_TRUE(next.valid && next.target.x == 4.0 && next.generation == 42U);
    // Begin/cancel do not pretend that an un-emitted zero has been executed.
    ASSERT_TRUE(next.applied_stamp == t + 20000000 && next.applied_command.x == 0.1);
    ASSERT_TRUE(lease.accept(next.generation, next.stamp, 2, {}, next.stamp, 1.05, cleared));
    EXPECT_EQ(cleared.status, ResetLease::REJECTED);
    lease.begin(43U, {NAN, 0, 0});
    ASSERT_TRUE(!lease.active());
}

TEST(ResetLease, RequestsAreSpacedAndInvalidInputsAreNotIssued) {
    ResetLease lease;
    lease.begin(1U, {0, 0, 0});
    ASSERT_TRUE(lease.issue({0, 0, 0}, t, 10.0).valid);
    EXPECT_FALSE(lease.issue({0, 0, 0}, t + 1000000, 10.01).valid);  // under 20 ms apart
    EXPECT_TRUE(lease.issue({0, 0, 0}, t + 25000000, 10.03).valid);
    EXPECT_FALSE(lease.issue({NAN, 0, 0}, t + 50000000, 10.06).valid);
    EXPECT_FALSE(lease.issue({0, 0, 0}, 0, 10.06).valid);
    EXPECT_FALSE(lease.issue({0, 0, 0}, t + 60000000, NAN).valid);
}

TEST(ResetLease, ConcurrentResponsesAndRequestsAreSafe) {
    ResetLease lease;
    lease.begin(5U, {1.0, 1.0, 0.0});
    std::atomic<bool> stop{false};
    std::atomic<int> accepted{0};
    std::atomic<uint64_t> issued_stamp{0};
    std::atomic<double> wall_now{100.0};
    std::thread responder([&] {
        uint64_t seen = 0;
        while (!stop) {
            const uint64_t stamp = issued_stamp.load();
            if (stamp != 0 && stamp != seen) {
                ResetLease::Clearance cleared;
                if (lease.accept(5U, stamp, 0, {0.1, 0, 0}, stamp, wall_now.load(), cleared)) {
                    ++accepted;
                }
                seen = stamp;
            }
        }
    });
    uint64_t stamp = t;
    for (int k = 0; k < 200; ++k) {
        stamp += 25000000ULL;
        wall_now = 100.0 + 0.03 * k;
        const auto request = lease.issue({0, 0, 0}, stamp, wall_now.load());
        if (request.valid) {
            issued_stamp = request.stamp;
        }
        lease.noteApplied({0.1, 0, 0}, stamp);
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    stop = true;
    responder.join();
    EXPECT_GT(accepted.load(), 0);
}

}  // namespace
