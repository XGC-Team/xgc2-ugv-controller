#include <gtest/gtest.h>
#include <ugv_reset_safety/fixed_executor.h>

#include <atomic>
#include <chrono>
#include <future>
TEST(FixedExecutor, BoundedWorkersCoalesceAndDetachWaitsForActualQuiescence) {
    ugv_reset_safety::FixedExecutor pool(2);
    std::atomic<int> running = 0, maximum = 0, executed = 0;
    std::promise<void> release;
    auto held = release.get_future().share();
    for (std::size_t i = 0; i < 12; ++i)
        pool.attach(i, [&] {
            auto n = ++running;
            auto old = maximum.load();
            while (old < n && !maximum.compare_exchange_weak(old, n)) {
            }
            held.wait();
            ++executed;
            --running;
        });
    for (std::size_t i = 0; i < 12; ++i)
        for (int n = 0; n < 100; ++n)
            ASSERT_TRUE(pool.submit(i));
    for (int n = 0; n < 100 && running < 2; ++n)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ASSERT_EQ(running, 2);
    EXPECT_EQ(maximum, 2);
    auto detach = std::async(std::launch::async, [&] { pool.detach(0); });
    EXPECT_EQ(detach.wait_for(std::chrono::milliseconds(20)), std::future_status::timeout);
    release.set_value();
    ASSERT_EQ(detach.wait_for(std::chrono::seconds(1)), std::future_status::ready);
    detach.get();
    EXPECT_FALSE(pool.submit(0));
    for (std::size_t i = 1; i < 12; ++i)
        pool.detach(i);
    EXPECT_LE(executed, 14);
    EXPECT_EQ(running, 0);
    EXPECT_EQ(maximum, 2);
}
