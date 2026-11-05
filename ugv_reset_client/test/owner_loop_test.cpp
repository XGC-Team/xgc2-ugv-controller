#include <gtest/gtest.h>
#include <ugv_reset_client/owner_loop.h>
#include <atomic>
#include <boost/make_shared.hpp>
#include <chrono>
#include <functional>
#include <thread>

namespace {
using namespace std::chrono_literals;
class Callback : public ros::CallbackInterface {
   public:
    explicit Callback(std::function<void()> value) : value_(std::move(value)) {}
    CallResult call() override { value_(); return Success; }
   private:
    std::function<void()> value_;
};
template<class Predicate> bool await(Predicate predicate) {
    const auto end = std::chrono::steady_clock::now() + 3s;
    while (!predicate() && std::chrono::steady_clock::now() < end) std::this_thread::sleep_for(1ms);
    return predicate();
}
TEST(OwnerLoop, ClockInputCompletionWallAndShutdownWakeOneSleepingOwner) {
    ugv_reset_client::OwnerLoop loop;
    ros::NodeHandle nh;
    loop.attach(nh);
    ASSERT_TRUE(loop.externalClock());
    auto clock = nh.advertise<rosgraph_msgs::Clock>("/clock", 1, true);
    std::atomic<unsigned> turns{0}, inputs{0}, wall_fires{0};
    std::atomic<double> observed_clock{0};
    std::atomic<bool> in_work{false}, release_work{false}, request_barrier{false};
    double wall_deadline = std::numeric_limits<double>::infinity();
    const auto owner = std::this_thread::get_id();
    auto enqueue = [&](std::function<void()> value) {
        loop.queue()->addCallback(boost::make_shared<Callback>(std::move(value)));
    };
    std::thread actor([&] {
        EXPECT_TRUE(await([&] { return turns.load() > 0 && clock.getNumSubscribers() >= 1; }));
        rosgraph_msgs::Clock message; message.clock = ros::Time(123.0); clock.publish(message);
        EXPECT_TRUE(await([&] { return observed_clock.load() == 123.0; }));
        std::this_thread::sleep_for(20ms);
        const auto paused = turns.load();
        std::this_thread::sleep_for(80ms);
        EXPECT_EQ(turns.load(), paused) << "paused domain deadline must not poll";
        enqueue([&] {
            EXPECT_EQ(std::this_thread::get_id(), owner);
            ++inputs;
            wall_deadline = loop.wallNow() + .04;
        });
        EXPECT_TRUE(await([&] { return inputs.load() == 1; }));
        EXPECT_TRUE(await([&] { return wall_fires.load() == 1; }));
        const auto fired = turns.load();
        std::this_thread::sleep_for(40ms);
        EXPECT_EQ(turns.load(), fired);
        // A completion while the owner is still working must remain latched across its wait.
        request_barrier = true;
        loop.notify();
        EXPECT_TRUE(await([&] { return in_work.load(); }));
        const auto during = turns.load();
        loop.notify();
        release_work = true;
        EXPECT_TRUE(await([&] { return turns.load() > during; }));
        ros::requestShutdown();
    });
    loop.run([&] {
        ++turns;
        observed_clock = loop.now();
        if (loop.wallNow() >= wall_deadline) {
            ++wall_fires;
            wall_deadline = std::numeric_limits<double>::infinity();
        }
        if (request_barrier.exchange(false)) {
            in_work = true;
            while (!release_work.load()) std::this_thread::yield();
        }
    }, [&] {
        ugv_reset_client::NextWake next;
        next.atDomain(124.0);
        next.atWall(wall_deadline);
        return next;
    });
    actor.join();
    EXPECT_EQ(inputs.load(), 1U);
    EXPECT_EQ(wall_fires.load(), 1U);
    EXPECT_LT(turns.load(), 12U);
}
}  // namespace
int main(int argc, char** argv) {
    ros::init(argc, argv, "ugv_owner_loop_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
