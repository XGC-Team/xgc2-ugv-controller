#include <gtest/gtest.h>
#include <ugv_fleet_host/reset_control.h>
using ugv_fleet_host::ResetControl;
const ResetControl::SceneFence fence = [] {
    ResetControl::SceneFence f;
    f.epoch[0] = 'w';
    f.revision = 1;
    f.simulation_time_epoch = 2;
    return f;
}();
void ready(ResetControl& control) {
    control.scene(fence, true, 1'000'000'000, 1);
}
using Session = ugv_reset_safety::ResetSession;
TEST(ResetControl, ReadyNeverCompletesAndNativeTerminalEventsRemainDistinct) {
    for (auto completion : {Session::Completion::Arrived, Session::Completion::Rejected,
                            Session::Completion::Expired}) {
        ResetControl control(
            1, [](auto) { return true; }, [](auto) { return true; });
        std::array<bool, 128> selected{};
        selected[0] = true;
        Session session;
        ready(control);
        ASSERT_TRUE(control.start(0, selected, 1'000'000'000, 1, fence));
        control.sample(0, session, false, false, true, 1'000'000'000, 1'020'000'000, 1.02);
        EXPECT_EQ(control.snapshot().state, ResetControl::State::Accepted);
        session.begin({});
        control.sample(0, session, true, false, true, 1'000'000'000, 1'040'000'000, 1.04);
        EXPECT_EQ(control.snapshot().state, ResetControl::State::Running);
        session.complete(completion);
        session.cancel();
        control.sample(0, session, false, false, false, 1'000'000'000, 1'060'000'000, 1.06);
        EXPECT_EQ(control.snapshot().state, ResetControl::State::Running);
        session.noteApplied({}, 1'060'000'000);
        session.noteApplied({}, 1'060'000'000);
        control.sample(0, session, false, false, true, 1'060'000'000, 1'060'000'000, 1.06);
        EXPECT_EQ(control.snapshot().state, completion == Session::Completion::Arrived
                                                ? ResetControl::State::Arrived
                                                : completion == Session::Completion::Expired
                                                      ? ResetControl::State::Expired
                                                      : ResetControl::State::Rejected);
    }
}
TEST(ResetControl, CancellationRequiresNativeExitAndNewActuallyPublishedZero) {
    ResetControl control(
        1, [](auto) { return true; }, [](auto) { return true; });
    std::array<bool, 128> selected{};
    selected[0] = true;
    Session session;
    ready(control);
    ASSERT_TRUE(control.start(0, selected, 1'000'000'000, 1, fence));
    session.begin({});
    control.sample(0, session, true, false, true, 1'000'000'000, 1'020'000'000, 1.02);
    const auto snapshot = control.snapshot();
    EXPECT_FALSE(control.cancel(snapshot.revision + 1, snapshot.operation, 1'040'000'000, 1.04));
    ASSERT_TRUE(control.cancel(snapshot.revision, snapshot.operation, 1'040'000'000, 1.04));
    session.cancel();
    control.sample(0, session, false, false, true, 1'000'000'000, 1'060'000'000, 1.06);
    EXPECT_EQ(control.snapshot().state, ResetControl::State::Cancelling);
    session.noteApplied({}, 1'060'000'000);
    control.sample(0, session, false, false, true, 1'060'000'000, 1'060'000'000, 1.06);
    EXPECT_EQ(control.snapshot().state, ResetControl::State::Cancelled);
}
TEST(ResetControl, RejectedAdmissionAndDualClockTimeoutNeverBecomeSuccess) {
    for (int scenario = 0; scenario < 3; ++scenario) {
        int stops = 0;
        ResetControl control(
            2, [&](auto i) { return i == 0; },
            [&](auto) {
                ++stops;
                return true;
            });
        std::array<bool, 128> selected{};
        selected[0] = selected[1] = true;
        Session session;
        ready(control);
        ASSERT_TRUE(control.start(0, selected, 1'000'000'000, 1, fence));
        EXPECT_EQ(control.snapshot().robots[1].state, ResetControl::State::Rejected);
        control.sample(0, session, false, scenario == 0, true, 1'000'000'000,
                       scenario == 1 ? 91'000'000'000ULL : 1'020'000'000ULL,
                       scenario == 2 ? 91 : 1.02);
        EXPECT_EQ(control.snapshot().state, ResetControl::State::Cancelling);
        {
            session.noteApplied({}, scenario == 1 ? 91'000'000'000ULL : 1'020'000'000ULL);
            control.sample(0, session, false, false, true, session.appliedStamp(),
                           scenario == 1 ? 91'000'000'000ULL : 1'020'000'000ULL,
                           scenario == 2 ? 91 : 1.02);
            EXPECT_EQ(control.snapshot().state,
                      scenario == 0 ? ResetControl::State::Rejected : ResetControl::State::Expired);
        }
        EXPECT_EQ(stops, 1);
    }
}
TEST(ResetControl, FailedStopEnqueueNeverFabricatesACompletedCancellation) {
    ResetControl control(
        1, [](auto) { return true; }, [](auto) { return false; });
    std::array<bool, 128> selected{};
    selected[0] = true;
    Session session;
    ready(control);
    ASSERT_TRUE(control.start(0, selected, 1'000'000'000, 1, fence));
    session.begin({});
    control.sample(0, session, true, false, false, 1'000'000'000, 1'020'000'000, 1.02);
    const auto snapshot = control.snapshot();
    ASSERT_TRUE(control.cancel(snapshot.revision, snapshot.operation, 1'040'000'000, 1.04));
    session.noteApplied({}, 1'060'000'000);
    control.sample(0, session, true, false, true, 1'060'000'000, 1'060'000'000, 1.06);
    EXPECT_EQ(control.snapshot().state, ResetControl::State::Cancelling);
    session.cancel();
    control.sample(0, session, false, false, false, 1'060'000'000, 1'080'000'000, 1.08);
    EXPECT_EQ(control.snapshot().state, ResetControl::State::Cancelling);
    session.noteApplied({}, 1'080'000'000);
    control.sample(0, session, false, false, true, 1'080'000'000, 1'080'000'000, 1.08);
    EXPECT_EQ(control.snapshot().state, ResetControl::State::Cancelled);
}
TEST(ResetControl, ScientificEpochFenceRejectsAndWaitsForNewActualZero) {
    int stops = 0;
    ResetControl control(
        1, [](auto) { return true; },
        [&](auto) {
            ++stops;
            return true;
        });
    std::array<bool, 128> selected{};
    selected[0] = true;
    Session session;
    EXPECT_FALSE(control.start(0, selected, 1'000'000'000, 1, fence));
    ready(control);
    auto wrong = fence;
    wrong.simulation_time_epoch++;
    EXPECT_FALSE(control.start(0, selected, 1'000'000'000, 1, wrong));
    ASSERT_TRUE(control.start(0, selected, 1'000'000'000, 1, fence));
    session.begin({});
    control.sample(0, session, true, false, true, 1'000'000'000, 1'020'000'000, 1.02);
    control.scene(wrong, true, 1'040'000'000, 1.04);
    EXPECT_EQ(stops, 1);
    EXPECT_EQ(control.snapshot().state, ResetControl::State::Cancelling);
    session.cancel();
    control.sample(0, session, false, false, true, 1'000'000'000, 1'060'000'000, 1.06);
    EXPECT_EQ(control.snapshot().state, ResetControl::State::Cancelling);
    session.noteApplied({}, 1'060'000'000);
    control.sample(0, session, false, false, true, 1'060'000'000, 1'060'000'000, 1.06);
    EXPECT_EQ(control.snapshot().state, ResetControl::State::Rejected);
}
TEST(ResetControl, ReplacementGenerationCannotCompleteTheOriginalOperation) {
    int stops = 0;
    ResetControl control(
        1, [](auto) { return true; },
        [&](auto) {
            ++stops;
            return true;
        });
    std::array<bool, 128> selected{};
    selected[0] = true;
    Session session;
    ready(control);
    ASSERT_TRUE(control.start(0, selected, 1'000'000'000, 1, fence));
    session.begin({});
    control.sample(0, session, true, false, true, 1'000'000'000, 1'020'000'000, 1.02);
    session.begin({});
    session.complete(Session::Completion::Arrived);
    session.noteApplied({}, 1'040'000'000);
    control.sample(0, session, true, false, true, session.appliedStamp(), 1'040'000'000, 1.04);
    EXPECT_EQ(control.snapshot().state, ResetControl::State::Cancelling);
    EXPECT_EQ(stops, 1);
    session.cancel();
    session.noteApplied({}, 1'060'000'000);
    control.sample(0, session, false, false, true, session.appliedStamp(), 1'060'000'000, 1.06);
    EXPECT_EQ(control.snapshot().state, ResetControl::State::Rejected);
}
