#include <gtest/gtest.h>
#include <ugv_reset_client/reset_executor.h>
#include "executor_test_support.h"
namespace rc = ugv_reset_client;
namespace {
rc::Measurement at(double now, double stamp) {
    rc::Measurement m; m.ros_now=now; m.wall_now=now; m.pose_stamp=stamp;
    m.healthy=true; return m;
}
rc::Grant decision(rc::ResetExecutor& e,rc::GrantState state,uint32_t id,uint32_t seq) {
    rc::Grant g;g.session=e.sessionId();g.request_generation=e.requestGeneration();
    g.state=state;g.grant_id=id;g.publication_seq=seq;g.goal=e.target();g.s_limit=1e18;
    return g;
}
void certify(rc::ResetExecutor& e,uint32_t id,uint32_t seq,double t) {
    e.onGrant(decision(e,rc::GrantState::Brake,id,seq),at(t,t-.05));
    e.onZeroPublished(t+.01,e.requestGeneration(),id);
    for(double stamp:{t+.02,t+.23}) {auto m=at(stamp+.05,stamp);e.observe(m);e.poll(m);}
}
}
TEST(ResetCausal, InitialGoNeedsExactBrakeRealZeroAndFreshPoseWindow) {
    rc::ResetExecutor e;e.begin(42,{1,0,0},true,at(10,9.95));
    auto go=decision(e,rc::GrantState::Go,8,1);e.onGrant(go,at(10.1,10.05));
    EXPECT_FALSE(e.tracker().active());
    e.onGrant(decision(e,rc::GrantState::Brake,7,2),at(10.1,10.05));
    EXPECT_EQ(e.makeSession(at(10.1,10.05)).acknowledged_grant_id,7u);
    EXPECT_FALSE(e.makeSession(at(10.1,10.05)).stop_certified);
    e.onZeroPublished(10.2);
    for(double stamp:{10.1,10.15}) {auto m=at(10.3,stamp);e.observe(m);e.poll(m);}
    EXPECT_FALSE(e.makeSession(at(10.3,10.15)).stop_certified);
    auto first=at(10.36,10.21);e.observe(first);e.poll(first);
    e.onZeroPublished(10.4); // repeated publication must not restart the first post-zero window
    auto second=at(10.57,10.42);e.observe(second);e.poll(second);
    EXPECT_TRUE(e.makeSession(at(10.57,10.42)).stop_certified);
    go.publication_seq=3;e.onGrant(go,at(10.58,10.43));
    EXPECT_FALSE(e.tracker().active());
    auto install=at(10.58,10.43);e.observe(install);e.step(install);EXPECT_TRUE(e.tracker().active());
    const auto s=e.makeSession(at(10.58,10.43));EXPECT_EQ(s.acknowledged_grant_id,8u);EXPECT_FALSE(s.stop_certified);
}
TEST(ResetCausal, OldGenerationAndRepeatedPublicationDoNotRenewSilence) {
    rc::ResetExecutor e;e.begin(42,{1,0,0},true,at(10,9.95));
    auto old=decision(e,rc::GrantState::Brake,700,100);--old.request_generation;
    e.onGrant(old,at(11.99,11.94));e.step(at(12.01,11.96));
    EXPECT_EQ(e.result().outcome,rc::Outcome::RejectedNoCoordinator);
    rc::ResetExecutor x;x.begin(42,{1,0,0},true,at(10,9.95));
    auto b=decision(x,rc::GrantState::Brake,7,0);x.onGrant(b,at(10.1,10.05));
    x.onGrant(b,at(12,11.95));x.step(at(12.11,12.06));
    EXPECT_EQ(x.result().outcome,rc::Outcome::AbortedCoordinatorLost);
}
TEST(ResetCausal, MutatedLogicalDecisionDoesNotRenewHeartbeat) {
    rc::ResetExecutor e;e.begin(42,{1,0,0},true,at(10,9.95));certify(e,7,0,10.1);
    auto go=decision(e,rc::GrantState::Go,8,1);e.onGrant(go,at(10.4,10.35));e.observe(at(10.4,10.35));e.step(at(10.4,10.35));
    go.goal.yaw=.5;go.publication_seq=2;e.onGrant(go,at(12.39,12.34));
    e.step(at(12.41,12.36));EXPECT_EQ(e.result().outcome,rc::Outcome::AbortedCoordinatorLost);
}
TEST(ResetCausal, TerminalNeedsNewBrakeCertificateAndCompletedHold) {
    rc::ResetExecutor e;e.begin(42,{1,0,0},true,at(10,9.95));
    const auto first=e.requestGeneration();e.abort(rc::Outcome::Stopped,"stop",at(10.1,10.05));e.finish();
    EXPECT_GT(e.requestGeneration(),first);EXPECT_TRUE(e.permissionPending());
    e.onGrant(decision(e,rc::GrantState::Hold,20,1),at(10.2,10.15));EXPECT_TRUE(e.permissionPending());
    certify(e,21,2,10.3);EXPECT_TRUE(e.permissionPending());
    e.onGrant(decision(e,rc::GrantState::Hold,22,3),at(10.6,10.55));EXPECT_FALSE(e.permissionPending());
    const auto s=e.makeSession(at(10.6,10.55));EXPECT_EQ(s.phase,rc::Phase::Idle);
    EXPECT_EQ(s.acknowledged_grant_id,22u);EXPECT_FALSE(s.stop_certified);EXPECT_TRUE(e.command().zero());
}
TEST(ResetCausal, NewResetDoesNotWaitForOldRetirementOrOverwriteOutcome) {
    rc::ResetExecutor e;e.begin(99,{1,0,0},true,at(10,9.95));
    e.abort(rc::Outcome::Stopped,"old",at(10.1,10.05));e.finish();const auto old=e.requestGeneration();
    ASSERT_TRUE(e.begin(1,{2,0,0},true,at(10.2,10.15)));EXPECT_GT(e.requestGeneration(),old);
    EXPECT_FALSE(e.reserveBegin());e.abort(rc::Outcome::Timeout,"new",at(10.3,10.25));
    rc::Result r;ASSERT_TRUE(e.peekResult(&r));EXPECT_EQ(r.session,99u);
    ASSERT_TRUE(e.peekResult(&r));EXPECT_EQ(r.session,99u); // failed commit must retain it
    e.commitResult();ASSERT_TRUE(e.takeResult(&r));EXPECT_EQ(r.session,1u);EXPECT_EQ(r.outcome,rc::Outcome::Timeout);
    EXPECT_FALSE(e.takeResult(&r));
}
TEST(ResetCausal, WrongReceiptUntrustedPoseAndOuterAxisCannotCertify) {
    rc::ResetExecutor e;e.begin(42,{1,0,0},true,at(10,9.95));
    e.onGrant(decision(e,rc::GrantState::Brake,7,0),at(10.1,10.05));
    e.onZeroPublished(10.2,e.requestGeneration()+1,7);
    for(double stamp:{10.21,10.42}) {auto m=at(stamp+.05,stamp);e.observe(m);e.poll(m);}
    EXPECT_FALSE(e.makeSession(at(10.47,10.42)).stop_certified);
    e.onZeroPublished(10.5,e.requestGeneration(),7);
    for(double stamp:{10.51,10.72}) {auto m=at(stamp+.05,stamp);m.pose_stamp_trusted=false;e.observe(m);e.poll(m);}
    EXPECT_FALSE(e.makeSession(at(10.77,10.72)).stop_certified);
    for(double stamp:{10.73,10.94}) {auto m=at(stamp+.05,stamp);m.external_command_zero=false;e.observe(m);e.poll(m);}
    EXPECT_FALSE(e.makeSession(at(10.99,10.94)).stop_certified);
}
TEST(ResetCausal, SessionPublicationBookkeepingWaitsForSuccessfulCommit) {
    rc::ResetExecutor e;auto m=at(10,9.95);ASSERT_TRUE(e.sessionDue(m));
    const auto seq=e.makeSession(m).publication_seq;EXPECT_TRUE(e.sessionDue(m));
    e.markSessionPublished(m);EXPECT_FALSE(e.sessionDue(m));EXPECT_EQ(e.makeSession(m).publication_seq,seq+1);
}
TEST(ResetCausal, SameSessionResidualFencesQueuedOldBrakeGoAndKeepsOriginalDeadline) {
    auto cfg=reset_test::wheeltecConfig();reset_test::PlantParams plant;plant.ell=-.04;
    const rc::Pose start{-6,2.2,-1.5943},goal{-6,-1.2,-.393};reset_test::SoloRig rig(cfg,start,plant);
    rig.exec().begin(1,goal,true,rig.measurement());const auto old=rig.exec().requestGeneration();
    bool injected=false;double next=0;
    ASSERT_TRUE(rig.run(90,[&](reset_test::SoloRig& r,const rc::StepOutput&) {
        if (!injected && r.exec().requestGeneration()>old) {
            injected=true;auto m=r.measurement();const auto g=r.exec().requestGeneration();
            auto stale=reset_test::goGrant(1,10000,start,{},goal,0,1e18);
            stale.request_generation=old;stale.publication_seq=10000;
            stale.state=rc::GrantState::Brake;r.exec().onGrant(stale,m);
            stale.state=rc::GrantState::Go;++stale.grant_id;++stale.publication_seq;r.exec().onGrant(stale,m);
            EXPECT_EQ(r.exec().requestGeneration(),g);EXPECT_EQ(r.exec().sessionId(),1u);
            EXPECT_EQ(r.exec().makeSession(m).acknowledged_grant_id,0u);EXPECT_FALSE(r.exec().tracker().active());
        }
        if (r.time()>=next) {next=r.time()+.4;r.deliver(reset_test::goGrant(1,1,start,{},goal,0,1e18),r.measurement());}
    }));
    EXPECT_TRUE(injected);EXPECT_EQ(rig.exec().result().outcome,rc::Outcome::Arrived);
    EXPECT_NEAR(rig.exec().result().duration,rig.time(),1e-8);
}
TEST(ResetCausal, BackwardClockCannotCertifyOrInstallPendingGo) {
    rc::ResetExecutor e;e.begin(42,{1,0,0},true,at(10,9.95));certify(e,7,0,10.1);
    auto go=decision(e,rc::GrantState::Go,8,1);e.onGrant(go,at(10.4,10.35));
    auto backwards=at(10.3,10.25);e.observe(backwards);e.poll(backwards);
    EXPECT_FALSE(e.tracker().active());EXPECT_FALSE(e.makeSession(backwards).stop_certified);
    EXPECT_TRUE(e.requiresZeroCommand());
}

TEST(ResetCausal, CertificateClearsIfCurrentEvidenceStopsProvingStopped) {
    rc::ResetExecutor e;e.begin(42,{1,0,0},true,at(10,9.95));certify(e,7,0,10.1);
    auto m=at(10.5,10.45);e.observe(m);e.poll(m);
    ASSERT_TRUE(e.makeSession(m).stop_certified);
    m.external_command_zero=false;e.poll(m);EXPECT_FALSE(e.makeSession(m).stop_certified);
    m.external_command_zero=true;m.healthy=false;e.poll(m);EXPECT_FALSE(e.makeSession(m).stop_certified);
    m.healthy=true;m.pose_stamp_trusted=false;e.poll(m);EXPECT_FALSE(e.makeSession(m).stop_certified);
    m.pose_stamp_trusted=true;m.pose_stamp=m.ros_now+.01;e.observe(m);e.poll(m);
    EXPECT_FALSE(e.makeSession(m).stop_certified);
    m.ros_now+=.02;e.poll(m);EXPECT_FALSE(e.makeSession(m).stop_certified); // rejected stamp was not a velocity sample
}
