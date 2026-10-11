// The ROS edge against a master that is not there: the start fails at once and says why, instead of
// waiting for it.

#include <gtest/gtest.h>

#include <chrono>

#include "test_host.h"

namespace {

using ugv_modules_test::ModuleLibrary;
using ugv_modules_test::TestHost;

TEST(RosEdgeWithoutMaster, StartFailsAtOnceAndSaysWhy) {
    ModuleLibrary library(EDGE_MODULE_PATH);
    TestHost host(library.desc());
    ASSERT_EQ(host.create("{\"master_uri\": \"http://127.0.0.1:1\"}"), XGC2_OK);
    const auto begin = std::chrono::steady_clock::now();
    EXPECT_EQ(host.start(), XGC2_ERR_INTERNAL);
    const auto elapsed = std::chrono::steady_clock::now() - begin;
    EXPECT_LT(elapsed, std::chrono::seconds(5));
    EXPECT_TRUE(host.logged(3, "the ROS master is not reachable: http://127.0.0.1:1"));
    // The instance is still a valid one: stopping and destroying it is fine.
    EXPECT_EQ(host.stop(), XGC2_OK);
    host.destroy();
}

}  // namespace
