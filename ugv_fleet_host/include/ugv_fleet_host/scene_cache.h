#pragma once
#include <ugv_reset_safety/scene_model.h>

#include <memory>
#include <string>
#include <xgc2/xrpc/bootstrap.hpp>
#include <xgc2/xrpc/runtime_policy.hpp>
namespace ugv_fleet_host {
struct SceneSnapshot {
    ugv_reset_safety::scene_model::Snapshot definition;
    ugv_reset_safety::scene_model::State state;
    uint64_t serial{0};
    bool applied{false}, playing{false};
};
SceneSnapshot parseSceneSnapshot(const std::string& body);
// One fixed caller-owned IO thread, one latest coalescing snapshot. The native
// turn never makes a network call or waits on the cache mutex.
class SceneCache {
   public:
    SceneCache(xgc2::xrpc::ServiceRef reference, const xgc2::xrpc::RuntimePolicy& policy);
    ~SceneCache();
    bool take(SceneSnapshot& snapshot);
    bool failed() const;

   private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace ugv_fleet_host
