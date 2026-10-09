#pragma once
#include <ugv_fleet_host/reset_control.h>

#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include <xgc2/xrpc/bootstrap.hpp>
#include <xgc2/xrpc/http.hpp>
#include <xgc2/xrpc/runtime_policy.hpp>
namespace ugv_fleet_host {
// Exactly one fixed SDK IO owner. Owner turn processes at most 16 cold commands;
// proposals, feedback and command publication never pass through this boundary.
class ResetHttpBoundary {
   public:
    static constexpr std::size_t command_capacity = 16, observer_capacity = 16;
    ResetHttpBoundary(const xgc2::xrpc::BootstrapInput& bootstrap, std::vector<std::string> robots,
                      const xgc2::xrpc::RuntimePolicy& policy);
    ~ResetHttpBoundary();
    ResetHttpBoundary(const ResetHttpBoundary&) = delete;
    ResetHttpBoundary& operator=(const ResetHttpBoundary&) = delete;
    void ownerTurn(ResetControl& control, uint64_t world_ns, double steady);
    void publish(const ResetControl::Snapshot& snapshot) noexcept;
    const std::string& instance() const;

   private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace ugv_fleet_host
