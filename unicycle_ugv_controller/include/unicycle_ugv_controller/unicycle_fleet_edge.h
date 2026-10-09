#pragma once
#include <ros/ros.h>
#include <ugv_reset_safety/fixed_executor.h>
#include <ugv_reset_safety/fleet_edge.h>
namespace unicycle_ugv_controller {
std::unique_ptr<ugv_reset_safety::FleetEdge> createFleetEdge(
    ros::NodeHandle& nh, ros::NodeHandle private_nh, ugv_reset_safety::FixedExecutor& executor,
    std::size_t slot);
}
