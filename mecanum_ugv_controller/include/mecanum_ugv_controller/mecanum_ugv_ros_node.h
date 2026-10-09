#pragma once
#include <ros/ros.h>
#include <ugv_reset_safety/fleet_edge.h>
namespace mecanum_ugv_controller {
std::unique_ptr<ugv_reset_safety::FleetEdge> createFleetEdge(ros::NodeHandle& nh,
                                                             ros::NodeHandle private_nh);
}
