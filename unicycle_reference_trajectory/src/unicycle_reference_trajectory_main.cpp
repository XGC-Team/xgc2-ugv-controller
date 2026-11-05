#include <ros/ros.h>

#include <cmath>

#include "unicycle_reference_trajectory/unicycle_reference_trajectory_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "unicycle_reference_trajectory_node");
    ros::NodeHandle nh;
    unicycle_reference_trajectory::ReferenceTrajectoryNode node(nh);
    node.run();
    return 0;
}
