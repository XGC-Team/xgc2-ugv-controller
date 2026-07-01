#include <ros/ros.h>

#include "unicycle_reference_trajectory/target_replanner_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "unicycle_target_replanner");
    ros::NodeHandle nh;
    unicycle_reference_trajectory::TargetReplannerNode node(nh);
    ros::spin();
    return 0;
}
