#include <ros/ros.h>
#include "lift_process.hpp"


int main(int argc, char **argv) {
    ros::init(argc, argv, "lift_process_node");
    ros::NodeHandle n;
    ROS_INFO("I am lift_process node");

    LiftProcess liftProcess(n, argc, argv);

    ROS_INFO("I am lift_process_node TOO");
    ros::spin();

    return 0;
}
