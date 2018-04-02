#include <ros/ros.h>
#include "lift_process.hpp"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char **argv) {
    ros::init(argc, argv, "lift_process_node");
    ros::NodeHandle n;
    ROS_INFO("I am lift_process node");

    interface::ProcessInterface process(argc, argv);

    LiftProcess liftProcess(n, argc, argv);
    // liftProcess.SetUpProcessInterface(&process);

    process.setGoalHandler(std::bind(&LiftProcess::liftCallback, liftProcess, std::placeholders::_1));
    process.setPreemptHandler(std::bind(&LiftProcess::preemtCallback, liftProcess));

    process.listen();

    ROS_INFO("I am lift_process_node TOO");
    ros::spin();

    return 0;
}
