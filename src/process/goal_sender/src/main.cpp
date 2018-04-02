#include <ros/ros.h>
#include "GoalSender.hpp"
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_sender_node");
    ros::NodeHandle n;
    ROS_INFO("I am goal_sender_node ");

    using interface::ProcessInterface;
    ProcessInterface::Result alibResultOk, alibResultFail;

    ProcessInterface process(argc, argv);

    GoalSender  goalSender(n, argc, argv);
    goalSender.SetUpProcessInterface(&process);

    process.setGoalHandler(std::bind(&GoalSender::goalCallback, goalSender,std::placeholders::_1));
    process.setPreemptHandler(std::bind(&GoalSender::preemtCallback,goalSender));

    process.listen();

    ROS_INFO("I am goal_sender_node TOO");
    ros::spin();
    return 0;
}