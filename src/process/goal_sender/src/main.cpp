#include <ros/ros.h>
#include "GoalSender.hpp"
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_sender_node");
    ros::NodeHandle n;
    ROS_INFO("I am goal_sender_node ");
    GoalSender  goalSender(n, argc, argv);
    ROS_INFO("I am goal_sender_node TOO");
    ros::spin();
    return 0;
}