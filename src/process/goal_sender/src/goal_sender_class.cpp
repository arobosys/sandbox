#include <map>
#include <unordered_map>
#include "GoalSender.hpp"

GoalSender::GoalSender(ros::NodeHandle &handle, int argc, char **argv) {

    rosLinkClientPtr = std::make_shared<interface::ProcessInterface>(argc, argv,
                       std::bind(&GoalSender::goalCallback, this,std::placeholders::_1),
                       std::bind(&GoalSender::preemtCallback,this));

    rosLinkClientPtr->listen();
}

template<typename T>
std::vector<T> splitByDelimiter(const std::string &s, char delim);

template<>
std::vector<double> splitByDelimiter(const std::string &s, char delim) {
    std::stringstream ss(s);
    std::string item;
    std::vector<double> result;
    while (std::getline(ss, item, delim)) {
        result.push_back(std::stod(item));
    }
    return result;
}

void GoalSender::parseTransforms(const std::map<std::string, std::string> &keyToValue) {

    std::string goal1_string = keyToValue.at(GOAL1);
    std::string goal2_string = keyToValue.at(GOAL2);
    std::string goal3_string = keyToValue.at(GOAL3);
    std::string goal4_string = keyToValue.at(GOAL4);
    std::string goal5_string = keyToValue.at(GOAL5);
    std::string goal6_string = keyToValue.at(GOAL6);
    std::string goal7_string = keyToValue.at(GOAL7);

    double goalToMove[num_goal][3];

    std::vector<double> goal = splitByDelimiter<double>(goal1_string, ';');
    ROS_INFO("From logic layer I get 2d_goal1: x=%f, y=%f, theta=%f",goal[0], goal[1], goal[2]);
    goalToMove[0][0]=goal[0];
    goalToMove[0][1]=goal[1];
    goalToMove[0][2]=goal[2];

    goal = splitByDelimiter<double>(goal2_string, ';');
    ROS_INFO("From logic layer I get 2d_goal2: x=%f, y=%f, theta=%f",goal[0], goal[1], goal[2]);
    goalToMove[1][0]=goal[0];
    goalToMove[1][1]=goal[1];
    goalToMove[1][2]=goal[2];

    goal = splitByDelimiter<double>(goal3_string, ';');
    ROS_INFO("From logic layer I get 2d_goal3: x=%f, y=%f, theta=%f",goal[0], goal[1], goal[2]);
    goalToMove[2][0]=goal[0];
    goalToMove[2][1]=goal[1];
    goalToMove[2][2]=goal[2];

    goal = splitByDelimiter<double>(goal4_string, ';');
    ROS_INFO("From logic layer I get 2d_goal4: x=%f, y=%f, theta=%f",goal[0], goal[1], goal[2]);
    goalToMove[3][0]=goal[0];
    goalToMove[3][1]=goal[1];
    goalToMove[3][2]=goal[2];

    goal = splitByDelimiter<double>(goal5_string, ';');
    ROS_INFO("From logic layer I get 2d_goal5: x=%f, y=%f, theta=%f",goal[0], goal[1], goal[2]);
    goalToMove[4][0]=goal[0];
    goalToMove[4][1]=goal[1];
    goalToMove[4][2]=goal[2];

    goal = splitByDelimiter<double>(goal6_string, ';');
    ROS_INFO("From logic layer I get 2d_goal6: x=%f, y=%f, theta=%f",goal[0], goal[1], goal[2]);
    goalToMove[5][0]=goal[0];
    goalToMove[5][1]=goal[1];
    goalToMove[5][2]=goal[2];

    goal = splitByDelimiter<double>(goal7_string, ';');
    ROS_INFO("From logic layer I get 2d_goal7: x=%f, y=%f, theta=%f",goal[0], goal[1], goal[2]);
    goalToMove[6][0]=goal[0];
    goalToMove[6][1]=goal[1];
    goalToMove[6][2]=goal[2];


    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(10.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    for(int i=0;i<num_goal;i++)
    {
        move_base_msgs::MoveBaseGoal goal;

        //The goal_(i+1)
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = goalToMove[i][0];
        goal.target_pose.pose.position.y = goalToMove[i][1];

        double radians=goalToMove[i][2]*(3.1415916/180);
        tf::Quaternion quaternion;
        quaternion = tf::createQuaternionFromYaw(radians);
        geometry_msgs::Quaternion qMsg;
        tf::quaternionTFToMsg(quaternion,qMsg);
        goal.target_pose.pose.orientation= qMsg;

        ROS_INFO("Sending goal %d to move_base",i+1);
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Malish, the robot moved to goal %d", i+1);
        else
            ROS_INFO("The robot failed to move to goal %d for some reason", i+1);
    }

}

void GoalSender::goalCallback(const interface::ProcessInterface::Parameters &params)
{
    auto data = params.key_value;
    std::map<std::string, std::string> keyToValue;
    for (const auto &a : data) {
        keyToValue[a.key] = a.value;
    }
    num_goal = std::stod(keyToValue[NUM_GOAL]);
    ROS_INFO("The number of goal = %d", num_goal);
    parseTransforms(keyToValue);
}

void GoalSender::preemtCallback() {}
