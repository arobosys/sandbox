#include <map>
#include <unordered_map>
#include "GoalSender.hpp"
#include <math.h>
#include <malish/JoyCMD.h>
// #include </home/zhuhua/AGRO/catkin_ws/src/malish/devel/include/malish/Lift.h>

GoalSender::GoalSender(ros::NodeHandle &handle, int argc, char **argv) {
    num_goal = 7;
    _gogogo = false;
    _restart = false;

	ros::NodeHandle nh_;

    joystik = nh_.subscribe("/joy/command", 10, &GoalSender::cmdCallback, this);
	if(!joystik)
		ROS_WARN("Cannot connect to joystik topic");

    //pub_lift = handle.advertise<Malish::Lift>("/lift", 10,cmdCallback());
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

    /*
    malish::Lift lift_msg;
    lift_msg.dio1 = True;
    lift_msg.dio2 = False;
    lift_msg.dio3 = False;
    pub_lift.publish(lift_msg);
     */
    ROS_INFO("I am lifting load");
    ros::Duration(3).sleep();

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(10.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    int curr_goal = 0;

  do{
	if(curr_goal == num_goal)
	{
		ros::Duration(5).sleep();
	}
    while(curr_goal < num_goal)
    {
        
        move_base_msgs::MoveBaseGoal goal;

        //The goal_(i+1)
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = goalToMove[curr_goal][0];
        goal.target_pose.pose.position.y = goalToMove[curr_goal][1];

        double radians=goalToMove[curr_goal][2]*(M_PI/180.0);
        tf::Quaternion quaternion;
        quaternion = tf::createQuaternionFromYaw(radians);
        geometry_msgs::Quaternion qMsg;
        tf::quaternionTFToMsg(quaternion,qMsg);
        goal.target_pose.pose.orientation= qMsg;

        int retry = 0;
        do {
            if (retry)
                ROS_INFO("Retry #%d sending a goal %d to move_base", retry, curr_goal + 1);
            ROS_INFO("Sending goal %d to move_base", curr_goal + 1);
            ac.sendGoal(goal);

            ac.waitForResult();

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Malish, the robot moved to goal %d", curr_goal + 1);
                if (curr_goal == 3) {
                    ROS_INFO("I am unloading");

                    int sec_to_wait = 15; // wait

                    while (sec_to_wait) {
                        ROS_INFO("Wait %d seconds", sec_to_wait);
                        if (_gogogo)
                            sec_to_wait = 0;
                        else {
                            ros::Duration(1).sleep();
                            sec_to_wait--;
                        }
                    }
                    /*
                    malish::Lift lift_msg;
                    lift_msg.dio1 = False;
                    lift_msg.dio2 = True;
                    lift_msg.dio3 = True;
                    pub_lift.publish(lift_msg);
                     */

                    ros::Duration(3).sleep();
                }
            }
            else {
                ROS_INFO("The robot failed to move to goal %d for some reason", curr_goal + 1);
                retry++;
                ros::Duration(5).sleep();
            }
        }while((ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) || _gogogo);

        curr_goal++;
    }//for num_of_goal
	ROS_INFO("The robot completed task.  Reset %d", _restart);
	if(_restart == true)
            curr_goal = 0;
  }while(1);
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

void GoalSender::cmdCallback(const malish::JoyCMD &message) {
    _gogogo = message.gogogo;
    _restart = message.restart;
	ROS_INFO("JoyCMD: gogogo %d reset %d", _gogogo , _restart);
}
