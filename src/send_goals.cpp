#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
//#include <fstream>
#include <unistd.h>
#include <memory>
#include "file_coordinates.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define FILE_NAME "1.txt"
#define TIME_TO_WAIT_MS 1700000

int main(int argc, char** argv){
  
  std::string s_file_name(FILE_NAME);
  std::vector<POINT> vec_points_from_file;
  std::unique_ptr<FileCoordinates> fCoordinates(new FileCoordinates);

  if(!fCoordinates->GetCoordinatesVectorFromFile(s_file_name, vec_points_from_file)) return -1;

  ros::init(argc, argv, "send_goals");
  std::cout << "Points count " << vec_points_from_file.size() << std::endl;

  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  for(int i=0;i<vec_points_from_file.size();++i)
  {
    goal.target_pose.pose.position.x = vec_points_from_file[i].x;
    goal.target_pose.pose.orientation.y = vec_points_from_file[i].ang;

    double radians = vec_points_from_file[i].ang * (M_PI/180);
    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians);

	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(quaternion, qMsg);

	goal.target_pose.pose.orientation = qMsg;
  
	ROS_INFO("Sending goal x %f y %f angle %f", vec_points_from_file[i].x, vec_points_from_file[i].y,  vec_points_from_file[i].ang);
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
	    ROS_INFO("Malish has been moved successfully");
	    usleep(TIME_TO_WAIT_MS);
	}
	else
	{
	    ROS_INFO("Malish failed to move " );
	    break;
	}
    }


  return 0;
}
