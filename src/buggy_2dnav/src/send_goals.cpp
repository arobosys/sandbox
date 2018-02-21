#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <string>
#include <iostream>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "send_goals");
  ros::NodeHandle n;

  int num_goal;
  n.getParam("num_goal",num_goal);

  double pose[num_goal][3];

/*
  for(int i=0;i<num_goal;i++)
  {
	  string goal_str="goal";
	  std::string robot_id=std::to_string(i);
	  goal_str+=robot_id;

	  string goal_x_str=goal_str+"_x";
	  string goal_y_str=goal_str+"_x";
	  string goal_theta_str=goal_str+"_x";

	  cout<<goal_x_str<<endl;

	  n.getParam(goal_x_str,pose[i][0]);
	  n.getParam(goal_y_str,pose[i][1]);
	  n.getParam(goal_theta_str,pose[i][2]);
  }
*/
  n.getParam("goal1_x",pose[0][0]);
  n.getParam("goal1_y",pose[0][1]);
  n.getParam("goal1_theta",pose[0][2]);
  n.getParam("goal2_x",pose[1][0]);
  n.getParam("goal2_y",pose[1][1]);
  n.getParam("goal2_theta",pose[1][2]);
  n.getParam("goal3_x",pose[2][0]);
  n.getParam("goal3_y",pose[2][1]);
  n.getParam("goal3_theta",pose[2][2]);
  n.getParam("goal4_x",pose[3][0]);
  n.getParam("goal4_y",pose[3][1]);
  n.getParam("goal4_theta",pose[3][2]);


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

	  goal.target_pose.pose.position.x = pose[i][0];
	  goal.target_pose.pose.position.y = pose[i][1];

	  double radians=pose[i][2]*(3.1415916/180);
	  tf::Quaternion quaternion;
	  quaternion = tf::createQuaternionFromYaw(radians);
	  geometry_msgs::Quaternion qMsg;
	  tf::quaternionTFToMsg(quaternion,qMsg);
	  goal.target_pose.pose.orientation= qMsg;

	  ROS_INFO("Sending goal %d",i+1);
	  ac.sendGoal(goal);

	  ac.waitForResult();

	  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		  ROS_INFO("Malish, the robot moved to goal %d", i+1);
	  else
		  ROS_INFO("The robot failed to move to goal %d for some reason", i+1);
  }

  return 0;
}
