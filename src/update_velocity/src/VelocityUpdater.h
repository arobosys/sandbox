#ifndef VELOCITY_UPDATER
#define VELOCITY_UPDATER

#include <cmath> 
#include <string> 

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  
#include <std_msgs/Bool.h>
#include <ros/console.h>


class VelocityUpdater {
private:
	//Velocities for different robot states. 
	const float MAX_LINEAR_VELOCITY_MODULE_IF_ROBOT_IS_LOADED = 0.3; 
	const float MAX_LINEAR_VELOCITY_MODULE_IF_ROBOT_IS_NOT_LOADED = 2;
	
	//Used topics names
	const std::string OLD_VELOCITY_TOPIC_NAME = "cmd_vel";
	const std::string NEW_VELOCITY_TOPIC_NAME = "new_cmd_vel"; 
	const std::string ROBOT_IS_LOADED_TOPIC_NAME = "robot_is_loaded";
public: 
	VelocityUpdater(ros::NodeHandle& nh);
	void Update();	
	void Publish();
	
private: 
	//Callback functions which read information from topics
	void RobotIsLoadedCallback(const std_msgs::Bool::ConstPtr& msgRobotIsLoaded);
	void VelocityCallback(const geometry_msgs::Twist::ConstPtr& msgOldVelocity);		
private: 
	float maxLinearVelocityModule; 
	geometry_msgs::Twist msgVelocity;
	
	//Publisher and Subscriber. We need them for correct work. 
	ros::Publisher pubNewVelocity;
	ros::Subscriber subVelocity;
	ros::Subscriber subRobotIsLoaded; 	
};

#endif
