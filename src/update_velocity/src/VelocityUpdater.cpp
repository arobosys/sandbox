#include "VelocityUpdater.h"


VelocityUpdater::VelocityUpdater(ros::NodeHandle& nh) {
	msgVelocity.linear.x = 0.0;
	msgVelocity.linear.y = 0.0;
	
	//We think the robot is loaded
	maxLinearVelocityModule = MAX_LINEAR_VELOCITY_MODULE_IF_ROBOT_IS_LOADED; 
	
	//Subscribe for the topic which gives us old velocity	
  subVelocity = nh.subscribe(OLD_VELOCITY_TOPIC_NAME.c_str(), 1000, &VelocityUpdater::VelocityCallback, this); 
  if (!subVelocity) {
  	ROS_WARN("Cannot connect to ", OLD_VELOCITY_TOPIC_NAME.c_str()," topic!");
  } 
  
  //Subscribe for the topic which gives us the robot load
	subRobotIsLoaded = nh.subscribe(ROBOT_IS_LOADED_TOPIC_NAME.c_str(), 1000, &VelocityUpdater::RobotIsLoadedCallback, this);	
	if (!subRobotIsLoaded) {
  	ROS_WARN("Cannot connect to ", ROBOT_IS_LOADED_TOPIC_NAME.c_str()," topic!");
  } 
	
	//Create Publisher which will publish new velocity
	pubNewVelocity = nh.advertise<geometry_msgs::Twist>(NEW_VELOCITY_TOPIC_NAME.c_str(), 1000); 
	if (!subRobotIsLoaded) {
  	ROS_WARN("Cannot connect to ", NEW_VELOCITY_TOPIC_NAME.c_str()," topic!");
  } 
		 	                                                   
}

//Callback function which subscribes for the topic which gives us the robot load	
void VelocityUpdater::RobotIsLoadedCallback(const std_msgs::Bool::ConstPtr& msgRobotIsLoaded) {
	ROS_INFO("RobotIsLoadedCallback function: ");
	
	if (!msgRobotIsLoaded) {
		ROS_WARN("Message from ", ROBOT_IS_LOADED_TOPIC_NAME.c_str(), " topic is incorrect!");
	} else {
		//Change max velocity according the robot load
		if (msgRobotIsLoaded->data) {
			ROS_INFO("Robot is loaded");
		
			maxLinearVelocityModule = MAX_LINEAR_VELOCITY_MODULE_IF_ROBOT_IS_LOADED;
		} else {
			ROS_INFO("Robot is not loaded");
		
			maxLinearVelocityModule = MAX_LINEAR_VELOCITY_MODULE_IF_ROBOT_IS_NOT_LOADED;
		}
	}
}	

//Callback function which subscribes for the topic which gives us the velocity	
void VelocityUpdater::VelocityCallback(const geometry_msgs::Twist::ConstPtr& msgOldVelocity) {
	ROS_INFO("VelocityCallback function: ");
	
	if (!msgOldVelocity) {
		ROS_WARN("Message from ", OLD_VELOCITY_TOPIC_NAME.c_str(), " topic is incorrect!");
	} else {
		msgVelocity.linear.x = msgOldVelocity->linear.x;
		msgVelocity.linear.y = msgOldVelocity->linear.y;	

		ROS_INFO("OldLinear.x = %f", msgVelocity.linear.x);
		ROS_INFO("OldLinear.y = %f", msgVelocity.linear.y);	
	}
}

void VelocityUpdater::Update() {
	//Velocity Module
	float speedModule = sqrt(msgVelocity.linear.x * msgVelocity.linear.x + msgVelocity.linear.y * msgVelocity.linear.y);

	//Change velocity if current velocity is big
	if (speedModule > maxLinearVelocityModule) {
		ROS_INFO("Velocity from the old topic is bigger than max velocity Module! Change velocity."); 
		
		msgVelocity.linear.x = msgVelocity.linear.x * maxLinearVelocityModule/speedModule;
		msgVelocity.linear.y = msgVelocity.linear.y * maxLinearVelocityModule/speedModule;
		
		ROS_INFO("NewLinear.x = %f", msgVelocity.linear.x);
		ROS_INFO("NewLinear.y = %f", msgVelocity.linear.y);	
	}

}

//Publish new velocity 
void VelocityUpdater::Publish() {
	pubNewVelocity.publish(msgVelocity);
}

