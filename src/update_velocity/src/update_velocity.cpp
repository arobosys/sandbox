#include <ros/ros.h>

#include "VelocityUpdater.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "update_velocity");
  ros::NodeHandle nh;
	
	//velocityUpdater works with the robot velocities from old topic and  the robot load.  
  VelocityUpdater velocityUpdater(nh); 
  
  ros::Rate rate(10);
 
 
  while(ros::ok()) { 	  
  	//velocityUpdater've gotten the robot velocities from the old topic and the robot load. velocityUpdater calculates the new velocity using the restriction by max velocity module.
  	velocityUpdater.Update();
  	
  	//Publish publishes the acceptable robot velocity in the new topic. 
	  velocityUpdater.Publish();

    ros::spinOnce();		
    rate.sleep();
	}
}
