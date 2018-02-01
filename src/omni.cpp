#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "arobo/OmniSpeed.h"

#include <sstream>

float rwheel, lx, ly; 
arobo::OmniSpeed msg;
 
void callback(const geometry_msgs::Twist::ConstPtr& data)
{
  //Converting
  msg.wfl = (data->linear.x - data->linear.y - data->angular.z*(lx+ly))/rwheel;
  msg.wfr = (data->linear.x + data->linear.y + data->angular.z*(lx+ly))/rwheel;
  msg.wrl = (data->linear.x + data->linear.y - data->angular.z*(lx+ly))/rwheel;
  msg.wrr = (data->linear.x - data->linear.y + data->angular.z*(lx+ly))/rwheel;
  ROS_INFO("Got new speed command:\nvx = %f,vy = %f, wz = %f", data->linear.x, data->linear.y, data->linear.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "omni_convertor");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<arobo::OmniSpeed>("/omni/command", 1000);
  ros::Subscriber sub = n.subscribe("/twist/command", 1000, callback);
  ros::Rate loop_rate(10);
  
  //Robot parametres
  //Robot is symmetric, using milimetres
  rwheel = 70;
  lx = 200;
  ly = 300;

  if (n.getParam("rwheel", rwheel))
    {
      ROS_INFO("Got omni param(rwheel): %f", rwheel);
    }
  else
    {
      ROS_WARN("Failed to get param 'omni_params', using default: %f", lx);
    }
  if (n.getParam("lx", lx))
    {
      ROS_INFO("Got omni param(rwheel): %f", lx);
    }
  else
    {
      ROS_WARN("Failed to get param 'omni_params', using default: %f", rwheel);
    }
  if (n.getParam("ly", ly))
    {
      ROS_INFO("Got omni param(rwheel): %f", ly);
    }
  else
    {
      ROS_WARN("Failed to get param 'omni_params', using default: %f", ly);
    }
  //Robot parametres
  
  int count = 0;

  msg.wfl = 0;
  msg.wfr = 0;
  msg.wrl = 0;
  msg.wrr = 0;
  
  while (ros::ok())
    {
      ROS_INFO("Omnispeed: %u, \t%u, \t%u, \t%u", msg.wfl, msg.wfr, msg.wrl, msg.wrr);
      pub.publish(msg);

      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }
  return 0;
}
