#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "arobo/OmniSpeed.h"

//#include <sstream>

//Including the UART wheel lib
//#include "Arduino.h"

//#include "MotorWheel.h"
arobo::OmniSpeed msg;

void callback(const arobo::OmniSpeed::ConstPtr& data)
{
  //Converting
  ROS_INFO("Omnispeed sent: %f \t%f \t%f \t%f", msg.wfl, msg.wfr, msg.wrl, msg.wrr);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uart_bridge");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/omni/command", 1000, callback);
  ros::Rate loop_rate(10);
  
  
  int count = 0;

  msg.wfl = 0;
  msg.wfr = 0;
  msg.wrl = 0;
  msg.wrr = 0;
  
  ros::spin();

  return 0;
}
