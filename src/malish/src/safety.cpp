// Created by Evgeny on 19.02.18.
/*!
 * \file safety.cpp
 *
 * Node which informs you if obstacles invade robot's safety vicinity.
 *
 * \authors Ostroumov Georgy
 * \authors Shtanov Evgeny
 *
*/

#include <sstream>
#include <list>
// ROS
#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/String.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <malish/Obstacle.h>
#include <malish/Diode.h>
#include <sensor_msgs/Range.h>

// OpenCV
#include <opencv2/imgproc.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>

//#define __DEBUG__ 1
const static int __DEBUG__ = 1;

/// Robot's width, [m].
static const float robot_width = 0.45;
/// Robot's length, [m].
static const float robot_length = 0.65;
/// Margin of safety vicinity, [m].
static const float margin = 0.88;
/// Margin of vicinity to light LED in yellow (notification), [m].
static const float yellow_margin = 1.25;
/// Maximal sonar distance.
static const float sonar_max_phisical = 1.03;
static const float sonar_max = 1.0;
static const float sonar_alert_front = 0.85;
static const float sonar_alert_bckg = 0.10;
/// Constant for comparision with float type zero value.
static const float eps = 1e-10;


void set_red(malish::Diode & msg) {
	msg.red = 255;
	msg.green = 0;
	msg.blue = 0;
}

void set_yellow(malish::Diode & msg) {
	msg.red = 255;
	msg.green = 127;
	msg.blue = 0;
}

void reset_LED(malish::Diode & msg) {
	msg.red = 0;
	msg.green = 0;
	msg.blue = 0;
}

// Returns mean of N elements
float push_and_mean_list(std::list<float> & queue, float val, float max_val, unsigned int size=5) {
	static unsigned int qsize = size;
	float mean = 0.0;

	if(queue.size() == qsize) {
		queue.pop_front();
	} else if (queue.size() > qsize) {
		while(queue.size() > qsize) {
			queue.pop_front();
		}
	}

	queue.push_back(val);

	for (std::list<float>::iterator it=queue.begin(); it != queue.end(); ++it) {
		mean += *it;
	}

	mean = mean / queue.size();

	if(queue.size() < size)
	    mean = max_val;

	return mean;
}

/**
 * class Safety.
 *
 * Obstacle occurrence analyzer.
 * Sends alarm message if some obstacle is in robot's safety zone.
 */
class Safety {
  public:;
    Safety() {
        ros::NodeHandle nh_;
        safety_pub_ = nh_.advertise<malish::Obstacle>("/safety", 10);
        led_pub_ = nh_.advertise<malish::Diode>("/led", 10);

        // Subscribe to Sonar's data.
        front_sonar_sub_ = nh_.subscribe("/sonar/front", 10, &Safety::frontSonarCallback, this);
        // rear_sonar_sub_ = nh_.subscribe("/sonar/rear", 10, &Safety::rearSonarCallback, this);
        // left_sonar_sub_ = nh_.subscribe("/sonar/left", 10, &Safety::leftSonarCallback, this);
        // right_sonar_sub_ = nh_.subscribe("/sonar/right", 10, &Safety::rightSonarCallback, this);

        // Init messages.
        safety_msg_.timestamp = ros::Time::now();
        safety_msg_.alert = false;

        safety_msg_.pos.orientation.w = 0.0;
        safety_msg_.pos.orientation.x = 0.0;
        safety_msg_.pos.orientation.y = 0.0;
        safety_msg_.pos.orientation.z = 0.0;

        safety_msg_.pos.position.x = 0.0;
        safety_msg_.pos.position.y = 0.0;
        safety_msg_.pos.position.z = 0.0;

        led_msg_.red = 0;
        led_msg_.green = 0;
        led_msg_.blue = 0;

        sonar_alarm = false;
        sonar_yellow = false;
    }

  void frontSonarCallback (sensor_msgs::Range const& std_sonar_msg) {
	  static std::list<float> range_list;

	  float mean = push_and_mean_list(range_list, std_sonar_msg.range, sonar_max, 5);

	  ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

	  if (mean > sonar_alert_front && mean < sonar_max) {
		  set_yellow(led_msg_);
		  led_pub_.publish(led_msg_);
		  sonar_alarm = false;
		  sonar_yellow = true;
	  } else if(mean < sonar_alert_front) {
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = true;

		  set_red(led_msg_);

		  ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

		  sonar_alarm = true;
		  sonar_yellow = false;
	      safety_pub_.publish(safety_msg_);
	      led_pub_.publish(led_msg_);
	  } else if(sonar_alarm == true){
		  sonar_alarm = false;
		  sonar_yellow = true;
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = false;
		  safety_pub_.publish(safety_msg_);
		  set_yellow(led_msg_);
		  led_pub_.publish(led_msg_);
	  } else if(sonar_yellow == true){
		  sonar_alarm = false;
		  sonar_yellow = false;
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = false;
		  safety_pub_.publish(safety_msg_);
		  reset_LED(led_msg_);
		  led_pub_.publish(led_msg_);
	  } else {
		  sonar_alarm = false;
		  sonar_yellow = false;
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = false;
	  }
  }

  /*void rearSonarCallback (sensor_msgs::Range const& std_sonar_msg) {
	  static std::list<float> range_list;

	  float mean = push_and_mean_list(range_list, std_sonar_msg.range, sonar_max, 5);

	  ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

	  if (mean > sonar_alert_bckg && mean < sonar_max) {
		  set_yellow(led_msg_);
		  led_pub_.publish(led_msg_);
		  sonar_alarm = false;
		  sonar_yellow = true;
	  } else if(mean < sonar_alert_bckg) {
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = true;

		  set_red(led_msg_);

		  ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

		  sonar_alarm = true;
		  sonar_yellow = false;
	      safety_pub_.publish(safety_msg_);
	      led_pub_.publish(led_msg_);
	  } else if(sonar_alarm == true){
		  sonar_alarm = false;
		  sonar_yellow = true;
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = false;
		  safety_pub_.publish(safety_msg_);
		  set_yellow(led_msg_);
		  led_pub_.publish(led_msg_);
	  } else if(sonar_yellow == true){
		  sonar_alarm = false;
		  sonar_yellow = false;
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = false;
		  safety_pub_.publish(safety_msg_);
		  reset_LED(led_msg_);
		  led_pub_.publish(led_msg_);
	  } else {
		  sonar_alarm = false;
		  sonar_yellow = false;
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = false;
	  }
  }

  void leftSonarCallback (sensor_msgs::Range const& std_sonar_msg) {
	  static std::list<float> range_list;

	  float mean = push_and_mean_list(range_list, std_sonar_msg.range, sonar_max, 5);

	  ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

	  if (mean > sonar_alert_bckg && mean < sonar_max) {
		  set_yellow(led_msg_);
		  led_pub_.publish(led_msg_);
		  sonar_alarm = false;
		  sonar_yellow = true;
	  } else if(mean < sonar_alert_bckg) {
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = true;

		  set_red(led_msg_);

		  ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

		  sonar_alarm = true;
		  sonar_yellow = false;
	      safety_pub_.publish(safety_msg_);
	      led_pub_.publish(led_msg_);
	  } else if(sonar_alarm == true){
		  sonar_alarm = false;
		  sonar_yellow = true;
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = false;
		  safety_pub_.publish(safety_msg_);
		  set_yellow(led_msg_);
		  led_pub_.publish(led_msg_);
	  } else if(sonar_yellow == true){
		  sonar_alarm = false;
		  sonar_yellow = false;
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = false;
		  safety_pub_.publish(safety_msg_);
		  reset_LED(led_msg_);
		  led_pub_.publish(led_msg_);
	  } else {
		  sonar_alarm = false;
		  sonar_yellow = false;
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = false;
	  }
  }

  void rightSonarCallback (sensor_msgs::Range const& std_sonar_msg) {
	  static std::list<float> range_list;

	  float mean = push_and_mean_list(range_list, std_sonar_msg.range, sonar_max, 5);

	  ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

	  if (mean > sonar_alert_bckg && mean < sonar_max) {
		  set_yellow(led_msg_);
		  led_pub_.publish(led_msg_);
		  sonar_alarm = false;
		  sonar_yellow = true;
	  } else if(mean < sonar_alert_bckg) {
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = true;

		  set_red(led_msg_);

		  ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

		  sonar_alarm = true;
		  sonar_yellow = false;
	      safety_pub_.publish(safety_msg_);
	      led_pub_.publish(led_msg_);
	  } else if(sonar_alarm == true){
		  sonar_alarm = false;
		  sonar_yellow = true;
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = false;
		  safety_pub_.publish(safety_msg_);
		  set_yellow(led_msg_);
		  led_pub_.publish(led_msg_);
	  } else if(sonar_yellow == true){
		  sonar_alarm = false;
		  sonar_yellow = false;
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = false;
		  safety_pub_.publish(safety_msg_);
		  reset_LED(led_msg_);
		  led_pub_.publish(led_msg_);
	  } else {
		  sonar_alarm = false;
		  sonar_yellow = false;
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = false;
	  }
  }*/

  /*void rearSonarCallback (sensor_msgs::Range const& std_sonar_msg) {
	  static std::list<float> range_list;

	  float mean = push_and_mean_list(range_list, std_sonar_msg.range, 5);

	  if (mean < sonar_max && !sonar_alarm) {
		  set_yellow(led_msg_);
		  led_pub_.publish(led_msg_);
	  } else if(sonar_alarm != true) {
		  reset_LED(led_msg_);
		  led_pub_.publish(led_msg_);
	  }
  }

  void leftSonarCallback (sensor_msgs::Range const& std_sonar_msg) {
	  static std::list<float> range_list;

	  float mean = push_and_mean_list(range_list, std_sonar_msg.range, 5);

	  if (mean < sonar_max && !sonar_alarm) {
		  set_yellow(led_msg_);
		  led_pub_.publish(led_msg_);
	  } else if(sonar_alarm != true) {
		  reset_LED(led_msg_);
		  led_pub_.publish(led_msg_);
	  }
  }

  void rightSonarCallback (sensor_msgs::Range const& std_sonar_msg) {
	  static std::list<float> range_list;

	  float mean = push_and_mean_list(range_list, std_sonar_msg.range, 5);

	  if (mean < sonar_max && !sonar_alarm) {
		  set_yellow(led_msg_);
		  led_pub_.publish(led_msg_);
	  } else if(sonar_alarm != true) {
		  reset_LED(led_msg_);
		  led_pub_.publish(led_msg_);
	  }
  }*/

  protected:;
    ros::Publisher safety_pub_;
    ros::Publisher led_pub_;
    ros::Subscriber front_sonar_sub_;
    ros::Subscriber rear_sonar_sub_;
    ros::Subscriber left_sonar_sub_;
    ros::Subscriber right_sonar_sub_;

    malish::Obstacle safety_msg_;
    malish::Diode led_msg_;
    bool sonar_alarm;
    bool sonar_yellow;
};

int main(int argc, char **argv)
{
    printf("Hello, ROS1!");

    ros::init(argc, argv, "safety_node");

    ROS_INFO("Hello, ROS!");

    Safety safety_checker;

    ros::spin();
}
