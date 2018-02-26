// Created by Evgeny on 19.02.18.
/*!
 * \file imu_transform.cpp
 *
 * Node which informs you, whee.
 *
 * \author Ostroumov Georgy
 *
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <malish/Obstacle.h>
#include <geometry_msgs/Pose.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "std_msgs/String.h"
#include <sstream>


using namespace cv;

static uint32_t seq_safety = 0;

class safety {
  public:;
    safety() {
        ros::NodeHandle nh_;
        // Subscribe to map and odometry topics
        map_sub_ = nh_.subscribe("/project/octomap", 10, &safety::mapCallback, this);
	    odom_sub_ = nh_.subscribe("/odom/filtered/sync", 10, &safety::odomCallback, this);
        safety_pub_ = nh_.advertise<malish::Obstacle>("/safety", 10);

        //Init messages
        safety_msg.alert = false;

        safety_msg.pos.orientation.w = 0;
        safety_msg.pos.orientation.x = 0;
        safety_msg.pos.orientation.y = 0;
        safety_msg.pos.orientation.z = 0;

        safety_msg.pos.position.x = 0;
        safety_msg.pos.position.y = 0;
        safety_msg.pos.position.z = 0;
    }

  // Analyze part of map and search largest obstacle
  void mapCallback (const nav_msgs::OccupancyGrid& map) {
      //Convert map to openCV
      std::vector<int8_t> Vf;

      Mat image = Mat(map.info.width, map.info.height, CV_8UC1, *map.data.data());
      //memcpy(image.data, Vf.data(), Vf.size()*sizeof(int));

      //Use openCV to find contours on map.
      std::vector<std::vector<cv::Point>> contours;
      std::vector<Vec4i> hierarchy;
      cv::findContours(image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);


      std::stringstream ss;
      ss << "hello world " << contours.size();
      ss.str();
      std::string tmp = ss.str();
      ROS_INFO("contours amount: %d", contours.size());
      //Extract largest contour.
      //Get the distance to largest contour.

  }

  void odomCallback (const nav_msgs::OccupancyGrid& odom) {
      //Check, how close is robot to an obstacle.
      //
  }

  private:;
    ros::Subscriber map_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher safety_pub_;
    malish::Obstacle safety_msg;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "safety_node");

    safety safety_checker;

    ros::spin();
}
