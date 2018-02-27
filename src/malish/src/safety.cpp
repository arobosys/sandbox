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
#include <ros/console.h>
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


// using namespace cv;

cv::RNG rng(12345);
//
static uint32_t seq_safety = 0;
// Map frames per second, Hz.
static const float MFPS = 6;
// Robot's width, meters.
static const float robot_width = 0.45;
// Robot's length, meters.
static const float robot_length = 0.65;
// Margin of safety vicinity, meters.
static const float margin = 1;

struct mapInfo {
  float resolution;
  geometry_msgs::Pose origin;
  unsigned int width;
  unsigned int height;
};

void map_to_mat(cv::Mat & mat, const nav_msgs::OccupancyGrid & map) {
    for(unsigned int i = 0; i < map.info.height; ++i) {
        for(unsigned int j = 0; j < map.info.width; ++j) {
            // Convert from int8 to uchar(grayscale image).
            mat.at<uchar>(j, i) = static_cast<uchar> (map.data[i * map.info.width + j]) + 1;
        }
    }

    // Normalize from [-1, 100] to [0, 255].

}

class Safety {
  public:;
    Safety() {
        ros::NodeHandle nh_;
//        float safe_zone = 1.25;
//        if (n.getParam("safe_zone", s))
//        {
//            ROS_INFO("Got param: %s", s.c_str());
//        }
//        else
//        {
//            ROS_ERROR("Failed to get param 'my_param'");
//        }
//        nh_.getParam("safe_zone", safe_zone);
        // Subscribe to map and odometry topics
        map_sub_ = nh_.subscribe("/rtabmap/grid_map", 10, &Safety::mapCallback, this);
	    odom_sub_ = nh_.subscribe("/odometry/filtered/sync", 10, &Safety::odomCallback, this);
        safety_pub_ = nh_.advertise<malish::Obstacle>("/safety", 10);

        //Init messages
        safety_msg_.alert = false;

        safety_msg_.pos.orientation.w = 0;
        safety_msg_.pos.orientation.x = 0;
        safety_msg_.pos.orientation.y = 0;
        safety_msg_.pos.orientation.z = 0;

        safety_msg_.pos.position.x = 0;
        safety_msg_.pos.position.y = 0;
        safety_msg_.pos.position.z = 0;
    }

  // Analyze part of map and search largest obstacle
  void mapCallback (const nav_msgs::OccupancyGrid& map) {
      /// Reaf map info.
      map_info.height = map.info.height;
      map_info.width = map.info.width;
      map_info.resolution = map.info.resolution;
      map_info.origin = map.info.origin;

      /// Convert map to openCV mat.
      map_image = cv::Mat(map.info.width, map.info.height, CV_8UC1);
      map_to_mat(map_image, map);

      cv::threshold(map_image, map_image, 2, 1, CV_THRESH_BINARY);

      /// Use openCV to find contours on map.
      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(map_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

      /// Draw contours
      cv::Mat drawing = cv::Mat::zeros(map_image.size(), CV_8UC3);
      for(int i = 0; i < contours.size(); i++) {
          cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
          cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
      }

      ROS_INFO("contours amount: %d", contours.size());

      /// Show in a window.
      cv::namedWindow( "image", CV_WINDOW_NORMAL);
      cv::imshow("image", map_image * 255);
      if(cv::waitKey(1000.0 / MFPS) == 27)
          return;

      /// Show in a window
      cv::namedWindow("Contours", CV_WINDOW_NORMAL);
      cv::imshow("Contours", drawing);
      if(cv::waitKey(1000.0 / MFPS) == 27)
          return;
  }

  void odomCallback (const nav_msgs::Odometry & odom) {
      // Check, how close is robot to an obstacle.
      float robo_x = ( - map_info.origin.position.x + odom.pose.pose.position.x) / map_info.resolution;
      float robo_y = ( - map_info.origin.position.y + odom.pose.pose.position.y) / map_info.resolution;

      ROS_INFO("odom.pose.pose.position.x: %f, odom.pose.pose.position.y: %f", odom.pose.pose.position.x, odom.pose.pose.position.y);
      ROS_INFO("map_info.origin.position.x: %f, map_info.origin.position.y: %f", map_info.origin.position.x, map_info.origin.position.y);
      ROS_INFO("robo_x: %f, robo_y: %f", robo_x, robo_y);
      ROS_INFO("map_info.resolution %f", map_info.resolution);

      // Render robot and vicinity into map's sized image.
      // Binary image with robot
      cv::Mat tmp_img = cv::Mat::zeros(map_image.size(), CV_8UC1);

      // tmp_img
      int radius = static_cast<int>((robot_length / 2.0 + margin) / map_info.resolution);
      cv::Scalar color = cv::Scalar(1, 1, 1);
      // Drae filled circle of safety vicinity.
      cv::circle(tmp_img, cv::Point(robo_y, robo_x), radius, color, -1, 8);

      /// Show in a window.
      cv::namedWindow( "Robot", CV_WINDOW_NORMAL);
      cv::imshow("Robot", tmp_img * 255);
      if(cv::waitKey(1000.0 / MFPS) == 27)
          return;

      // Find intersection.
      // cv::Mat intersect_map = tmp_img * map_image;
      cv::bitwise_and(tmp_img, map_image, tmp_img);

      /// Show in a window.
      cv::namedWindow( "Intersection", CV_WINDOW_NORMAL);
      cv::imshow("Intersection", tmp_img * 255);
      if(cv::waitKey(1000.0 / MFPS) == 27)
          return;

      // Analyse obstacles.

      // Fill out safety message form.
      // safety_msg_

      // Publish safety alarm message.
      safety_pub_.publish(safety_msg_);
  }

  protected:;
    ros::Subscriber map_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher safety_pub_;
    malish::Obstacle safety_msg_;
    cv::Mat map_image;
    mapInfo map_info;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "safety_node");

    Safety safety_checker;

    ros::spin();
}
