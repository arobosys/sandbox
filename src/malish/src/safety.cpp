// Created by Evgeny on 19.02.18.
/*!
 * \file safety.cpp
 *
 * Node which informs you if obstacles invade robot's safety vicinity.
 *
 * \author Ostroumov Georgy
 *
*/

#include <sstream>
#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/String.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <malish/Obstacle.h>
#include <geometry_msgs/Pose.h>
// #include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#define __DEBUG__ 1

// Random generator.
cv::RNG rng(12345);
//
static uint32_t seq_safety = 0;
/// Map frames per second, Hz.
static const float MFPS = 6.0;
/// Robot's width, meters.
static const float robot_width = 0.45;
/// Robot's length, meters.
static const float robot_length = 0.65;
/// Margin of safety vicinity, meters.
static const float margin = 1.25;
/// Minimal blob area to consider it as obstacle, [pixels].
static const float min_blob_area = 2.0;

// typedef std::vector<cv::Point2f> cvContour;
typedef std::vector<cv::Point> cvContour;


struct mapInfo {
  float resolution;
  geometry_msgs::Pose origin;
  unsigned int width;
  unsigned int height;
};

/*
 * Center of mass by moments.
 */
cv::Point2f center_by_moments(cvContour const& cnt) {
    cv::Moments M = cv::moments(cnt);
    float Cx = int(M.m10 / M.m00);
    float Cy = int(M.m01 / M.m00);
    return cv::Point2f(Cx, Cy);
}

/*
 * Number of nonzero pixels in image.
 */
double blob_area(cv::Mat const& img) {
    double area = 0.0;
    for(unsigned int i = 0; i < img.rows; ++i) {
        for (unsigned int j = 0; j < img.cols; ++j) {
            if(img.at<uchar>(j, i) > 0.0)
                area += 1.0;
        }
    }
    return area;
}

/*
 * Converter map to mat.
 *
 * @param mat output cv::Mat;
 * @param map input nav_msgs::OccupancyGrid.map.
 */
void map_to_mat(cv::Mat & mat, nav_msgs::OccupancyGrid const& map) {
    for(unsigned int i = 0; i < map.info.height; ++i) {
        for(unsigned int j = 0; j < map.info.width; ++j) {
            // Convert from int8 to uchar (grayscale image).
            mat.at<uchar>(j, i) = static_cast<uchar>(map.data[i * map.info.width + j]) + 1;
        }
    }

    // Normalize from [-1, 100] to [0, 255].

}

/**
 * class Safety.
 *
 *
 *
 */
class Safety {
  public:;
    Safety() {
        ros::NodeHandle nh_;
        // Subscribe to map and odometry topics.
        map_sub_ = nh_.subscribe("/rtabmap/grid_map", 10, &Safety::mapCallback, this);
	    odom_sub_ = nh_.subscribe("/odometry/filtered/sync", 10, &Safety::odomCallback, this);
        safety_pub_ = nh_.advertise<malish::Obstacle>("/safety", 10);

        // Init messages.
        safety_msg_.alert = false;

        safety_msg_.pos.orientation.w = 0.0;
        safety_msg_.pos.orientation.x = 0.0;
        safety_msg_.pos.orientation.y = 0.0;
        safety_msg_.pos.orientation.z = 0.0;

        safety_msg_.pos.position.x = 0.0;
        safety_msg_.pos.position.y = 0.0;
        safety_msg_.pos.position.z = 0.0;
    }

  /// Takes map from rtab_map's message.
  void mapCallback (const nav_msgs::OccupancyGrid& map) {
      // Read map info.
      map_info.height = map.info.height;
      map_info.width = map.info.width;
      map_info.resolution = map.info.resolution;
      map_info.origin = map.info.origin;

      // Convert map to openCV mat.
      map_image = cv::Mat(map.info.width, map.info.height, CV_8UC1);
      map_to_mat(map_image, map);

      // Binarization.
      cv::threshold(map_image, map_image, 2, 1, CV_THRESH_BINARY);

      if(__DEBUG__) {
          // Show in a window.
          cv::namedWindow("image", CV_WINDOW_NORMAL);
          cv::imshow("image", map_image * 255);
          if (cv::waitKey(1000.0 / MFPS) == 27)
              return;
          ROS_INFO("map_info.resolution %f", map_info.resolution);
      }
  }

  /// Takes odometry data and analyzes robot's safety zone on obstacle occurrence.
  void odomCallback (const nav_msgs::Odometry & odom) {
      // Robot coordinate in map's SC.
      float robo_x = (-map_info.origin.position.x + odom.pose.pose.position.x) / map_info.resolution;
      float robo_y = (-map_info.origin.position.y + odom.pose.pose.position.y) / map_info.resolution;
      ROS_INFO_COND(__DEBUG__ > 0, "robo_x: %f, robo_y: %f", robo_x, robo_y);

      // Render robot and vicinity into image with map's size.
      // Binary image with robot and safety vicinity.
      cv::Mat tmp_img = cv::Mat::zeros(map_image.size(), CV_8UC1);

      int radius = static_cast<int>((robot_length / 2.0 + margin) / map_info.resolution);
      ROS_INFO_COND(__DEBUG__ > 0, "Robot safety zone radius: %d", radius);

      cv::Scalar color = cv::Scalar(1, 1, 1);
      // Draw filled circle of safety vicinity.
      cv::circle(tmp_img, cv::Point(robo_y, robo_x), radius, color, -1, 8);

      if (__DEBUG__) {
          // Show in a window.
          cv::namedWindow("Robot", CV_WINDOW_NORMAL);
          cv::imshow("Robot", tmp_img * 255);
          if (cv::waitKey(1000.0 / MFPS) == 27)
              return;
      }

      // Find intersection map.
      cv::Mat intersect_img = cv::Mat::zeros(map_image.size(), CV_8UC1);
      cv::bitwise_and(tmp_img, map_image, intersect_img);

      if (__DEBUG__) {
          // Show in a window.
          cv::namedWindow("Intersection", CV_WINDOW_NORMAL);
          cv::imshow("Intersection", intersect_img * 255);
          if (cv::waitKey(1000.0 / MFPS) == 27)
              return;
      }

      // Find contours on intersection map.
      std::vector<cvContour> contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(intersect_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(1,1));

      // Draw contours
      cv::Mat drawing = cv::Mat::zeros(map_image.size(), CV_8UC3);
      for(int i = 0; i < contours.size(); i++) {
          cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
          cv::drawContours(drawing, contours, i, color, -1, 8, hierarchy, 0, cv::Point(1,1));
      }

      if (__DEBUG__) {
          // Show in a window.
          cv::namedWindow("Intersection", CV_WINDOW_NORMAL);
          cv::imshow("Intersection", drawing);
          if (cv::waitKey(1000.0 / MFPS) == 27)
              return;
      }

      // Analyse obstacles. Find blob with maximal area.
      double max_blob_area = 0.0;
      cvContour best_contour;

      for(unsigned int i = 0; i < contours.size(); i++) {
          cvContour approx;
          cv::approxPolyDP(contours[i], approx, 1, true);
          double area1 = cv::contourArea(approx);

          double area = cv::contourArea(contours[i]);

          area = area > area1 ? area : area1;

          // cv::Mat obstacles = cv::Mat::zeros(map_image.size(), CV_8UC3);
          // cv::Scalar color = cv::Scalar(1, 1, 1);
          // cv::drawContours(obstacles, contours[i], 0, color, CV_FILLED);

          // Show in a window.
          // cv::namedWindow( "obstacles", CV_WINDOW_NORMAL);
          // cv::imshow("obstacles", obstacles * 255);
          // if(cv::waitKey(1000.0 / MFPS) == 27)
              // return;

          // cv::Mat obstacles_bw = cv::Mat::zeros(map_image.size(), CV_8UC1);
          // cv::threshold(obstacles, obstacles_bw, 0.5, 1, CV_THRESH_BINARY);
          double area2 = blob_area(intersect_img);

          ROS_INFO_COND(__DEBUG__ > 0, "area = %f, area1 = %f, area2 = %f, approx poly vertices = %d", area, area1, area2, approx.size());

          area = area > area2 ? area : area2;

          // Show in a window.
          // cv::namedWindow( "maxObsatcle", CV_WINDOW_NORMAL);
          // cv::imshow("maxObsatcle", obstacles * 255);
          // if(cv::waitKey(1000.0 / MFPS) == 27)
             // return;

          if(area > max_blob_area) {
              max_blob_area = area;
              best_contour = contours[i];
          }
      }

      ROS_INFO_COND(__DEBUG__ > 0, "max_blob_area: %f", max_blob_area);

      // Send alarm message in case of obstacle occurrence.
      if(max_blob_area > min_blob_area) {
          // Find center of blob in map's SC.
          // cv::Point2f center_blob = center_by_moments(best_contour);
          cv::Point2f center_blob;
          float blob_radius;
          minEnclosingCircle(best_contour, center_blob, blob_radius);
          center_blob = center_blob;

          ROS_INFO_COND(__DEBUG__ > 0, "center_blob.x: %f, center_blob.y: %f", center_blob.x, center_blob.y);

          // Fill out safety message form.
          safety_msg_.alert = true;

          // Angle between robot and obstacle.
          float angle = atan2(center_blob.y - robo_y, center_blob.x - robo_x);
          ROS_INFO_COND(__DEBUG__ > 0, "angle = %f", angle * 180 / 3.14);
          tf::Quaternion q_tf;
          q_tf = tf::createQuaternionFromRPY(angle, 0.0, 0.0);

          safety_msg_.pos.orientation.w = q_tf.getW();
          safety_msg_.pos.orientation.x = q_tf.getX();
          safety_msg_.pos.orientation.y = q_tf.getY();
          safety_msg_.pos.orientation.z = q_tf.getZ();

          safety_msg_.pos.position.x = (center_blob.x - robo_x) * map_info.resolution;
          safety_msg_.pos.position.y = (center_blob.y - robo_y) * map_info.resolution;
          safety_msg_.pos.position.z = 0.0;

          // Publish safety alarm message.
          safety_pub_.publish(safety_msg_);
      }
      else if(safety_msg_.alert == true) {
          // Reset message.
          safety_msg_.alert = false;

          safety_msg_.pos.orientation.w = 0.0;
          safety_msg_.pos.orientation.x = 0.0;
          safety_msg_.pos.orientation.y = 0.0;
          safety_msg_.pos.orientation.z = 0.0;

          safety_msg_.pos.position.x = 0.0;
          safety_msg_.pos.position.y = 0.0;
          safety_msg_.pos.position.z = 0.0;

          // Publish safety alarm message.
          safety_pub_.publish(safety_msg_);
      }
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
