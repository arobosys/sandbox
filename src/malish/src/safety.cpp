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

// Random generator.
cv::RNG rng(12345);
/// Map frames per second, [Hz].
static const float MFPS = 6.0;
/// Robot's width, [m].
static const float robot_width = 0.45;
/// Robot's length, [m].
static const float robot_length = 0.65;
/// Margin of safety vicinity, [m].
static const float alert_margin = 0.05;
/// Margin of vicinity to light LED in yellow (notification), [m].
static const float yellow_margin = 1.25;
/// Maximal sonar distance.
static const float sonar_max_phisical = 1.03;
static const float sonar_max = 1.0;
static const float sonar_alert = 0.85;
/// Minimal blob area to consider it as obstacle, [pixels].
static const float min_blob_area = 20.0;
/// Constant for comparision with float type zero value.
static const float eps = 1e-10;

typedef std::vector<cv::Point> cvContour;

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
float push_and_mean_list(std::list<float> & queue, float val, unsigned int size=5) {
	static unsigned int qsize = size;
	float mean = 0.0;

	queue.push_back(val);

	if(queue.size() == qsize) {
		queue.pop_front();
	} else if (queue.size() > qsize) {
		while(queue.size() > qsize) {
			queue.pop_front();
		}
	}

	for (std::list<float>::iterator it=queue.begin(); it != queue.end(); ++it) {
		mean += *it;
	}

	return mean / queue.size();
}

/**
 * mapInfo - short version of nav_msgs::OccupancyGrid's map.info field.
 */
struct mapInfo {
  /// Map's resolution(size of pixel) [m].
  float resolution;
  /// Origin of map's SC [m, m, rad].
  geometry_msgs::Pose origin;
  /// Size of map as image.
  unsigned int width;
  unsigned int height;
};

/*
 * Center of mass by moments.
 */
cv::Point2f
center_by_moments(cvContour const& cnt) {
    cv::Moments M = cv::moments(cnt);
    float Cx = int(M.m10 / M.m00);
    float Cy = int(M.m01 / M.m00);
    return cv::Point2f(Cx, Cy);
}

/*
 * Number of nonzero pixels in image.
 */
double
blob_area(cv::Mat const& img) {
    double area = 0.0;
    for(unsigned int i = 0; i < img.rows; ++i) {
        for (unsigned int j = 0; j < img.cols; ++j) {
            if(img.at<uchar>(j, i) > 0)
                area += 1.0;
        }
    }
    return area;
}

/*
 * Converter map to mat.
 *
 * @param mat output cv::Mat, range [0, 101];
 * @param map input nav_msgs::OccupancyGrid.map, range [-1, 100].
 */
void
map_to_mat(cv::Mat & mat, nav_msgs::OccupancyGrid const& map) {
    for(unsigned int i = 0; i < map.info.height; ++i) {
        for(unsigned int j = 0; j < map.info.width; ++j) {
            // Convert from int8 to uchar (grayscale image).
            mat.at<uchar>(j, i) = static_cast<uchar>(map.data[i * map.info.width + j]) + 1;
        }
    }
}

/*
 * Find blob with maximal area. Returns its area.
 *
 * @param max_contour output largest contour;
 * @param contours input set of contours;
 * @return max blob's area.
 */
double
find_largest_blob(cvContour & max_contour, std::vector<cvContour> const& contours, cv::Mat const& intersect_img) {
    double max_blob_area = 0.0;

    for(unsigned int i = 0; i < contours.size(); i++) {
        cvContour approx;
        cv::approxPolyDP(contours[i], approx, 1, true);
        double area1 = cv::contourArea(approx);

        double area = cv::contourArea(contours[i]);

        area = area > area1 ? area : area1;

        double area2 = blob_area(intersect_img);

        ROS_INFO_COND(__DEBUG__ > 0, "area = %f, area1 = %f, area2 = %f, approx poly vertices = %d",
                      area, area1, area2, (int)approx.size());

        area = area > area2 ? area : area2;

        if(area > max_blob_area) {
            max_blob_area = area;
            max_contour = contours[i];
        }
    }

    return max_blob_area;
}

cv::Mat
sector_map(cv::Mat map, cv::Point center, unsigned int radius, int direction, unsigned int angle_range, const cv::Scalar& color) {
    ellipse(map, center, cv::Size(radius, radius), direction, direction + angle_range / 2, direction - angle_range / 2, color);

    return map;
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
        // Subscribe to map and odometry topics.
        // map_sub_ = nh_.subscribe("/rtabmap/grid_map", 10, &Safety::mapCallback, this);
	    // odom_sub_ = nh_.subscribe("/odometry/filtered/sync", 10, &Safety::odomCallback3, this);
        safety_pub_ = nh_.advertise<malish::Obstacle>("/safety", 10);
        led_pub_ = nh_.advertise<malish::Diode>("/led", 10);

        // Subscribe to Sonar's data.
        front_sonar_sub_ = nh_.subscribe("/sonar/front", 10, &Safety::frontSonarCallback, this);
        rear_sonar_sub_ = nh_.subscribe("/sonar/rear", 10, &Safety::rearSonarCallback, this);
        left_sonar_sub_ = nh_.subscribe("/sonar/left", 10, &Safety::leftSonarCallback, this);
        right_sonar_sub_ = nh_.subscribe("/sonar/right", 10, &Safety::rightSonarCallback, this);

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
    }

  /// Takes map from rtab_map's message.
  /*void mapCallback (const nav_msgs::OccupancyGrid& map) {
	  ROS_INFO_COND(__DEBUG__ > 0, "map_info.resolution %f", map_info.resolution);
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
          // ROS_INFO_COND(__DEBUG__ > 0, "map_info.resolution %f", map_info.resolution);
      }
  }*/

  void frontSonarCallback (sensor_msgs::Range const& std_sonar_msg) {
	  static std::list<float> range_list;

	  float mean = push_and_mean_list(range_list, std_sonar_msg.range, 5);

	  ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

	  if (mean > sonar_alert && mean < sonar_max) {
		  set_yellow(led_msg_);
		  led_pub_.publish(led_msg_);
		  sonar_alarm = false;
	  } else if(mean < sonar_alert) {
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = true;

		  set_red(led_msg_);

		  ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

		  sonar_alarm = true;
	      safety_pub_.publish(safety_msg_);
	      led_pub_.publish(led_msg_);
	  } else if(sonar_alarm == true){
		  sonar_alarm = false;
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = false;
		  safety_pub_.publish(safety_msg_);
		  reset_LED(led_msg_);
		  led_pub_.publish(led_msg_);
	  } else {
		  sonar_alarm = false;
		  safety_msg_.timestamp = ros::Time::now();
		  safety_msg_.alert = false;
	  }
  }

  void rearSonarCallback (sensor_msgs::Range const& std_sonar_msg) {
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
  }

  /// Takes map from rtab_map's message.
  /*void mapCallback_costmap2d_ROS (const nav_msgs::OccupancyGrid& map) {
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
          ROS_INFO_COND(__DEBUG__ > 0, "map_info.resolution %f", map_info.resolution);
      }
  }*/

  /// Takes odometry data and analyzes robot's safety zone on obstacle occurrence.
  /*void odomCallback (const nav_msgs::Odometry & odom) {
      if(map_info.resolution < eps) {
          ROS_WARN("Warning: empty rtab map.");
          return;
      }
      // Robot coordinate in map's SC.
      float robo_x = (-map_info.origin.position.x + odom.pose.pose.position.x) / map_info.resolution;
      float robo_y = (-map_info.origin.position.y + odom.pose.pose.position.y) / map_info.resolution;
      // ROS_INFO_COND(__DEBUG__ > 0, "robo_x: %f, robo_y: %f", robo_x, robo_y);

      // Render robot and vicinity into image with map's size.
      // Binary image with robot and safety vicinity.
      cv::Mat tmp_img = cv::Mat::zeros(map_image.size(), CV_8UC1);

      int radius = static_cast<int>((robot_length / 2.0 + alert_margin) / map_info.resolution);
      // ROS_INFO_COND(__DEBUG__ > 0, "Robot safety zone radius: %d", radius);

      // Draw filled circle of safety vicinity.
      cv::Scalar color = cv::Scalar(1, 1, 1);
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

      // Find external contours on intersection map.
      std::vector<cvContour> contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(intersect_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(1,1));

      if (__DEBUG__) {
          // Draw contours
          cv::Mat drawing = cv::Mat::zeros(map_image.size(), CV_8UC3);
          for(int i = 0; i < contours.size(); i++) {
              cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
              cv::drawContours(drawing, contours, i, color, -1, 8, hierarchy, 0, cv::Point(1,1));
          }

          // Show in a window.
          cv::namedWindow("Intersection", CV_WINDOW_NORMAL);
          cv::imshow("Intersection", drawing);
          if (cv::waitKey(1000.0 / MFPS) == 27)
              return;
      }

      // Analyse obstacles. Find blob with maximal area.
      double max_blob_area = 0.0;
      cvContour best_contour;
      max_blob_area = find_largest_blob(best_contour, contours, intersect_img);

      ROS_INFO_COND(__DEBUG__ > 0, "max_blob_area: %f", max_blob_area);

      // Send alarm message in case of obstacle occurrence.
      if(max_blob_area > min_blob_area) {
          // Find center of blob in map's SC.
          cv::Point2f center_blob;
          float blob_radius;
          // center_blob = center_by_moments(best_contour);
          minEnclosingCircle(best_contour, center_blob, blob_radius);

          ROS_INFO_COND(__DEBUG__ > 0, "center_blob.x: %f, center_blob.y: %f", center_blob.x, center_blob.y);

          // Fill out safety message form.
          safety_msg_.timestamp = ros::Time::now();
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
          // LED strip lightening.
          set_red(led_msg_);
      }
      else if(safety_msg_.alert == true) {
          // Reset message.
    	  safety_msg_.timestamp = ros::Time::now();
          safety_msg_.alert = false;
          safety_msg_.pos.orientation.w = 0.0;
          safety_msg_.pos.orientation.x = 0.0;
          safety_msg_.pos.orientation.y = 0.0;
          safety_msg_.pos.orientation.z = 0.0;
          safety_msg_.pos.position.x = 0.0;
          safety_msg_.pos.position.y = 0.0;
          safety_msg_.pos.position.z = 0.0;
          // LED strip resetting.
          reset_LED(led_msg_);
      }
      // Publish safety alarm message.
      safety_pub_.publish(safety_msg_);
      led_pub_.publish(led_msg_);
  }

    // Seek obstacles in tmp_img.
    bool seek_zone(cv::Mat const& tmp_img, nav_msgs::Odometry const& odom, cv::Point robot_c) {
        // Find intersection map.
        cv::Mat intersect_img = cv::Mat::zeros(map_image.size(), CV_8UC1);
        cv::bitwise_and(tmp_img, map_image, intersect_img);

        if (__DEBUG__) {
            // Show in a window.
            cv::namedWindow("Intersection", CV_WINDOW_NORMAL);
            cv::imshow("Intersection", intersect_img * 255);
            if (cv::waitKey(1000.0 / MFPS) == 27)
                return false;
        }

        // Find external contours on intersection map.
        std::vector<cvContour> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(intersect_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(1,1));

        if (__DEBUG__) {
            // Draw contours
            cv::Mat drawing = cv::Mat::zeros(map_image.size(), CV_8UC3);
            for(int i = 0; i < contours.size(); i++) {
                cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
                cv::drawContours(drawing, contours, i, color, -1, 8, hierarchy, 0, cv::Point(1,1));
            }

            // Show in a window.
            cv::namedWindow("Intersection contours", CV_WINDOW_NORMAL);
            cv::imshow("Intersection contours", drawing);
            if (cv::waitKey(1000.0 / MFPS) == 27)
                return false;
        }

        // Analyse obstacles. Find blob with maximal area.
        double max_blob_area = 0.0;
        cvContour best_contour;
        max_blob_area = find_largest_blob(best_contour, contours, intersect_img);

        ROS_INFO_COND(__DEBUG__ > 0, "max_blob_area: %f", max_blob_area);

        // Send alarm message in case of an obstacle occurrence.
        if((max_blob_area > min_blob_area)) {
            // Find center of blob in map's SC.
            cv::Point2f center_blob;
            float blob_radius;
            // center_blob = center_by_moments(best_contour);
            minEnclosingCircle(best_contour, center_blob, blob_radius);

            ROS_INFO_COND(__DEBUG__ > 0, "center_blob.x: %f, center_blob.y: %f", center_blob.x, center_blob.y);

            // Fill out safety message form.
            safety_msg_.timestamp = ros::Time::now();
            safety_msg_.alert = true;

            // Angle between robot and obstacle.
            float angle = atan2(center_blob.y - robot_c.y, center_blob.x - robot_c.x);
            ROS_INFO_COND(__DEBUG__ > 0, "angle = %f", angle * 180 / 3.14);
            tf::Quaternion q_tf;
            q_tf = tf::createQuaternionFromRPY(angle, 0.0, 0.0);

            safety_msg_.pos.orientation.w = q_tf.getW();
            safety_msg_.pos.orientation.x = q_tf.getX();
            safety_msg_.pos.orientation.y = q_tf.getY();
            safety_msg_.pos.orientation.z = q_tf.getZ();

            safety_msg_.pos.position.x = (center_blob.x - robot_c.x) * map_info.resolution;
            safety_msg_.pos.position.y = (center_blob.y - robot_c.y) * map_info.resolution;
            safety_msg_.pos.position.z = 0.0;
        }
        else if(safety_msg_.alert == true) {
            // Reset message.
        	safety_msg_.timestamp = ros::Time::now();
            safety_msg_.alert = false;
            safety_msg_.pos.orientation.w = 0.0;
            safety_msg_.pos.orientation.x = 0.0;
            safety_msg_.pos.orientation.y = 0.0;
            safety_msg_.pos.orientation.z = 0.0;
            safety_msg_.pos.position.x = 0.0;
            safety_msg_.pos.position.y = 0.0;
            safety_msg_.pos.position.z = 0.0;
        }

        return safety_msg_.alert;
    }

    /// Takes odometry data and analyzes robot's safety zone on obstacle occurrence.
    void odomCallback3 (const nav_msgs::Odometry & odom) {
        if(map_info.resolution < eps) {
            ROS_WARN("Warning: empty rtab map.");
            return;
        }
        // Robot coordinate in map's SC.
        float robo_x = (-map_info.origin.position.x + odom.pose.pose.position.x) / map_info.resolution;
        float robo_y = (-map_info.origin.position.y + odom.pose.pose.position.y) / map_info.resolution;
        // Robot orientation in map's SC.
        double robo_yaw = tf::getYaw (odom.pose.pose.orientation);
        ROS_INFO_COND(__DEBUG__ > 0, "robo_x: %f, robo_y: %f, yaw = %f", robo_x, robo_y, robo_yaw);

        // Binary image with robot and safety vicinity.
        cv::Mat tmp_img = cv::Mat::zeros(map_image.size(), CV_8UC1);
        // Render robot and vicinity into image with map's size.
        int radius = static_cast<int>((robot_length / 2.0 + alert_margin) / map_info.resolution);
        // ROS_INFO_COND(__DEBUG__ > 0, "Robot red safety zone radius: %d", radius);

        // Draw filled circle of safety vicinity.
        cv::Scalar color = cv::Scalar(1, 1, 1);
        // tmp_img = sector_map(tmp_img, cv::Point(robo_y, robo_x), radius, robo_yaw, 60 * 3.14 / 180, color);
        cv::circle(tmp_img, cv::Point(robo_y, robo_x), radius, color, -1, 8);

        if (__DEBUG__) {
            // Show in a window.
            cv::namedWindow("Red", CV_WINDOW_NORMAL);
            cv::imshow("Red", tmp_img * 255);
            if (cv::waitKey(1000.0 / MFPS) == 27)
                return;
        }

        // If sonar give alert than send alarm message
        if(sonar_alarm == true) {
        	led_pub_.publish(led_msg_);
        	safety_pub_.publish(safety_msg_);
        }
        // Look for red zone.
        else if(!seek_zone(tmp_img, odom, cv::Point(robo_y, robo_x))) {
            radius = static_cast<int>((robot_length / 2.0 + yellow_margin) / map_info.resolution);
            // ROS_INFO_COND(__DEBUG__ > 0, "Robot safety zone radius: %d", radius);

            // Draw filled circle of safety vicinity.
            cv::circle(tmp_img, cv::Point(robo_y, robo_x), radius, color, -1, 8);

            if (__DEBUG__) {
                // Show in a window.
                cv::namedWindow("Yellow", CV_WINDOW_NORMAL);
                cv::imshow("Yellow", tmp_img * 255);
                if (cv::waitKey(1000.0 / MFPS) == 27)
                    return;
            }

            // Look for yellow zone if red is empty.
            if(!seek_zone(tmp_img, odom, cv::Point(robo_y, robo_x))) {
            	reset_LED(led_msg_);
            }
            else {
                // LED strip notification lightening.
            	set_yellow(led_msg_);
            }
            if(safety_msg_.alert == true) {
                // Reset message.
            	safety_msg_.timestamp = ros::Time::now();
                safety_msg_.alert = false;
                safety_msg_.pos.orientation.w = 0.0;
                safety_msg_.pos.orientation.x = 0.0;
                safety_msg_.pos.orientation.y = 0.0;
                safety_msg_.pos.orientation.z = 0.0;
                safety_msg_.pos.position.x = 0.0;
                safety_msg_.pos.position.y = 0.0;
                safety_msg_.pos.position.z = 0.0;
                safety_pub_.publish(safety_msg_);
            }
        }
        else { // If obstacle in red zone.
            // LED strip alert lightening.
        	set_red(led_msg_);
            // Publish safety alarm message.
            safety_pub_.publish(safety_msg_);
        }

        // Publish LED lightening message.
        led_pub_.publish(led_msg_);
    }*/

  protected:;
    ros::Subscriber map_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher safety_pub_;
    ros::Publisher led_pub_;
    ros::Subscriber front_sonar_sub_;
    ros::Subscriber rear_sonar_sub_;
    ros::Subscriber left_sonar_sub_;
    ros::Subscriber right_sonar_sub_;

    malish::Obstacle safety_msg_;
    malish::Diode led_msg_;
    cv::Mat map_image;
    mapInfo map_info;
    bool sonar_alarm;
};

int main(int argc, char **argv)
{
    printf("Hello, ROS1!");

    ros::init(argc, argv, "safety_node");

    ROS_INFO("Hello, ROS!");

    Safety safety_checker;

    ros::spin();
}
