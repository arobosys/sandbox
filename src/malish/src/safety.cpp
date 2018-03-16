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

// #define __DEBUG__ 1
const static int __DEBUG__ = 0;

/// Robot's width, [m].
static const float robot_width = 0.45;
/// Robot's length, [m].
static const float robot_length = 0.65;
/// Maximal sonar distance, [m].
static const float sonar_max_phisical = 1.03;
static const float sonar_max = 1.0;
static const float sonar_max_front = 1.0;
static const float sonar_alert_front = 0.65;
static const float sonar_alert_bckg = 0.15;
/// Constant for comparision with float type zero value.
static const float eps = 1e-10;
/// Queue to suppress noise in sonars range measurement.
static const unsigned int queue_size = 5;

static const unsigned char RESET = 0;
static const unsigned char FRONT_YELLOW = 1;
static const unsigned char FRONT_RED = 2;
static const unsigned char RIGHT_YELLOW = 4;
static const unsigned char RIGHT_RED = 8;
static const unsigned char REAR_YELLOW = 16;
static const unsigned char REAR_RED = 32;
static const unsigned char LEFT_YELLOW = 64;
static const unsigned char LEFT_RED = 128;
static const unsigned char ANY_YELLOW = 85;
static const unsigned char ANY_RED = 170;

enum LED_state {
  NONE = 0,
  WHITE = 1,
  YELLOW = 2,
  RED = 3
};

/* Standard operations with bitmaksk. */
unsigned char ToggleNthBit(unsigned char num, unsigned char n) {
  if(num & (1 << n))
    num &= ~(1 << n);
  else
    num |= (1 << n);

  return num;
}

unsigned char set_nth_bit(unsigned char num, unsigned char n) {

    return (num | 1 << n);
}

unsigned char clear_nth_bit(unsigned char num, unsigned char n) {

    return (num & ~( 1 << n));
}

unsigned char toggle_nth_bit(unsigned char num, unsigned char n) {

    return num ^ (1 << n);
}

unsigned char check_nth_bit(unsigned char num, unsigned char n) {

    return num & (1 << n);
}


bool check_mask(unsigned int const& mask, unsigned char flag) {
  unsigned int check_mask = 0;
  check_mask = mask & flag;

  return check_mask > 0 ? true : false;
}

void set_mask(unsigned char *mask, unsigned char flag) {
  *mask |= flag;
}

void reset_front(unsigned char *mask) {
  *mask = clear_nth_bit(*mask, 1);
  *mask = clear_nth_bit(*mask, 0);
}

void reset_right(unsigned char *mask) {
  *mask = clear_nth_bit(*mask, 3);
  *mask = clear_nth_bit(*mask, 2);
}

void reset_rear(unsigned char *mask) {
  *mask = clear_nth_bit(*mask, 5);
  *mask = clear_nth_bit(*mask, 4);
}

void reset_left(unsigned char *mask) {
  *mask = clear_nth_bit(*mask, 7);
  *mask = clear_nth_bit(*mask, 6);
}

void set_red_LED(malish::Diode & msg) {
  msg.red = 255;
  msg.green = 0;
  msg.blue = 0;
}

void set_yellow_LED(malish::Diode & msg) {
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
      sonar_yellow = false;
      mask = 0; // 0000 0000
      // 00 - white blink;
      // 01 - yellow blink;
      // 10 or 11 - reb blink
    }

    void message_sender() {
      ros::Rate rate(26);
      static unsigned char prev_state = NONE;
      if (state != prev_state) {
        safety_msg_.timestamp = ros::Time::now();
        if(check_mask(mask, ANY_RED)) {
          safety_msg_.alert = true;
          set_red_LED(led_msg_);
          safety_pub_.publish(safety_msg_);
          led_pub_.publish(led_msg_);
        } else if(check_mask(mask, ANY_YELLOW)) {
          safety_msg_.alert = false;
          set_yellow_LED(led_msg_);
          safety_pub_.publish(safety_msg_);
          led_pub_.publish(led_msg_);
        } else {
          safety_msg_.alert = false;
          reset_LED(led_msg_);
          safety_pub_.publish(safety_msg_);
          led_pub_.publish(led_msg_);
        }
      }
      //Delays untill it is time to send another message
      rate.sleep();
    }

    void frontSonarCallback (sensor_msgs::Range const& std_sonar_msg) {
      static std::list<float> range_list;

      float mean = push_and_mean_list(range_list, std_sonar_msg.range, sonar_max, queue_size);

      ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

      if (mean > sonar_alert_front && mean < sonar_max_front) {
        if (check_mask(mask, FRONT_RED)) {
          // Reset mask
          reset_front(&mask);
        }
        // Set state
        if (!check_mask(mask, ANY_RED)) {
          // Set color
          set_yellow_LED(led_msg_);
          // Reset mask
          reset_front(&mask);
          // Set mask
          set_mask(&mask, FRONT_YELLOW);
          state = YELLOW;
        }
        // Send mesage
        message_sender();
      } else if(mean < sonar_alert_front) {
        // Set color
        set_red_LED(led_msg_);

        // Set mask
        set_mask(&mask, FRONT_RED);

        // Set state
        state = RED;

        ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

        // Send mesage
        message_sender();
      } else {
        // Reset mask
        reset_front(&mask);
        // Set state
        if (!check_mask(mask, ANY_RED) && !check_mask(mask, ANY_YELLOW)) {
          state = WHITE;
        }

        ROS_INFO_COND(__DEBUG__ > 0, "mask after reset %d", mask);
        // Send message
        message_sender();
      }
    }

    void rearSonarCallback (sensor_msgs::Range const& std_sonar_msg) {
      static std::list<float> range_list;

      float mean = push_and_mean_list(range_list, std_sonar_msg.range, sonar_max, queue_size);

      ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

      if (mean > sonar_alert_bckg && mean < sonar_max) {
        if (check_mask(mask, REAR_RED)) {
          // Reset mask
          reset_front(&mask);
        }
        // Set mask
        set_mask(&mask, REAR_YELLOW);
        // Set state
        if (!check_mask(mask, ANY_RED)) {
          // Set color
          set_yellow_LED(led_msg_);
          // Reset mask
          reset_rear(&mask);
          // Set mask
          set_mask(&mask, REAR_YELLOW);
          state = YELLOW;
        }
        // Send mesage
        message_sender();
      } else if(mean < sonar_alert_bckg) {
        // Set color
        set_red_LED(led_msg_);

        // Set mask
        set_mask(&mask, REAR_RED);

        // Set state
        state = RED;

        ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

        // Send mesage
        message_sender();
      } else {
        // Reset mask
        reset_rear(&mask);
        // Set state
        if (!check_mask(mask, ANY_RED) && !check_mask(mask, ANY_YELLOW)) {
          state = WHITE;
        }

        ROS_INFO_COND(__DEBUG__ > 0, "mask after reset %d", mask);
        // Send message
        message_sender();
      }
    }

    void rightSonarCallback (sensor_msgs::Range const& std_sonar_msg) {
      static std::list<float> range_list;

      float mean = push_and_mean_list(range_list, std_sonar_msg.range, sonar_max, queue_size);

      ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

      if (mean > sonar_alert_bckg && mean < sonar_max) {
        if (check_mask(mask, RIGHT_RED)) {
          // Reset mask
          reset_front(&mask);
        }
        // Set mask
        set_mask(&mask, RIGHT_YELLOW);
        // Set state
        if (!check_mask(mask, ANY_RED)) {
          // Set color
          set_yellow_LED(led_msg_);
          // Reset mask
          reset_right(&mask);
          // Set mask
          set_mask(&mask, RIGHT_YELLOW);
          state = YELLOW;
        }
        // Send mesage
        message_sender();
      } else if(mean < sonar_alert_bckg) {
        // Set color
        set_red_LED(led_msg_);

        // Set mask
        set_mask(&mask, RIGHT_RED);

        // Set state
        state = RED;

        ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

        // Send mesage
        message_sender();
      } else {
        // Set state
        if (!check_mask(mask, ANY_RED) && !check_mask(mask, ANY_YELLOW)) {
          state = WHITE;
        }

        // Reset mask
        reset_right(&mask);

        ROS_INFO_COND(__DEBUG__ > 0, "mask after reset %d", mask);
        // Send message
        message_sender();
      }
    }

    void leftSonarCallback (sensor_msgs::Range const& std_sonar_msg) {
      static std::list<float> range_list;

      float mean = push_and_mean_list(range_list, std_sonar_msg.range, sonar_max, queue_size);

      ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

      if (mean > sonar_alert_bckg && mean < sonar_max) {
        if (check_mask(mask, LEFT_RED)) {
          // Reset mask
          reset_front(&mask);
        }
        // Set mask
        set_mask(&mask, LEFT_YELLOW);

        // Set state
        if (!check_mask(mask, ANY_RED)) {
          // Set color
          set_yellow_LED(led_msg_);
          // Reset mask
          reset_left(&mask);
          // Set mask
          set_mask(&mask, LEFT_YELLOW);
          state = YELLOW;
        }
        // Send mesage
        message_sender();
      } else if(mean < sonar_alert_bckg) {
        // Set color
        set_red_LED(led_msg_);

        // Set mask
        set_mask(&mask, LEFT_RED);

        // Set state
        state = RED;

        ROS_INFO_COND(__DEBUG__ > 0, "front sonar mean range %f", mean);

        // Send mesage
        message_sender();
      } else {
        // Reset mask
        reset_left(&mask);

        // Set state
        if (!check_mask(mask, ANY_RED) && !check_mask(mask, ANY_YELLOW)) {
          state = WHITE;
        }

        ROS_INFO_COND(__DEBUG__ > 0, "mask after reset %d", mask);

        // Send message
        message_sender();
      }
    }

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
    unsigned char mask;
    unsigned char state;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "safety_node");

  ROS_INFO("Hello, ROS!");

  Safety safety_checker;

  ros::spin();
}
