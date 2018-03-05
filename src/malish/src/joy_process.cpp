// Created by Evgeny on 19.02.18.
/*!
 * \file led_strip_alarm.cpp
 *
 * Gets safety message (obstacle.msg) and publishes r,g,b values(diode.msg) to Arduino.
 *
 * \authors Evgeny Shtanov
 *
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <malish/Obstacle.h>
#include <malish/Diode.h>

static uint32_t seq_led = 0;

class AlarmLED {
  public:;
    AlarmLED() {
        ros::NodeHandle nh_;
        // Subscribe to IMU data publishing from arduino.
        obstacle_sub_ = nh_.subscribe("/safety", 10, &AlarmLED::alarmLEDCallback, this);

        led_pub_ = nh_.advertise<malish::Diode>("/led", 10);
    }

    // Publish as std IMU sensor message.
    void alarmLEDCallback (const malish::Obstacle& obstacle) {
        if(obstacle.alert == true) {
            led_msg_.red = 128;
            led_msg_.green = 128;
            led_msg_.blue = 128;
            led_pub_.publish(led_msg_);
        }
        else {
            led_msg_.red = 0;
            led_msg_.green = 0;
            led_msg_.blue = 0;
            led_pub_.publish(led_msg_);
        }
    }

  protected:;
    ros::Subscriber obstacle_sub_;
    ros::Publisher led_pub_;
    malish::Diode led_msg_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "led_alarm_node");

    AlarmLED alarm_led;

    ros::spin();
}
