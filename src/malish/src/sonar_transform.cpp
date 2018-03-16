/*!
 * \file sonar_transform.cpp
 *
 * Gets sonar data from Arduino and transforms to standard ROS sensor_msgs/Sonar.msg messages.
 *
 * \author Evgeny Shtanov
 *
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Range.h>
#include <malish/ArduSonar.h>

#define leftSonarFrame "sonar_left"
#define rightSonarFrame "sonar_right"
#define frontSonarFrame "sonar_front"
#define rearSonarFrame "sonar_rear"

// Consecutively increasing sequence ID.
static uint32_t sonar_seq[4] = {0};
static const float scale = 100.0;
static const float sonar_max = 1.03;
static const float front_sonar_max = 1.03;

class SonarTransform {
public:;
    SonarTransform() {
        ros::NodeHandle nh_;
        // Subscribe to Sonar data publishing from arduino.
        ardu_sonar_sub_ = nh_.subscribe("/sonars", 10, &SonarTransform::arduSonarCallback, this);

        std_sonar_left_pub_ = nh_.advertise<sensor_msgs::Range>("/sonar/left", 10);
        std_sonar_right_pub_ = nh_.advertise<sensor_msgs::Range>("/sonar/right", 10);
        std_sonar_front_pub_ = nh_.advertise<sensor_msgs::Range>("/sonar/front", 10);
        std_sonar_rear_pub_ = nh_.advertise<sensor_msgs::Range>("/sonar/rear", 10);

        std_sonar_msg.radiation_type = 0;
        std_sonar_msg.field_of_view = 0.2618;
        std_sonar_msg.min_range = 0.01;
        std_sonar_msg.max_range = 1.03;
    }
  
    // Publish as std Sonar sensor message.
    void arduSonarCallback (const malish::ArduSonar& ardu_sonar) {
        // Left
        std_sonar_msg.header.seq = ++sonar_seq[0];
        std_sonar_msg.header.stamp = ardu_sonar.timestamp;
        std_sonar_msg.header.frame_id = leftSonarFrame;
        std_sonar_msg.range = ardu_sonar.sonLeft / scale;
        std_sonar_msg.max_range = sonar_max;
        std_sonar_left_pub_.publish(std_sonar_msg);
        // Right
        std_sonar_msg.header.seq = ++sonar_seq[1];
        std_sonar_msg.header.stamp = ardu_sonar.timestamp;
        std_sonar_msg.header.frame_id = rightSonarFrame;
        std_sonar_msg.range = ardu_sonar.sonRight / scale;
        std_sonar_right_pub_.publish(std_sonar_msg);
        // Rear
        std_sonar_msg.header.seq = ++sonar_seq[3];
        std_sonar_msg.header.stamp = ardu_sonar.timestamp;
        std_sonar_msg.header.frame_id = rearSonarFrame;
        std_sonar_msg.range = ardu_sonar.sonRear / scale;
        std_sonar_rear_pub_.publish(std_sonar_msg);
        // Front
        std_sonar_msg.header.seq = ++sonar_seq[2];
        std_sonar_msg.header.stamp = ardu_sonar.timestamp;
        std_sonar_msg.header.frame_id = frontSonarFrame;
        std_sonar_msg.range = ardu_sonar.sonFront / scale;
        std_sonar_msg.max_range = front_sonar_max;
        std_sonar_front_pub_.publish(std_sonar_msg);
    }

protected:;
    ros::Subscriber ardu_sonar_sub_;
    ros::Publisher std_sonar_left_pub_;
    ros::Publisher std_sonar_right_pub_;
    ros::Publisher std_sonar_front_pub_;
    ros::Publisher std_sonar_rear_pub_;
    sensor_msgs::Range std_sonar_msg;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sonar_transform_node");

    SonarTransform sonar_transformer;

    ros::spin();
}
