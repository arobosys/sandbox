// Created by Evgeny on 19.02.18.
/*!
 * \file imu_transform.cpp
 *
 * Gets imu data from Arduino and transforms to standard ROS sensor_msgs/Imu.msg messages.
 *
 * \author Evgeny Shtanov
 *
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <malish/ArduImu.h>

#define imuFrame "imu"
static uint32_t seq_imu = 0;

class IMU_transform {
  public:;
    IMU_transform() {
        ros::NodeHandle nh_;
        // Subscribe to IMU data publishing from arduino.
        ardu_imu_sub_ = nh_.subscribe("/arduino/imu", 10, &IMU_transform::arduImuCallback, this);

        std_imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/transform", 10);

        std_imu_msg.header.frame_id = imuFrame;
    }

    // Publish as std IMU sensor message.
    void arduImuCallback (const malish::ArduImu& ardu_imu) {
        std_imu_msg.linear_acceleration.x = ardu_imu.linear_acceleration.x;
        std_imu_msg.linear_acceleration.y = ardu_imu.linear_acceleration.y;
        std_imu_msg.linear_acceleration.z = ardu_imu.linear_acceleration.z;
        std_imu_msg.angular_velocity.x = ardu_imu.angular_velocity.x;
        std_imu_msg.angular_velocity.y = ardu_imu.angular_velocity.y;
        std_imu_msg.angular_velocity.z = ardu_imu.angular_velocity.z;
        std_imu_msg.header.stamp = ardu_imu.timestamp;
        std_imu_msg.header.seq = ++seq_imu;

        std_imu_pub_.publish(std_imu_msg);
    }

  private:;
    ros::Subscriber ardu_imu_sub_;
    ros::Publisher std_imu_pub_;
    sensor_msgs::Imu std_imu_msg;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_transform_node");

    IMU_transform imu_transformer;

    ros::spin();
}
