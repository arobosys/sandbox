// Library calculate odometry from wheel encoder data

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "malish/ArduOdom.h"

#include <sstream>

class OmniWheelOdometry
{
public:

    OmniWheelOdometry()
    {
        vx = 0.0;
        vy = 0.0;
        vth = 0.0;

        x = 0.0;
        y = 0.0;
        th = 0.0;

        ros::NodeHandle nh_;

        odom_sub_ = nh_.subscribe("ardu_odom", 10, &OmniWheelOdometry::odomCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("omni/odom", 10);

        //TODO:'"move param to constructor setable"
        //Robot parametres
        //Robot is symmetric, dimention in metres
        rwheel = 0.07;
        lx = 0.2;
        ly = 0.3;

        if (nh_.getParam("rwheel", rwheel))
        {
            ROS_INFO("Got omni param(rwheel): %f", rwheel);
        }
        else
        {
            ROS_WARN("Failed to get param 'omni_params', using default: %f", rwheel);
        }
        if (nh_.getParam("lx", lx))
        {
            ROS_INFO("Got omni param(rwheel): %f", lx);
        }
        else
        {
            ROS_WARN("Failed to get param 'omni_params', using default: %f", lx);
        }
        if (nh_.getParam("ly", ly))
        {
            ROS_INFO("Got omni param(rwheel): %f", ly);
        }
        else
        {
            ROS_WARN("Failed to get param 'omni_params', using default: %f", ly);
        }
    }

    void odomCallback (const malish::ArduOdom& wheel)
    {
        //Converting matrix
        vx = 0.25*rwheel*(wheel.wfl + wheel.wfr + wheel.wrl + wheel.wrr  );
        vy = 0.25*rwheel*(-wheel.wfl + wheel.wfr + wheel.wrl - wheel.wrr );
        vth = 0.25*rwheel/(lx+ly)*(-wheel.wfl + wheel.wfr - wheel.wrl + wheel.wrr);
        //vth = ((lx+ly))/rwheel; size = scan_in->intensities.size();
        //compute odometry in a typical way given the velocities of the robot

        current_time = wheel.timestamp;
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
#ifdef __ODOM_BROADCASTER
        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);
#endif
        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
#ifdef __ODOM_BROADCASTER
        odom.header.frame_id = "odom";
#else
        odom.header.frame_id = "base_link";
#endif

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
	odom.pose.covariance =  boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
                                                       (0) (1e-3)  (0)  (0)  (0)  (0)
                                                       (0)   (0)  (1e6) (0)  (0)  (0)
                                                       (0)   (0)   (0) (1e6) (0)  (0)
                                                       (0)   (0)   (0)  (0) (1e6) (0)
                                                       (0)   (0)   (0)  (0)  (0)  (0.03) ; // 1e3


        //set the velocity
#ifdef __ODOM_BROADCASTER
        odom.child_frame_id = "base_link";
#else
        odom.child_frame_id = "omni_odom";
#endif
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
	odom.twist.covariance =  boost::assign::list_of(1e-3) (0)   (0)  (0)  (0)  (0)
                                                      (0) (1e-3)  (0)  (0)  (0)  (0)
                                                      (0)   (0)  (1e6) (0)  (0)  (0)
                                                      (0)   (0)   (0) (1e6) (0)  (0)
                                                      (0)   (0)   (0)  (0) (1e6) (0)
                                                      (0)   (0)   (0)  (0)  (0)  (0.03) ;

        odom_pub_.publish(odom);

        last_time = current_time;
    }

protected:
    ros::Subscriber odom_sub_;
    ros::Publisher odom_pub_;

    float rwheel, lx, ly;

    double vx, vy, vth;
    double x, y, th;

    ros::Time current_time, last_time;
#ifdef __ODOM_BROADCASTER
    tf::TransformBroadcaster odom_broadcaster;
#endif
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wheel_odometry_node");

    OmniWheelOdometry omni;

    ros::spin();
}