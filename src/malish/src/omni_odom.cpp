// Library calculate odometry from wheel encoder data

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "malish/ArduOdom.h"
#include <boost/assign.hpp>

#include "XmlRpcException.h"

//#define __OMNI_ODOM_DEBUG
//#define __ODOM_BROADCASTER

const int COV_SIZE = 36;

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
#ifdef __OMNI_ODOM_DEBUG
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("omni/odom_debug", 10);
#else
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("omni/odom", 10);
#endif

        //TODO:'"move param to constructor setable"
        //Robot parametres
        //Robot is symmetric, dimention in metres
        rwheel = 0.05;
        lx = 0.245;
        ly = 0.1925;
        time_not_init = true;

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
            ROS_INFO("Got omni param(lx): %f", lx);
        }
        else
        {
            ROS_WARN("Failed to get param 'omni_params', using default: %f", lx);
        }
        if (nh_.getParam("ly", ly))
        {
            ROS_INFO("Got omni param(ly): %f", ly);
        }
        else
        {
            ROS_WARN("Failed to get param 'omni_params', using default: %f", ly);
        }

        XmlRpc::XmlRpcValue cov_matrix_param;

        try
        {
            nh_.getParam("omni_odom/covariance", cov_matrix_param);

            ROS_ASSERT(cov_matrix_param.getType() == XmlRpc::XmlRpcValue::TypeArray);

            if (cov_matrix_param.size() != COV_SIZE)
            {
                ROS_WARN_STREAM("Configuration vector for covariance should have 36 entries.");
            }

            for (int i = 0; i < cov_matrix_param.size(); i++)
            {
                cov_matrix[i] = static_cast<double>(cov_matrix_param[i]);
            }
        }
        catch (XmlRpc::XmlRpcException &e)
        {
            ROS_FATAL_STREAM("Could not read covariance matrix" <<
                        " (type: " << cov_matrix_param.getType() << ", expected: " << XmlRpc::XmlRpcValue::TypeArray
                        << "). Error was " << e.getMessage() << "\n");
        }
    }

    void odomCallback (const malish::ArduOdom& wheel)
    {
        if (time_not_init)
        {
            last_time = wheel.timestamp;
            time_not_init = false;
            return;
        }
        //Converting matrix
        vx = 0.25*rwheel*(-wheel.wfl + wheel.wfr - wheel.wrl + wheel.wrr );         // ++++
        vy = 0.25*rwheel*( -wheel.wfl - wheel.wfr + wheel.wrl + wheel.wrr );         // -++-
        vth = 0.25*rwheel/(lx+ly)*(-wheel.wfl - wheel.wfr - wheel.wrl - wheel.wrr ); // -+-+
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
//#ifdef __ODOM_BROADCASTER
        odom.header.frame_id = "odom";
//#else
//        odom.header.frame_id = "map";
//#endif

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.pose.covariance = cov_matrix;
	    /*odom.pose.covariance =  boost::assign::list_of (1e-3) (0) (0)  (0)  (0)  (0)
                                                       (0) (1e-3)  (0)  (0)  (0)  (0)
                                                       (0)   (0)  (1e6) (0)  (0)  (0)
                                                       (0)   (0)   (0) (1e6) (0)  (0)
                                                       (0)   (0)   (0)  (0) (1e6) (0)
                                                       (0)   (0)   (0)  (0)  (0)  (0.03) ; // 1e3*/


        //set the velocity
//#ifdef __ODOM_BROADCASTER
        odom.child_frame_id = "base_link";
//#else
//        odom.child_frame_id = "omni_odom";
//#endif
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
	    odom.twist.covariance = cov_matrix;
	    /*odom.twist.covariance =  boost::assign::list_of (1e-3) (0)   (0)  (0)  (0)  (0)
                                                      (0) (1e-3)  (0)  (0)  (0)  (0)
                                                      (0)   (0)  (1e6) (0)  (0)  (0)
                                                      (0)   (0)   (0) (1e6) (0)  (0)
                                                      (0)   (0)   (0)  (0) (1e6) (0)
                                                      (0)   (0)   (0)  (0)  (0)  (0.03) ;*/

        odom_pub_.publish(odom);

        last_time = current_time;
    }

protected:
    ros::Subscriber odom_sub_;
    ros::Publisher odom_pub_;

    float rwheel, lx, ly;

    double vx, vy, vth;
    double x, y, th;
    bool time_not_init;

    ros::Time current_time, last_time;
#ifdef __ODOM_BROADCASTER
    tf::TransformBroadcaster odom_broadcaster;
#endif

    boost::array<double, COV_SIZE> cov_matrix;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wheel_odometry_node");

    OmniWheelOdometry omni;

    ros::spin();
}
