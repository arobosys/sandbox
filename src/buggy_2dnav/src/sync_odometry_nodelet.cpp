#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <tf/transform_broadcaster.h>

#include <csignal>
#include <mutex>
#include <thread>


#ifdef __GNUC__
#include <features.h>
#if __GNUC_PREREQ(7,0)
//      If  gcc_version >= 7.0
typedef std::scoped_lock<std::mutex> std_lock;
#define GCC7 1
#elif __GNUC_PREREQ(3,2)
//       If gcc_version >= 3.2
typedef std::lock_guard<std::mutex> std_lock;
#else
//       Else
#endif
#else
//    If not gcc
#endif

namespace buggy_2dnav {

    using namespace sensor_msgs;
    using namespace message_filters;
    using namespace nav_msgs;

    class SyncOdometryNodelet : public nodelet::Nodelet {
        ros::NodeHandle nh;

        ros::Publisher odomPub, laserPub;
        ros::Subscriber infoSub, odomSub, laserSub;

        tf::TransformBroadcaster broadcaster;

        OdometryPtr lastOdom;
        LaserScanPtr lastScan;
        CameraInfoConstPtr lastInfo;

        std::unique_ptr<std::thread> syncThread;
        std::mutex odomMutex, scanMutex, infoMutex;

        static void sigintHandler(int) {
            ros::shutdown();
        }

    public:
        void odomCallback(const OdometryPtr& odom) {
            std_lock lock(odomMutex);
            lastOdom = odom;
        }

        void infoCallback(const CameraInfoConstPtr& info) {
            std_lock lock(infoMutex);
            lastInfo = info;
        }

        void scanCallback(const LaserScanPtr& scan) {
            std_lock lock(scanMutex);
            lastScan = scan;
        }

        void syncLoop() {
            ros::Rate rate(60);

            while (ros::ok()) {
                {
#ifdef GCC7
                    std::scoped_lock lock(odomMutex, infoMutex, scanMutex);
#else
                    std_lock lock1(odomMutex);
                    std_lock lock2(infoMutex);
                    std_lock lock3(scanMutex);
#endif
                    if (lastInfo && lastOdom) {
                        lastOdom->header.stamp = lastInfo->header.stamp;
                        odomPub.publish(lastOdom);

                        //lastScan->header.stamp = lastInfo->header.stamp;
                        //laserPub.publish(lastScan);
                    }
                }

                tf::Transform transform;
                transform.setOrigin(tf::Vector3(0.014, 0.0, 0.2));
                tf::Quaternion q;
                q.setRPY(0.0, 0.0, 0.0);
                transform.setRotation(q);
                broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now() + ros::Duration(2, 0), "base_link", "laser"));
                rate.sleep();
            }
        }

        void onInit() override {
            nh = getMTNodeHandle();

            signal(SIGINT, sigintHandler);

            odomPub = nh.advertise<Odometry>("/odometry/filtered/sync", 1);
            odomSub = nh.subscribe("/odometry/filtered", 1, &SyncOdometryNodelet::odomCallback, this);
            infoSub = nh.subscribe("/zed/rgb/camera_info", 1, &SyncOdometryNodelet::infoCallback, this);

            laserPub = nh.advertise<LaserScan>("/scan/sync", 1);
            laserSub = nh.subscribe("/scan", 1, &SyncOdometryNodelet::scanCallback, this);

            syncThread = std::make_unique<std::thread>(&SyncOdometryNodelet::syncLoop, this);
        }
    };

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(buggy_2dnav::SyncOdometryNodelet, nodelet::Nodelet);
