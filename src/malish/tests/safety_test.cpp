// Created by Evgeny on 06.03.18.
/*!
 * \file safety.cpp
 *
 * Test for the Node which informs you if obstacles invade robot's safety vicinity.
 *
 * \authors Shtanov Evgeny
 *
*/

#include <sstream>
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
static const float alert_margin = 0.88;
/// Margin of vicinity to light LED in yellow (notification), [m].
static const float yellow_margin = 1.25;
/// Minimal blob area to consider it as obstacle, [pixels].
static const float min_blob_area = 20.0;
/// Constant for comparision with float type zero value.
static const float eps = 1e-10;

typedef std::vector<cv::Point> cvContour;

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
    ellipse(map, center, cv::Size(radius, radius), direction, direction - angle_range / 2, direction + angle_range / 2, color);

    return map;
}
