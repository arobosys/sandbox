cmake_minimum_required(VERSION 2.8.3)
project(buggy_2dnav)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        message_filters
        nodelet
        sensor_msgs
        roscpp
        tf
        move_base_msgs
        actionlib
        )

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_library(sync_odometry src/sync_odometry_nodelet.cpp)
target_link_libraries(sync_odometry ${catkin_LIBRARIES})
