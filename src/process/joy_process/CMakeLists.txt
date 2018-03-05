cmake_minimum_required(VERSION 2.8.3)
project(joy_process)
set(CMAKE_CXX_FLAGS "-std=c++14")

#add_dependencies(process_interface)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  process_interface
  geometry_msgs
  tf
  move_base_msgs
  actionlib
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS} include)

add_library(joy_sender STATIC
        include/joy_process.hpp src/joy_sender_class.cpp)

add_executable(joy_process src/main.cpp)

target_link_libraries(joy_process ${catkin_LIBRARIES})
target_link_libraries(joy_process joy_sender_class)


