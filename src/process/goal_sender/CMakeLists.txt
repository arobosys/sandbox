cmake_minimum_required(VERSION 2.8.3)
project(goal_sender)
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

add_library(goal_sender_class STATIC
        include/GoalSender.hpp src/goal_sender_class.cpp)

add_executable(goal_sender src/main.cpp)

target_link_libraries(goal_sender_class ${catkin_LIBRARIES})
target_link_libraries(goal_sender goal_sender_class)


