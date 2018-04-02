cmake_minimum_required(VERSION 2.8.3)
project(lift_process)
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

set(CMAKE_CXX_FLAGS "-std=c++14")

include_directories(${catkin_INCLUDE_DIRS} include)
add_library(lift_process_class STATIC
        include/lift_process.hpp src/lift_process_class.cpp)


add_executable(lift_process src/main.cpp)
target_link_libraries(lift_process lift_process_class ${catkin_LIBRARIES})
add_dependencies(lift_process malish_generate_messages_cpp)



