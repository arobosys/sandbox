cmake_minimum_required(VERSION 2.8.3)
project(joy_process)
set(CMAKE_CXX_FLAGS "-std=c++14")

find_package(catkin REQUIRED COMPONENTS
  	roscpp
  	rospy
  	std_msgs
	malish
        process_interface
)

catkin_package(
	CATKIN_DEPENDS malish std_msgs 
)
include_directories(${catkin_INCLUDE_DIRS} include)

add_library(joy_process_class STATIC
        include/joy_process_class.hpp src/joy_process_class.cpp)

add_executable(joy_process src/main.cpp)
target_link_libraries(joy_process joy_process_class ${catkin_LIBRARIES})
add_dependencies(joy_process ${catkin_EXPORTED_TARGETS})



