cmake_minimum_required(VERSION 2.8.3)
project(joy_process)
set(CMAKE_CXX_FLAGS "-std=c++14")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

#find_package(malish)

#link_directories(${catkin_LIBRARY_DIRS})

#add_message_files(FILES JoyCMD.msg)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS roscpp
    malish
)

#include_directories(${catkin_INCLUDE_DIRS} include)
#add_library(joy_process_class STATIC
#        include/joy_process_class.hpp src/joy_process_class.cpp)

#add_executable(joy_process src/main.cpp)


#target_link_libraries(joy_process joy_process_class ${catkin_LIBRARIES})


