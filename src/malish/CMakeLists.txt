cmake_minimum_required(VERSION 2.8.3)
project(malish)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
)
#add_service_files(
#  FILES
#  )
add_message_files(
  FILES
  OmniSpeed.msg
  NewTwist.msg
  ArduOdom.msg
  Sonar.msg
  Diode.msg
  )
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
 )
catkin_package(
  CATKIN_DEPENDS message_runtime
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  )
add_executable(omni_odom src/omni_odom.cpp)
target_link_libraries(omni_odom ${catkin_LIBRARIES})
add_dependencies(omni_odom malish_generate_messages_cpp)

