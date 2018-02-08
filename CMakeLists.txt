cmake_minimum_required(VERSION 2.8.3)
project(malish)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)
add_service_files(
  FILES
  FindDistance.srv
  FindText.srv
  )
add_message_files(
  FILES
  OmniSpeed.msg
  NewTwist.msg
  ArduOdom.msg
  Sonar.msg
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

#include_directories(/home/georgy/catkin_ws/src/arobo/include/lib/MotorWheel)
#include_directories(/home/georgy/catkin_ws/src/arobo/include/lib/PID_Beta6)
#include_directories(/home/georgy/catkin_ws/src/arobo/include/lib/PinChangeInt)
#include_directories(/home/georgy/Documents/arduino-1.8.5/hardware/arduino/avr/cores/arduino)
#include_directories(/home/georgy/Documents/arduino-1.8.5/hardware/tools/avr/avr/include)



add_executable(omni src/omni.cpp)
target_link_libraries(omni ${catkin_LIBRARIES})
add_dependencies(omni malish_generate_messages_cpp)

add_executable(uart_bridge src/uart_bridge.cpp)
target_link_libraries(uart_bridge ${catkin_LIBRARIES})
add_dependencies(uart_bridge malish_generate_messages_cpp)
