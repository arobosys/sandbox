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
  xmlrpcpp
)
#add_service_files(
#  FILES
#  )
add_message_files(
  FILES
  OmniSpeed.msg
  NewTwist.msg
  ArduOdom.msg
  ArduSonar.msg
  Diode.msg
  Vector3_32.msg
  ArduImu.msg
  Obstacle.msg
  Lift.msg
  JoyCMD.msg	
)
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)
catkin_package(
  	CATKIN_DEPENDS message_runtime xmlrpcpp
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package( OpenCV REQUIRED )

IF(NOT OpenCV_FOUND)
    MESSAGE(FATAL_ERROR "OpenCV not found")
ENDIF()
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(omni_odom src/omni_odom.cpp)
target_link_libraries(omni_odom ${catkin_LIBRARIES})
add_dependencies(omni_odom malish_generate_messages_cpp)

add_executable(imu_transform src/imu_transform.cpp)
target_link_libraries(imu_transform ${catkin_LIBRARIES})
add_dependencies(imu_transform malish_generate_messages_cpp)

add_executable(sonar_transform src/sonar_transform.cpp)
target_link_libraries(sonar_transform ${catkin_LIBRARIES})
add_dependencies(sonar_transform malish_generate_messages_cpp)

add_executable(safety src/safety.cpp)
target_link_libraries(safety ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(safety malish_generate_messages_cpp)
