cmake_minimum_required(VERSION 2.8.3)
project(console_node)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++1y" COMPILER_SUPPORTS_CXX1Y)
if (COMPILER_SUPPORTS_CXX1Y)
    set(CMAKE_CXX_FLAGS "-std=c++1y")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++1y support. Please use a different C++ compiler.")
endif ()

find_package(catkin REQUIRED COMPONENTS
  actionlib
  core_msgs
  roscpp
)

catkin_package(
        CATKIN_DEPENDS core_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(console_node src/console_node.cpp)
target_link_libraries(console_node ${catkin_LIBRARIES})

add_dependencies(console_node ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})