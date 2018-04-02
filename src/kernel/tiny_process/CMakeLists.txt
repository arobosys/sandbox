cmake_minimum_required(VERSION 2.8.3)
project(tiny_process)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Threads REQUIRED)
find_package(catkin REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
)

include_directories(${catkin_INCLUDE_DIRS} include)

add_library(${PROJECT_NAME} SHARED
        src/process.cpp
        )
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
