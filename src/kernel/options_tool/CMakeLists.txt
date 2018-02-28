cmake_minimum_required(VERSION 2.8.3)
project(options_tool)

find_package(catkin REQUIRED)
find_package(Boost COMPONENTS regex REQUIRED)
set(Boost_USE_STATIC_LIBS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall")

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES options_tool
)

# # # # # # # #  


include_directories(include ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED src/optionstooljson.cpp src/processoptionshandler.cpp src/procptree_boostptree.cpp src/optionstool.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

add_executable(test_poh src/test.cpp)
target_link_libraries(test_poh ${PROJECT_NAME} ${Boost_LIBRARIES})

# # # # # # # #

install(
   DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
   DIRECTORY include/options_tool/valijson
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../
)

install(
   DIRECTORY include/options_tool/compat
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../
)

install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
