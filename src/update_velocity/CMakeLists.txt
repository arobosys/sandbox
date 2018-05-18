# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(update_velocity)

add_compile_options(-std=c++11)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_msgs)

# Declare our catkin package.
catkin_package()

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files.
add_executable(update_velocity src/update_velocity.cpp src/VelocityUpdater.cpp)


# Specify libraries against which to link.
target_link_libraries(update_velocity ${catkin_LIBRARIES})


