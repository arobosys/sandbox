#!/bin/bash -e
source ~/.bashrc
catkin_make --pkg malish
source ~/.bashrc
catkin_make --pkg process_interface
source ~/.bashrc
catkin_make
source ~/.bashrc
