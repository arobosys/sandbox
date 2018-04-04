#!/bin/bash -e
echo "Writing an ROS_IP command in a file:"
addres=ip route get 8.8.4.4 | head -1 | cut -d' ' -f8
echo "export ROS_IP=" $addres >>~/.bashrc
echo "export ROS_IP=" $addres >>~/.zshrc
