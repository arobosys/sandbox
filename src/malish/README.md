# malish
Репо по минироботу
Добавлена тестовая ветвь.

requirements:

sudo apt-get update
sudo apt-get upgrade
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_IP="hostname -I"" >> ~/.bashrc

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
source ~/.bashrc
catkin_make

sudo apt-get install terminator xboxdrv

sudo apt-get install ros-kinetic-joy ros-kinetic-rosserial ros-kinetic-pcl-ros ros-kinetic-tf2-geometry-msgs ros-kinetic-rtabmap ros-kinetic-rtabmap-ros ros-kinetic-urg-node ros-kinetic-image-view ros-kinetic-robot-localization ros-kinetic-move-base ros-kinetic-teb-local-planner ros-kinetic-global-planner ros-kinetic-teb-local-planner ros-kinetic-range-sensor-layer

Arduino depends on:
- TroykaIMU.h https://github.com/amperka/Troyka-IMU
- Ultrasonic.h https://github.com/JRodrigoTech/Ultrasonic-HC-SR04
In Ultrasonic.cpp change line21: Time_out=6000;
