# malish
Репо по минироботу
------------------


### System setup:

* ROS setup.

```bash
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

sudo apt-get install ros-kinetic-joy ros-kinetic-rosserial ros-kinetic-pcl-ros ros-kinetic-tf2-geometry-msgs ros-kinetic-rtabmap ros-kinetic-rtabmap-ros ros-kinetic-urg-node ros-kinetic-image-view ros-kinetic-robot-localization ros-kinetic-move-base ros-kinetic-teb-local-planner ros-kinetic-global-planner ros-kinetic-teb-local-planner ros-kinetic-range-sensor-layer
```

* User interface:

```bash
sudo apt-get install terminator xboxdrv
```

* GCC/G++ 7

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-7 g++-7
sudo update-alternatives --remove-all gcc
sudo update-alternatives --remove-all g++
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 10
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 10
```

### Building of source code

```bash
sudo rm -r ~/catkin_ws
git clone https://github.com/arobosys/malish catkin_ws
cd ~/catkin_ws
catkin_make -DCATKIN_BLACKLIST_PACKAGES="zed-ros-wrapper" #For pc without cuda
```

### System settings
Add ros uri to bash
Modify /etc/hosts