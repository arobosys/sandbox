# malish  
## Репо по минироботу  
Добавлена тестовая ветвь.


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
sudo apt-get install terminator xboxdrv ssh
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
git clone git+ssh://git@github.com/arobosys/malish.git catkin_ws
git submodule update --recursive --remote # If you need submodules
cd ~/catkin_ws
catkin_make -DCATKIN_BLACKLIST_PACKAGES="zed-ros-wrapper" #For pc without cuda
catkin_make -pkg malish #To make only one package
```

### System settings
Add ros uri to bash
Modify /etc/hosts

### Arduino devices  
#### IMU connection  
'V' - 3.3V;  
'G' - ground;  
Arduino UNO:  
'D' - A3;  
'C' - A4;  
Arduino MEGA 2560  
'D' - SDA 20;  
'C' - SCL 21;  
Depends upon:  
TroykaIMU.h https://github.com/amperka/Troyka-IMU  

#### Sonars connection 
            left, right, front, rear   
trig pins =   50,    46,    38,   42;  
echo pins =   48,    44,    36,   40;  


Depends upon:
Ultrasonic.h https://github.com/JRodrigoTech/Ultrasonic-HC-SR04
In Ultrasonic.cpp change line21: Time_out=6000;

#### Github:
Set up for fast access.
```bash
git clone --recurse-submodules git+ssh://git@github.com/arobosys/malish.git catkin_ws #With submodules (like ZED-wrapper)
git remote set-url origin git+ssh://git@github.com/arobosys/malish.git
ssh-keygen
cat ~/.ssh/id_rsa.pub
#Then you manually add the key to GIT
git checkout devel
```

#### ZSH/Terminator/ssh recommendations
*Copying the settings from .bashrc-->.zshrc:
```bash
sudo apt-get install zsh
echo "source /opt/ros/kinetic/setup.zsh" >> ~/.zshrc
echo "source ~/catkin_ws/devel/setup.zsh" >> ~/.zshrc
echo "export ROS_IP="hostname -I"" >> ~/.zshrc
```
Then you check again if this files have identical statements.

*Installing oh-my-zsh:
```bash
sh -c "$(wget https://raw.githubusercontent.com/robbyrussell/oh-my-zsh/master/tools/install.sh -O -)"
```
*For ssh use: 
```bash
ssh -X -C username@address
```
and you will have access to X-server as well. 
