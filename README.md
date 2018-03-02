# Task for B1-35 B1-59

Copy file from github.com
```
git clone https://github.com/arobosys/malish/tree/B1-35
gedit .bashrc
```

In file .bashrc, add
```
source ~/AGRO/catkin_ws/devel/setup.bash
export AVIS_ROOT="/home/zhuhua/AGRO/catkin_ws/"
```

In console write command
```
roslaunch malish kernel.launch
```

In another console publish topic
```
rostopic pub  /CI_input std_msgs/String "start malish 5 10"
```

