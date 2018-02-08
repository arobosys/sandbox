#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from malish.msg import NewTwist
from sensor_msgs.msg import Joy
from math import atan2,pi,sqrt

t1 = NewTwist()
t1.linear_vel = 0;
t1.orient = 0.0;
t1.angle_vel = 0.0;

def callback(data):
    global pub, t1

    #Dead zone constant
    thresh = 0.15

    #Constants
    norm_lin_vel = 1000
    norm_orient = pi/2.0
    norm_angle_vel = pi/8.0

    #Joystick Buttons
    lr = data.axes[0]
    ud = data.axes[1]
    throttle = data.buttons[0]

    left_turn = data.axes[4]#2]
    right_turn = data.axes[5]#5]

    #Dead zone logick
    if lr < thresh and lr > -thresh:
        lr = 0
    if ud < thresh and ud > -thresh:
        ud = 0
    #Dead zone turn
    turn = left_turn - right_turn
    if turn < thresh*2 and turn > -thresh*2:
        turn = 0
    print(turn)    
    t1.linear_vel = int(throttle * norm_lin_vel * sqrt(lr**2+ud**2))
    t1.orient = norm_orient + atan2(lr, -ud)  
    t1.angle_vel = throttle * turn * norm_angle_vel 

    #pub.publish(t1)
    

def talker():
    global pub, t1
    rospy.init_node('twist_joy')
    pub = rospy.Publisher('/twist/command', NewTwist, queue_size=10)
    rospy.Subscriber("/joy", Joy, callback)
    rate = rospy.Rate(100) # 100hz

    while not rospy.is_shutdown():
        pub.publish(t1)
        rate.sleep()
                                            
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
