#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from malish.msg import NewTwist
from sensor_msgs.msg import Joy
from math import atan2,pi,sqrt


def callback(data):
    global pub
    t1 = NewTwist()
    t1.orient = 0.0
    t1.angle_vel = 0.0
    t1.linear_vel = 0

    norm_lin_vel = 100
    norm_orient = pi/2.0
    norm_angle_vel = 0

    lr = data.axes[0]
    ud = data.axes[1]
    throttle = data.buttons[0]
    left_turn = data.axes[2]
    right_turn = data.axes[5]
    
    if lr<0.15 and lr>-0.15:
        lr = 0
    if ud<0.15 and ud>-0.15:
        ud = 0


    t1.linear_vel = int(throttle * norm_lin_vel * sqrt(lr**2+ud**2))
    t1.orient = norm_orient + atan2(lr, -ud)  
    t1.angle_vel = (left_turn - right_turn) * pi/10

    pub.publish(t1)
    

def talker():
    global pub
    rospy.init_node('twist_joy', anonymous=True)
    pub = rospy.Publisher('/twist/command', NewTwist, queue_size=10)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
