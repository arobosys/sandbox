#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from malish.msg import NewTwist, Diode, Obstacle, Lift, JoyCMD
from sensor_msgs.msg import Joy
from math import atan2,pi,sqrt
from geometry_msgs.msg import Twist

vel_msg = Twist()
vel_msg.linear.x = 0.0
vel_msg.linear.y = 0.0
vel_msg.linear.z = 0.0
vel_msg.angular.x = 0.0
vel_msg.angular.y = 0.0
vel_msg.angular.z = 0.0
 
t1 = NewTwist()
t1.linear_vel = 0;
t1.orient = 0.0;
t1.angle_vel = 0.0;

t2 = NewTwist()
t2.linear_vel = 0;
t2.orient = 0.0;
t2.angle_vel = 0.0;

t3 = NewTwist()
t3.linear_vel = 0;
t3.orient = 0.0;
t3.angle_vel = 0.0;

flag_no_cmd = False;

lift = False;
alert = False;
safe_toggle_prev = False;
restart_prev = False;

def callback(data):
    global pub, t1, flag_no_cmd, pub_lift, safe_toggle_prev, restart_prev

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
    lifter = data.buttons[4]
    unlifter = data.buttons[5]
    safe_toggle = data.buttons[1]
    restart = data.buttons[2]
    joy_cmd = JoyCMD()

    if (safe_toggle==1):
    	joy_cmd.safety_toggle = True
    else:
    	joy_cmd.safety_toggle = False

    if (restart==1):
    	joy_cmd.restart = True
    else:
    	joy_cmd.restart = False
    
    #pub.publish(joy cmd)
    if (safe_toggle!=safe_toggle_prev) or(restart!=restart_prev):
    	pub_command.publish(joy_cmd)

    safe_toggle_prev = bool(safe_toggle)
    restart_prev = bool(restart) 

    flag_no_cmd = bool(data.buttons[3])

    left_turn = data.axes[4]#2]
    right_turn = data.axes[5]#5]

    lift_msg = Lift()
    if (lifter==1) and (unlifter!=1):
        lift_msg.dio1 = True
        lift_msg.dio2 = False
        lift_msg.dio3 = False

    if (lifter!=1) and (unlifter==1):
        lift_msg.dio1 = False
        lift_msg.dio2 = True
        lift_msg.dio3 = True
    
    #Dead zone logick
    if lr < thresh and lr > -thresh:
        lr = 0
    if ud < thresh and ud > -thresh:
        ud = 0
    

    #Dead zone turn
    turn = left_turn - right_turn
    if turn < thresh*2 and turn > -thresh*2:
        turn = 0
    t1.linear_vel = int(throttle * norm_lin_vel * sqrt(lr**2+ud**2))
    t1.orient = norm_orient + atan2(lr, -ud)  
    t1.angle_vel = throttle * turn * norm_angle_vel 
	
    #pub.publish(t1)
    pub_lift.publish(lift_msg)

def vel_callback(msg):
    global vel_msg, t2
    norm_orient = pi/2.0
    vel_msg = msg
    t2.linear_vel = int(1000*(sqrt(vel_msg.linear.x**2 + vel_msg.linear.y**2)))
    t2.orient = norm_orient + atan2(vel_msg.linear.y, -vel_msg.linear.x)
    t2.angle_vel = vel_msg.angular.z

def safety_callback(msg):
    global alert
    alert = msg.alert

def talker():
    global pub, t1, t2, flag_no_cmd, pub_lift, alert, pub_command
    rospy.init_node('twist_joy')
    pub = rospy.Publisher('/twist/command', NewTwist, queue_size=10)
    pub_lift = rospy.Publisher('/lift', Lift , queue_size=10)
    pub_command = rospy.Publisher('/joy/command', JoyCMD , queue_size=10)

    rospy.Subscriber("/joy", Joy, callback)
    rospy.Subscriber("/cmd_vel", Twist, vel_callback)
    rospy.Subscriber("/safety", Obstacle, safety_callback)


    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        if (flag_no_cmd):
            if alert:
                pub.publish(t3)
            else:
                pub.publish(t2)
        else:
            pub.publish(t1)
        rate.sleep()
                                            
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
