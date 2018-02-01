#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from arobo.srv import FindText

def main(rec):
    print rec.filename
    print rec.text
    filename = "/home/georgy/catkin_ws/src/arobo/data/"+rec.filename
    f = open(filename, "r")
    word = rec.text
    mass = []
    for c,i in enumerate(f):
        if word in i:
            mass += [c]
    
    f.close()
    print mass
    if mass:
        return [mass]
    else:
        return [[-1]]

            
if __name__ == "__main__":
    rospy.init_node("text_finder")
    s = rospy.Service("find_text", FindText, main)
    rospy.spin()
