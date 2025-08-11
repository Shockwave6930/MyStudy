#!/usr/bin/env python

from robot_class2_dandan.srv import *
from telnetlib import Telnet
import rospy

def handle_open_door(open_time):
    tn = Telnet('localhost', 18899)
    tn.write('open_door ' + str(open_time) + '\n')
    res = tn.read_until('\n')
    res = res.replace('\n', '')
    return OpenDoorResponse(res)

def open_door_server():
    rospy.init_node('open_door_server')
    s = rospy.Service('open_door', OpenDoor, handle_open_door)
    rospy.spin()

if(__name__ == '__main__'):
    open_door_server()
