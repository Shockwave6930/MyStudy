#!/usr/bin/env python

from robot_class2_dandan.srv import *
from telnetlib import Telnet
import rospy

def handle_door_status(req):
    tn = Telnet('localhost', 18899)
    tn.write('door_status\n')
    res = tn.read_until('\n')
    res = res.replace('\n', '')
    return DoorStatusResponse(float(res))

def door_status_server():
    rospy.init_node('door_status_server')
    s = rospy.Service('door_status', DoorStatus, handle_door_status)
    rospy.spin()

if(__name__ == '__main__'):
    door_status_server()
