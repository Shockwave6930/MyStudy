#!/usr/bin/env python

import sys
import rospy
from robot_class2_dandan.srv import *

def door_status_client():
    rospy.wait_for_service('door_status')
    try:
   	door_service = rospy.ServiceProxy('door_status', DoorStatus)
        res = door_service()
        return res
    except rospy.ServiceException, e:
        print('Service call failed: %s'%e)

if(__name__ == '__main__'):
    print(door_status_client()) 

