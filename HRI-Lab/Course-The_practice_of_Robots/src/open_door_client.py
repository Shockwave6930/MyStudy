#!/usr/bin/env python

import sys
import rospy
from robot_class2_dandan.srv import *

def open_door_client(open_time):
    rospy.wait_for_service('open_door')
    try:
        door_service = rospy.ServiceProxy('open_door', OpenDoor)
        res = door_service(open_time)
        return res
    except rospy.ServiceException, e:
        print('Service call failed: %s'%e)

if(__name__ == '__main__'):
    if(sys.argv[1] == 'open_door'):
        open_time = float(sys.argv[2])
        print(open_door_client(open_time)) 

