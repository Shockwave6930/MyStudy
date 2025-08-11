#!/usr/bin/env python

import rospy
import tf
import math
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Vector3, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray

apple_counter = [] #boolの配列で保持、Trueが5つになったならそこで停止
apple_position = [[3.4923, -2.3363, 0.0000], #A
                [6.3800, -2.2004, 0.0000], #B
                [8.6732, -6.1583, 0.0000], #C
                [14.9412, -5.8525, 0.0000], #D
                [13.6333,  0.1097, 0.0000], #E
                [18.1517,  0.0588, 0.0000], #F
                [1.6238,  0.3475, 0.0000],
                [6.7877,  2.6237, 0.0000],
                [1.7258,  9.1295, 0.0000],
                [7.9088,  7.5158, 0.0000],
                [13.9390,  9.1805, 0.0000],
                [15.2979,  3.1333, 0.0000],
                [17.3703,  2.8276, 0.0000],
                [17.2344,  9.0446, 0.0000],
                [17.3703, 14.0386, 0.0000]]
nav_position = [[1.58, -4.76], #通るべき地点を事前に指定
                [4.2, -6.2],
                [6.5, -6.2],
                [1.58, -4.76],
                [11, -2],
                [11, 1],
                [11, 5],
                [11, 6]] #適当な位置デバッグ用
nav_state = 0 #現在何番目のNav_positionへの移動をしているかを保持
scan_position_flug = [True, False, False, True, False, False, False] #nav_positionのうち、どのpositionでscanを行うのかを保持

angle_min = 0.0
angle_max = 1.0
angle_increment = 1.0
ranges = [0, 0]
_odom_x_ = 0.0
_odom_y_ = 0.0
_odom_theta_ = 0.0
pub = None
THETA_ERROR_RANGE = 10.0 * math.pi / 180.0
DISTANCE_ERROR_RANGE = 1

def callback_scan(msg):
    global angle_min, angle_max, angle_increment, ranges
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    angle_increment = msg.angle_increment
    ranges = msg.ranges

def callback_odom(msg):
    global _odom_x_, _odom_y_, _odom_theta_
    _odom_x_ = msg.pose.pose.position.x
    _odom_y_ = msg.pose.pose.position.y
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    q = (qx, qy, qz, qw)
    e = euler_from_quaternion(q)
    _odom_theta_ = e[2]


def move_to(des_x, des_y, rate):
    global _odom_x_, _odom_y_,  _odom_theta_
    dist_diff = DISTANCE_ERROR_RANGE * 2
    while not rospy.is_shutdown() and not (-DISTANCE_ERROR_RANGE <= dist_diff and dist_diff <= DISTANCE_ERROR_RANGE):
        x_diff = des_x - _odom_x_
        y_diff = des_y - _odom_y_
        dist_diff = np.sqrt((x_diff) ** 2 + (y_diff) ** 2)
        des_theta = np.arctan2(y_diff, x_diff)
        theta_diff = des_theta - _odom_theta_
#        rospy.loginfo(str(_odom_x_) + '   ' + str(_odom_y_) + '   ' + str(des_x) + '   ' + str(des_y))
        msg = Twist()
        if -THETA_ERROR_RANGE <= theta_diff and theta_diff <= THETA_ERROR_RANGE:
            msg.linear.x = 1
            rospy.loginfo('DIST: ' + str(dist_diff))
        else:
            if theta_diff < 0:
                msg.angular.z = -0.7
            else:
                msg.angular.z = 0.7
#            rospy.loginfo(str(des_theta) + '   ' + str(_odom_theta_))
        pub.publish(msg)
        rate.sleep()
    pub.publish(Twist())

def talker():
    global pub, apple_counter, apple_position
    rospy.init_node('manipulater')
    rospy.Subscriber('scan', LaserScan, callback_scan)
    rospy.Subscriber('odom', Odometry, callback_odom)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    rate = rospy.Rate(10)
    while np.count_nonzero(apple_counter) < 5 and not rospy.is_shutdown():
        for [des_x, des_y] in nav_position[nav_state]:
            move_to(des_x, des_y, rate)
            if(nav_state == 5):
                rospy.wait_for_service('open_door')
                rospy.wait_for_service('door_status')
                try:
                    ratio = -1.0
                    res = ''
                    print(open_door.open_door_client(10).response)
                    while(ratio != 100.0):
                        ratio = float(door_status.door_status_client().ratio)
                        if(res == 'OK'):
                            continue
                        res = open_door.open_door_client(10).response
                    if(ratio == 100.0):
                        move_to(nav_position[6][0], nav_position[6][1], rate) #ドア後
                except rospy.ServiceException, e:
                    print('Service call failed: %s'%e)
            if(nav_state != 5):
                nav_state += 1
            else:
                nav_state += 2
        rate.sleep()


if(__name__ == '__main__'):
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

