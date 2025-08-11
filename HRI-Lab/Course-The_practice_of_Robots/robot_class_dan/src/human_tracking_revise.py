#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Vector3, Point, Quaternion, Twist, QuaternionStamped
from visualization_msgs.msg import Marker, MarkerArray
from layer2.msg import HTEntityList


angle_min = 0.0
angle_max = 1.0
angle_increment = 1.0
ranges = [0, 0]
_robot_x_ = 0.0
_robot_y_ = 0.0
_robot_theta_ = 0.0
pub_manipurator = None
pub_talkgesture = None
pub_talkgesture2 = None
THETA_ERROR_RANGE = 10.0 * math.pi / 180.0
DISTANCE_ERROR_RANGE = 0.5
_ht_x_ = 0.0
_ht_y_ = 0.0












def set_robot_position(listener):
    global _robot_x_, _robot_y_, _robot_theta_
    try:
        now = rospy.Time(0)
        robot_bl = PointStamped()
        robot_bl.header.stamp = now
        robot_bl.header.frame_id = 'base_link'
        robot_bl.point.x = 0
        robot_bl.point.y = 0
        robot_bl.point.z = 0
        listener.waitForTransform('/map', '/base_link', now, rospy.Duration(10.0))
        robot_in_map = listener.transformPoint('/map', robot_bl)
        _robot_x_, _robot_y_ = robot_in_map.point.x, robot_in_map.point.y

        robot_bl_quat = QuaternionStamped()
        robot_bl_quat.header.stamp = now
        robot_bl_quat.header.frame_id = "/base_link"
        robot_bl_quat.quaternion.x = 0
        robot_bl_quat.quaternion.y = 0
        robot_bl_quat.quaternion.z = 0
        robot_bl_quat.quaternion.w = 1
        robot_in_map_quat = listener.transformQuaternion('/map', robot_bl_quat)
        robot_in_map_quat = robot_in_map_quat.quaternion
        explicit_quat = [robot_in_map_quat.x, robot_in_map_quat.y, robot_in_map_quat.z, robot_in_map_quat.w]
        (roll, pitch, theta) = euler_from_quaternion(explicit_quat)
        _robot_theta_ = theta
    except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr(e)


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


def callback_ht(msg):
    global _ht_x_, _ht_y_
    _ht_x_, _ht_y_ = msg.list[0].x, msg.list[0].y
#    rospy.loginfo(str(_ht_x_), str(_ht_y_))


def move_to(des_x, des_y, listener):
    global _robot_x_, _robot_y_,  _robot_theta_, _ht_x_, _ht_y_
    dist_diff = DISTANCE_ERROR_RANGE * 2
    while not rospy.is_shutdown() and not (-DISTANCE_ERROR_RANGE <= dist_diff and dist_diff <= DISTANCE_ERROR_RANGE):
        set_robot_position(listener)
        x_diff = des_x - _robot_x_
        y_diff = des_y - _robot_y_
        dist_diff = np.sqrt((x_diff) ** 2 + (y_diff) ** 2)
        des_theta = np.arctan2(y_diff, x_diff)
        theta_diff = des_theta - _robot_theta_
        rospy.loginfo("robot_x, robot_y: " + str(_robot_x_) + " " + str(_robot_y_))
        rospy.loginfo("ht_x, ht_y: " + str(des_x) + " " + str(des_y))
        rospy.loginfo("theta_diff: " + str(theta_diff * 180 / math.pi))
        msg = Twist()
        if -THETA_ERROR_RANGE <= theta_diff and theta_diff <= THETA_ERROR_RANGE:
            msg.linear.x = 0.5
 #           rospy.loginfo('DIST: ' + str(dist_diff))
        else:
            if theta_diff < 0:
                msg.angular.z = -0.3
            else:
                msg.angular.z = 0.3
#            rospy.loginfo(str(des_theta) + '   ' + str(_odom_theta_))
        pub_manipurator.publish(msg)
#        rate.sleep()
    pub_manipurator.publish(Twist())

def talker():
    global pub_manipurator, pub_talkgesture, pub_talkgesture2, _ht_x_, _ht_y_, angle_min, angle_max, angle_increment, ranges, _robot_x_, _robot_y_, _robot_theta_
    rospy.init_node('manipulater')
    ##rospy.Subscriber('odom', Odometry, callback_odom)
    rospy.Subscriber('ht', HTEntityList, callback_ht) #####################################
    pub_manipurator = rospy.Publisher('tos_cmd_vel', Twist, queue_size = 1)
    pub_talkgesture = rospy.Publisher('tos_implicit_gestures', String, queue_size = 1)
    pub_talkgesture2 = rospy.Publisher('tos_voice_gestures', String, queue_size = 1)
    listener = tf.TransformListener()
#    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        set_robot_position(listener)
        msg1 = String()
        msg1.data = 'talker'
        pub_talkgesture.publish(msg1)
        msg2 = String()
        msg2.data = '<gesture type=\"small\">こんにちは<pause time=\"3000\"></gesture>'
#        pub_talkgesture2.publish(msg2)
#        move_to(_ht_x_, _ht_y_, rate)
        move_to(_ht_x_, _ht_y_, listener)
        rospy.sleep(2.0)
        pub_talkgesture2.publish(msg2)
#        msg3 = String()
#        msg3.data = '<gesture type=\"small\">ころすぞ<pause time=\"3000\"></gesture>'
#        pub_talkgesture2.publish(msg3)
        rospy.loginfo(str(_robot_x_) + " " + str(_robot_y_) + "             " + str(_ht_x_) + " " + str(_ht_y_))
        rospy.sleep(2.0)
#        rate.sleep()


if(__name__ == '__main__'):
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

