#!/usr/bin/env python

import rospy
import tf
import math
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped

angle_min = 0.0
angle_max = 1.0
angle_increment = 1.0
ranges = [0, 0]
_odom_x_ = 0.0
_odom_y_ = 0.0
_odom_theta_ = 0.0

def callback(msg):
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
    a = str(_odom_x_)
    b = str(_odom_y_)
    c = str(_odom_theta_)
#    rospy.loginfo(a + '   '  + b + '   ' + c)

def follower():
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.Subscriber("odom", Odometry, callback_odom)

def talker():
    pub = rospy.Publisher('closest_point', PointStamped, queue_size = 10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        follower()
        global angle_min, angle_max, angle_increment, ranges, _odom_x_, _odom_y_, _odom_theta_
        x = 0
        y = 0
        for i in range(int(1+(angle_max-angle_min)/angle_increment)):
            theta = _odom_theta_ - angle_min + i * angle_increment
            x = _odom_x_ + ranges[i] * math.cos(theta)
            y = _odom_y_ + ranges[i] * math.sin(theta)
            if(ranges[i] == min(ranges)):
                break
        msg = PointStamped()
        msg.header.frame_id = '/base_link'
        msg.point.x, msg.point.y = x, y
        pub.publish(msg)
        rate.sleep()

if(__name__ == '__main__'):
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
