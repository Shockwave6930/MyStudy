#!/usr/bin/env python
import roslib
import rospy
import tf
import math
import open_door_client as open_door
import door_status_client as door_status
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist

state = 0
apple_counter = 0
goal =  [[3.4923, -2.3363, 0.0000],
         [6.3800, -2.2004, 0.0000],
         [8.6732, -6.1583, 0.0000],
         [14.9412, -5.8525, 0.0000],
         [13.6333,  0.1097, 0.0000],
         [18.1517,  0.0588, 0.0000],
         [1.6238,  0.3475, 0.0000],
         [6.7877,  2.6237, 0.0000],
         [1.7258,  9.1295, 0.0000],
         [7.9088,  7.5158, 0.0000],
         [13.9390,  9.1805, 0.0000],
         [15.2979,  3.1333, 0.0000],
         [17.3703,  2.8276, 0.0000],
         [17.2344,  9.0446, 0.0000],
         [17.3703, 14.0386, 0.0000]]

goal = [[4.13, -7.0, 0.0],
        [10.3, 4.7, 0.0],
        [-4.06, 10.9, 0.0]]

angle_min = 0.0
angle_max = 1.0
angle_increment = 1.0
ranges = [0, 0]
pub = None
_odom_x_ = 0.0
_odom_y_ = 0.0
_odom_theta_ = 0.0

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

THETA_ERROR_RANGE = 10.0 * math.pi / 180.0
DISTANCE_ERROR_RANGE = 1

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
    #rospy.loginfo(str(x) + '   ' + str(y))
#    rotation_flug = False
#            move_flug = False
#            begin_arg = _odom_theta_
#            move_arg = 0
#            rate2 = rospy.Rate(10)
#            while  move_arg < des_theta:
#                follower()
#                move_arg = _odom_theta_ - begin_arg
#                rotation_flug = True
#                msg = Twist()
#                if(rotation_flug):
#                    msg.angular.z = 0.5
#                else:
#                    msg.linear.x = 1
#                pub.publish(msg)
#                rate2.sleep()
#            msg = Twist()
#            if(rotation_flug):
#                msg.angular.z = 0.5
#            else:
#                msg.linear.x = 1
#            pub.publish(msg)
#    if(flug):
#    if(des_theta - 0.03 <= _odom_theta_ and _odom_theta_ <= des_theta + 0.03):
#        if(first_time_flug):
#        rotation_flug = False
#                    move_flug = True
#            flug = False
#        #first_time_flug = False
#    else:
#        rotation_flug = True
#                    move_flug = False
#    msg = Twist()
#    if(rotation_flug):
#        msg.angular.z = 0.5
#    else:
#        msg.linear.x = 1
#    pub.publish(msg)
#            rospy.Duration(10)
#    else:
#        rospy.wait_for_service('open_door')
#        rospy.wait_fo


def talker():
    global pub
    rospy.init_node('manipulater')
    rospy.Subscriber('scan', LaserScan, callback_scan)
    rospy.Subscriber('odom', Odometry, callback_odom)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    rate = rospy.Rate(10)
    listener = tf.TransformListener()
    flug = True
    if not rospy.is_shutdown():
#        if(first_time_flug):
#            start_time = rospy.Time.now()
#            first_time_flug = False
        try:
            now = rospy.Time(0)
            goal_in_map = PointStamped()
            goal_in_map.header.stamp = now
            goal_in_map.header.frame_id = '/map'
            goal_in_map.point.x = 4.13
            goal_in_map.point.y = -7.0
            goal_in_map.point.z = 0.0
            listener.waitForTransform('/base_link' ,'/map', now, rospy.Duration(10.0))
            goal_in_map = listener.transformPoint('base_link', goal_in_map)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)
        #move_to(goal_in_map.point.x, goal_in_map.point.y, rate)




        move_to(4.2, -6.2, rate)
        move_to(6.5, -6.2, rate)
        move_to(11, -2.0, rate)
        move_to(11, 1, rate)
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
                move_to(11, 5, rate)
        except rospy.ServiceException, e:
            print('Service call failed: %s'%e)

###
        pub = rospy.Publisher('closest_point', PointStamped, queue_size = 10)
        while not rospy.is_shutdown():
            global angle_min, angle_max, angle_increment, ranges, _odom_x_, _odom_y_, _odom_theta_
            x = 0
            y = 0
            for i in range(len(ranges)):
                theta = _odom_theta_ + angle_min + i * angle_increment
                x = _odom_x_ + ranges[i] * math.cos(theta)
                y = _odom_y_ + ranges[i] * math.sin(theta)
                if ranges[i] <= min_range:
		    min_angle = theta
                    min_range = ranges[i]
            msg = PointStamped()
            msg.header.stamp = rospy.Time()
            msg.header.frame_id = '/map'
            msg.point = Point( _odom_x_ + min_range * math.cos(min_angle), _odom_y_ + min_range * math.sin(min_angle$
            pub.publish(msg)
            rospy.loginfo(str(msg))
###
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

