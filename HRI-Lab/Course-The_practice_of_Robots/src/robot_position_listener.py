#!/usr/bin/env python
import roslib, rospy, math, tf
from geometry_msgs.msg import PointStamped

if(__name__ == '__main__'):
    rospy.init_node('robot_position_listener')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            now = rospy.Time(0)
            robot_in_base_link = PointStamped()
	    robot_in_base_link.header.stamp = now
            robot_in_base_link.header.frame_id = '/base_link'
            robot_in_base_link.point.x = 0
            robot_in_base_link.point.y = 0
            robot_in_base_link.point.z = 0
            listener.waitForTransform('/map' ,'/base_link', now, rospy.Duration(10.0))
            robot_in_map = listener.transformPoint('/map', robot_in_base_link)
            rospy.loginfo(robot_in_map)
            rate.sleep()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)
            continue
