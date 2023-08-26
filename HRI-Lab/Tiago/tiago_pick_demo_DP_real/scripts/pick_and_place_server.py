#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Sam Pfeiffer
#   * Job van Dieten
#   * Jordi Pages

import rospy
import tf
import tf2_geometry_msgs
import tf2_ros
from spherical_grasps_server import SphericalGrasps
from actionlib import SimpleActionClient, SimpleActionServer
from moveit_commander import PlanningSceneInterface, MoveGroupCommander, RobotCommander
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult, MoveItErrorCodes, Constraints, OrientationConstraint
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal, PickUpPoseResult, PickUpPoseFeedback
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
from std_srvs.srv import Empty, EmptyRequest
from copy import deepcopy
from random import shuffle
import numpy as np
import copy
import math
import actionlib
import trajectory_msgs.msg
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Twist


moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
	if not name[:1] == '_':
		code = MoveItErrorCodes.__dict__[name]
		moveit_error_dict[code] = name


def createPickupGoal(group="arm_torso", target="part", grasp_pose=PoseStamped(), possible_grasps=[], links_to_allow_contact=None):
	""" Create a PickupGoal with the provided data"""
	pug = PickupGoal()
	pug.target_name = target
	pug.group_name = group
	pug.possible_grasps.extend(possible_grasps)
	pug.allowed_planning_time = 35.0
	pug.planning_options.planning_scene_diff.is_diff = True
	pug.planning_options.planning_scene_diff.robot_state.is_diff = True
	pug.planning_options.plan_only = False
	pug.planning_options.replan = True
	pug.planning_options.replan_attempts = 1  # 10
	pug.allowed_touch_objects = []
	pug.attached_object_touch_links = ['<octomap>']
	pug.attached_object_touch_links.extend(links_to_allow_contact)

	return pug


def createPlaceGoal(place_pose,
					place_locations,
					group="arm_torso",
					target="part",
					links_to_allow_contact=None):
	"""Create PlaceGoal with the provided data"""
	placeg = PlaceGoal()
	placeg.group_name = group
	placeg.attached_object_name = target
	placeg.place_locations = place_locations
	placeg.allowed_planning_time = 15.0
	placeg.planning_options.planning_scene_diff.is_diff = True
	placeg.planning_options.planning_scene_diff.robot_state.is_diff = True
	placeg.planning_options.plan_only = False
	placeg.planning_options.replan = True
	placeg.planning_options.replan_attempts = 1
	placeg.allowed_touch_objects = ['<octomap>']
	placeg.allowed_touch_objects.extend(links_to_allow_contact)


	return placeg

class PickAndPlaceServer(object):
        def __init__(self):
		rospy.loginfo("Initalizing PickAndPlaceServer...")
		self.sg = SphericalGrasps()
		rospy.loginfo("Connecting to pickup AS")
		self.pickup_ac = SimpleActionClient('/pickup', PickupAction)
		self.pickup_ac.wait_for_server()
		rospy.loginfo("Succesfully connected.")
		rospy.loginfo("Connecting to place AS")
		self.place_ac = SimpleActionClient('/place', PlaceAction)
		self.place_ac.wait_for_server()
		rospy.loginfo("Succesfully connected.")
                rospy.loginfo("Connecting to gripper_controller")
                self.gripper_control_client = actionlib.SimpleActionClient("/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
                self.gripper_control_client.wait_for_server()
                rospy.loginfo("Successfully conttented.")
                self.scene = PlanningSceneInterface()               ######################################################################################
                self.robot = RobotCommander()                       ######################################################################################
                self.move_group = MoveGroupCommander("arm_torso")   ######################################################################################

		rospy.loginfo("Connecting to /get_planning_scene service")
		self.scene_srv = rospy.ServiceProxy(
			'/get_planning_scene', GetPlanningScene)
		self.scene_srv.wait_for_service()
		rospy.loginfo("Connected.")

		rospy.loginfo("Connecting to clear octomap service...")
		self.clear_octomap_srv = rospy.ServiceProxy(
			'/clear_octomap', Empty)
		self.clear_octomap_srv.wait_for_service()
		rospy.loginfo("Connected!")

		# Get the object size
		self.object_height = rospy.get_param('~object_height')
		self.object_width = rospy.get_param('~object_width')
		self.object_depth = rospy.get_param('~object_depth')

		# Get the links of the end effector exclude from collisions
		self.links_to_allow_contact = rospy.get_param('~links_to_allow_contact', None)
		if self.links_to_allow_contact is None:
			rospy.logwarn("Didn't find any links to allow contacts... at param ~links_to_allow_contact")
		else:
			rospy.loginfo("Found links to allow contacts: " + str(self.links_to_allow_contact))

		self.pick_as = SimpleActionServer(
			'/pickup_pose', PickUpPoseAction,
			execute_cb=self.pick_cb, auto_start=False)
		self.pick_as.start()   #pick_client.pyではこのアクショントピックにゴールを送信することでpick_and_place_server.pyで定義されているオートマニピュレーションのための動作を実行することでpickを行っている
		self.place_as = SimpleActionServer(
			'/place_pose', PickUpPoseAction,
			execute_cb=self.place_cb, auto_start=False)
		self.place_as.start()
                rospy.loginfo("Connecting to '/play_motion' AS...")
                self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
                if not self.play_m_as.wait_for_server(rospy.Duration(20)):
                        rospy.logerr("Could not connect to /play_motion AS")
                        exit()
                rospy.loginfo("Connected!")

                #self.prepare_placing_object()   # to check place action
                self.move_base_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
                self.tfBuffer = tf2_ros.Buffer()
                self.listerner = tf2_ros.TransformListener(self.tfBuffer)
                #self.get_tiago_information(True)
                #self.push()

	def pick_cb(self, goal):
		"""
		:type goal: PickUpPoseGoal
		"""
		error_code = self.grasp_object(goal.object_pose)
		p_res = PickUpPoseResult()
		p_res.error_code = error_code
		if error_code != 1:
			self.pick_as.set_aborted(p_res)
		else:
			self.pick_as.set_succeeded(p_res)

	def place_cb(self, goal):
		"""
		:type goal: PickUpPoseGoal
		"""
		error_code = self.place_object(goal.object_pose)
		p_res = PickUpPoseResult()
		p_res.error_code = error_code
		if error_code != 1:
			self.place_as.set_aborted(p_res)
		else:
			self.place_as.set_succeeded(p_res)

	def wait_for_planning_scene_object(self, object_name='part'):
		rospy.loginfo(
			"Waiting for object '" + object_name + "'' to appear in planning scene...")
		gps_req = GetPlanningSceneRequest()
		gps_req.components.components = gps_req.components.WORLD_OBJECT_NAMES
		
		part_in_scene = False
		while not rospy.is_shutdown() and not part_in_scene:
			# This call takes a while when rgbd sensor is set
			gps_resp = self.scene_srv.call(gps_req)
			# check if 'part' is in the answer
			for collision_obj in gps_resp.scene.world.collision_objects:
				if collision_obj.id == object_name:
					part_in_scene = True
					break
			else:
				rospy.sleep(1.0)

		rospy.loginfo("'" + object_name + "'' is in scene!")

	def grasp_object(self, object_pose):
		rospy.loginfo("Removing any previous 'part' object")
		self.scene.remove_attached_object("arm_tool_link")
                self.scene.remove_world_object("part")
		self.scene.remove_world_object("table")
		rospy.loginfo("Clearing octomap")
		self.clear_octomap_srv.call(EmptyRequest())
		rospy.sleep(2.0)  # Removing is fast
		rospy.loginfo("Adding new 'part' object")

                print("\n\n\n\n\n=====================DEBUG==================")
                print("pick_and_place_server.py: ", object_pose.pose)
                print("=====================DEBUG==================\n\n\n\n\n")
		rospy.loginfo("Object pose: %s", object_pose.pose)
                #object_pose.pose.position.z += 0.05 #in the simulation
                object_pose.pose.position.z += 0.016   #pose ###########################################################################must use in the case of using real tiago
		###object_pose.pose.position.z -= 0.01   #pose
		#Add object description in scene
		#self.scene.add_box("part", object_pose, (self.object_depth, self.object_width, self.object_height)) ###default
                #self.scene.add_cylinder("part", object_pose, self.object_height, 0.033) #empty can
                ###self.scene.add_sphere("part", object_pose, radius=0.031)   ###tennis ball
                ###self.scene.add_sphere("part", object_pose, radius=0.040)   ###ball
                self.scene.add_cylinder("part", object_pose, self.object_height, 0.0325) #empty can and mugitya
                ###self.scene.add_cylinder("part", object_pose, self.object_height, 0.0350) #empty can and mugitya

		rospy.loginfo("Second%s", object_pose.pose)
		table_pose = copy.deepcopy(object_pose)

		#define a virtual table below the object
		table_height = object_pose.pose.position.z - 0.016 - self.object_height/2 + 0.015
                ###table_height = object_pose.pose.position.z - self.object_height/2 - 0.01
		table_width  = 5.0
		table_depth  = 1.2
		table_pose.pose.position.x = 1.1
		table_pose.pose.position.z = table_height/2
                table_pose.pose.orientation.x = 0
                table_pose.pose.orientation.y = 0
                table_pose.pose.orientation.z = 0
                table_pose.pose.orientation.w = 1

		self.scene.add_box("table", table_pose, (table_depth, table_width, table_height))

		# # We need to wait for the object part to appear
		self.wait_for_planning_scene_object()
		self.wait_for_planning_scene_object("table")

		# compute grasps
		possible_grasps = self.sg.create_grasps_from_object_pose(object_pose)
		self.pickup_ac
		goal = createPickupGoal(
			"arm_torso", "part", object_pose, possible_grasps, self.links_to_allow_contact)

		rospy.loginfo("Sending goal")
		self.pickup_ac.send_goal(goal)
		rospy.loginfo("Waiting for result")
		self.pickup_ac.wait_for_result()
		result = self.pickup_ac.get_result()
		rospy.logdebug("Using torso result: " + str(result))
		rospy.loginfo(
			"Pick result: " +
		str(moveit_error_dict[result.error_code.val]))
                #rospy.sleep(5.0)
                if(moveit_error_dict[result.error_code.val] == "SUCCESS"):
                        self.prepare_placing_object()   #drop
                        #self.push()   #push
		return result.error_code.val

        def euler_to_quaternion(self, euler):
                """Convert Euler Angles to Quaternion
                euler: geometry_msgs/Vector3
                quaternion: geometry_msgs/Quaternion
                """
                q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
                return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        def quaternion_to_euler(self, quaternion):   #指定された角度回転するまで/cmd_velにメッセージ>を送信し続ける
                """Convert Quaternion to Euler Angles
                quarternion: geometry_msgs/Quaternion
                euler: geometry_msgs/Vector3
                """
                e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
                return Vector3(x=e[0], y=e[1], z=e[2])

        def get_tiago_information(self, option=False):
                rospy.loginfo("\n\n\n\n\n")
                if(option):
                        rospy.loginfo("==============================================================visualize all parameters===============================================================")
                        planning_frame = self.move_group.get_planning_frame()
                        print("Planning frame: %s" % planning_frame)   #プランニングの実行対象となっている基準座標名を取得
                        eef_link = self.move_group.get_end_effector_link()
                        print("End effector link: %s" % eef_link)   #エンドエフェクタとして定義されているリンク名を取得
                        print("Available Planning Groups:", self.robot.get_group_names())   #Tiagoのモデルに定義されているグループのリストを取得
                        print("Printing robot state:", self.robot.get_current_state())   #tiago全体の現在の状態を取得 #OK
                        print("Printing initially setting group as default:", self.move_group.get_name())   #現在設定されているグループ名を取得
                        print("Printing links in current group", self.robot.get_link_names(self.move_group.get_name()))   #OK
                        print("Printing pose reference frame:", self.move_group.get_pose_reference_frame())   #エンドエフェクタの姿勢表現に使われる基準座標を取得 #OK
                        #print("Printing trajectory constraints:", self.move_group.get_trajectory_constraints())
                        print("Printing end-effector pose:", self.move_group.get_current_pose())   #エンドエフェクタのポジションを取得 #No
                        print("Printing end-effector rpy:", self.move_group.get_current_rpy())   #エンドエフェクタの姿勢を取得 #No
                        print("Printing current joint values:", self.move_group.get_current_joint_values())   #現在の関節角度をリストとして取得
                        print("Printing planner's id list:", self.move_group.get_interface_description())   #プランナーIDのリストを取得
                        print("Printing constraints' name:", self.move_group.get_known_constraints())   #現在紐づけられている制約の名前のリストを取得
                        print("Printing get path constraints:", self.move_group.get_path_constraints())   #現在設定されているパス制約を取得
                        print("Printing planner id:", self.move_group.get_planner_id())   #現在設定されているプランナーIDを取得
                        print("Printing planning time:", self.move_group.get_planning_time())   #プランニング時間を取得
                else:
                        rospy.loginfo("==============================================================visualize main parameters==============================================================")
                        print("Printing end-effector pose:", self.move_group.get_current_pose())   #エンドエフェクタのポジションを取得 #No
                        print("Printing end-effector rpy:", self.move_group.get_current_rpy())   #エンドエフェクタの姿勢を取得 #No
                        print("Printing current joint values:", self.move_group.get_current_joint_values())   #現在の関節角度をリストとして取得 #No
                        print("Printing planner's id list:", self.move_group.get_interface_description())   #プランナーIDのリストを取得
                        print("Printing constraints' name:", self.move_group.get_known_constraints())   #現在紐づけられている制約の名前のリストを取得
                        print("Printing get path constraints:", self.move_group.get_path_constraints())   #現在設定されているパス制約を取得
                        print("Printing planner id:", self.move_group.get_planner_id())   #現在設定されているプランナーIDを取得
                        print("Printing planning time:", self.move_group.get_planning_time())   #プランニング時間を取得
                        #print("Printing trajectory constraints:", self.move_group.get_trajectory_constraints())
                rospy.loginfo("=========================================================================================================================================================")
                rospy.loginfo("\n\n\n\n\n")

        def push(self):
                flag = False
                robot_pose = tf2_geometry_msgs.PoseStamped()
                robot_pose.header.frame_id = "base_footprint"
                robot_pose.header.stamp = rospy.Time(0)
                # Orientationも変換したい場合はwを1.0にする。
                robot_pose.pose.orientation.w = 1.0
                first_z = 0
                while not rospy.is_shutdown() and not flag:
                        try:
                                # base_linkフレームの原点をmapフレーム上の座標に変換
                                global_pose = self.tfBuffer.transform(robot_pose, "odom")
                                print(global_pose.pose.orientation)
                                e = self.quaternion_to_euler(global_pose.pose.orientation)   #ラジアン
                                first_z = e.z * 180 / math.pi
                                if(first_z < 0):
                                        first_z += 360
                                flag = True
                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                                rospy.sleep(0.1)
                                continue
                first = first_z
                now_z = first_z
                rate = rospy.Rate(100) # 10hz
                while not rospy.is_shutdown() and abs(now_z - first_z) < 25:
                        msg = Twist()
                        msg.linear.x = 0
                        msg.linear.y = 0
                        msg.linear.z = 0
                        msg.angular.x = 0
                        msg.angular.y = 0
                        msg.angular.z = 0.5
                        #print(msg)
                        self.move_base_pub.publish(msg)
                        robot_pose = tf2_geometry_msgs.PoseStamped()
                        robot_pose.header.frame_id = "base_footprint"
                        robot_pose.header.stamp = rospy.Time(0)
                        # Orientationも変換したい場合はwを1.0にする。
                        robot_pose.pose.orientation.w = 1.0

                        try:
                                # base_linkフレームの原点をmapフレーム上の座標に変換
                                global_pose = self.tfBuffer.transform(robot_pose, "odom")
                                print(global_pose.pose.orientation)
                                e = self.quaternion_to_euler(global_pose.pose.orientation)   #ラジア>ン
                                now_z = e.z * 180 / math.pi
                                if(now_z < 0):
                                        now_z += 360
                                print(now_z)   #度
                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                                print("WARNING: tf map to base_link not found.")
                                rospy.sleep(1)
                                continue
                        rate.sleep()
                print("result:", first_z, now_z)
                print("finished first motion===================================================================")
                first_z = now_z
                rate = rospy.Rate(100) # 10hz
                while not rospy.is_shutdown() and abs(now_z - first_z) < 50:
                        msg = Twist()
                        msg.linear.x = 0
                        msg.linear.y = 0
                        msg.linear.z = 0
                        msg.angular.x = 0
                        msg.angular.y = 0
                        msg.angular.z = -0.5
                        #print(msg)
                        self.move_base_pub.publish(msg)
                        robot_pose = tf2_geometry_msgs.PoseStamped()
                        robot_pose.header.frame_id = "base_footprint"
                        robot_pose.header.stamp = rospy.Time(0)
                        # Orientationも変換したい場合はwを1.0にする。
                        robot_pose.pose.orientation.w = 1.0

                        try:
                                # base_linkフレームの原点をmapフレーム上の座標に変換
                                global_pose = self.tfBuffer.transform(robot_pose, "odom")
                                print(global_pose.pose.orientation)
                                e = self.quaternion_to_euler(global_pose.pose.orientation)   #ラジア>ン
                                now_z = e.z * 180 / math.pi
                                if(now_z < 0):
                                        now_z += 360
                                print(now_z)   #度
                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                                print("WARNING: tf map to base_link not found.")
                                rospy.sleep(1)
                                continue
                        rate.sleep()
                print("result:", first_z, now_z)
                print("finished second motion====================================================================")
                first_z = now_z
                rate = rospy.Rate(100) # 10hz
                while not rospy.is_shutdown() and abs(now_z - first) > 1:
                        msg = Twist()
                        msg.linear.x = 0
                        msg.linear.y = 0
                        msg.linear.z = 0
                        msg.angular.x = 0
                        msg.angular.y = 0
                        msg.angular.z = 0.1
                        #print(msg)
                        self.move_base_pub.publish(msg)
                        robot_pose = tf2_geometry_msgs.PoseStamped()
                        robot_pose.header.frame_id = "base_footprint"
                        robot_pose.header.stamp = rospy.Time(0)
                        # Orientationも変換したい場合はwを1.0にする。
                        robot_pose.pose.orientation.w = 1.0

                        try:
                                # base_linkフレームの原点をmapフレーム上の座標に変換
                                global_pose = self.tfBuffer.transform(robot_pose, "odom")
                                print(global_pose.pose.orientation)
                                e = self.quaternion_to_euler(global_pose.pose.orientation)   #ラジア>ン
                                now_z = e.z * 180 / math.pi
                                if(now_z < 0):
                                        now_z += 360
                                print(now_z)   #度
                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                                print("WARNING: tf map to base_link not found.")
                                rospy.sleep(1)
                                continue
                        rate.sleep()
                print("result:", first, now_z)
                print("finished third motion====================================================================")


        def prepare_placing_object(self):
                rospy.loginfo("Start test!!!")
                self.get_tiago_information(True)   #全情報の取得
                self.move_group.set_planner_id("arm[RRTConnectkConfigDefault]")
                self.move_group.allow_replanning(True)
                
                result = self.drop()
                if(not result):
                        rospy.loginfo("Trying to place with arm and torso")
                        self.move_group.set_planner_id("arm_torso[RRTConnectkConfigDefault]")
                        result = self.drop()
                
                if(result):
                        rospy.sleep(2.0)
                        rospy.loginfo("prepare_place result: SUCCESS")
                        rospy.loginfo("overview after execute prepare_place")
                        self.get_tiago_information()
                        gripper_goal = FollowJointTrajectoryGoal()
                        gripper_goal.trajectory.joint_names.append("gripper_left_finger_joint")
                        gripper_goal.trajectory.joint_names.append("gripper_right_finger_joint")
                        point = trajectory_msgs.msg.JointTrajectoryPoint()
                        point.positions = [0.04, 0.04]
                        point.velocities = [0.0, 0.0]
                        point.time_from_start = rospy.Duration(2.0)
                        gripper_goal.trajectory.points.append(point)
                        #How long it takes to start trajectory after the time that Action interface accepted message
                        gripper_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
                        rospy.loginfo("send gripper goal")
                        self.gripper_control_client.send_goal(gripper_goal)
                        self.gripper_control_client.wait_for_result()
                        result_gripper = self.gripper_control_client.get_result()
                        if(result_gripper.error_code == 0):
                                print("place result: SUCCESS")
                        else:
                                print("place result: FAILED")
                rospy.loginfo("Removing previous 'part' object")
                self.scene.remove_world_object("part")
                #self.prepare_robot() #prepare for next pick_and_place

        def drop(self):
                pose_goal = Pose()
                pose_goal.orientation.x = self.move_group.get_current_pose().pose.orientation.x   #エンドエフェクタの姿勢を取得 #No
                pose_goal.orientation.y = self.move_group.get_current_pose().pose.orientation.y
                pose_goal.orientation.z = self.move_group.get_current_pose().pose.orientation.z
                pose_goal.orientation.w = self.move_group.get_current_pose().pose.orientation.w
                pose_goal.position.x = self.move_group.get_current_pose().pose.position.x
                pose_goal.position.y = self.move_group.get_current_pose().pose.position.y
                h = float(np.random.randint(5, 15)) / 100   #ランダムな高さを[5, 15]の範囲で生成
                pose_goal.position.z = self.move_group.get_current_pose().pose.position.z + h
                print("DEBUG: height = ", h)
                self.move_group.set_position_target([pose_goal.position.x, pose_goal.position.y, pose_goal.position.z])

                constraints = Constraints()
                constraints.name = "fix_rotation"
                orientation_constraint = OrientationConstraint()
                orientation_constraint.header.frame_id = self.move_group.get_planning_frame()
                orientation_constraint.link_name = self.move_group.get_end_effector_link()
                #orientation_constraint.link_name = "arm_7_link"
                orientation_constraint.orientation = self.move_group.get_current_pose().pose.orientation
                orientation_constraint.absolute_x_axis_tolerance = 0.05
                orientation_constraint.absolute_y_axis_tolerance = 0.05
                orientation_constraint.absolute_z_axis_tolerance = 0.05
                orientation_constraint.weight = 1.0
                constraints.orientation_constraints.append(orientation_constraint)
                self.move_group.set_path_constraints(constraints)
                rospy.loginfo("Send goal of prepare_place!")
                result = self.move_group.go(wait=True)
                self.move_group.stop()
                self.move_group.clear_pose_targets()
                print(result)
                return result


	def place_object(self, object_pose):
                rospy.loginfo("Clearing octomap")
                self.clear_octomap_srv.call(EmptyRequest())
                possible_placings = self.sg.create_placings_from_object_pose(
                        object_pose)
                # Try only with arm
                rospy.loginfo("Trying to place using only arm")
                goal = createPlaceGoal(
                        object_pose, possible_placings, "arm", "part", self.links_to_allow_contact)
                rospy.loginfo("Sending goal")
                self.place_ac.send_goal(goal)
                rospy.loginfo("Waiting for result")

                self.place_ac.wait_for_result()
                result = self.place_ac.get_result()
                rospy.loginfo(str(moveit_error_dict[result.error_code.val]))

                if str(moveit_error_dict[result.error_code.val]) != "SUCCESS":
                        rospy.loginfo(
                                "Trying to place with arm and torso")
                        # Try with arm and torso
                        goal = createPlaceGoal(
                                object_pose, possible_placings, "arm_torso", "part", self.links_to_allow_contact)
                        rospy.loginfo("Sending goal")
                        self.place_ac.send_goal(goal)
                        rospy.loginfo("Waiting for result")

                        self.place_ac.wait_for_result()
                        result = self.place_ac.get_result()
                        rospy.logerr(str(moveit_error_dict[result.error_code.val]))

                # print result
                rospy.loginfo(
                        "Result: " +
                        str(moveit_error_dict[result.error_code.val]))
                rospy.loginfo("Removing previous 'part' object")
                self.scene.remove_world_object("part")

                return result.error_code.val


if __name__ == '__main__':
	rospy.init_node('pick_and_place_server')
	paps = PickAndPlaceServer()
        rospy.spin()
