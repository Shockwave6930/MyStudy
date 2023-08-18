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
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult, MoveItErrorCodes
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation, OrientationConstraint, Constraints
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal, PickUpPoseResult, PickUpPoseFeedback
import trajectory_msgs.msg
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
from std_srvs.srv import Empty, EmptyRequest
from copy import deepcopy
from random import shuffle
import copy
import math
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Twist

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
	if not name[:1] == '_':
		code = MoveItErrorCodes.__dict__[name]
		moveit_error_dict[code] = name


def createPickupGoal(group="arm_torso", target="part",
					 grasp_pose=PoseStamped(),
					 possible_grasps=[],
					 links_to_allow_contact=None):
	""" Create a PickupGoal with the provided data"""
	pug = PickupGoal()
	pug.target_name = target   #planning scene内でのオブジェクト名
	pug.group_name = group   #pickupに用いるグループ名
	pug.possible_grasps.extend(possible_grasps)   #possoble_graspsの内容を追加
	pug.allowed_planning_time = 35.0   #motion plannerのplannningの制限時間
	pug.planning_options.planning_scene_diff.is_diff = True   #planning sceneで差分を考慮する
	pug.planning_options.planning_scene_diff.robot_state.is_diff = True   #このシーンを他のシーンとの差分として解釈する
	pug.planning_options.plan_only = False   #実行可能なプランをそのまま実行
	pug.planning_options.replan = True   #実行中にプランが無効になった場合、そのプランを再計算して実行を再開する
	pug.planning_options.replan_attempts = 1  # 10   #再計画の最大試行回数
	pug.allowed_touch_objects = []   #把握の過程で触って/押して/移動できる障害物のリスト
	pug.attached_object_touch_links = ['<octomap>']   #付加するオブジェクトが触れることを許可するリンクの名前
	pug.attached_object_touch_links.extend(links_to_allow_contact)   #links_to_allow_contactと結合

	return pug


def createPlaceGoal(place_pose,
					place_locations,
					group="arm_torso",
					target="part",
					links_to_allow_contact=None):
	"""Create PlaceGoal with the provided data"""
	placeg = PlaceGoal()
	placeg.group_name = group   #planning scene内でのオブジェクト名
	placeg.attached_object_name = target   #placeに用いるグループ名
	placeg.place_locations = place_locations   #place_locationで掴む際の姿勢、エンドエフェクタの位置、アプローチモーション、後退モーション、掴む過程で触って/押して/移動できる障害物のリスト
	placeg.allowed_planning_time = 15.0   #motion plannerのplannningの制限時間
	placeg.planning_options.planning_scene_diff.is_diff = True   #planning sceneで差分を考慮する
	placeg.planning_options.planning_scene_diff.robot_state.is_diff = True   #このシーンを他のシーンとの差分として解釈する
	placeg.planning_options.plan_only = False   #実行可能なプランをそのまま実行
	placeg.planning_options.replan = True   #実行中にプランが無効になった場合、そのプランを再計算して実行を再開する
	placeg.planning_options.replan_attempts = 1   #再計画の最大試行回数
	placeg.allowed_touch_objects = ['<octomap>']   #付加するオブジェクトが触れることを許可するリンクの名前
	placeg.allowed_touch_objects.extend(links_to_allow_contact)   #links_to_allow_contactと結合

	return placeg

class PickAndPlaceServer(object):
	def __init__(self):
		rospy.loginfo("Initalizing PickAndPlaceServer...")
		self.sg = SphericalGrasps()
		rospy.loginfo("Connecting to pickup AS")
		self.pickup_ac = SimpleActionClient('/pickup', PickupAction)   #Action server:/pickupのAction clientを宣言<---------------------------------------------------------------------
		self.pickup_ac.wait_for_server()   #Action serverの起動を待つ
		rospy.loginfo("Succesfully connected.")
		rospy.loginfo("Connecting to place AS")
		self.place_ac = SimpleActionClient('/place', PlaceAction)   #Action server:/placeのAction clientを宣言<---------------------------------------------------------------------
		self.place_ac.wait_for_server()   #Action serverの起動を待つ
		rospy.loginfo("Succesfully connected.")
		rospy.loginfo("Connecting to gripper_controller")
		self.gripper_control_client = actionlib.SimpleActionClient("/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
		self.gripper_control_client.wait_for_server()
		rospy.loginfo("Successfully conttented.")
		self.scene = PlanningSceneInterface()               ######################################################################################プランニングシーンを更新するためのインターフェース
		self.robot = RobotCommander()                       ######################################################################################ロボットの状態管理用
		self.move_group = MoveGroupCommander("arm_torso")   ######################################################################################指定されたグループに対してコマンドを実行
		rospy.loginfo("Connecting to /get_planning_scene service")
		self.scene_srv = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)   #service server:/get_planning_sceneのservice clientの宣言
		self.scene_srv.wait_for_service()   #Service serverの起動を待つ
		rospy.loginfo("Connected.")

		rospy.loginfo("Connecting to clear octomap service...")
		self.clear_octomap_srv = rospy.ServiceProxy('/clear_octomap', Empty)   #service server:/clear_octmapのsercice clientの宣言。どこでこのサーバーを宣言しているのか不明<---------------------------------------------------------------------
		self.clear_octomap_srv.wait_for_service()   #Service serverの起動を待つ
		rospy.loginfo("Connected!")

		#rosparamからオブジェクトの各パラメータをロード(tiago_pick_demo/launch/pick_demo.launchに記載)
		self.object_height = rospy.get_param('~object_height')
		self.object_width = rospy.get_param('~object_width')
		self.object_depth = rospy.get_param('~object_depth')

		#rosparamからエンドエフェクタのlink名をロード：衝突回避のプランニングにおいてこのエンドエフェクタの部分を除外する必要があるため
        #(tiago_pick_demo/config/pick_and_place_params.yamlに記載)
		self.links_to_allow_contact = rospy.get_param('~links_to_allow_contact', None)   #get_paramして内容がemptyだった場合にはNoneをデフォルト値として代入
		if self.links_to_allow_contact is None:
			rospy.logwarn("Didn't find any links to allow contacts... at param ~links_to_allow_contact")
		else:
			rospy.loginfo("Found links to allow contacts: " + str(self.links_to_allow_contact))
		
		self.pick_as = SimpleActionServer('/pickup_pose', PickUpPoseAction, execute_cb=self.pick_cb, auto_start=False)   #pick_client.pyではこのアクショントピックにゴールを送信することでpick_and_place_server.pyで定義されているオートマニピュレーションのための動作を実行することでpickを行っている
		self.pick_as.start()   #Action server:/pickup_poseのAction serverを宣言(pick_demo用に独自定義:tiago_pick_demo/action/PickUpPose.action)

		self.place_as = SimpleActionServer('/place_pose', PickUpPoseAction, execute_cb=self.place_cb, auto_start=False)   #Action server:/place_poseのAction serverを宣言(pick_demo用に独自定義:tiago_pick_demo/action/PickUpPose.action)
		self.place_as.start()   #Action serverの起動
		##self.prepare_placing_object() ###trying
		self.move_base_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
		self.tfBuffer = tf2_ros.Buffer()
		self.listerner = tf2_ros.TransformListener(self.tfBuffer)
		#self.get_tiago_information(True)
		self.push()

	def pick_cb(self, goal):   #pick_asのコールバック関数だが、実際のプランニングシーンへの物体追加からプランニング、実行まではgrasp_object関数が担っており、実体としてはエラーコード管理用である
		"""
		:type goal: PickUpPoseGoal
		"""
		error_code = self.grasp_object(goal.object_pose)
		p_res = PickUpPoseResult()
		p_res.error_code = error_code
		if error_code != 1:   #error_code=1が成功のため、1なら1を返し、1以外ならサーバーの強制終了し、さらにそのエラーコードの値を返す
			self.pick_as.set_aborted(p_res)   #サーバーの強制終了
		else:
			self.pick_as.set_succeeded(p_res)   #Resultの返却

	def place_cb(self, goal):   #place_asのコールバック関数だが、実際のプランニングシーンへの物体追加からプランニング、実行まではgrasp_object関数が担っており、実体としてはエラーコード管理用である
		"""
		:type goal: PickUpPoseGoal
		"""
		error_code = self.place_object(goal.object_pose)
		p_res = PickUpPoseResult()
		p_res.error_code = error_code
		if error_code != 1:   #error_code=1が成功のため、1なら1を返し、1以外ならサーバーの強制終了し、さらにそのエラーコードの値を返す
			self.place_as.set_aborted(p_res)   #サーバーの強制終了
		else:
			self.place_as.set_succeeded(p_res)   #Resultの返却

	def wait_for_planning_scene_object(self, object_name='part'):   #プランニングシーンにobject_nameの物体が追加されたことが確認できるまでwhileループし続ける関数、チェック用
		rospy.loginfo("Waiting for object '" + object_name + "'' to appear in planning scene...")
		gps_req = GetPlanningSceneRequest()   #<---------------------------------------------------------------------
		gps_req.components.components = gps_req.components.WORLD_OBJECT_NAMES   #<---------------------------------------------------------------------
		
		part_in_scene = False
		while not rospy.is_shutdown() and not part_in_scene:
			# This call takes a while when rgbd sensor is set
			gps_resp = self.scene_srv.call(gps_req)   #<---------------------------------------------------------------------
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
		self.scene.remove_world_object("part")   #planning sceneからロボットに接続されたオブジェクト名=partのオブジェクト(掴む物体)を削除
		self.scene.remove_world_object("table")   #planning sceneからロボットに接続されたオブジェクト名=tableのオブジェクト(机)を削除
		rospy.loginfo("Clearing octomap")
		self.clear_octomap_srv.call(EmptyRequest())   #このサービスがどこで宣言されているか不明なため、詳細な処理は調べる必要がある<---------------------------------------------------------------------
		rospy.sleep(2.0)  # Removing is fast
		rospy.loginfo("Adding new 'part' object")

		rospy.loginfo("Object pose: %s", object_pose.pose)
		
		#Add object description in scene
		#対象の物体を"part"と命名し、プランニングシーンのobject_poseの座標に中心が来るように、サイズを[self.object_depth, self.object_width, self.object_height]として追加
		self.scene.add_box("part", object_pose, (self.object_depth, self.object_width, self.object_height))

		rospy.loginfo("Second%s", object_pose.pose)
		table_pose = copy.deepcopy(object_pose)   #y座標はそのまま使えるためひとまずコピー

		#define a virtual table below the object
		table_height = object_pose.pose.position.z - self.object_width/2  
		table_width  = 1.8
		table_depth  = 0.5
		table_pose.pose.position.z += -(2*self.object_width)/2 -table_height/2
		table_height -= 0.008 #remove few milimeters to prevent contact between the object and the table

		self.scene.add_box("table", table_pose, (table_depth, table_width, table_height))   #プランニングシーンにtable_poseを中心とした、サイズ(table_depth, table_width, table_height)の机を追加

		# # We need to wait for the object part to appear
		self.wait_for_planning_scene_object()   #partが追加されたかどうかを確認
		self.wait_for_planning_scene_object("table")   #tableが追加されたかどうかを確認

                # compute grasps
		possible_grasps = self.sg.create_grasps_from_object_pose(object_pose)   #class SphericalGrasps()のメソッド、これが最も重要な処理を行っている部分<---------------------------------------------------------------------
		self.pickup_ac   #??? これ必要か?
		goal = createPickupGoal("arm_torso", "part", object_pose, possible_grasps, self.links_to_allow_contact)   #ゴール作成
		
		rospy.loginfo("Sending goal")
		self.pickup_ac.send_goal(goal)   #action serverにゴールを送信, 結局pickup action serverの中身ってどこに定義されているのか<---------------------------------------------------------------------
		rospy.loginfo("Waiting for result")
		self.pickup_ac.wait_for_result()   #resultが帰ってくるまで待機
		result = self.pickup_ac.get_result()   #resultを取得
		rospy.logdebug("Using torso result: " + str(result))
		rospy.loginfo("Pick result: " + str(moveit_error_dict[result.error_code.val]))
		rospy.loginfo("Sleeping in 5 seconds!")
		rospy.sleep(5.0)
		#destination_pose = Pose()
		#destination_pose.position.x = self.move_group.get_current_pose().pose.position.x
		#destination_pose.position.y = self.move_group.get_current_pose().pose.position.y
		#destination_pose.position.z = object_pose.pose.position.z + 0.05
		#self.prepare_placing_object()   #そのままplace動作に入る
		return result.error_code.val   #moveitのerror codeを返却

	def euler_to_quaternion(self, euler):
		"""Convert Euler Angles to Quaternion
		euler: geometry_msgs/Vector3
		quaternion: geometry_msgs/Quaternion
		"""
		q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
		return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

	def quaternion_to_euler(self, quaternion):   #指定された角度回転するまで/cmd_velにメッセージを送信し続ける
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
			print("Printing robot state:", self.robot.get_current_state())   #tiago全体の現在の状態を取得
			print("Printing initially setting group as default:", self.move_group.get_name())   #現在設定されているグループ名を取得
			print("Printing pose reference frame:", self.move_group.get_pose_reference_frame())   #エンドエフェクタの姿勢表現に使われる基準座標を取得
			#print("Printing trajectory constraints:", self.move_group.get_trajectory_constraints())
			print("Printing end-effector pose:", self.move_group.get_current_pose())   #エンドエフェクタのポジションを取得
			print("Printing end-effector rpy:", self.move_group.get_current_rpy())   #エンドエフェクタの姿勢を取得
			print("Printing current joint values:", self.move_group.get_current_joint_values())   #現在の関節角度をリストとして取得
			print("Printing planner's id list:", self.move_group.get_interface_description())   #プランナーIDのリストを取得
			print("Printing constraints' name:", self.move_group.get_known_constraints())   #現在紐づけられている制約の名前のリストを取得
			print("Printing get path constraints:", self.move_group.get_path_constraints())   #現在設定されているパス制約を取得
			print("Printing planner id:", self.move_group.get_planner_id())   #現在設定されているプランナーIDを取得
			print("Printing planning time:", self.move_group.get_planning_time())   #プランニング時間を取得
		else:
			rospy.loginfo("==============================================================visualize main parameters==============================================================")
			print("Printing end-effector pose:", self.move_group.get_current_pose())   #エンドエフェクタのポジションを取得
			print("Printing end-effector rpy:", self.move_group.get_current_rpy())   #エンドエフェクタの姿勢を取得
			print("Printing current joint values:", self.move_group.get_current_joint_values())   #現在の関節角度をリストとして取得
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
				flag = True
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				rospy.sleep(0.1)
				continue
		now_z = first_z
		rate = rospy.Rate(3) # 10hz
		while not rospy.is_shutdown() and abs(now_z - first_z) < 90:
			msg = Twist()
			msg.linear.x = 0
			msg.linear.y = 0
			msg.linear.z = 0
			msg.angular.x = 0
			msg.angular.y = 0
			msg.angular.z = 0.3
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
				e = self.quaternion_to_euler(global_pose.pose.orientation)   #ラジアン
				now_z = e.z * 180 / math.pi
				print(now_z)   #度
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				print("WARNING: tf map to base_link not found.")
				rospy.sleep(1)
				continue
			rate.sleep()

	def prepare_placing_object(self):
		#self.clear_octomap_srv.call(EmptyRequest())
		self.get_tiago_information(True)   #全情報の取得
		self.move_group.set_planner_id("arm_torso[RRTConnectkConfigDefault]")
		self.move_group.allow_replanning(True)
		#joint_goal = move_group.get_current_joint_values()
		#joint_goal[0] = 0.34   #torso_lift_joint
		#joint_goal[1] = 0.08   #arm_1_joint
		#joint_goal[2] = -0.20   #arm_2_joint
		#joint_goal[3] = -1.44   #arm_3_joint
		#joint_goal[4] = 1.63   #arm_4_joint
		#joint_goal[5] = -1.82   #arm_5_joint
		#joint_goal[6] = 1.27   #arm_6_joint
		#joint_goal[7] = -1.81   #arm_7_joint
		#move_group.go(joint_goal, wait=True)
		#euler = Vector3(0, 0, 3.14)
		#orientation = self.euler_to_quaternion(euler)
		#Pose goalの作成
		pose_goal = Pose()
		pose_goal.orientation.x = self.move_group.get_current_pose().pose.orientation.x   #エンドエフェクタの姿勢を取得
		pose_goal.orientation.y = self.move_group.get_current_pose().pose.orientation.y
		pose_goal.orientation.z = self.move_group.get_current_pose().pose.orientation.z
		pose_goal.orientation.w = self.move_group.get_current_pose().pose.orientation.w
		pose_goal.position.x = self.move_group.get_current_pose().pose.position.x 
		pose_goal.position.y = self.move_group.get_current_pose().pose.position.y
		pose_goal.position.z = self.move_group.get_current_pose().pose.position.z + 0.05
		#pose_goal.position.x = object_pose.position.x
		#pose_goal.position.y = object_pose.position.y
		#pose_goal.position.z = object_pose.position.z
		#print(object_pose)
		#print(self.object_height)
		print("Printing current pose goal1:", pose_goal)
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
		rospy.loginfo("Send goal1!")
		result = self.move_group.go(wait=True)
		print("prepare_place result:", result)
		self.move_group.stop()
		self.move_group.clear_pose_targets()
		rospy.loginfo("overview agter execute goal1")
		self.get_tiago_information()

		gripper_goal = FollowJointTrajectoryGoal()
		gripper_goal.trajectory.joint_names.append("gripper_left_finger_joint")
		gripper_goal.trajectory.joint_names.append("gripper_right_finger_joint")
		point = trajectory_msgs.msg.JointTrajectoryPoint()
		point.positions = [0.04, 0.04]
		point.velocities = [0.0, 0.0]
		point.time_from_start = rospy.Duration(3.0)
		gripper_goal.trajectory.points.append(point)
		rospy.loginfo("Opening gripper")
		#How long it takes to start trajectory after the time that Action interface accepted message
		gripper_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
		self.gripper_control_client.send_goal(gripper_goal)
		self.gripper_control_client.wait_for_result()
		result_gripper = self.gripper_control_client.get_result()
		print("place result:", result_gripper)

		#print("========================================================================================================\n========================================================================================================\n========================================================================================================\n========================================================================================================\n========================================================================================================\n========================================================================================================n========================================================================================================\n========================================================================================================n========================================================================================================\n")
		##euler = Vector3(0, 0, 3.14)
		##orientation = self.euler_to_quaternion(euler)
		#pose_goal = Pose()
		#pose_goal.orientation.x = self.move_group.get_current_pose().pose.orientation.x
		#pose_goal.orientation.y = self.move_group.get_current_pose().pose.orientation.y
		#pose_goal.orientation.z = self.move_group.get_current_pose().pose.orientation.z
		#pose_goal.orientation.w = self.move_group.get_current_pose().pose.orientation.w
		#pose_goal.position.x = self.move_group.get_current_pose().pose.position
		#pose_goal.position.y = self.move_group.get_current_pose().pose.position
		#pose_goal.position.z = self.move_group.get_current_pose().pose.position 
		#print("Printing current pose goal2:", pose_goal)
		#self.move_group.set_pose_target(pose_goal)
		#constraints = Constraints()
		#constraints.name = "lock_rotation"
		#orientation_constraint = OrientationConstraint()
		#orientation_constraint.header.frame_id = self.move_group.get_planning_frame()
		#orientation_constraint.link_name = self.move_group.get_end_effector_link()
		##orientation_constraint.link_name = "arm_7_link"
		#orientation_constraint.orientation = self.move_group.get_current_pose().pose.orientation
		#orientation_constraint.absolute_x_axis_tolerance = 0.05
		#orientation_constraint.absolute_y_axis_tolerance = 0.05
		#orientation_constraint.absolute_z_axis_tolerance = 0.05
		#orientation_constraint.weight = 1.0
		#constraints.orientation_constraints.append(orientation_constraint)
		#self.move_group.set_path_constraints(constraints)
		#plan = self.move_group.go(wait=True)
		#rospy.loginfo("Sent goal2!")
		## Calling `stop()` ensures that there is no residual movement
		#self.move_group.stop()
		## It is always good to clear your targets after planning with poses.
		## Note: there is no equivalent function for clear_joint_value_targets()
		#self.move_group.clear_pose_targets()
		#rospy.loginfo("overview agter execute goal2")
		#self.get_tiago_information()

	def place_object(self, object_pose):
		rospy.loginfo("Clearing octomap")
		self.clear_octomap_srv.call(EmptyRequest())   #このサービスがどこで宣言されているか不明なため、詳細な処理は調べる必要がある<---------------------------------------------------------------------
		possible_placings = self.sg.create_placings_from_object_pose(object_pose)   #class SphericalGrasps()のメソッド、これが最も重要な処理を行っている部分<---------------------------------------------------------------------
		# Try only with arm
		rospy.loginfo("Trying to place using only arm")
		goal = createPlaceGoal(object_pose, possible_placings, "arm", "part", self.links_to_allow_contact)   #ゴール作成、ただしこちらはpickの際のgoalと違い、planning groupは"arm_torso"ではなく"arm"を指定
		rospy.loginfo("Sending goal")
		self.place_ac.send_goal(goal)   #action serverにゴールを送信, 結局place action serverの中身ってどこに定義されているのか<---------------------------------------------------------------------
		rospy.loginfo("Waiting for result")

		self.place_ac.wait_for_result()   #resultが返却されるのを待つ
		result = self.place_ac.get_result()   #resultの取得
		rospy.loginfo(str(moveit_error_dict[result.error_code.val]))

		if str(moveit_error_dict[result.error_code.val]) != "SUCCESS":   #もしarmだけでやって失敗したらarm_torsoで二回目を実行する
			rospy.loginfo("Trying to place with arm and torso")
			# Try with arm and torso
			goal = createPlaceGoal(
				object_pose, possible_placings, "arm_torso", "part", self.links_to_allow_contact)   #ゴール作成、今度はplanning groupをarm_torsoに指定
			rospy.loginfo("Sending goal")
			self.place_ac.send_goal(goal)   #action serverにゴールを送信, 結局place action serverの中身ってどこに定義されているのか<---------------------------------------------------------------------
			rospy.loginfo("Waiting for result")

			self.place_ac.wait_for_result()   #resultが返却されるのを待つ
			result = self.place_ac.get_result()   #resultの取得
			rospy.logerr(str(moveit_error_dict[result.error_code.val]))

		# print result
		#準備動作の変更はここを変更した上で初期状態への関数呼び出しで調整できるはず
		rospy.loginfo("Result: " +str(moveit_error_dict[result.error_code.val]))
		rospy.loginfo("Removing previous 'part' object")
		self.scene.remove_world_object("part")   #planning sceneからロボットに接続されたオブジェクト名=partのオブジェクト(掴む物体)のみを削除。これは継続して新たな物体のpick, placeを実行する際に既存のオブジェクトは考慮する必要があるため、tableは残していると考えられる

		return result.error_code.val


if __name__ == '__main__':
	rospy.init_node('pick_and_place_server')
	paps = PickAndPlaceServer()
	rospy.spin()
