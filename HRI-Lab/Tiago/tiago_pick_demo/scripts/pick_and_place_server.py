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
from spherical_grasps_server import SphericalGrasps
from actionlib import SimpleActionClient, SimpleActionServer
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult, MoveItErrorCodes
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal, PickUpPoseResult, PickUpPoseFeedback
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
from std_srvs.srv import Empty, EmptyRequest
from copy import deepcopy
from random import shuffle
import copy

#MoveItErrorCodesのkeyとvalueを交換したmoveit_error_dictを定義
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
	placeg.attached_object_name = target   #planning scene内でのオブジェクト名
	placeg.group_name = group   #placeに用いるグループ名
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
		self.scene = PlanningSceneInterface()   #ワールド管理用
		rospy.loginfo("Connecting to /get_planning_scene service")
		self.scene_srv = rospy.ServiceProxy(
			'/get_planning_scene', GetPlanningScene)   #service server:/get_planning_sceneのservice clientの宣言
		self.scene_srv.wait_for_service()   #Service serverの起動を待つ
		rospy.loginfo("Connected.")

		rospy.loginfo("Connecting to clear octomap service...")
		self.clear_octomap_srv = rospy.ServiceProxy(
			'/clear_octomap', Empty)   #service server:/clear_octmapのsercice clientの宣言。どこでこのサーバーを宣言しているのか不明<---------------------------------------------------------------------
		self.clear_octomap_srv.wait_for_service()   #Service serverの起動を待つ
		rospy.loginfo("Connected!")

		# Get the object size
        #rosparamからオブジェクトの各パラメータをロード(tiago_pick_demo/launch/pick_demo.launchに記載)
		self.object_height = rospy.get_param('~object_height')
		self.object_width = rospy.get_param('~object_width')
		self.object_depth = rospy.get_param('~object_depth')

		# Get the links of the end effector exclude from collisions
        #rosparamからエンドエフェクタのlink名をロード：衝突回避のプランニングにおいてこのエンドエフェクタの部分を除外する必要があるため
        #(tiago_pick_demo/config/pick_and_place_params.yamlに記載)
		self.links_to_allow_contact = rospy.get_param('~links_to_allow_contact', None)   #get_paramして内容がemptyだった場合にはNoneをデフォルト値として代入
		if self.links_to_allow_contact is None:
			rospy.logwarn("Didn't find any links to allow contacts... at param ~links_to_allow_contact")
		else:
			rospy.loginfo("Found links to allow contacts: " + str(self.links_to_allow_contact))

		self.pick_as = SimpleActionServer(
			'/pickup_pose', PickUpPoseAction,
			execute_cb=self.pick_cb, auto_start=False)   #Action server:/pickup_poseのAction serverを宣言(pick_demo用に独自定義:tiago_pick_demo/action/PickUpPose.action)
		self.pick_as.start()   #Action serverの起動

		self.place_as = SimpleActionServer(
			'create_placings_from_object_pose', PickUpPoseAction,
			execute_cb=self.place_cb, auto_start=False)   #Action server:/place_poseのAction serverを宣言(pick_demo用に独自定義:tiago_pick_demo/action/PickUpPose.action)
		self.place_as.start()   #Action serverの起動

	def pick_cb(self, goal):   #pick_asのコールバック関数だが、実際のプランニングシーンへの物体追加からプランニング、実行まではgrasp_object関数が担っており、実体としてはエラーコード管理用である
		"""
		:type goal: PickUpPoseGoal
		"""
		error_code = self.grasp_object(goal.object_pose)
		p_res = PickUpPoseResult()
		p_res.error_code = error_code
		if error_code != 1:   #error_code=1が成功のため、1なら1を返し、1以外ならサーバーの強制終了し、さらにそのエラーコードの値を返す
			self.pick_as.set_aborted(p_res)
		else:
			self.pick_as.set_succeeded(p_res)

	def place_cb(self, goal):   #place_asのコールバック関数だが、実際のプランニングシーンへの物体追加からプランニング、実行まではplace_object関数が担っており、実体としてはエラーコード管理用である
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
		rospy.loginfo(
			"Waiting for object '" + object_name + "'' to appear in planning scene...")
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
		self.scene.remove_attached_object("arm_tool_link")   #arm_tool_linkはどれのこと？
		self.scene.remove_world_object("part")   #planning sceneからロボットに接続されたオブジェクト名=partのオブジェクト(掴む物体)を削除
		self.scene.remove_world_object("table")   #planning sceneからロボットに接続されたオブジェクト名=tableのオブジェクト(机)を削除
		rospy.loginfo("Clearing octomap")
		self.clear_octomap_srv.call(EmptyRequest())   #このサービスがどこで宣言されているか不明なため、詳細な処理は調べる必要がある<---------------------------------------------------------------------
		rospy.sleep(2.0)  # Removing is fast
		rospy.loginfo("Adding new 'part' object")

		rospy.loginfo("Object pose: %s", object_pose.pose)
		object_pose.pose.position.z += 0.016   #marker検出座標に対するキャリブレーション
		
		#Add object description in scene
        #対象の物体を"part"と命名し、プランニングシーンのobject_poseの座標に中心が来るように、サイズを[self.object_depth, self.object_width, self.object_height]として追加
		self.scene.add_box("part", object_pose, (self.object_depth, self.object_width, self.object_height))   #default box
		#self.scene.add_cylinder("part", object_pose, self.object_height, 0.033)   #empty can
		#self.scene.add_sphere("part", object_pose, radius=0.031)   #tennis ball
		#self.scene.add_sphere("part", object_pose, radius=0.040)   #ball
		#self.scene.add_sylinder("part", object_pose, self.object_height, 0.0325)   #mugitya
		#self.scene.add_sylinder("part", object_pose, self.object_height, 0.0350)   #plastic cap
		rospy.loginfo("Second%s", object_pose.pose)
		table_pose = copy.deepcopy(object_pose)   #y座標はそのまま使えるためひとまずコピー

		#define a virtual table below the object
		#この辺の詳細な処理は研究ノートを参照
		table_height = object_pose.pose.position.z - 0.016 - self.object_height/2 + 0.015
		table_width  = 1.0
		table_depth  = 1.2
		table_pose.pose.position.x = 1.1
		table_pose.pose.position.z = table_height/2

		self.scene.add_box("table", table_pose, (table_depth, table_width, table_height))   #プランニングシーンにtable_poseを中心とした、サイズ(table_depth, table_width, table_height)の机を追加

		# # We need to wait for the object part to appear
		self.wait_for_planning_scene_object()   #partが追加されたかどうかを確認
		self.wait_for_planning_scene_object("table")   #tableが追加されたかどうかを確認

		# compute grasps
		possible_grasps = self.sg.create_grasps_from_object_pose(object_pose)   #class SphericalGrasps()のメソッド、これが最も重要な処理を行っている部分<---------------------------------------------------------------------
		self.pickup_ac   #??? これ必要か?
		goal = createPickupGoal(
			"arm_torso", "part", object_pose, possible_grasps, self.links_to_allow_contact)   #ゴール作成

		rospy.loginfo("Sending goal")
		self.pickup_ac.send_goal(goal)   #action serverにゴールを送信, 結局pickup action serverの中身ってどこに定義されているのか<---------------------------------------------------------------------
		rospy.loginfo("Waiting for result")
		self.pickup_ac.wait_for_result()   #resultが帰ってくるまで待機
		result = self.pickup_ac.get_result()   #resultを取得
		rospy.logdebug("Using torso result: " + str(result))
		rospy.loginfo(
			"Pick result: " +
		str(moveit_error_dict[result.error_code.val]))

		return result.error_code.val   #moveitのerror codeを返却
	
	def prepare_placing_object(self):
		rospy.loginfo("")
		

	def place_object(self, object_pose):
		rospy.loginfo("Clearing octomap")
		self.clear_octomap_srv.call(EmptyRequest())   #このサービスがどこで宣言されているか不明なため、詳細な処理は調べる必要がある<---------------------------------------------------------------------
		possible_placings = self.sg.create_placings_from_object_pose(
			object_pose)   #class SphericalGrasps()のメソッド、これが最も重要な処理を行っている部分<---------------------------------------------------------------------
		# Try only with arm
		rospy.loginfo("Trying to place using only arm")
		goal = createPlaceGoal(
			object_pose, possible_placings, "arm", "part", self.links_to_allow_contact)   #ゴール作成、ただしこちらはpickの際のgoalと違い、planning groupは"arm_torso"ではなく"arm"を指定
		rospy.loginfo("Sending goal")
		self.place_ac.send_goal(goal)   #action serverにゴールを送信, 結局place action serverの中身ってどこに定義されているのか<---------------------------------------------------------------------
		rospy.loginfo("Waiting for result")

		self.place_ac.wait_for_result()   #resultが返却されるのを待つ
		result = self.place_ac.get_result()   #resultの取得
		rospy.loginfo(str(moveit_error_dict[result.error_code.val]))

		if str(moveit_error_dict[result.error_code.val]) != "SUCCESS":   #もしarmだけでやって失敗したらarm_torsoで二回目を実行する
			rospy.loginfo(
				"Trying to place with arm and torso")
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
		rospy.loginfo(
			"Result: " +
			str(moveit_error_dict[result.error_code.val]))
		rospy.loginfo("Removing previous 'part' object")
		self.scene.remove_world_object("part")   #planning sceneからロボットに接続されたオブジェクト名=partのオブジェクト(掴む物体)のみを削除。これは継続して新たな物体のpick, placeを実行する際に既存のオブジェクトは考慮する必要があるため、tableは残していると考えられる

		return result.error_code.val


if __name__ == '__main__':
	rospy.init_node('pick_and_place_server')
	paps = PickAndPlaceServer()
	rospy.spin()
