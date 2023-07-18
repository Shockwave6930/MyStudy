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
import numpy as np
import math
from math import radians, pi
import copy
from copy import deepcopy

import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion, Point
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.msg import Grasp, GripperTranslation
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation
from visualization_msgs.msg import MarkerArray, Marker

from tf import transformations
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, unit_vector, quaternion_multiply

from dynamic_reconfigure.server import Server
from tiago_pick_demo.cfg import SphericalGraspConfig

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

# http://stackoverflow.com/questions/17044296/quaternion-rotation-without-euler-angles

def quaternion_from_vectors(v0, v1):
    if type(v0) == Point():
        v0 = [v0.x, v0.y, v0.z]
    if type(v1) == Point():
        v1 = [v1.x, v1.y, v1.z]

    v0 = normalize(v0)
    v1 = normalize(v1)
    c = np.cross(v0, v1)
    d = np.dot(v0, v1)
    try:
        s = math.sqrt((1.0 + d) * 2)
    except ValueError:
        s = 0.0
    if s == 0.0:
        # print "s == 0.0, we cant compute"
        return None  # [0.0, 0.0, 0.0, 1.0]

    q = [0.0, 0.0, 0.0, 0.0]
    q[0] = c[0] / s
    q[1] = c[1] / s
    q[2] = c[2] / s
    q[3] = s / 2.0
    return q

def filter_poses(sphere_poses, object_pose,
                 filter_behind=False,
                 filter_under=True):
    """Given the generated poses and the object pose
    filter out the poses that are behind or under (if set to True)
    :type sphere_poses: []
        list of Pose
    :type object_pose: PoseStamped
    :rtype: []"""
    new_list = []
    for pose in sphere_poses:
        # if pose is further away than object, ditch it
        if filter_behind:
            if pose.position.x > object_pose.pose.position.x:
                continue
        # if pose if under the object, ditch it
        if filter_under:
            if pose.position.z < object_pose.pose.position.z:
                continue

        new_list.append(pose)
    return new_list


def sort_by_height(sphere_poses):
    # We prefer to grasp from top to be safer
    newlist = sorted(
        sphere_poses, key=lambda item: item.position.z, reverse=False)
    sorted_list = newlist
    return sorted_list


class SphericalGrasps(object):
    def __init__(self):
        rospy.loginfo("Initializing SphericalGrasps...")
        # Get server parameters from param server by using dynamic reconfigure
        # This is an advantage as you can test your grasp configuration
        # dynamically
        self.dyn_rec_srv = Server(SphericalGraspConfig, self.dyn_rec_callback)

        # Setup Markers for debugging
        self.poses_pub = rospy.Publisher(
            '/sphere_poses', PoseArray, latch=True)
        self.grasps_pub = rospy.Publisher(
            '/grasp_poses', PoseArray, latch=True)
        self.object_pub = rospy.Publisher(
            '/object_marker', Marker, latch=True)

        rospy.loginfo("SphericalGrasps initialized!")

    def dyn_rec_callback(self, config, level):

        rospy.loginfo("Received reconf call: " + str(config))
        self._grasp_postures_frame_id = config["grasp_postures_frame_id"]
        self._gripper_joint_names = config["gripper_joint_names"]
        self._gripper_pre_grasp_positions = config[
            "gripper_pre_grasp_positions"]
        self._gripper_grasp_positions = config["gripper_grasp_positions"]
        self._time_pre_grasp_posture = config["time_pre_grasp_posture"]
        self._time_grasp_posture = config["time_grasp_posture"]
        self._time_grasp_posture_final = config["time_grasp_posture_final"]
        self._grasp_pose_frame_id = config["grasp_pose_frame_id"]

        self._grasp_desired_distance = config["grasp_desired_distance"]
        self._grasp_min_distance = config["grasp_min_distance"]

        self._pre_grasp_direction_x = config["pre_grasp_direction_x"]
        self._pre_grasp_direction_y = config["pre_grasp_direction_y"]
        self._pre_grasp_direction_z = config["pre_grasp_direction_z"]

        self._post_grasp_direction_x = config["post_grasp_direction_x"]
        self._post_grasp_direction_y = config["post_grasp_direction_y"]
        self._post_grasp_direction_z = config["post_grasp_direction_z"]

        self._grasp_quality = config["grasp_quality"]
        self._max_contact_force = config["max_contact_force"]
        self._allowed_touch_objects = config["allowed_touch_objects"]

        self._fix_tool_frame_to_grasping_frame_roll = config[
            "fix_tool_frame_to_grasping_frame_roll"]
        self._fix_tool_frame_to_grasping_frame_pitch = config[
            "fix_tool_frame_to_grasping_frame_pitch"]
        self._fix_tool_frame_to_grasping_frame_yaw = config[
            "fix_tool_frame_to_grasping_frame_yaw"]

        self._step_degrees_yaw = config["step_degrees_yaw"]
        self._step_degrees_pitch = config["step_degrees_pitch"]
        self._min_degrees_yaw = config["min_degrees_yaw"]
        self._max_degrees_yaw = config["max_degrees_yaw"]
        self._min_degrees_pitch = config["min_degrees_pitch"]
        self._max_degrees_pitch = config["max_degrees_pitch"]
        print("dyn_rec_callback関数, self._grasp_postures_frame_id:", self._grasp_postures_frame_id)   #debug #arm_tool_link
        print("dyn_rec_callback関数, self._gripper_joint_names:", self._gripper_joint_names)   #debug #'gripper_left_finger_joint gripper_right_finger_joint')
        print("dyn_rec_callback関数, self._gripper_pre_grasp_positions:", self._gripper_pre_grasp_positions)   #debug
        print("dyn_rec_callback関数, self._gripper_grasp_positions:", self._gripper_grasp_positions)   #debug
        print("dyn_rec_callback関数, self._time_pre_grasp_posture:", self._time_pre_grasp_posture)   #debug
        print("dyn_rec_callback関数, self._time_grasp_posture:", self._time_grasp_posture)   #debug
        print("dyn_rec_callback関数, self._time_grasp_posture_final:", self._time_grasp_posture_final)   #debug
        print("dyn_rec_callback関数, self._grasp_pose_frame_id:", self._grasp_pose_frame_id)   #debug
        print("dyn_rec_callback関数, self._grasp_desired_distance:", self._grasp_desired_distance)   #debug
        print("dyn_rec_callback関数, self._grasp_min_distance:", self._grasp_min_distance)   #debug
        print("dyn_rec_callback関数, self._pre_grasp_direction_x:", self._pre_grasp_direction_x)   #debug
        print("dyn_rec_callback関数, self._pre_grasp_direction_y:", self._pre_grasp_direction_y)   #debug
        print("dyn_rec_callback関数, self._pre_grasp_direction_z:", self._pre_grasp_direction_z)   #debug
        print("dyn_rec_callback関数, self._post_grasp_direction_x:", self._post_grasp_direction_x)   #debug
        print("dyn_rec_callback関数, self._post_grasp_direction_y:", self._post_grasp_direction_y)   #debug
        print("dyn_rec_callback関数, self._post_grasp_direction_z:", self._post_grasp_direction_z)   #debug
        print("dyn_rec_callback関数, self._grasp_quality:", self._grasp_quality)   #debug
        print("dyn_rec_callback関数, self._max_contact_force:", self._max_contact_force)   #debug
        print("dyn_rec_callback関数, self._allowed_touch_objects:", self._allowed_touch_objects)   #debug
        print("dyn_rec_callback関数, self._fix_tool_frame_to_grasping_frame_roll:", self._fix_tool_frame_to_grasping_frame_roll)   #debug
        print("dyn_rec_callback関数, self._fix_tool_frame_to_grasping_frame_pitch:", self._fix_tool_frame_to_grasping_frame_pitch)   #debug
        print("dyn_rec_callback関数, self._fix_tool_frame_to_grasping_frame_yaw:", self._fix_tool_frame_to_grasping_frame_yaw)   #debug
        return config

    def generate_grasp_poses(self, object_pose):
        # Compute all the points of the sphere with step X
        # http://math.stackexchange.com/questions/264686/how-to-find-the-3d-coordinates-on-a-celestial-spheres-surface
        radius = self._grasp_desired_distance
        ori_x = 0.0
        ori_y = 0.0
        ori_z = 0.0
        sphere_poses = []
        rotated_q = quaternion_from_euler(0.0, 0.0, math.radians(180))

        yaw_qtty = int((self._max_degrees_yaw - self._min_degrees_yaw) / self._step_degrees_yaw)  # NOQA
        pitch_qtty = int((self._max_degrees_pitch - self._min_degrees_pitch) / self._step_degrees_pitch)  # NOQA
        info_str = "Creating poses with parameters:\n" + \
            "Radius: " + str(radius) + "\n" \
            "Yaw from: " + str(self._min_degrees_yaw) + \
            " to " + str(self._max_degrees_yaw) + " with step " + \
            str(self._step_degrees_yaw) + " degrees.\n" + \
            "Pitch from: " + str(self._min_degrees_pitch) + \
            " to " + str(self._max_degrees_pitch) + \
            " with step " + str(self._step_degrees_pitch) + " degrees.\n" + \
            "Total: " + str(yaw_qtty) + " yaw * " + str(pitch_qtty) + \
            " pitch = " + str(yaw_qtty * pitch_qtty) + " grap poses."
        rospy.loginfo(info_str)

        # altitude is yaw
        for altitude in range(self._min_degrees_yaw, self._max_degrees_yaw, self._step_degrees_yaw):  # NOQA
            altitude = math.radians(altitude)
            # azimuth is pitch
            for azimuth in range(self._min_degrees_pitch, self._max_degrees_pitch, self._step_degrees_pitch):  # NOQA
                azimuth = math.radians(azimuth)
                # This gets all the positions
                x = ori_x + radius * math.cos(azimuth) * math.cos(altitude)
                y = ori_y + radius * math.sin(altitude)
                z = ori_z + radius * math.sin(azimuth) * math.cos(altitude)
                # this gets all the vectors pointing outside of the center
                # quaternion as x y z w
                q = quaternion_from_vectors([radius, 0.0, 0.0], [x, y, z])
                # Cannot compute so the vectors are parallel
                if q is None:
                    # with this we add the missing arrow
                    q = rotated_q
                # We invert the orientations to look inwards by multiplying
                # with a quaternion 180deg rotation on yaw
                q = quaternion_multiply(q, rotated_q)

                # We actually want roll to be always 0.0 so we approach
                # the object with the gripper always horizontal
                # this rotation can be tuned with the dynamic params
                # multiplying later on
                roll, pitch, yaw = euler_from_quaternion(q)
                q = quaternion_from_euler(math.radians(0.0), pitch, yaw)

                x += object_pose.pose.position.x
                y += object_pose.pose.position.y
                z += object_pose.pose.position.z
                current_pose = Pose(
                    Point(x, y, z), Quaternion(*q))
                sphere_poses.append(current_pose)
        return sphere_poses

    def publish_grasps(self, grasps):   #Publishing PoseArrays for debugging pourposes
        pa = PoseArray()
        pa.header.frame_id = self._grasp_pose_frame_id
        pa.header.stamp = rospy.Time.now()
        for grasp in grasps:
            pa.poses.append(grasp.grasp_pose.pose)
        #print("publish_grasps関数, grasps:", grasps)   #debug
        #print("publish_grasps関数, pa:", pa)   #debug
        self.grasps_pub.publish(pa)

    def publish_poses(self, sphere_poses):   #Publishing PoseArrays for debugging pourposes
        pa = PoseArray()
        pa.header.frame_id = self._grasp_pose_frame_id
        pa.header.stamp = rospy.Time.now()
        for pose in sphere_poses:
            pa.poses.append(pose)
        #print("publish_poses関数, sphere_poses:", sphere_poses)   #debug
        #print("publish_poses関数, pa:", pa)   #debug
        self.poses_pub.publish(pa)

    def publish_object_marker(self, object_pose, width=0.03):   # Publishing PoseArrays for debugging pourposes #オブジェクトの中心座標に対して、マーカーの色等を設定して一度だけpublishする関数
        m = Marker()
        m.action = m.ADD   #Marker()にはデフォルトでADD=0と設定されており、Marker.action=0だとadd/modify an objectを意味する
        m.color.r = 1.0   #RGBAのRを1に設定
        m.color.a = 1.0   #RGBAのAを1に設定、上のと合わせると透過なしの真っ黒になる
        m.type = m.CUBE   #Marker()にはデフォルトでCUBE=1と設定されており、またMarker.typeはオブジェクトのタイプを示す
        m.id = 8787   #名前空間と組み合わせて後でオブジェクトを操作したり削除したりする際に便利なオブジェクトIDのこと
        m.header.frame_id = self._grasp_pose_frame_id   #base_footprint
        m.pose = object_pose.pose   #オブジェクトのポーズ(ここでのオブジェクトポーズはmarkerの検知座標ではなく、オブジェクトの中心位置になっているため、z方向のみ少し異なる)
        m.scale.x = width   #オブジェクトのスケールのことでx, y, z = 1, 1, 1がデフォルト(通常は1メートル四方を意味する)
        m.scale.y = width   #オブジェクトのスケールのことでx, y, z = 1, 1, 1がデフォルト(通常は1メートル四方を意味する)
        m.scale.z = 0.1   #オブジェクトのスケールのことでx, y, z = 1, 1, 1がデフォルト(通常は1メートル四方を意味する)
        print("publish_object_marker関数, m:", m)   #debug
        self.object_pub.publish(m)

    def create_grasps_from_poses(self, sphere_poses):
        """
        :type sphere_poses: []
            [] of Pose
        """
        grasps = []
        for idx, pose in enumerate(sphere_poses):   #generate_grasp_poses関数で作成されたsphere_posesの数だけcreate_grasp関数を呼び出す
            print("create_grasp number",idx)
            grasps.append(
                self.create_grasp(pose,
                                  "grasp_" + str(idx)))
        #print("create_grasps_from_poses関数, sphere_poses:", sphere_poses)   #debug
        #print("create_grasps_from_poses関数, grasps:", grasps)   #debug
        return grasps

    def create_grasp(self, pose, grasp_id):   #generate_grasp_poses関数で作成されたsphere_posesの数だけ呼び出される
        """
        :type pose: Pose
            pose of the gripper for the grasp
        :type grasp_id: str
            name for the grasp
        :rtype: Grasp
        """
        g = Grasp()
        g.id = grasp_id   #string型のid

        #pre_grasp_postureの動作を記述
        pre_grasp_posture = JointTrajectory()
        pre_grasp_posture.header.frame_id = self._grasp_postures_frame_id   #self._grasp_postures_frame_id = arm_tool_link
        pre_grasp_posture.joint_names = [
            name for name in self._gripper_joint_names.split()]   #self._gripper_joint_names = 'gripper_left_finger_joint gripper_right_finger_joint'なので、空白で二つに分割してリスト化
        jtpoint = JointTrajectoryPoint()
        jtpoint.positions = [
            float(pos) for pos in self._gripper_pre_grasp_positions.split()]   #self._gripper_pre_grasp_positions = '0.038 0.038'なので分割してfloatにしてリスト化
        jtpoint.time_from_start = rospy.Duration(self._time_pre_grasp_posture)   #(self._time_pre_grasp_posture) = 2.0
        pre_grasp_posture.points.append(jtpoint)   #pre_grasp_postureの動作を登録(1つのwaypointを持つ)
        print("create_grasp関数, pre_grasp_posture:", pre_grasp_posture)   #debug

        #grasp_postureの動作を記述して登録
        grasp_posture = copy.deepcopy(pre_grasp_posture)   #全く同じpostureなのはこの姿勢で1秒間待機させるため
        grasp_posture.points[0].time_from_start = rospy.Duration(
            self._time_pre_grasp_posture + self._time_grasp_posture)   #(self._time_pre_grasp_posture(= 2.0) + self._time_grasp_posture(=1.0)) = 3.0
        jtpoint2 = JointTrajectoryPoint()
        jtpoint2.positions = [
            float(pos) for pos in self._gripper_grasp_positions.split()]   #self._gripper_grasp_positions = '0.015 0.015'なので分割してfloatにしてリスト化
        jtpoint2.time_from_start = rospy.Duration(
            self._time_pre_grasp_posture +
            self._time_grasp_posture + self._time_grasp_posture_final)   #(self._time_pre_grasp_posture(= 2.0) + self._time_grasp_posture(= 1.0) + self._time_grasp_posture_final(= 3.0)) = 6.0
        grasp_posture.points.append(jtpoint2)   #grasp_postureの動作を登録(2つのwaypointを持つ)
        print("create_grasp関数, grasp_posture:", grasp_posture)   #debug

        g.pre_grasp_posture = pre_grasp_posture   #pre_grasp_postureの動作を登録
        g.grasp_posture = grasp_posture   #grasp_postureの動作を登録

        header = Header()
        header.frame_id = self._grasp_pose_frame_id   #arm_tool_link
        q = [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w]
        # Fix orientation from gripper_link to parent_link (tool_link)
        fix_tool_to_gripper_rotation_q = quaternion_from_euler(
            math.radians(self._fix_tool_frame_to_grasping_frame_roll),   #self._fix_tool_frame_to_grasping_frame_roll = -90.0 #math.radians関数で度からラジアンに変換
            math.radians(self._fix_tool_frame_to_grasping_frame_pitch),   #self._fix_tool_frame_to_grasping_frame_pitch = 0.0 #math.radians関数で度からラジアンに変換
            math.radians(self._fix_tool_frame_to_grasping_frame_yaw),   #self._fix_tool_frame_to_grasping_frame_yaw = 0.0 #math.radians関数で度からラジアンに変換
        )
        q = quaternion_multiply(q, fix_tool_to_gripper_rotation_q)
        fixed_pose = copy.deepcopy(pose)
        fixed_pose.orientation = Quaternion(*q)

        g.grasp_pose = PoseStamped(header, fixed_pose)   #Grasp.grasp_poseはgraspのためのエンドエフェクタの位置を示すが、内容はエンドエフェクタのparentのリンクのポーズであり、実際にはエンドエフェクタ内のリンクではない。一般的にはエンドエフェクタが始まる前の最も遠い位置の手首のリンクのポーズになる
        g.grasp_quality = self._grasp_quality   #self._grasp_quality = 0.1

        g.pre_grasp_approach = GripperTranslation()
        g.pre_grasp_approach.direction.vector.x = self._pre_grasp_direction_x  # NOQA   #self._pre_grasp_direction_x = 1.0
        g.pre_grasp_approach.direction.vector.y = self._pre_grasp_direction_y  # NOQA   #self._pre_grasp_direction_y = 0.0
        g.pre_grasp_approach.direction.vector.z = self._pre_grasp_direction_z  # NOQA   #self._pre_grasp_direction_z = 0.0
        g.pre_grasp_approach.direction.header.frame_id = self._grasp_postures_frame_id  # NOQA   #self._grasp_postures_frame_id = arm_took_link
        #g.pre_grasp_approach.direction.header.frame_id = "base_footprint" #base_footprintに設定すると、本当にその座標系でのx軸方向から物体に対して侵入を行う。ただし本デモではグリッパーのコリジョンが無視されているので、Rviz上では物体をアームが通過する
        g.pre_grasp_approach.desired_distance = self._grasp_desired_distance  # NOQA   #self._grasp_desired_distance = 0.2
        g.pre_grasp_approach.min_distance = self._grasp_min_distance   #self._grasp_min_distance = 0.0
        g.post_grasp_retreat = GripperTranslation()
        #g.post_grasp_retreat.direction.vector.x = self._post_grasp_direction_x  # NOQA   #self._post_grasp_direction_x = -1.0
        #g.post_grasp_retreat.direction.vector.y = self._post_grasp_direction_y  # NOQA   #self._post_grasp_direction_y = 0.0
        #g.post_grasp_retreat.direction.vector.z = self._post_grasp_direction_z  # NOQA   #self._post_grasp_direction_z = 0.0
        g.post_grasp_retreat.direction.vector.x = 0.0  # NOQA   #self._post_grasp_direction_x = -1.0
        g.post_grasp_retreat.direction.vector.y = 0.0  # NOQA   #self._post_grasp_direction_y = 0.0
        g.post_grasp_retreat.direction.vector.z = 1.0  # NOQA   #self._post_grasp_direction_z = 0.0
        #g.post_grasp_retreat.direction.header.frame_id = self._grasp_postures_frame_id  # NOQA   #self._grasp_postures_frame_id = arm_took_link
        g.post_grasp_retreat.direction.header.frame_id = "base_footprint"  # NOQA   #self._grasp_postures_frame_id = arm_took_link
        #g.post_grasp_retreat.desired_distance = self._grasp_desired_distance  # NOQA   #self._grasp_desired_distance = 0.2
        #g.post_grasp_retreat.min_distance = self._grasp_min_distance   #self._grasp_min_distance = 0.0
        g.post_grasp_retreat.desired_distance = 0.0  # NOQA   #self._grasp_desired_distance = 0.2
        g.post_grasp_retreat.min_distance = 0.0   #self._grasp_min_distance = 0.0

        g.max_contact_force = self._max_contact_force   #self._max_contact_force = 0.0
        g.allowed_touch_objects = self._allowed_touch_objects   #self._allowed_touch_objects = ''

        return g

    def create_grasps_from_object_pose(self, object_pose):
        """
        :type object_pose: PoseStamped
        """
        tini = rospy.Time.now()
        sphere_poses = self.generate_grasp_poses(object_pose)
        filtered_poses = filter_poses(sphere_poses, object_pose,
                                      filter_behind=False, filter_under=True)
        sorted_poses = sort_by_height(filtered_poses)
        grasps = self.create_grasps_from_poses(sorted_poses)
        tend = rospy.Time.now()
        rospy.loginfo("Generated " + str(len(grasps)) +
                      " grasps in " + str((tend - tini).to_sec()))   #二つのrospy.Time()の差分時間で何個のgraspsが作成されたかをログ出力
        # Publishing PoseArrays for debugging pourposes
        #self.publish_poses(sphere_poses)
        #self.publish_grasps(grasps)
        #self.publish_object_marker(object_pose)
        return grasps

    def create_placings_from_object_pose(self, posestamped):
        """ Create a list of PlaceLocation of the object rotated every 15deg"""
        place_locs = []
        pre_grasp_posture = JointTrajectory()
        # Actually ignored....
        pre_grasp_posture.header.frame_id = self._grasp_pose_frame_id
        pre_grasp_posture.joint_names = [
            name for name in self._gripper_joint_names.split()]
        jtpoint = JointTrajectoryPoint()
        jtpoint.positions = [
            float(pos) for pos in self._gripper_pre_grasp_positions.split()]
        jtpoint.time_from_start = rospy.Duration(self._time_pre_grasp_posture)
        pre_grasp_posture.points.append(jtpoint)
        # Generate all the orientations every step_degrees_yaw deg
        for yaw_angle in np.arange(0.0, 2.0 * pi, radians(self._step_degrees_yaw)):
            pl = PlaceLocation()
            pl.place_pose = posestamped
            newquat = quaternion_from_euler(0.0, 0.0, yaw_angle)
            pl.place_pose.pose.orientation = Quaternion(
                newquat[0], newquat[1], newquat[2], newquat[3])
            # TODO: the frame is ignored, this will always be the frame of the gripper
            # so arm_tool_link
            pl.pre_place_approach = self.createGripperTranslation(
                Vector3(1.0, 0.0, 0.0))
            pl.post_place_retreat = self.createGripperTranslation(
                Vector3(-1.0, 0.0, 0.0))

            pl.post_place_posture = pre_grasp_posture
            place_locs.append(pl)

        return place_locs

    def createGripperTranslation(self, direction_vector,
                                 desired_distance=0.15,
                                 min_distance=0.01):
        """Returns a GripperTranslation message with the
         direction_vector and desired_distance and min_distance in it.
        Intended to be used to fill the pre_grasp_approach
         and post_grasp_retreat field in the Grasp message."""
        g_trans = GripperTranslation()
        g_trans.direction.header.frame_id = self._grasp_postures_frame_id
        g_trans.direction.header.stamp = rospy.Time.now()
        g_trans.direction.vector.x = direction_vector.x
        g_trans.direction.vector.y = direction_vector.y
        g_trans.direction.vector.z = direction_vector.z
        g_trans.desired_distance = desired_distance
        g_trans.min_distance = min_distance
        return g_trans

if __name__ == '__main__':
    rospy.init_node("spherical_grasps_server")
    sg = SphericalGrasps()
    # rospy.spin()

    # For debugging pourposes
    ps = PoseStamped()
    ps.header.frame_id = 'base_footprint'
    ps.pose.position.x = 1.0
    ps.pose.position.y = 0.0
    ps.pose.position.z = 1.0
    ps.pose.orientation.w = 1.0
    while not rospy.is_shutdown():
        sg.create_grasps_from_object_pose(ps)
        rospy.sleep(1.0)
