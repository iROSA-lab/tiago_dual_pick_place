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
#   * Daljeet Nandha

import rospy
from grasps_server import Grasps
from arm_conf import ArmConf
from actionlib import SimpleActionClient, SimpleActionServer
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult, MoveItErrorCodes
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from tiago_dual_pick_place.msg import PlaceAutoObjectAction, PlaceAutoObjectResult, PickUpObjectAction, PickUpObjectResult, PickPlacePoseAction, PickPlacePoseResult
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
from std_srvs.srv import Empty, EmptyRequest
from copy import deepcopy
from random import shuffle
import copy

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


def createPickupGoal(group, target="part",
                     grasp_pose=PoseStamped(),
                     possible_grasps=[],
                     links_to_allow_contact=None):
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
                    group,
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
        rospy.loginfo("Connecting to pickup AS")
        self.pickup_ac = SimpleActionClient('/pickup', PickupAction)
        self.pickup_ac.wait_for_server()
        rospy.loginfo("Succesfully connected.")
        rospy.loginfo("Connecting to place AS")
        self.place_ac = SimpleActionClient('/place', PlaceAction)
        self.place_ac.wait_for_server()
        rospy.loginfo("Succesfully connected.")
        self.scene = PlanningSceneInterface()
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

        self.arm_conf_l = ArmConf(
            rospy.get_param('~move_group_l0'),
            rospy.get_param('~move_group_l1'),
            rospy.get_param('~grasp_postures_frame_id_l'),
            rospy.get_param('~gripper_joint_names_l')
        )
        self.arm_conf_r = ArmConf(
            rospy.get_param('~move_group_r0'),
            rospy.get_param('~move_group_r1'),
            rospy.get_param('~grasp_postures_frame_id_r'),
            rospy.get_param('~gripper_joint_names_r')
        )

        # Get the links of the end effector exclude from collisions
        self.links_to_allow_contact = rospy.get_param(
            '~links_to_allow_contact', None)
        if self.links_to_allow_contact is None:
            rospy.logwarn(
                "Didn't find any links to allow contacts... at param ~links_to_allow_contact")
        else:
            rospy.loginfo("Found links to allow contacts: " +
                          str(self.links_to_allow_contact))

        # Start action servers
        self.pick_as = SimpleActionServer(
            '/pickup_pose', PickPlacePoseAction,
            execute_cb=self.pick_cb, auto_start=False)
        self.pick_as.start()

        self.place_as = SimpleActionServer(
            '/place_pose', PickPlacePoseAction,
            execute_cb=self.place_cb, auto_start=False)
        self.place_as.start()

        self.pick_obj_as = SimpleActionServer(
            '/pickup_object', PickUpObjectAction,
            execute_cb=self.pick_obj_cb, auto_start=False)
        self.pick_obj_as.start()

        self.place_obj_as = SimpleActionServer(
            '/place_object', PlaceAutoObjectAction,
            execute_cb=self.place_obj_cb, auto_start=False)
        self.place_obj_as.start()

        # Initialize grasp generator
        self.sg = Grasps()

    def pick_obj_cb(self, goal):
        """
        :type goal: PickUpObjectGoal
        """
        p_res = PickUpObjectResult()
        arm_conf = self.arm_conf_r if goal.left_right == 'right' else self.arm_conf_l
        object_pose = self.get_object_pose(goal.object_name)
        if object_pose is None:
            rospy.logerr("Object with name %s not found", goal.object_name)
            p_res.error_code = 99999
        else:
            p_res.error_code = self.grasp_object(
                arm_conf, object_pose, part=goal.object_name)
            p_res.object_pose = object_pose

        if p_res.error_code != 1:
            self.pick_obj_as.set_aborted(p_res)
        else:
            self.pick_obj_as.set_succeeded(p_res)

    def place_obj_cb(self, goal):
        """
        :type goal: PlaceAutoObjectGoal
        """
        p_res = PlaceAutoObjectResult()

        objects = self.scene.get_attached_objects()
        arm_conf = None
        for oid in objects:
            if oid != goal.object_name:
                continue
            link = objects[oid].link_name
            if self.arm_conf_r.is_link(link):
                arm_conf = self.arm_conf_r
            elif self.arm_conf_l.is_link(link):
                arm_conf = self.arm_conf_l

        if arm_conf is not None:
            p_res.error_code = self.place_object(
                arm_conf, goal.target_pose, part=goal.object_name, simple_place=False)
        else:
            rospy.logerr("Object not in gripper", goal.object_name)
            p_res.error_code = 99999

        if p_res.error_code != 1:
            self.place_obj_as.set_aborted(p_res)
        else:
            self.place_obj_as.set_succeeded(p_res)

    def pick_cb(self, goal):
        """
        :type goal: PickPlacePoseGoal
        """
        p_res = PickPlacePoseResult()
        arm_conf = self.arm_conf_r if goal.left_right == 'right' else self.arm_conf_l
        self.remove_part(arm_conf.grasp_frame)
        p_res.error_code = self.grasp(arm_conf, goal.object_pose)
        if p_res.error_code != 1:
            self.pick_as.set_aborted(p_res)
        else:
            self.pick_as.set_succeeded(p_res)

    def place_cb(self, goal):
        """
        :type goal: PickPlacePoseGoal
        """
        p_res = PickPlacePoseResult()
        arm_conf = self.arm_conf_r if goal.left_right == 'right' else self.arm_conf_l
        p_res.error_code = self.place_object(
            arm_conf, goal.object_pose, simple_place=True)
        if p_res.error_code != 1:
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

    def get_object_pose(self, object_name):
        rospy.loginfo("Looking for object: %s", object_name)
        obj_poses = self.scene.get_object_poses([object_name])
        if len(obj_poses) < 1:
            return None
        object_pose = PoseStamped()
        object_pose.header.frame_id = "base_footprint"
        object_pose.pose = obj_poses[object_name]
        rospy.loginfo("Object pose: %s", object_pose.pose)
        return object_pose

    def remove_part(self, grasp_frame, object_name="part"):
        rospy.loginfo("Removing any previous '"+str(object_name)+"' object")
        self.scene.remove_attached_object(grasp_frame)
        self.scene.remove_world_object(object_name)
        rospy.sleep(2.0)  # Removing is fast

    def add_part(self, object_pose, object_name="part"):
        rospy.loginfo("Adding new '"+str(object_name)+"' object")
        # Add object description in scene
        self.scene.add_box(object_name, object_pose,
                           (self.object_depth, self.object_width, self.object_height))

    def grasp_object(self, arm_conf, object_pose, part="part"):
        rospy.loginfo("Clearing octomap")
        self.clear_octomap_srv.call(EmptyRequest())

        rospy.loginfo("Second%s", object_pose.pose)

        # We need to wait for the object part to appear
        self.wait_for_planning_scene_object(part)

        # compute grasps
        possible_grasps = self.sg.create_grasps_from_object_pose(
            object_pose, arm_conf, single=False)
        goal = createPickupGoal(
            arm_conf.group_arm, part, object_pose, possible_grasps, self.links_to_allow_contact)

        rospy.loginfo("Sending goal")
        self.pickup_ac.send_goal(goal)
        rospy.loginfo("Waiting for result")
        self.pickup_ac.wait_for_result()
        result = self.pickup_ac.get_result()
        rospy.logdebug("Using torso result: " + str(result))
        rospy.loginfo("Pick result: " + str(
            moveit_error_dict[result.error_code.val]) + "(" + str(result.error_code.val) + ")")

        return result.error_code.val

    def grasp(self, arm_conf, object_pose):
        # create scene object at pose of grasp

        # remove any old obstacles/tables (these will be added again as per the current grasp call)
        self.remove_part()
        self.scene.remove_world_object()  # NOTE: this removes everything!

        rospy.loginfo("Object pose: %s", object_pose.pose)

        rospy.loginfo("Clearing octomap")
        self.clear_octomap_srv.call(EmptyRequest())

        rospy.loginfo("Second%s", object_pose.pose)

        # Get all objects to be used for planning using an object Marker Array
        obj_markers = rospy.wait_for_message('/obj_markers', MarkerArray)
        for marker in obj_markers.markers:
            obj_id = marker.id
            obj_pose = PoseStamped()
            obj_pose.header = marker.header
            obj_pose.pose = marker.pose
            obj_dimensions = (marker.scale.x, marker.scale.y, marker.scale.z)
            # '42' is the main "part" object to be grasped
            if obj_id != 42:
                self.scene.add_box("obj"+str(obj_id), obj_pose, obj_dimensions)
        self.add_part(object_pose)  # grasp for object 42

        # We need to wait for the object 'part' to appear
        self.wait_for_planning_scene_object()

        # compute grasps
        possible_grasps = self.sg.create_grasps_from_object_pose(
            object_pose, arm_conf, single=True)
        goal = createPickupGoal(
            arm_conf.group_arm, "part", object_pose, possible_grasps, self.links_to_allow_contact)
        rospy.loginfo("Sending goal")
        self.pickup_ac.send_goal(goal)
        rospy.loginfo("Waiting for result")
        self.pickup_ac.wait_for_result()
        result = self.pickup_ac.get_result()
        rospy.logdebug("Using torso result: " + str(result))
        rospy.loginfo("Pick result: " + str(
            moveit_error_dict[result.error_code.val]) + "(" + str(result.error_code.val) + ")")

        # Remove objects that aren't necessary anymore
        for marker in obj_markers.markers:
            obj_id = marker.id
            # remove everything except '42' (="part" object to be grasped)
            if obj_id != 42:
                self.scene.remove_world_object("obj"+str(obj_id))

        return result.error_code.val

    def place_object(self, arm_conf, object_pose, part="part", simple_place=False):

        rospy.loginfo("Clearing octomap")
        self.clear_octomap_srv.call(EmptyRequest())

        # Get all objects to be used for planning using an object Marker Array
        obj_markers = rospy.wait_for_message('/obj_markers', MarkerArray)
        for marker in obj_markers.markers:
            obj_id = marker.id
            obj_pose = PoseStamped()
            obj_pose.header = marker.header
            obj_pose.pose = marker.pose
            obj_dimensions = (marker.scale.x, marker.scale.y, marker.scale.z)
            # '42' is the main "part" object to be grasped
            if obj_id != 42:
                self.scene.add_box("obj"+str(obj_id), obj_pose, obj_dimensions)

        self.wait_for_planning_scene_object("obj"+str(obj_id))

        possible_placings = self.sg.create_placings_from_object_pose(
            object_pose, simple_place)
        rospy.loginfo(
            "Trying to place with arm and torso")
        rospy.loginfo(
            "MOVE GROUP is:" + str(self.move_group_1))
        # Try with arm and torso
        goal = createPlaceGoal(
            object_pose, possible_placings, arm_conf.group_arm_torso, part, self.links_to_allow_contact)
        rospy.loginfo("Sending goal")
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

    #         rospy.loginfo("Removing previous object: %s", part)
        # self.scene.remove_world_object(part)
    #         # Also remove any old obstacles/tables (these will be added again as per the current grasp call)
    #         self.scene.remove_world_object() # NOTE: this removes everything

        return result.error_code.val


if __name__ == '__main__':
    rospy.init_node('pick_and_place_server')
    paps = PickAndPlaceServer()
    rospy.spin()
