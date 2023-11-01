#!/usr/bin/env python

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
import time
from tiago_dual_pick_place.msg import PlaceAutoObjectAction, PlaceAutoObjectGoal, PickUpObjectAction, PickUpObjectGoal, PickPlacePoseAction, PickPlacePoseGoal
from tiago_dual_pick_place.srv import PickPlaceObject, PickPlaceObjects, PickPlaceAutoObject, PickPlaceSimple
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient

import copy

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

import numpy as np
from std_srvs.srv import Empty

import cv2
from cv_bridge import CvBridge

from moveit_msgs.msg import MoveItErrorCodes

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


class GraspsService(object):
    def __init__(self):
        rospy.loginfo("Starting Grasps Service")
        self.pick_type = PickPlace()
        rospy.loginfo("Finished GraspsService constructor")
        self.place_gui = rospy.Service(
            "/place", PickPlaceSimple, self.start_place_simple)
        self.pick_gui = rospy.Service(
            "/pick", PickPlaceSimple, self.start_pick_simple)
        self.pick_object = rospy.Service(
            "/pick_object", PickPlaceObject, self.start_pick_object)
        self.pick_objects = rospy.Service(
            "/pick_objects", PickPlaceObjects, self.start_pick_objects)
        self.place_object = rospy.Service(
            "/place_object", PickPlaceAutoObject, self.start_place_object)

    def start_pick_simple(self, req):
        return self.pick_type.pick_simple(req.left_right)

    def start_place_simple(self, req):
        return self.pick_type.place_simple(req.left_right)

    def start_pick_object(self, req):
        return self.pick_type.pick_object(req.object_name, req.left_right)

    def start_pick_objects(self, req):
        return self.pick_type.pick_objects(req.left_object_name, req.right_object_name)

    def start_place_object(self, req):
        return self.pick_type.place_object(req.object_name)


class PickPlace(object):
    def __init__(self):
        rospy.loginfo("Initalizing...")
        self.bridge = CvBridge()
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_l = tf2_ros.TransformListener(self.tfBuffer)

        rospy.loginfo("Waiting for /pick_as AS...")
        self.pick_as = SimpleActionClient('/pick_as', PickPlacePoseAction)
        time.sleep(1.0)
        if not self.pick_as.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to /pick_as AS")
            exit()
        rospy.loginfo("Waiting for /place_as AS...")
        self.place_as = SimpleActionClient('/place_as', PickPlacePoseAction)
        self.place_as.wait_for_server()

        self.pick_obj_as = SimpleActionClient(
            '/pickup_object', PickUpObjectAction)
        time.sleep(1.0)
        if not self.pick_obj_as.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to /pickup_object AS")
            exit()
        rospy.loginfo("Waiting for /place_object AS...")
        self.place_obj_as = SimpleActionClient(
            '/place_object', PlaceAutoObjectAction)
        self.place_obj_as.wait_for_server()

        rospy.loginfo("Setting publishers to torso and head controller...")
        self.torso_cmd = rospy.Publisher(
            '/torso_controller/command', JointTrajectory, queue_size=1)
        self.head_cmd = rospy.Publisher(
            '/head_controller/command', JointTrajectory, queue_size=1)
        self.detected_pose_pub = rospy.Publisher('/detected_grasp_pose',
                                                 PoseStamped,
                                                 queue_size=1,
                                                 latch=True)

        rospy.loginfo("Waiting for '/play_motion' AS...")
        self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
        if not self.play_m_as.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to /play_motion AS")
            exit()
        rospy.loginfo("Connected!")
        rospy.sleep(1.0)

        # TODO: unify
        # PickPlacePoseGoal for pick/place simple
        self.place_g = {}
        # PoseStamped for pick/place object
        self.place_pose = {}

    def strip_leading_slash(self, s):
        return s[1:] if s.startswith("/") else s

    # TODO: decide which arm by some sort of optimization
    def pick_object(self, object_name, left_right):
        """
        The pick_object function is used to pick up an object. It makes the robot prepare, executes the pick motion
        and saves the pose to be able to place the object back in the future.

        :param self: class instance
        :param object_name: Specify the name of the object to be picked
        :param left_right: Determine which arm to use
        :return: The error_code
        :doc-author: Trelent
        """
        # Move torso to its maximum height
        self.lift_torso()

        self.prepare_robot(left_right)
        rospy.sleep(2.0)

        rospy.loginfo("Start picking %s", object_name)
        goal = PickUpObjectGoal()
        goal.left_right = left_right
        goal.object_name = object_name
        rospy.loginfo("Sending pick command...")
        self.pick_obj_as.send_goal_and_wait(goal)

        result = self.pick_obj_as.get_result()
        if str(moveit_error_dict[result.error_code]) != "SUCCESS":
            rospy.logerr("Failed to pick, not trying further")
            return result.error_code

        # Move torso to its maximum height
        # self.lift_torso()

        # Raise arm
        rospy.loginfo("Moving arm to a safe pose")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'pick_final_pose_' + left_right[0]  # first char
        pmg.skip_planning = False
        rospy.loginfo("Sending raise arm command...")
        self.play_m_as.send_goal_and_wait(pmg)

        # Save pose for placing
        self.place_pose[object_name] = copy.deepcopy(result.object_pose)
        self.place_pose[object_name].pose.position.z += 0.025

        return result.error_code

    def pick_objects(self, left_object_name, right_object_name):
        """
        The pick_objects function is used to pick up objects from the world. It takes in two arguments:
        left_object_name and right_object_name, which are strings that represent the names of objects in the Gazebo
        world. The function first moves the torso to its maximum height, then prepares the robot for picking by
        moving its arms into a safe position. Next, it sends goals to moveit! using actionlib and waits for them to
        complete before sending another goal. Finally, it raises both arms into a safe pose.

        :param self: Refer to the object itself
        :param left_object_name: Specify the name of the object to be picked up with the left arm
        :param right_object_name: Specify the name of the object to be picked up by the right arm
        :return: The error code of the pick_obj_as action server
        :doc-author: Trelent
        """
        # Move torso to its maximum height
        self.lift_torso()

        objects = {'left': left_object_name, 'right': right_object_name}
        for lr in sorted(objects):
            self.prepare_robot(lr)

        for lr in sorted(objects):
            rospy.loginfo("Start picking %s with %s arm", objects[lr], lr)
            goal = PickUpObjectGoal()
            goal.left_right = lr
            goal.object_name = objects[lr]
            self.pick_obj_as.send_goal_and_wait(goal)
            rospy.loginfo("Sending pick %s command...", lr)

            result = self.pick_obj_as.get_result()
            if str(moveit_error_dict[result.error_code]) != "SUCCESS":
                rospy.logerr("Failed to pick, not trying further")
                return result.error_code
            # Save pose for placing
            self.place_pose[objects[lr]] = copy.deepcopy(result.object_pose)
            self.place_pose[objects[lr]].pose.position.z += 0.025

        # Raise left/right arm
        rospy.loginfo("Moving arms to a safe pose")
        for lr in ['l', 'r']:
            pmg = PlayMotionGoal()
            pmg.motion_name = 'pick_final_pose_' + lr
            pmg.skip_planning = False
            rospy.loginfo("Sending raise arm command...")
            self.play_m_as.send_goal_and_wait(pmg)

        return result.error_code

    def pick_simple(self, left_right):
        """
        The pick_simple function is a simple pick function that takes in the left_right argument and uses it to
        determine which arm to use. It then waits for a grasp pose, lowers the torso, creates a PickPlacePoseGoal
        object with the grasp pose as its position and orientation, and sends this goal to the pick action server. It
        then lifts the torso again before lowering it back down after 4 seconds. Finally, it opens both grippers.

        :param self: Represent the instance of the class
        :param left_right: Determine which arm to use
        :return: The resulting error code
        :doc-author: Trelent
        """
        rospy.loginfo("Pick: Waiting for a grasp pose")
        
        grasp_ps = self.wait_for_pose('/grasp/pose')

        self.lower_torso()
        rospy.sleep(3.5)

        pick_g = PickPlacePoseGoal()
        pick_g.left_right = left_right
        pick_g.object_pose.pose = grasp_ps.pose
        # pick_g.object_pose.pose.position = grasp_ps.pose.position
        # pick_g.object_pose.pose.position.z -= 0.1*(1.0/2.0)
        # pick_g.object_pose.pose.orientation.w = 1.0
        rospy.loginfo("grasp pose in base_footprint:" + str(pick_g))
        pick_g.object_pose.header.frame_id = 'base_footprint'

        self.detected_pose_pub.publish(pick_g.object_pose)
        rospy.loginfo("Gonna pick:" + str(pick_g))

        # Optional: prepare robot # TODO: why optional? assume that robot is already prepared?
        # self.prepare_robot(left_right)
        # rospy.sleep(2.0)

        self.pick_as.send_goal_and_wait(pick_g)
        rospy.loginfo("Done!")

        result = self.pick_as.get_result()
        if str(moveit_error_dict[result.error_code]) != "SUCCESS":
            # if INVALIDATED_BY_ENVIRONMENT_CHANGE, just continue # TODO: Try to find out why this error occurs
            if str(moveit_error_dict[result.error_code]) != "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE":
                if str(moveit_error_dict[result.error_code]) != "CONTROL_FAILED":
                    rospy.logerr("Failed to pick, not trying further")
                    return result.error_code

        # TODO: huh?
        # Move torso to its maximum height
        self.lift_torso()

        # Lower torso back and place object back
        # sleep for 4 seconds
        rospy.sleep(4.0)
        self.lower_torso()
        rospy.sleep(3.0)

        # Open grippers
        rospy.loginfo("Opening grippers")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'open_gripper_' + left_right[0]  # take first char
        print(pmg.motion_name)
        pmg.skip_planning = True
        self.play_m_as.send_goal_and_wait(pmg)

        # Move torso to its maximum height
        self.lift_torso()

        # OR

        # # Raise arm
        # rospy.loginfo("Moving arm to a safe pose")
        # pmg = PlayMotionGoal()
        # pmg.motion_name = 'pick_final_pose_' + left_right[0]  # take first char
        # print(pmg.motion_name)
        # pmg.skip_planning = False
        # rospy.loginfo("Sending final arm command...")
        # self.play_m_as.send_goal_and_wait(pmg)
        # rospy.sleep(1.0)

        # # Save pose for optional immediate placing back
        # self.place_g[left_right] = copy.deepcopy(pick_g)
        # self.place_g[left_right].object_pose.pose.position.z += 0.0125

        return result.error_code

    def wait_for_pose(self, topic, timeout=None):
        """
        The wait_for_pose function is used to transform a pose from one frame to another.
        The function takes in two arguments: the topic and timeout.
        The topic argument is the name of the ROS topic that contains a PoseStamped message, which will be transformed
        into base_footprint frame.
        If no message has been received on this topic within timeout seconds, then None will be returned.

        :param self: Refer to the object itself
        :param topic: Specify the topic to listen for a posestamped message on
        :param timeout: Set a time limit on how long the function will wait for a message
        :return: A posestamped object
        :doc-author: Trelent
        """
        try:
            grasp_pose = rospy.wait_for_message(
                topic, PoseStamped, timeout=timeout)
        except rospy.ROSException as e:
            return None

        grasp_pose.header.frame_id = self.strip_leading_slash(
            grasp_pose.header.frame_id)
        rospy.loginfo("Got: " + str(grasp_pose))

        rospy.loginfo("Transforming from frame: " +
                      grasp_pose.header.frame_id + " to 'base_footprint'")
        ps = PoseStamped()
        ps.pose.position = grasp_pose.pose.position
        ps.pose.orientation = grasp_pose.pose.orientation
        ps.header.stamp = self.tfBuffer.get_latest_common_time(
            "base_footprint", grasp_pose.header.frame_id)
        ps.header.frame_id = grasp_pose.header.frame_id
        transform_ok = False

        while not transform_ok and not rospy.is_shutdown():
            try:
                transform = self.tfBuffer.lookup_transform("base_footprint",
                                                           ps.header.frame_id,
                                                           rospy.Time(0))
                ps_trans = do_transform_pose(ps, transform)
                transform_ok = True
            except tf2_ros.ExtrapolationException as e:
                rospy.logwarn(
                    "Exception on transforming point... trying again \n(" +
                    str(e) + ")")
                rospy.sleep(0.01)
                ps.header.stamp = self.tfBuffer.get_latest_common_time(
                    "base_footprint", grasp_pose.header.frame_id)
            rospy.loginfo("Setting pose")

        return ps_trans

    def place_object(self, object_name):
        """
        The place_object function is used to place an object in a desired location.
        The function takes the name of the object as input and places it at a pre-defined location.
        If no pre-defined location exists, then the user can define one by publishing to /place/pose topic.

        :param self: Represent the instance of the class
        :param object_name: Identify the object that is being placed
        :return: An error code
        :doc-author: Trelent
        """
        rospy.loginfo("Start placing %s", object_name)
        goal = PlaceAutoObjectGoal()

        rospy.loginfo("Place: Waiting for a /place/pose")
        place_pose = self.wait_for_pose('/place/pose', timeout=10.)
        if place_pose is None:
            # use previously stored pickup position
            # if it doesn't exist -> wait forever
            if not object_name in self.place_pose:
                place_pose = self.wait_for_pose('/place/pose')
            else:
                place_pose = self.place_pose[object_name]

        goal.target_pose = place_pose
        goal.object_name = object_name
        # goal.left_right = left_right  # not needed with 'Auto'
        print("Sending place...")
        self.place_obj_as.send_goal_and_wait(goal)

        result = self.place_obj_as.get_result()
        if str(moveit_error_dict[result.error_code]) != "SUCCESS":
            rospy.logerr("Failed to place, not trying further")

        return result.error_code

    def place_simple(self, left_right):
        """
        The place_simple function is a function that takes in the left_right parameter and then waits for a place pose.
        If there is no place pose set, it will try to place back where it was (by using previous place_g). If there is a
        place pose, it will receive the place pose and add an offset of 0.0125 to not crash into other objects.

        :param self: Refer to the object itself
        :param left_right: Determine which arm to use
        :doc-author: Trelent
        """
        rospy.loginfo("Place: Waiting for a place pose")
        place_pose = self.wait_for_pose('/place/pose', timeout=1.0)
        if place_pose is None:
            rospy.loginfo(
                "No place pose set. Gonna try placing back where it was")
        else:
            rospy.loginfo("Received a place pose")
            self.place_g[left_right].object_pose.pose = place_pose.pose
            # Add small offset to not crash into stuff
            self.place_g[left_right].object_pose.pose.position.z += 0.0125
        # Place the object
        self.place_as.send_goal_and_wait(self.place_g[left_right])
        rospy.loginfo("Done!")

    def lift_torso(self):
        """
        The lift_torso function moves the torso up to a height of 0.24 meters.
    
        :param self: Refer to the object itself
        :doc-author: Trelent
        """
        rospy.loginfo("Moving torso up")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.24]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        self.torso_cmd.publish(jt)
    
    def lower_torso(self):
        """    
        The lower_torso function moves the torso down to a position of 0.14 radians.

        :param self: Represent the instance of the class
        :return: A jointtrajectory object
        :doc-author: Trelent
        """
        rospy.loginfo("Moving torso down")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.14]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        self.torso_cmd.publish(jt)

    def lower_head(self):
        """
        The lower_head function moves the head down and left.


        :param self: Represent the instance of the class
        :doc-author: Trelent
        """
        rospy.loginfo("Moving head down and left")
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.75, -0.75]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
        rospy.loginfo("Done.")

    def prepare_robot(self, left_right):
        """
        The prepare_robot function is used to prepare the robot for grasping by unfolding the arm. It uses the
        play_motion pregrasp_{l/r}.

        :param self: Refer to the instance of the class
        :param left_right: Determine which arm to use
        :doc-author: Trelent
        """
        # TODO: maybe adjust to be able to do both arms at the same time?
        rospy.loginfo("Unfold arm safely")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'pregrasp_' + left_right[0]
        pmg.skip_planning = False
        self.play_m_as.send_goal_and_wait(pmg)
        rospy.loginfo("Done.")

        # self.lower_head()

        rospy.loginfo("Robot prepared.")


if __name__ == '__main__':
    rospy.init_node('pick_place')
    srv = GraspsService()
    rospy.spin()
