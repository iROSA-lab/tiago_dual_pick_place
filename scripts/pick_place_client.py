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
from tiago_dual_pick_place.msg import PlaceObjectAction, PlaceObjectGoal, PickUpObjectAction, PickUpObjectGoal, PickUpPoseAction, PickUpPoseGoal
from tiago_dual_pick_place.srv import PickPlaceObject
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
                self.place_gui = rospy.Service("/place", Empty, self.start_place)  # old place service, assumes object name to be 'part'
                self.pick_gui = rospy.Service("/pick", Empty, self.start_pick)
                self.pick_object = rospy.Service("/pick_object", PickPlaceObject, self.start_pick_object)
                self.place_object = rospy.Service("/place_object", PickPlaceObject, self.start_place_object)

        def start_pick(self, req):
                self.pick_type.pick_place("pick")
                return {}

        def start_place(self, req):
                self.pick_type.pick_place("place")
                return {}

        def start_pick_object(self, req):
                self.pick_type.pick_object(req.object_name)
                return {}

        def start_place_object(self, req):
                self.pick_type.place_object(req.object_name)
                return {}

class PickPlace(object):
        def __init__(self):
                rospy.loginfo("Initalizing...")
                self.bridge = CvBridge()
                self.tfBuffer = tf2_ros.Buffer()
                self.tf_l = tf2_ros.TransformListener(self.tfBuffer)
                
                rospy.loginfo("Waiting for /pickup_pose AS...")
                self.pick_as = SimpleActionClient('/pickup_pose', PickUpPoseAction)
                time.sleep(1.0)
                if not self.pick_as.wait_for_server(rospy.Duration(20)):
                        rospy.logerr("Could not connect to /pickup_pose AS")
                        exit()
                rospy.loginfo("Waiting for /place_pose AS...")
                self.place_as = SimpleActionClient('/place_pose', PickUpPoseAction)
                self.place_as.wait_for_server()

                self.pick_obj_as = SimpleActionClient('/pickup_object', PickUpObjectAction)
                time.sleep(1.0)
                if not self.pick_obj_as.wait_for_server(rospy.Duration(20)):
                        rospy.logerr("Could not connect to /pickup_object AS")
                        exit()
                rospy.loginfo("Waiting for /place_object AS...")
                self.place_obj_as = SimpleActionClient('/place_object', PlaceObjectAction)
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

                # place goal
                self.place_g = PickUpPoseGoal()
                self.place_pose = PoseStamped()

        def strip_leading_slash(self, s):
                return s[1:] if s.startswith("/") else s
                
        def pick_object(self, object_name):
                #self.prepare_robot()
                #rospy.sleep(2.0)
                rospy.loginfo("Start picking %s", object_name)
                goal = PickUpObjectGoal()
                goal.object_name = object_name
                self.pick_obj_as.send_goal_and_wait(goal)
                rospy.loginfo("Pick done!")

                result = self.pick_obj_as.get_result()
                if str(moveit_error_dict[result.error_code]) != "SUCCESS":
                        rospy.logerr("Failed to pick, not trying further")
                        return

                # Move torso to its maximum height
                self.lift_torso()

                # Raise arm
                rospy.loginfo("Moving arm to a safe pose")
                pmg = PlayMotionGoal()
                pmg.motion_name = 'pick_final_pose'
                pmg.skip_planning = False
                self.play_m_as.send_goal_and_wait(pmg)
                rospy.loginfo("Raise object done.")

                # Save pose for placing
                self.place_pose = copy.deepcopy(result.object_pose)
                self.place_pose.pose.position.z += 0.05

        def wait_for_pose(self, topic, timeout=None):
                try:
                    grasp_pose = rospy.wait_for_message(topic, PoseStamped, timeout=timeout)
                except rospy.ROSException as e:
                    return None

                grasp_pose.header.frame_id = self.strip_leading_slash(grasp_pose.header.frame_id)
                rospy.loginfo("Got: " + str(grasp_pose))


                rospy.loginfo("Transforming from frame: " +
                grasp_pose.header.frame_id + " to 'base_footprint'")
                ps = PoseStamped()
                ps.pose.position = grasp_pose.pose.position
                ps.pose.orientation = grasp_pose.pose.orientation
                ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", grasp_pose.header.frame_id)
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
                                ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", grasp_pose.header.frame_id)
                        rospy.loginfo("Setting pose")

                return ps_trans

        def place_object(self, object_name):
                rospy.loginfo("Start placing %s", object_name)
                goal = PlaceObjectGoal()

                rospy.loginfo("Place: Waiting for a place pose")
                place_pose = self.wait_for_pose('/place/pose', timeout=10.)
                if place_pose is None:
                    place_pose = self.place_pose  # use previously stored pickup position

                goal.target_pose = place_pose
                goal.object_name = object_name
                self.place_obj_as.send_goal_and_wait(goal)
                rospy.loginfo("Place done!")

                result = self.place_obj_as.get_result()
                if str(moveit_error_dict[result.error_code]) != "SUCCESS":
                        rospy.logerr("Failed to place, not trying further")
                        return

        def pick_place(self, string_operation):
                transform_ok = True
                if string_operation == "pick":
                        #self.prepare_robot()
                        #rospy.sleep(2.0)

                        rospy.loginfo("Pick: Waiting for a grasp pose")
                        grasp_ps = self.wait_for_pose('/grasp/pose')

                        pick_g = PickUpPoseGoal()
                        pick_g.object_pose.pose = grasp_ps.pose
                        #pick_g.object_pose.pose.position = grasp_ps.pose.position
                        #pick_g.object_pose.pose.position.z -= 0.1*(1.0/2.0)
                        #pick_g.object_pose.pose.orientation.w = 1.0
                        rospy.loginfo("grasp pose in base_footprint:" + str(pick_g))
                        pick_g.object_pose.header.frame_id = 'base_footprint'

                        self.detected_pose_pub.publish(pick_g.object_pose)
                        rospy.loginfo("Gonna pick:" + str(pick_g))

                        self.pick_as.send_goal_and_wait(pick_g)
                        rospy.loginfo("Done!")

                        result = self.pick_as.get_result()
                        if str(moveit_error_dict[result.error_code]) != "SUCCESS":
                                rospy.logerr("Failed to pick, not trying further")
                                return

                        # Move torso to its maximum height
                        self.lift_torso()

                        # Raise arm
                        rospy.loginfo("Moving arm to a safe pose")
                        pmg = PlayMotionGoal()
                        pmg.motion_name = 'pick_final_pose'
                        pmg.skip_planning = False
                        self.play_m_as.send_goal_and_wait(pmg)
                        rospy.loginfo("Raise object done.")

                        # Save pos for placing
                        self.place_g = pick_g
                        self.place_g.object_pose.pose.position.z += 0.05

                elif string_operation == "place":
                        # Place the object back to its position
                        rospy.loginfo("Gonna place near where it was")
                        self.place_as.send_goal_and_wait(self.place_g)
                        rospy.loginfo("Done!")

        def lift_torso(self):
                rospy.loginfo("Moving torso up")
                jt = JointTrajectory()
                jt.joint_names = ['torso_lift_joint']
                jtp = JointTrajectoryPoint()
                jtp.positions = [0.34]
                jtp.time_from_start = rospy.Duration(2.5)
                jt.points.append(jtp)
                self.torso_cmd.publish(jt)

        def lower_head(self):
                rospy.loginfo("Moving head down and left")
                jt = JointTrajectory()
                jt.joint_names = ['head_1_joint', 'head_2_joint']
                jtp = JointTrajectoryPoint()
                jtp.positions = [0.75, -0.75]
                jtp.time_from_start = rospy.Duration(2.0)
                jt.points.append(jtp)
                self.head_cmd.publish(jt)
                rospy.loginfo("Done.")

        def prepare_robot(self):
                rospy.loginfo("Unfold arm safely")
                pmg = PlayMotionGoal()
                pmg.motion_name = 'pregrasp'
                pmg.skip_planning = False
                self.play_m_as.send_goal_and_wait(pmg)
                rospy.loginfo("Done.")

                #self.lower_head()

                rospy.loginfo("Robot prepared.")


if __name__ == '__main__':
        rospy.init_node('pick_place')
        srv = GraspsService()
        rospy.spin()

