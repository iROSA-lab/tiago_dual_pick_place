import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("arm_left_torso")
plan = group.plan()
print(plan)
