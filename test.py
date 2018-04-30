#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_python
from moveit_msgs.msg import MoveItErrorCodes

rospy.init_node("tmp")

right_arm_move_group = moveit_python.move_group_interface.MoveGroupInterface("right_arm", "base")
left_arm_move_group = moveit_python.move_group_interface.MoveGroupInterface("left_arm", "base")

left_joint_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2', 'left_hand', 'left_endpoint']
right_joint_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2', 'right_hand', 'right_endpoint']

pose = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

left_arm_move_group.moveToJointPosition(left_joint_names, pose)
left_arm_move_group.get_move_action().wait_for_result()
result = left_arm_move_group.get_move_action().get_result()

if result.error_code.val == MoveItErrorCodes.SUCCESS:
    print("Successful!")
else:
    print("Error!")
