#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospy
from std_msgs.msg import String

pos_left = [0.0, 0.0, 0.0]
pos_right = [0.0, 0.0, 0.0]
s0_left = 0.0
e0_left = 0.0
w0_left = 0.0
s0_right = 0.0
e0_right = 0.0
w0_right = 0.0

print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)

# instantiate robot commander object
robot = moveit_commander.RobotCommander()

# instantiate planning scene interface object
scene = moveit_commander.PlanningSceneInterface()

# instantiate arm move group commanders
left = moveit_commander.MoveGroupCommander("left_arm")
right = moveit_commander.MoveGroupCommander("right_arm")

# publishes trajectory to visualize
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)
# Gather basic information
print "============ Reference frame left: %s" % left.get_planning_frame()
print "============ Reference frame right: %s" % right.get_planning_frame()

print "============ End effector left: %s" % left.get_end_effector_link()
print "============ End effector right: %s" % right.get_end_effector_link()

print "============ Robot Groups:"
print robot.get_group_names()

def move_to_pos(data):
    global pos_left, pos_right, s0_left, e0_left, w0_left, s0_right, e0_right, w0_right
    pos = data.data.split(',')
    position = [float(x) for x in pos]
    pos_left = position[0:3]
    pos_right = position[3:6]

    s0_left = position[6]
    e0_left = position[7]
    w0_left = position[8]
    s0_right = position[9]
    e0_right = position[10]
    w0_right = position[11]

    move(pos_left, pos_right, s0_left, e0_left, w0_left, s0_right, e0_right, w0_right)

def listener():
    rospy.init_node('mover', anonymous=True)
    rospy.Subscriber("joints", String, move_to_pos)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def move(pos_left, pos_right, shoulder_left, elbow_left, wrist_left, shoulder_right, elbow_right, wrist_right):    
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"
    
    print "============ Printing robot variables"
    print robot.get_current_variable_values()
    print "============"
    
    print "============ Generating EE position plans"
    left.set_position_target(pos_left)
    left.set_goal_tolerance(0.25)
    right = moveit_commander.MoveGroupCommander("right_arm")
    right.set_goal_tolerance(0.25)
    #left.set_random_target()   
 
    print "============ Generating joint angle plans"
    # current set of left arm joint values
    left_joints = left.get_current_joint_values()
    right_joints = right.get_current_joint_values()
    print "============ Joint values left: %s" % left_joints
    print "============ Joint values right: %s" % right_joints
    d_left = {"left_s0":shoulder_left, "left_e0":elbow_left, "left_w0":wrist_left}
    d_right = {"right_s0":shoulder_right, "right_e0":elbow_right, "right_w0":wrist_right}

    left.set_joint_value_target(d_left)
    right.set_joint_value_target(d_right)

    plan_l = left.plan()
    plan_r = right.plan()
    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(5)
    
    # Uncomment below line when working with a real robot
    # group.go(wait=True)
    
    # Use execute instead if you would like the robot to follow
    # the plan that has already been computed
    #left.execute(plan1)
    
    left.clear_pose_targets()
    right.clear_pose_targets()
        

if __name__=='__main__':
    listener()
    
moveit_commander.roscpp_shutdown()
