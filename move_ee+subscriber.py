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

print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

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

def find_joints(data):
    global pos_left
    global pos_right
    position = data.data.split(',')
    pos_left = position[0:3]
    pos_right = position[3:6]
    move(pose_left, pos_right)

def listener():
    rospy.init_node('mover', anonymous=True)
    rospy.Subscriber("joints", String, find_joints)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def move_EE(pos_left, pos_right):    
    # Gather basic information
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"
    
    print "============ Printing robot variables"
    print robot.get_current_variable_values()
    print "============"
    
    print "============ Generating plan 1 for left end_effector"
    left.set_position_target(pos_left)
    right.set_position_target(pos_right)
    #left.set_random_target()   
 

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
    while True:
        listener()
        move_EE(pos_left, pos_right)
    
 moveit_commander.roscpp_shutdown()
