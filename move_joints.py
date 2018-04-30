#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def move_joints():
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
    
    # Gather basic information
    print "============ Reference frame left: %s" % left.get_planning_frame()
    print "============ Reference frame right: %s" % right.get_planning_frame()
    
    print "============ End effector left: %s" % left.get_end_effector_link()
    print "============ End effector right: %s" % right.get_end_effector_link()
    
    print "============ Robot Groups:"
    print robot.get_group_names()
    
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"
    
    print "============ Printing robot variables"
    print robot.get_current_variable_values()
    print "============"

    print "============ Printing left hand values"
    print "%s" % left.get_end_effector_link()
    
    # current set of left arm joint values
    #group_variable_values = left.get_current_joint_values()
    #print "============ Joint values left: %s" % group_variable_values
    #group_variable_values[0] = 1.0
    d = {"right_s0":0.0, "right_s1":0.0, "right_e0":0.0, "right_e1":0.0, "right_w0":0.0, "right_w1":0.0, "right_w2":0.0}
    right.set_joint_value_target(d)
    
    plan = left.plan()
    
    print "============ Waiting while RVIZ displays plan2..."
    rospy.sleep(5)
    
    print "============ Printing robot variables"
    print robot.get_current_variable_values()
    print "============"
    
    #left.execute(plan2)

    left.clear_pose_targets()
    
    moveit_commander.roscpp_shutdown()

if __name__=='__main__':
    move_joints()




