#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def move_group_python_interface_tutorial():
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
    
    print "============ Generating plan 1 for left end_effector"
    left.set_start_state_to_current_state()
    left.set_position_target([0.1817, 0.8461, -0.5083])
    #left.set_random_target()   
 

    plan1 = left.plan()
    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(5)
    
    # Uncomment below line when working with a real robot
    # group.go(wait=True)
    
    # Use execute instead if you would like the robot to follow
    # the plan that has already been computed
    #left.execute(plan1)
    
    left.clear_pose_targets()
    
    moveit_commander.roscpp_shutdown()

if __name__=='__main__':
    move_group_python_interface_tutorial()




