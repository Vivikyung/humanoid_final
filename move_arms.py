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
    
    print "============ Generating plan 1 for left end_effector"
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = 0.7
    pose_target.position.y = -0.05
    pose_target.position.z = 1.1
    left.set_pose_target(pose_target)
    
    plan1 = left.plan()
    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(5)
    
    # Uncomment below line when working with a real robot
    # group.go(wait=True)
    
    # Use execute instead if you would like the robot to follow
    # the plan that has already been computed
    #left.execute(plan1)
    
    left.clear_pose_targets()
    # current set of left arm joint values
    group_variable_values = left.get_current_joint_values()
    print "============ Joint values left: ", group_variable_values
    group_variable_values[0] = 1.0
    group.set_joint_value_target(group_variable_values)
    
    plan2 = left.plan()
    
    print "============ Waiting while RVIZ displays plan2..."
    rospy.sleep(5)
    
    #left.execute(plan2)
    
    moveit_commander.roscpp_shutdown()

if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass




