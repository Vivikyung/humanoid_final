#!/usr/bin/env python

import argparse
import sys
import rospy
 
import baxter_interface
import baxter_external_devices
 
from baxter_interface import CHECK_VERSION

def set_neutral():
    """
    Sets both arms back into a neutral pose.
    """
    print("Moving to neutral pose...")
    baxter_interface.limb.Limb("left").move_to_neutral()
    baxter_interface.limb.Limb("right").move_to_neutral()

    # Sets head into neutral pose
    baxter_interface.Head().set_pan(0.0)

def set_j_limb(limb, pos, accuracy):
    """
    Set the selected joint to given pos.
    @param limb: the limb to move
    @param pos: desired joint angle
    """
    limb.move_to_joint_positions(waypoint, timeout=20.0,
                                                   threshold=accuracy)
    rospy.sleep(3.0)

def input_control(speed, accuracy):
    """
    Takes user input to move the limbs
    """

    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    # Set the speeds for the limbs to move
    left.set_joint_position_speed(speed)
    right.set_joint_position_speed(speed)

    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
            elif c == '?':
                """
                TODO FILL THIS OUT!!!!
                """
                print("Enter joints and their values in corresponding order: ")
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))

def main():
    """RSDK Joint Position Example: Keyboard Control
 
   Use your dev machine's keyboard to control joint positions.
 
   Each key corresponds to increasing or decreasing the angle
   of a joint on one of Baxter's arms. Each arm is represented
   by one side of the keyboard and inner/outer key pairings
   on each row for each joint.
   """
    # Takes care of arguments when starting file
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-s', '--speed', default=0.3, type=float,
        help='joint position motion speed ratio [0.0-1.0] (default:= 0.3)'
    )
    parser.add_argument(
        '-a', '--accuracy',
        default=baxter_interface.settings.JOINT_ANGLE_TOLERANCE, type=float,
        help='joint position accuracy (rad) at which waypoints must achieve'
    )
    args = parser.parse_args(rospy.myargv()[1:])
 
    # Initializes robot
    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_input")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    # Move into neutral position to start
    set_neutral()

    def clean_shutdown():
        set_neutral()
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)
    print("Enabling robot... ")
    rs.enable()
 
    input_control(args.speed, args.accuracy)
    print("Done.")
 
if __name__ == '__main__':
    main()
















