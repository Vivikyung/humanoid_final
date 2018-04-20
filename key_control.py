#!/usr/bin/env python

import argparse
 
import rospy
 
import baxter_interface
import baxter_external_devices
 
from baxter_interface import CHECK_VERSION

def set_j(limb, joint_name, delta):
    """
    Set the selected joint to current pos + delta.
    @param limb: the limb to get the pos from
    @param joint_name: joint name
    @param delta: delta to update the joint by
    """
    current_position = limb.joint_angle(joint_name)
    joint_command = {joint_name: current_position + delta}
    limb.set_joint_positions(joint_command)

def set_neutral():
    """
    Sets both arms back into a neutral pose.
    """
    print("Moving to neutral pose...")
    baxter_interface._left_arm.move_to_neutral()
    baxter_interface._right_arm.move_to_neutral()

    #Sets head into neutral pose
    baxter_interface._head.set_pan(0.0)

def map_keyboard():
    """
    Maps keyboard input to joint position commands.
    @param keyboard: a list of keys
    """
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()
 
    bindings = {
    #   key: (function, args, description)
        '9': (set_j, [left, lj[0], 0.1], "left_s0 increase"),
        '6': (set_j, [left, lj[0], -0.1], "left_s0 decrease"),
        '8': (set_j, [left, lj[1], 0.1], "left_s1 increase"),
        '7': (set_j, [left, lj[1], -0.1], "left_s1 decrease"),
        'o': (set_j, [left, lj[2], 0.1], "left_e0 increase"),
        'y': (set_j, [left, lj[2], -0.1], "left_e0 decrease"),
        'i': (set_j, [left, lj[3], 0.1], "left_e1 increase"),
        'u': (set_j, [left, lj[3], -0.1], "left_e1 decrease"),
        'l': (set_j, [left, lj[4], 0.1], "left_w0 increase"),
        'h': (set_j, [left, lj[4], -0.1], "left_w0 decrease"),
        'k': (set_j, [left, lj[5], 0.1], "left_w1 increase"),
        'j': (set_j, [left, lj[5], -0.1], "left_w1 decrease"),
        '.': (set_j, [left, lj[6], 0.1], "left_w2 increase"),
        'n': (set_j, [left, lj[6], -0.1], "left_w2 decrease"),
        ',': (grip_left.close, [], "left: gripper close"),
        'm': (grip_left.open, [], "left: gripper open"),
        '/': (grip_left.calibrate, [], "left: gripper calibrate"),
 
        '4': (set_j, [right, rj[0], 0.1], "right_s0 increase"),
        '1': (set_j, [right, rj[0], -0.1], "right_s0 decrease"),
        '3': (set_j, [right, rj[1], 0.1], "right_s1 increase"),
        '2': (set_j, [right, rj[1], -0.1], "right_s1 decrease"),
        'r': (set_j, [right, rj[2], 0.1], "right_e0 increase"),
        'q': (set_j, [right, rj[2], -0.1], "right_e0 decrease"),
        'e': (set_j, [right, rj[3], 0.1], "right_e1 increase"),
        'w': (set_j, [right, rj[3], -0.1], "right_e1 decrease"),
        'f': (set_j, [right, rj[4], 0.1], "right_w0 increase"),
        'a': (set_j, [right, rj[4], -0.1], "right_w0 decrease"),
        'd': (set_j, [right, rj[5], 0.1], "right_w1 increase"),
        's': (set_j, [right, rj[5], -0.1], "right_w1 decrease"),
        'v': (set_j, [right, rj[6], 0.1], "right_w2 increase"),
        'z': (set_j, [right, rj[6], -0.1], "right_w2 decrease"),
        'c': (grip_right.close, [], "right: gripper close"),
        'x': (grip_right.open, [], "right: gripper open"),
        'b': (grip_right.calibrate, [], "right: gripper calibrate"),
    }

    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
            elif c in bindings:
                cmd = bindings[c]
                #expand binding to something like "set_j(right, 's0', 0.1)"
                cmd[0](*cmd[1])
                print("command: %s" % (cmd[2],))
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
    epilog = """
    See help inside the example with the '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])
 
    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    print("Moving into neutral position...")
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
 
    map_keyboard()
    print("Done.")
 
if __name__ == '__main__':
    main()
















