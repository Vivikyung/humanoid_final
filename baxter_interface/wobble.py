#!/usr/bin/env python

import math
import argparse 
import random
import rospy 
from std_msgs.msg import (
    UInt16,
)
import baxter_interface
from baxter_interface import CHECK_VERSION

class Wobbler(object):
    def __init__(self):
        #Wobbles head
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._done = False
        self._head = baxter_interface.Head()
        self._left_arm = baxter_interface.limb.Limb("left")
        self._right_arm = baxter_interface.limb.Limb("right")
        self._left_joint_names = self._left_arm.joint_names()
        self._right_joint_names = self._right_arm.joint_names()

        #Make sure robot is enabled
        print("Getting robot state...")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled

        print("Enabling robot...")
        self._rs.enable()

        print("Program start!")
        self._pub_rate.publish(100)

    def _reset_control_modes(self):
        rate = rospy.Rate(100)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._left_arm.exit_control_mode()
            self._right_arm.exit_control_mode()
            self._pub_rate.publish(100)
            rate.sleep()
        return True

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        print("Moving to neutral pose...")
        self._left_arm.move_to_neutral()
        self._right_arm.move_to_neutral()

        #Sets head into neutral pose
        self._head.set_pan(0.0)

    def clean_shutdown(self):
        #Clean end to program
        print("Done with program")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        if self._done:
            self.set_neutral()
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def wobble(self):
        self.set_neutral()

        self._head.command_nod()
        command_rate = rospy.Rate(1)
        control_rate = rospy.Rate(100)
        start = rospy.Time.now()

        def make_v_func():
            """
            returns a randomly parameterized cosine function to control a
            specific joint.
            """
            period_factor = random.uniform(0.3, 0.5)
            amplitude_factor = random.uniform(0.1, 0.2)
     
            def v_func(elapsed):
                w = period_factor * elapsed.to_sec()
                return amplitude_factor * math.cos(w * 2 * math.pi)
            return v_func

        v_funcs = [make_v_func() for _ in self._right_joint_names]

        def make_cmd(joint_names, elapsed):
                return dict([(joint, v_funcs[i](elapsed))
                             for i, joint in enumerate(joint_names)])

        print("Wobbling. Press Ctrl-C to stop...")
        while not rospy.is_shutdown():
            self._pub_rate.publish(100)
            angle = random.uniform(-1.5, 1.5)
            elapsed = rospy.Time.now() - start
            cmd = make_cmd(self._left_joint_names, elapsed)
            self._left_arm.set_joint_velocities(cmd)
            cmd = make_cmd(self._right_joint_names, elapsed)
            self._right_arm.set_joint_velocities(cmd)
            self._head.set_pan(angle, speed=.3, timeout=0)
            control_rate.sleep()

        self._done = True
        rospy.signal_shutdown("Example finished.")

def main():
    """RSDK Head Example: Wobbler
 
   Nods the head and pans side-to-side towards random angles.
   Demonstrates the use of the baxter_interface.Head class.
   Commands joint velocities of randomly parameterized cosine waves
   to each joint. Demonstrates Joint Velocity Control Mode.
   """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.parse_args(rospy.myargv()[1:])
 
    print("Initializing nodes...")
    rospy.init_node("rsdk_body_wobbler")
 
    wobble = Wobbler()

    rospy.on_shutdown(wobble.clean_shutdown)
    print("Wobbling... ")
    wobble.wobble()
    print("Done.")
 
if __name__ == '__main__':
    main()

















