#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('joints', String, queue_size=10)
    rospy.init_node('baxter_map', anonymous=True)
    rate = rospy.Rate(0.5) # 10hz
    while not rospy.is_shutdown():
        pose_str = "0.1817,0.8461,-0.5083,0.1817,0.2,-0.5083"
        rospy.loginfo(pose_str)
        pub.publish(pose_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
