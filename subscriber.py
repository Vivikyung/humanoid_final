import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard %s",data.data)
    
def listener():
    rospy.init_node('node_name')
    rospy.Subscriber("chatter", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
