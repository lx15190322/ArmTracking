#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16
sign = 0

def callback(data):
    global sign
    sign = data.data
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", sign)
    
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    while(1):
        rospy.Subscriber("lock", UInt16, callback)
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", sign)
    #rospy.spin()
