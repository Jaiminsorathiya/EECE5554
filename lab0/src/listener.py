#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    original_message = data.data
    modified_message = ' '.join([word + '@' for word in original_message.split()])
    rospy.loginfo("Received and modified message: %s", modified_message)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
