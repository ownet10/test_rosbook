#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16

def recv_buzzer(self):
    rospy.loginfo(type(self))
    rospy.loginfo(self.data)

if __name__ == '__main__':
    rospy.init_node('buzzer')
    rospy.Subscriber("buzzer", UInt16, recv_buzzer)
