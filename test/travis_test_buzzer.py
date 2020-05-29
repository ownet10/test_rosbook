#!/usr/bin/env python
import rospy, unittest, rostest
import rosnode
import time
from std_msgs.msg import UInt16

class BuzzerTest(unittest.TestCase):
    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/buzzer' ,nodes, "node does not exist")

    def test_put_value(self):
        pub = rospy.Publisher('/buzzer', UInt16)
        for i in range(10):
            pub.publish(1234)
            time.sleep(0.1)

if __name__ == '__main__':
    time.sleep(3)
    rospy.init_node('travis_test_buzzer')
    rostest.rosrun('test_rosbook','travis_test_buzzer',BuzzerTest)
