#!/usr/bin/env python
import unittest, rostest
import rosnode, rospy
import time
from test_rosbook.msg import MotorFreqs
from test_rosbook.srv import TimedMotion #add
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse #add

class MotorTest(unittest.TestCase):
    def setUp(self):
        rospy.wait_for_service('/motor_on')
        rospy.wait_for_service('/motor_off')
        rospy.wait_for_service('/timed_motion') #add
        on = rospy.ServiceProxy('/motor_on', Trigger)
        ret = on()

    def file_check(self, dev, value, message):
        with open("/dev/" + dev,"r") as f:
            self.assertEqual(f.readline(),str(value)+"\n",message)

    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/motors', nodes, "node does not exist")

    def test_put_freq(self):
        pub = rospy.Publisher('/motor_raw', MotorFreqs)
        m = MotorFreqs()
        m.left_hz = 123
        m.right_hz = 456
        for i in range(10):
            pub.publish(m)
            time.sleep(0.1)

        self.file_check("rtmotor_raw_l0", m.left_hz, "wrong left value from motor_raw")
        self.file_check("rtmotor_raw_r0", m.right_hz, "wrong left value from motor_raw")
     
    def test_put_cmd_vel(self):
        pub = rospy.Publisher('/cmd_vel', Twist)
        m  = Twist()
        m.linear.x = 0.1414
        m.angular.z = 1.57
        for i in range(10):
            pub.publish(m)
            time.sleep(0.1)
        
        self.file_check("rtmotor_raw_l0",200,"wrong left value from cmd_vel")
        self.file_check("rtmotor_raw_r0",600,"wrong right value from cmd_vel")

        time.sleep(1.1)
        self.file_check("rtmotor_raw_r0",0,"don't stop after l[s]")
        self.file_check("rtmotor_raw_l0",0,"don't stop after l[s]")

    def test_on_off(self):     #add
        off = rospy.ServiceProxy('/motor_off', Trigger)
        ret = off()
        self.assrtEqual(ret.success, True, "motor off does not succeeded")
        self.assrtEqual(ret.message, "OFF", "motor off wrong message")
        with open("/dev/rtmotoren0","r") as f:
            data = f.readline()
            self.assertEqual(data,"\n","wrong value in rtmotor0 at motor off")
        
        on = rospy.ServiceProxy('/motor_on', Trigger)
        ret = on()
        self.assertEqual(ret.success, True, "motor on does not succeeded")
        self.assertEqual(ret.message, "ON", "motor on wrong message")
        with open("/dev/rtmotoren0","r") as f:
            data = d.readline()
            self.assertEqual(data, "1\n", "wrong value in rtmotor0 at motor on")
    
    def test_put_value_timed(self):
        tm = rospy.ServiceProxy('/timed_motion', TimedMotion)
        tm(-321,654,1500)
        with open("/dev/rtmotor0","r") as f:
            data = f.readline()
            self.assertEqual(data, "-321 654 1500\n", "value does not written tortmotor0")

if __name__ == '__main__':
    # time.sleep(3)
    rospy.init_node('travis_test_motors')
    rostest.rosrun('test_rosbook','travis_test_motors', MotorTest)
