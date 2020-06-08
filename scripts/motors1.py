#!/usr/bin/env python
import sys, rospy, math
from test_rosbook.msg import MotorFreqs
from geometry_msgs.msg import Twist

class Motor():
    def __init__(self):
        if not self.power(True): sys.exist(1)

        rospy.on_shutdown(self.power)
        self.sub_raw = rospy.Subscriber('motor_raw', MotorFreqs, self.callback_raw_freq)
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.callback_cmd_vel)
        self.last_time = rospy.Time.now()
 
    def callback_raw_freq(self, message):
        self.raw_freq(message.left_hz,message.right_hz)
    
    def callback_cmd_vel(self, message):
        forward_hz = 80000.0*message.linear.x/(9*math.pi)
        rot_hz = 400.0*message.angular.z/math.pi
        self.set_raw_freq(forward_hz-rot_hz, forward_hz+rot_hz)
        self.using_cmd_vel = True
        self.last_time = rospy.Time.now()

    def power(self,onoff = False):
        dev = '/dev/rtmotoren0'
        try:
            with open(dev,'w') as f:
                f.write("1\n" if onoff else "0\n")
            self.is_on = onoff
            return True
        except OSError:
            rospy.logerr("can't write to" + dev)
        return False    
    
    def raw_freq(self,left_hz,right_hz):
        if not is_on:
            rospy.logerr("not empowered")
            return
        try:
            with open("/dev/rtmotor_raw_l0",'w') as lf,\
                    open("/dev/rtmotor_raw_r0",'w') as rf:
                        lf.write(str(int(round(left_hz))) + "\n")
                        rf.write(str(int(round(right_hz))) + "\n")
        except OSError:
            rospy.logerr("can't write to rtmotor_raw_*")

if __name__ == '__main__':
    rospy.init_node('motors')
    m = Motor()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if m.using_cmd_vel and rospy.Time.now().to_sec() - m.last_time.to_sec() >= 1.0:
            m.set_raw_freq(0,0)
            m.using_cmd_vel = False
        rate.sleep()
