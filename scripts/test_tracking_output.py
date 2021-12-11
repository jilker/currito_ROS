#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import ColorRGBA
class TLeds():
    def __init__(self):
        self.publish = rospy.Publisher("/leds",ColorRGBA,queue_size=100)
        self.msg = ColorRGBA()
        self.msg.r = 255
        self.msg.g = 0
        self.msg.b = 0
        self.msg.a = 0
    def pub(self,event=None):
        self.publish.publish(self.msg)

if __name__ == "__main__":
    nh = rospy.init_node('Test_leds')
    leds = TLeds()
    rate = rospy.Rate(5)
    rospy.Timer(rospy.Duration(0.5), leds.pub)
    while not rospy.is_shutdown():
        if leds.msg.a > 360:
            leds.msg.a = 0
        else:
            leds.msg.a+=1
        rate.sleep()
