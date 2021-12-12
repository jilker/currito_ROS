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
        if leds.msg.a > 22:
            leds.msg.a = 0
        else:
            leds.msg.a+=1

if __name__ == "__main__":
    nh = rospy.init_node('Test_leds')
    leds = TLeds()
    rate = rospy.Rate(2)
    rospy.Timer(rospy.Duration(0.1), leds.pub)
    while not rospy.is_shutdown():
        rate.sleep()
    # while not rospy.is_shutdown():
    #     value = int(input("value: "))
    #     leds.msg.a = value
    #     rate.sleep()