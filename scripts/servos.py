#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16

class Servo():
    def __init__(self):
        self.nh = rospy.init_node('pc_servo')
        self.pub = rospy.Publisher("/servo",UInt16,queue_size=10)
    def move(self,value):
        msg = UInt16()
        msg.data = value
        self.pub.publish(msg)
if __name__ == "__main__":
    servo = Servo()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        value = int(input("value: "))
        servo.move(value)
        rate.sleep()

    
