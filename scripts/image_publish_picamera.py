#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Publish a video as ROS messages.
"""

import argparse

import numpy as np

import cv2

import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo


from cv_bridge import CvBridge, CvBridgeError
import picamera, picamera.array



bridge = CvBridge()

def main():
    """Publish a video as ROS messages.
    """
    # Patse arguments.



    # Set up node.
    rospy.init_node("video_publisher", anonymous=True)
    img_pub = rospy.Publisher("/camera/image_raw", Image,
                              queue_size=10)

    # Open video.
    # video = cv2.VideoCapture(0)
    with picamera.PiCamera() as camera:
        camera.start_preview()
        time.sleep(2)
        with picamera.array.PiRGBArray(camera) as stream:
            camera.capture(stream, format='bgr')
            # At this point the image is available as stream.array
            image = stream.array

            try:
                # Publish image.
                img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
                # img_msg = bridge.cv2_to_imgmsg(img_out, "rgba8")
                # img_msg.header.stamp = rospy.Time.now()
                # img_msg.header.frame_id = 1
                img_pub.publish(img_msg)


            except CvBridgeError as err:
                print(err)

            # if cv2.waitKey(1) == 27: 
            #     break  # esc to quit
        return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass