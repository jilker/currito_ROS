#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse

import numpy as np

import cv2

import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np

class CameraCurrito():
    def __init__(self):
        self.node = rospy.init_node("image_currito", anonymous=True)
        self.img_pub = rospy.Publisher("/camera/image_raw", Image,queue_size=10)
        self.video = cv2.VideoCapture(0)
    def loop(self):
        while not rospy.is_shutdown() and self.video.grab():
            tmp,img = self.get_image()
            mask = self.get_mask(img)
            res = cv2.bitwise_and(img,img,mask=mask)
            contours  = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for it in contours[1]:
                cv2.drawContours(img, it, -1, (0,255,0), 3)
            c = max(contours[1], key=cv2.contourArea)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(img, center, 5, (0, 0, 255), -1)
            cv2.imshow('mask',mask)
            cv2.imshow('res',res)
            cv2.imshow("frame", img)
            # cv2.imshow("cnts", cnts)
            cv2.waitKey(1)
    def get_mask(self,img):
        img = cv2.GaussianBlur(img, (11, 11), 0)
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        lower_red_1 = np.array([160,100,20])
        upper_red_1 = np.array([180,255,255])
        mask_A = cv2.inRange(hsv,lower_red_1,upper_red_1)
        mask_A = cv2.erode(mask_A, None, iterations=2)
        mask_A = cv2.dilate(mask_A, None, iterations=2)
        lower_red_2 = np.array([0,100,20])
        upper_red_2 = np.array([10,255,255])
        mask_B = cv2.inRange(hsv,lower_red_2,upper_red_2)
        mask_B = cv2.erode(mask_B, None, iterations=2)
        mask_B = cv2.dilate(mask_B, None, iterations=2)
        return mask_A+mask_B
    def get_image(self):
        tmp, img = self.video.read()
        # cv2.imshow("test", img)
        if not tmp:
            print("Could not grab frame.")
            return 0
        else:
            return tmp,img

if __name__ == '__main__':
    try:
       camera_currito = CameraCurrito()
       camera_currito.loop()
    except rospy.ROSInterruptException:
        pass