#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse

import numpy as np
import time
import cv2

import rospy

from sensor_msgs.msg import Image
# from sensor_msgs.msg import CameraInfo
import numpy as np

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Joy



class CameraCurrito():
    def __init__(self):
        self.node = rospy.init_node("image_currito", anonymous=True)
        self.img_pub = rospy.Publisher("/camera/image_raw", Image,queue_size=10)
        print("Initializing Camera")
        self.video = cv2.VideoCapture("/dev/video0",cv2.CAP_V4L)
        print("Camera initialized")
        self.bridge = CvBridge()

        # self.height = 0
        # self.width = 0

        self.pub = rospy.Publisher("/datos_pelota", Joy, queue_size=10)
        self.msg_pelota = Joy()
        self.FPS_target = 15


    def loop(self):
        print("Running main loop")
        while not rospy.is_shutdown():
            start = time.time()
            img = self.get_image()
            img = cv2.resize(img, (250, 250))

            # self.height,self.width = img.shape[0:2]

            mask = self.get_mask(img)
            res = cv2.bitwise_and(img,img,mask=mask)
            res = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)

            circles = cv2.HoughCircles(res,cv2.HOUGH_GRADIENT,1,600,param1=50,param2=12,minRadius=10,maxRadius=500)
            try:
                circles = np.uint16(np.around(circles))

                for circle in circles[0,:]:
                    # print(circle)
                    # draw the outer circle
                    cv2.circle(img,(circle[0],circle[1]),circle[2],(0,255,0),2)
                    # draw the center of the circle
                    cv2.circle(img,(circle[0],circle[1]),2,(0,0,255),3)

                # Publicamos la información del circulo
                # Asumiendo que el circulo 0 sea el bueno
                circle = circles[0,0]
                self.msg_pelota.axes = [circle[0]/img.shape[0]*2-1, circle[1]/img.shape[1]*2-1, circle[2]]
                self.pub.publish(self.msg_pelota)
            except:
                pass
            
            try:
                img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                # img_msg = bridge.cv2_to_imgmsg(img_out, "rgba8")
                # img_msg.header.stamp = rospy.Time.now()
                # img_msg.header.frame_id = 1
                self.img_pub.publish(img_msg)

            except CvBridgeError as err:
                print(err)

            if 1/(time.time()-start) > self.FPS_target:
                time_to_wait = 1/self.FPS_target - (time.time()-start)
                time.sleep(time_to_wait)
            # cv2.imshow('detected circles',img)
            print("FPS = ", 1/(time.time()-start))
            cv2.waitKey(1)

    def get_mask(self,img):
        img = cv2.GaussianBlur(img, (11, 11), 0)
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        color = 0
        if color == 1:
            lower_red_1 = np.array([160,100,20])
            upper_red_1 = np.array([175,255,255])
            mask_A = cv2.inRange(hsv,lower_red_1,upper_red_1)
            mask_A = cv2.erode(mask_A, None, iterations=2)
            mask_A = cv2.dilate(mask_A, None, iterations=2)
            lower_red_2 = np.array([0,100,10])
            upper_red_2 = np.array([10,255,255])
            mask_B = cv2.inRange(hsv,lower_red_2,upper_red_2)
            mask_B = cv2.erode(mask_B, None, iterations=2)
            mask_B = cv2.dilate(mask_B, None, iterations=2)
            return mask_A+mask_B
        else:
            lower_red_1 = np.array([150,43,183])
            upper_red_1 = np.array([179,229,255])
            mask_A = cv2.inRange(hsv,lower_red_1,upper_red_1)
            mask_A = cv2.erode(mask_A, None, iterations=2)
            mask_A = cv2.dilate(mask_A, None, iterations=2)
            return mask_A
    def get_image(self):
        tmp, img = self.video.read()
        if not tmp:
            print("Could not grab frame.")
            return 0
        else:
            return img

if __name__ == '__main__':
    try:
       camera_currito = CameraCurrito()
       camera_currito.loop()
    except rospy.ROSInterruptException:
        pass
