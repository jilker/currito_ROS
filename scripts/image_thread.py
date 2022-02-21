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

import queue, threading, time

import queue as Queue



class CameraCurrito():
    def __init__(self):
        self.node = rospy.init_node("image_currito", anonymous=True)
        self.img_pub = rospy.Publisher("/camera/image_raw", Image,queue_size=10)
        print("Initializing Camera")

        self.video = cv2.VideoCapture("/dev/video0")

        self.q = queue.Queue()
        t = threading.Thread(target=self._reader)
        t.daemon = True
        t.start()

    # read frames as soon as they are available, keeping only most recent one

        self.video.set(cv2.CAP_PROP_BUFFERSIZE, -1)
        print("Camera initialized")
        self.bridge = CvBridge()

        # self.height = 0
        # self.width = 0

        self.pub = rospy.Publisher("/datos_pelota", Joy, queue_size=10)
        self.msg_pelota = Joy()

        self.kernel = np.ones((4,4),np.uint8)
        try:
            with open("hsv.txt", "r") as f:
                self.hsv_min = np.array(f.readline(),np.uint8)
                self.hsv_max = np.array(f.readline(),np.uint8)
        except:
            print("NOT READING HSV FILE")
            self.hsv_min = np.array([15.042006133540337, 154.2345059435631, 151.06768973892002],np.uint8)
            self.hsv_max = np.array([124.61171741707324, 198.58528129576243, 217.8428054394965],np.uint8)
        self.FPS_target = 5

    def _reader(self):
        while True:
            ret, frame = self.video.read()
            if not ret:
                break
            if not self.q.empty():
                try:
                    self.q.get_nowait()   # discard previous (unprocessed) frame
                except queue.Empty:
                    pass
        self.q.put(frame)

    def read(self):
        return self.q.get()

    def loop(self):
        print("Running main loop")
        while not rospy.is_shutdown():
            start = time.time()
            img = self.get_image()
            img = cv2.resize(img, (250, 250))

            # self.height,self.width = img.shape[0:2]

            mask = self.get_mask(img)
            # res = cv2.bitwise_and(img,img,mask=mask)
            # res = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)



            contours, _  = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)

                ((x, y), radius) = cv2.minEnclosingCircle(c)
                try:
                    cv2.circle(img, (int(x),int(y)), 15, (0, 0, 255), -1)
                except:
                    pass
            
                self.msg_pelota.axes = [x/img.shape[0]*2-1, y/img.shape[1]*2-1, radius]
                self.pub.publish(self.msg_pelota)

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
            cv2.imshow('detected circles',img)
            print("FPS = ", 1/(time.time()-start))
            cv2.waitKey(1)

    def get_mask(self,img):

        hsv_image = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        threshed_image = cv2.inRange(hsv_image, self.hsv_min, self.hsv_max)


        threshed_image = cv2.morphologyEx(threshed_image, cv2.MORPH_CLOSE, self.kernel)
        return threshed_image

    def get_image(self):
        ret, img = self.video.read()
        print(img)
        return img

if __name__ == '__main__':
    try:
       camera_currito = CameraCurrito()
       camera_currito.loop()
    except rospy.ROSInterruptException:
        pass
