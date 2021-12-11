#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

import pygame


class Main_window():
    def __init__(self, debug=False ,ser= None):
        self.debug = debug
        self.source = None
        self.status = True
        self.ser = ser
        self.Drive = None
        self.Turn = None
        pygame.init()
    
        # Set the width and height of the screen [width,height]
        self.size = [500, 980]
        self.img_size = (640//2, 480//2)
        self.screen = pygame.display.set_mode(self.size)

        pygame.display.set_caption("Joystick")


        # Used to manage how fast the screen updates
        self.clock = pygame.time.Clock()

        # Initialize the joysticks
        self.screen.fill((255,255,255))
        pygame.display.update()

        # Get ready to print
        self.bridge = CvBridge()
        pos_center = [500//2, 980//4]
        self.pos_center = [pos_center[0] - self.img_size[0]//2, pos_center[1] - self.img_size[1]//2]


        rospy.init_node('Joy_listener', anonymous=True)

        rospy.Subscriber("/joy", Joy, self.cb_joy)
        rospy.Subscriber("/camera/image_raw", Image, self.cb_image)


    def cvimage_to_pygame(self, image):
        return pygame.image.frombuffer(image.tobytes(), image.shape[1::-1], "RGB")

    def cb_joy(self, data):

        self.axes = data.axes
        buttons = data.buttons

    def cb_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
            cv_image = cv2.resize(cv_image, self.img_size)
            self.image = self.cvimage_to_pygame(cv_image)
            
        except CvBridgeError as e:
            print(e)

        
    def update(self):

        quit = False
        while not quit:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: 
                    quit = True 

            try:
                self.screen.fill((255 * (self.axes[0] + 1)/2,255 * (self.axes[1] + 1)/2, 255 * (self.axes[2] + 1)/2))
            except:
                pass

            try:
                self.screen.blit(self.image, self.pos_center)
            except:
                pass

            pygame.display.update()

            self.clock.tick(60)



        # spin() simply keeps python from exiting until this node is stopped
        # rospy.spin()

Window1 = Main_window()
Window1.update()
