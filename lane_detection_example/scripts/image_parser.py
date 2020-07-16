#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from utils import warp_image
from utils import BEVTransform

class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.source_prop = np.float32([[0.05, 0.65],
                                    [0.5 - 0.15, 0.52],
                                    [0.5 + 0.15, 0.52],
                                    [1 - 0.05, 0.65]])
    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)
        
        self.img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        lower_wlane = np.array([25,5,240])
        upper_wlane = np.array([40,15,255])

        self.img_wlane = cv2.inRange(self.img_hsv, lower_wlane, upper_wlane)
        self.img_wlane = cv2.cvtColor(self.img_wlane, cv2.COLOR_GRAY2BGR)

        img_concat = np.concatenate([img_bgr,self.img_hsv,self.img_wlane], axis=1)

        img_warp = warp_image(self.img_wlane, self.source_prop) 

        
        cv2.imshow('mouseRGB',img_warp)
        cv2.namedWindow('mouseRGB')
        cv2.setMouseCallback('mouseRGB',self.mouseRGB)
        cv2.imshow('display',img_concat)
        cv2.namedWindow('display')
        
        cv2.waitKey(1)

    def mouseRGB(self, event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN:
            colorsB = self.img_hsv[y,x,0]
            colorsG = self.img_hsv[y,x,1]
            colorsR = self.img_hsv[y,x,2]
            colors = self.img_hsv[y,x]
            print("red: ",colorsR)
            print("Green: ",colorsG)
            print("Blue: ",colorsB)
            print("BRG format:",colors)
            print("Coodinates of pixel: x: ",x,"y: ",y)

if __name__ == "__main__":
    rospy.init_node('image_parser',anonymous=True)
    image_parser = IMGParser()
    rospy.spin()