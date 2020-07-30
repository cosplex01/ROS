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
        self.img_wlane = None
        self.img = None
        self.set_cam(1) #TX borad connected cam

   def set_cam(self, _index):
       self.cam = cv2.VideoCapture(int(_index))
       
   def get_image(self):
       ret, img = self.cam.read()
       return ret, img

   def get_bi_img(self):
       ret, img_bgr = self.get_image()
       img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

       lower_wlane = np.array([75,0,220])
       upper_wlane = np.array([175,20,255])
       img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)

       return img_wlane

   def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)
        
        self.img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        h = img_bgr.shape[0]
        w = img_bgr.shape[1]

        lower_wlane = np.array([25,5,240])
        upper_wlane = np.array([40,15,255])

        upper_sig_y = np.array([35,255,255])
        lower_sig_y = np.array([15,20,245])

        upper_sig_r = np.array([15,255,255])
        lower_sig_r = np.array([0,20,245])

        upper_sig_g = np.array([90,255,255])
        lower_sig_g = np.array([60,20,245])

        img_r = cv2.resize(cv2.inRange(self.img_hsv, lower_sig_r, upper_sig_r), (w/2,h/2))
        img_y = cv2.resize(cv2.inRange(self.img_hsv, lower_sig_y, upper_sig_y), (w/2,h/2))
        img_g = cv2.resize(cv2.inRange(self.img_hsv, lower_sig_g, upper_sig_g), (w/2,h/2))

        img_r[int(h/3/2):,:]=0
        img_y[int(h/3/2):,:]=0
        img_g[int(h/3/2):,:]=0
        img_concat = np.concatenate([img_r, img_y, img_g], axis=1)

        self.img_wlane = cv2.inRange(self.img_hsv, lower_wlane, upper_wlane)
       
        #cv2.namedWindow('mouseRGB')
        #cv2.imshow('image_check', img_concat)
#         cv2.imshow('mouseRGB', self.img_hsv)
#         cv2.setMouseCallback('mouseRGB', self.mouseRGB)
        #cv2.waitKey(1)
     
#    def mouseRGB(self, event,x,y,flags,param):
#         if event == cv2.EVENT_LBUTTONDOWN:
#             colorsB = self.img_hsv[y,x,0]
#             colorsG = self.img_hsv[y,x,1]
#             colorsR = self.img_hsv[y,x,2]
#             colors = self.img_hsv[y,x]
#             print("red: ",colorsR)
#             print("Green: ",colorsG)
#             print("Blue: ",colorsB)
#             print("BRG format:",colors)
#             print("Coodinates of pixel: x: ",x,"y: ",y)

if __name__ == "__main__":
    rospy.init_node('image_parser',anonymous=True)
    image_parser = IMGParser()
    rospy.spin()