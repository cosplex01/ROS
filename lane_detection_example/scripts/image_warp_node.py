#!/user/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from utils import BEVTransform, draw_lane_img
from lane_detection import curve_learner

class IMGParser:
    def __init__(self):
        self.img_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.img_wlane = None
    
    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            imb_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)
        
        img_hsv = cv2.cvtColor(imb_bgr, cv2.COLOR_BGR2HSV)

        lower_wlane = np.array([0,0,190])
        upper_wlane = np.array([30,30,220])
        self.img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)
    
if __name__ == '__main__':
    rp = rospkg.RosPack()

    currentPath = rp.get_path("lane_detection_example")
    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)
    params_cam = sensor_params["params_cam"]

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()
    bev_op = BEVTransform(params_cam=params_cam)

    rate = rospy.Rate(30)

    