#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from utils import BEVTransform, CURVEFit, draw_lane_img
from image_parser import IMGParser

if __name__ == '__main__':
    rp = rospkg.RosPack()

    currentPath = rp.get_path("lane_detection_example")

    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)
    
    params_cam = sensor_params["params_cam"]
    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()
    bev_op = BEVTransform(params_cam=params_cam)
    curve_learner = CURVEFit(order=3)

    rate = rospy.Rate(20)