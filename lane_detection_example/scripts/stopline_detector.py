#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg
import json
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from image_parser import IMGParser

from utils import BEVTransform, STOPLineEstimator

if __name__ == "__main__":

    rp = rospkg.RosPack()

    currentPath = rp.get_path("lane_detection_example")

    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)
    
    params_cam = sensor_params["params_cam"]
    
    rospy.init_node('stoplane_detector', anonymous=True)
    image_parser = IMGParser()
    bev_op = BEVTransform(params_cam=params_cam)
    sline_detectpr = STOPLineEstimator()

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        if image_parser.img_wlane is not None:
            lane_pts = bev_op.recon_lane_pts(image_parser.img_wlane)

            sline_detectpr.get_x_points(lane_pts)
            sline_detectpr.estimate_dist(0.3)
            sline_detectpr.pub_sline()

            rate.sleep()