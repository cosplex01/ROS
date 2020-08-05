#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import rospkg

from sensor_msgs.msg import LaserScan, PointCloud, Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from morai_msgs.msg import GPSMessage
from math import cos, sin, pi, sqrt, pow
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry,Path

import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class make_path :
    def __init__(self):        
        rospy.init_node('make_path',anonymous=True)
        #path on odometry to save on subProc
        #rospy.Subscriber('/odom', Odometry, self.odom_callback)
        #path data send to subscriber
        self.path_pub = rospy.Publisher('/sensor_msgs/PointCloud',PointCloud, queue_size=1)
        self.is_odom = False
        self.path_msg=PointCloud()
        self.path_msg.header.frame_id='/map'
        self.prev_x=0.0
        self.prev_y=0.0
        self.prev_z=0.0        

        self.x_Offset= GPSMessage().eastOffset
        self.y_Offset= GPSMessage().northOffset

        #read on path at saveFile
        rospack=rospkg.RosPack()
        #save to path odometry in File that find pakageFile
        pkg_path=rospack.get_path('lane_detection_example')
        #make file on created Folder
        full_path=pkg_path+'/kcity_PM0138'+'/A1LANE_CenterLine_*.csv'
        self.f=open(full_path,'r')
        #if path loading
        lines=self.f.readlines()[8:]
        for line in lines:
            tmp=line.split()
            read_Point = Point32()
            #read_pose=PoseStamped()
            read_Point.x = float(tmp[0]) - self.x_Offset
            read_Point.y = float(tmp[1]) - self.y_Offset
            read_Point.z = float(tmp[2])
            #read_pose.pose.orientation.w = 1
            print(read_Point.x, read_Point.y, read_Point.z)
            self.path_msg.points.append(read_Point)
        #load and replay on path
        rate = rospy.Rate(20)

        #if path loading
        self.f.close()

        while not rospy.is_shutdown():    
            #don`t close __init__        
            #rospy.spin()
            #close __init__ and load path
            self.path_pub.publish(self.path_msg)
            rate.sleep()
        #if path making 
        #self.f.close()
   

if __name__ == '__main__' :
    try:
        test_track = make_path()
    except rospy.ROSInterruptException :
        pass
        