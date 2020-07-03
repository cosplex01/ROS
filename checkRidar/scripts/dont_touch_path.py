#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import rospkg

from sensor_msgs.msg import LaserScan, PointCloud, Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos, sin, pi, sqrt, pow
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Odometry,Path

import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class make_path :
    def __init__(self):        
        rospy.init_node('make_path',anonymous=True)
        #path on odometry to save on subProc
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        #path data send to subscriber
        self.path_pub = rospy.Publisher('/path',Path, queue_size=1)
        self.is_odom = False
        self.path_msg=Path()
        self.path_msg.header.frame_id='/odom'
        self.prev_x=0
        self.prev_y=0
        #read on path at saveFile
        rospack=rospkg.RosPack()
        #save to path odometry in File that find pakageFile
        pkg_path=rospack.get_path('checkRidar')
        #make file on created Folder
        full_path=pkg_path+'/path'+'/path.txt'
        self.f=open(full_path,'w')
        #if path loading
        #lines=self.f.readlines()
        #for line in lines:
        #    tmp=line.split()
        #    read_pose=PoseStamped()
        #    read_pose.pose.position.x = float(tmp[0])
        #    read_pose.pose.position.y = float(tmp[1])
        #    read_pose.pose.orientation.w = 1
        #    self.path_msg.poses.append(read_pose)
        #load and replay on path
        #rate = rospy.Rate(20)

        #if path loading
        #self.f.close()

        while not rospy.is_shutdown():    
            #don`t close __init__        
            rospy.spin()
            #close __init__ and load path
            #self.path_pub.publish(self.path_msg)
            #rate.sleep()
        #if path making 
        self.f.close()

    def odom_callback(self, msg):
        #position value make      
        waypoint_pose=PoseStamped()
        #position recieive to msg.pose
        x= msg.pose.pose.position.x
        y= msg.pose.pose.position.y    
        if self.is_odom == True:     
            #position recieve time setting
            distance = sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
            if distance > 0.1:
                #recieve position send position value
                waypoint_pose.pose.position.x = x 
                waypoint_pose.pose.position.y = y
                waypoint_pose.pose.orientation.w = 1
                #position value send path_msg
                self.path_msg.poses.append(waypoint_pose)
                #stack to pathPoint at Format PoseStamped in path_msg
                self.path_pub.publish(self.path_msg)
                data = '{0}\t{1}\n'.format(x,y)
                self.f.write(data)
                self.prev_x = x
                self.prev_y = y
                print(x,y)
            
        else :
            self.is_odom =True
            self.prev_x = x
            self.prev_y = y

if __name__ == '__main__' :
    try:
        test_track = make_path()
    except rospy.ROSInterruptException :
        pass
        