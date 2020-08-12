#-*- coding: utf-8 -*-
#!/usr/bin/env python

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
#경로파일을 읽어들인 다음 경로데이터를 ROS에 전달합니다
class make_path :
    def __init__(self):        
        rospy.init_node('make_path',anonymous=True)
        #path on odometry to save on subProc
        #rospy.Subscriber('/odom', Odometry, self.odom_callback)
        #path data send to subscriber
        self.path_pub = rospy.Publisher('/path',Path, queue_size=1)
        self.path_local = rospy.Publisher('/local_path',Path, queue_size=1)
        self.is_odom = False
        self.path_msg=Path()
        #로컬 패스를 담을 공간을 지정하여 초기화
        self.local_msg=Path()
        #self.path_msg.header.frame_id='/odom'
        #gps conncection to map loading is /map
        self.path_msg.header.frame_id='/map'
        self.local_msg.header.frame_id='/map'       
        self.prev_x=0.0
        self.prev_y=0.0
        
        #read on path at saveFile
        rospack=rospkg.RosPack()
        #save to path odometry in File that find pakageFile
        pkg_path=rospack.get_path('checkridar')
        #load and replay on path
        full_path=pkg_path+'/path'+'/path.txt'
        self.f=open(full_path,'r')

        #if path loading
        lines=self.f.readlines()
        #로컬 패스에 최대치로 표시(읽어들일)구간 간격을 지정한다
        self.local_index = 3
        #읽어들인 패스의 데이터 인덱스범위를 지정한다(우선은 10000으로)
        self.temp_index=10000
            
        for line in lines:
            tmp=line.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.orientation.w = 1            
            self.path_msg.poses.append(read_pose)
            self.path_pub.publish(self.path_msg)
        self.f.close()
        rate = rospy.Rate(20)        
        #읽어들인 경로의 인덱스를 계산하기 위한 변수지정
        linecount=len(self.path_msg.poses)
        print(linecount)

        #자동차 본인의 일부 구간경로를 읽어들인다
        while not rospy.is_shutdown():    
            #read on path at saveFile
            rospack=rospkg.RosPack()
            #make file on created Folder
            full_path=pkg_path+'/path'+'/path.txt'
            self.f2=open(full_path,'r')
            #if path loading
            lines=self.f2.readlines()
            #읽어들인 패스의 데이터 인덱스범위를 지정한다(우선은 10000으로)
            self.temp_index=10000

            for line in lines:
                tmp=line.split()
                tmp2=tmp
                read_pose1=PoseStamped()
                read_pose1.pose.position.x = float(tmp[0])
                read_pose1.pose.position.y = float(tmp[1])
                read_pose1.pose.orientation.w = 1            
                
                nowline = sqrt((self.prev_x-float(tmp[0]))**2 + (self.prev_y-float(tmp[1]))**2)
                if nowline<=self.temp_index:
                    self.temp_index = nowline
                    tmp2[0] = tmp[0]
                    tmp2[1] = tmp[1]
                self.local_msg.poses.append(read_pose1)
                self.path_local.publish(self.local_msg)
            #if path loading
        self.f2.close()
        rate.sleep()
    
if __name__ == '__main__' :
    try:
        test_track = make_path()
    except rospy.ROSInterruptException :
        pass
        
