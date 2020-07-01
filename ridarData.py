#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from math import cos, sin, pi
from geometry_msgs.msg import Point32

class simple_controller:
    def __init__(self):
        rospy.init_node("simple_controller", anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        self.motor_pub = rospy.Publisher("commands/motor/speed", Float64, queue_size=1)
        # get the motor rpm
        self.servo_pub = rospy.Publisher("commands/servo/position", Float64, queue_size=1)
        # get the angle driving wheel
        self.pcd_pub = rospy.Publisher("Laser2pcd", PointCloud, queue_size=1)
        # get the lidar data(rectangular coordinate system)

        while not rospy.is_shutdown():
            rospy.spin()
    
    def laser_callback(self, msg):
        pcd = PointCloud()
        motor_msg = Float64()
        servo_msg = Float64()
        pcd.header.frame_id = msg.header.frame_id
        angle = 0

        # check the theta and r degree
        #for theta,r in enumerate(msg.ranges):
        #    print(theta,r)

        # lidar scan data is conv to designation matrix
        # conversion to rectangular coordinate system
        for r in msg.ranges:
            tmp_point=Point32()
            tmp_point.x=r*cos(angle)
            tmp_point.y=r*sin(angle)
            print(angle, tmp_point.x, tmp_point.y)
            angle=angle+(1.0/180*pi)
            if r<12:
                pcd.points.append(tmp_point)
        #stop the RC at the found object
        count=0
        wall=0
        Handlemax = 3.0
        Handlemin =-3.0
        for point in pcd.points:
            if (point.x > 0.3 and point.x < 2.0) and (point.y >-1.0 and point.y <1.0): #point.x = ridar range, point.y = handle
                count = count + 1
            if (point.x >0.0 and point.x<0.3):
                if point.y >= 0.2:
                    if point.y < Handlemax:
                        Handlemax = point.y
                elif point.y <-0.2:
                    if point.y > Handlemin:
                        Handlemin = point.y
        # check the destance of the object found by lidar        
        if count > 20:
            motor_msg.data=4000
            if (abs(Handlemax)>abs(Handlemin)):
                servo_msg.data = 0.15
            elif (abs(Handlemax)<abs(Handlemin)):
                servo_msg.data = 0.85
        else :
            motor_msg.data=6000 # 6000 is max on simulator
            servo_msg.data=0.5304
        print(count, Handlemax, Handlemin)
        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_msg)
        self.pcd_pub.publish(pcd)

if __name__ == "__main__":
    try:
        test_track=simple_controller()
    except rospy.ROSInterruptException:
        pass