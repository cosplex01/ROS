#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from math import cos, sin, pi, sqrt
from geometry_msgs.msg import Point32

class simple_controller:
    def __init__(self):
        rospy.init_node("simple_controller", anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        self.motor_pub = rospy.Publisher("commands/motor/speed", Float64, queue_size=1)
        self.servo_pub = rospy.Publisher("commands/servo/position", Float64, queue_size=1)

        self.pcd_pub = rospy.Publisher("Laser2pcd", PointCloud, queue_size=1)


        while not rospy.is_shutdown():
            rospy.spin()

    def laser_callback(self, msg):
        pcd = PointCloud()
        motor_msg = Float64()
        servo_msg = Float64()
        pcd.header.frame_id = msg.header.frame_id
        angle = 0

        for r in msg.ranges:
            tmp_point=Point32()
            tmp_point.x=r*cos(angle)
            tmp_point.y=r*sin(angle)

            #print(angle, tmp_point.x, tmp_point.y)
            angle=angle+(1.0/180*pi)
            if r<12:
                pcd.points.append(tmp_point)
        
        count=0
        for point in pcd.points:
            if 0 < point.x and point.x < 2.5:
                if -1 < point.y and point.y < 1:
                    count = count + 1
        if count > 20:
            avgLeft = 0.0
            avgRight = 0.0
            ls = 0
            rs = 0
            for point in pcd.points:
                if 0 < point.x:
                    c = (point.x * point.x) + (point.y * point.y)
                    if point.y < 0:
                        avgLeft += sqrt(c)
                        ls += 1
                    else:
                        avgRight += sqrt(c)
                        rs += 1
            avgLeft = avgLeft / ls
            avgRight = avgRight / rs

            if avgLeft < avgRight:
                servo_msg.data = 0.15
                motor_msg.data=20000
            else:
                servo_msg.data = 0.85
                motor_msg.data=20000
        else :
            motor_msg.data=20000
            servo_msg.data = 0.5304
        
        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_msg)
        self.pcd_pub.publish(pcd)
    




if __name__ == "__main__":
    try:
        test_track=simple_controller()
    except rospy.ROSInterruptException:
        pass