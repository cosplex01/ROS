#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos, sin, pi
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
#IMU function have eulerAngle and QuaternionAngle
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class simple_kinematic :

    def __init__(self):
        rospy.init_node('simple_kinematic', anonymous=True)

        rospy.Subscriber('/sensors/core', VescStateStamped, self.status_callback)
        rospy.Subscriber('/sensors/servo_position_command', Float64, self.servo_command_callback)
        #send imu calculator data to topic
        rospy.Subscriber('/imu', VescStateStamped, self.imu_callback)        
        #base init (motorRPM, servo, imu)
        self.is_speed = False
        self.is_servo = False
        self.is_imu = False
        self.servo_msg=Float64()
        #odometry link and deta send (to need the acting tf procedure)
        self.odom_pub = rospy.Publisher('/odom', Odometry , queue_size=1)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = '/odom'
        #basic value setting
        self.rpm_gain=4614
        self.steering_angle_to_servo_gain = -1.2135
        self.steering_angle_to_servo_offset = 0.5304
        self.theta=0
        self.L=0.5

        rate = rospy.Rate(20)
        #kinematic model function and odometry function
        while not rospy.is_shutdown():
            if self.is_servo == True and self.is_speed == True :
                #sending data to Terminal Console (motorRPM, handleUse)
                print(self.speed, self.servo_angle_rad*180/pi)
                #kinematic model function
                x_dot = self.speed*cos(self.theta+self.servo_angle_rad)/20
                y_dot = self.speed*sin(self.theta+self.servo_angle_rad)/20
                theta_dot = self.speed*sin(self.servo_angle_rad)/self.L/20
                #odometry function
                self.odom_msg.pose.pose.position.x = self.odom_msg.pose.pose.position.x+x_dot
                self.odom_msg.pose.pose.position.y = self.odom_msg.pose.pose.position.y+y_dot
                #odometry function to linkdata on kinematic model data
                self.theta = self.theta+theta_dot
                quaternion = quaternion_from_euler(0,0,self.theta)
                self.odom_msg.pose.pose.orientation.x =quaternion[0]
                self.odom_msg.pose.pose.orientation.y =quaternion[1]
                self.odom_msg.pose.pose.orientation.z =quaternion[2]
                self.odom_msg.pose.pose.orientation.w =quaternion[3]        
                #linkdata to broadcast to RViz and topic Messages
                self.odom_pub.publish(self.odom_msg)
                br = tf.TransformBroadcaster()
                br.sendTransform((self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y, self.odom_msg.pose.pose.position.z),
                                  quaternion, rospy.Time.now(), "base_link", "odom",)              
            rate.sleep()

    #motorRPM calculate
    def status_callback(self,msg):
        self.is_speed = True
        rpm = msg.state.speed
        self.speed = rpm/self.rpm_gain
    #handle control data calculate
    def servo_command_callback(self, msg):
        self.is_servo=True
        servo_value = msg.data
        self.servo_angle_rad = (servo_value-self.steering_angle_to_servo_offset)/self.steering_angle_to_servo_gain
    #imu data calculate
    def imu_callback(self, msg):  
        imu_quanternion = (msg.orientation.x ,msg.orientation.y, msg.orientation.z ,msg.orientation.w)
        if self.is_imu == False:
            self.is_imu=True
            #roll,pitch,yaw - (_ = roll,_ = pitch,)
            _,_,self.theta_offset=euler_from_quaternion(imu_quanternion)
        else:
            _,_,raw_theta=euler_from_quaternion(imu_quanternion)
            self.theta=raw_theta-self.theta 


if __name__ == "__main__":
    try:
        test=simple_kinematic()
    except rospy.ROSInterruptException:
        pass
    