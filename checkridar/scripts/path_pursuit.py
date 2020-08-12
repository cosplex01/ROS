# -*- coding: utf-8 -*-
#!/usr/bin/env python

import rospy
import rospkg
from sensor_msgs.msg import LaserScan,PointCloud,Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point32,PoseStamped,Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path

import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        rospy.Subscriber("path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        # 시뮬레이터에서 차량의 속도와 방향을 가진 토픽을 얻기
        rospy.Subscriber("sensor/core", VescStateStamped, self.vesc_callback)
        # 사용자가 시뮬레이터에 지정하는 토픽을 생성하자
        rospy.Subscriber("user/speed", Float64, self.user_callback)
        # rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64, queue_size=1)
        self.motor_msg=Float64()
        self.servo_msg=Float64()
        self.is_path=False
        self.is_odom=False
        self.is_amcl=False
        self.is_vesc=False
        self.is_speed=False
        self.forward_point=Point()
        self.current_postion=Point()
        self.is_look_forward_point=False
        self.vehicle_length=1
        # 차량의 초기값
        self.user_speed=50000
        # localpath의 리딩 초기값
        self.lfd=2.5
        self.steering=0

        self.steering_angle_to_servo_gain =-1.2135
        self.steering_angle_to_servo_offset=0.5304   
        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            if self.is_path ==True and (self.is_odom==True or self.is_amcl==True or self.is_vesc==True) :
                
                vehicle_position=self.current_postion
                rotated_point=Point()
                self.is_look_forward_point= False

                # 시뮬레이션에서 보내주는 차량제어 토픽이 맞는가
                if self.is_vesc == True:
                    # 차량속도를 지정한다
                    vehicle_speed = self.vehicle_speed/4616*3.6
                    if vehicle_speed < 5:
                        self.lfd = 2.5
                    elif vehicle_speed >= 5 and vehicle_speed < 20:
                        self.lfd = 0.5 * vehicle_speed
                    else:
                        self.lfd = 10.0

                for num,i in enumerate(self.path.poses) :
                    path_point=i.pose.position
                    dx= path_point.x - vehicle_position.x
                    dy= path_point.y - vehicle_position.y
                    rotated_point.x=cos(self.vehicle_yaw)*dx +sin(self.vehicle_yaw)*dy
                    rotated_point.y=sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy
            
                    dis=sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))
                    if dis >= self.lfd :
                        self.forward_point=path_point
                        self.is_look_forward_point=True
                        break
        
                theta=-atan2(rotated_point.y,rotated_point.x)
                if self.is_look_forward_point :
                    self.steering=atan2((2*self.vehicle_length*sin(theta)),self.lfd) #rad
                    print(self.steering*180/pi) #degree
                    self.motor_msg.data= self.user_speed

                else : 
                    self.steering=0
                    print("no found forward point")
                    self.motor_msg.data=0
                
                self.steering_command=(self.steering_angle_to_servo_gain*self.steering)+self.steering_angle_to_servo_offset 
                self.servo_msg.data=self.steering_command
                
                self.servo_pub.publish(self.servo_msg)
                self.motor_pub.publish(self.motor_msg)
            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  #nav_msgs/Path 

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def amcl_callback(self,msg):
        self.is_amcl=True
        amcl_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(amcl_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y
    
    def vesc_callback(self,msg):
        self.is_vesc=True
        self.vehicle_speed =msg.state.speed
    # 사용자가 차량 속도지정 토픽 데이터 전달
    def user_callback(self,msg):
        self.user_speed=msg.data


if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass