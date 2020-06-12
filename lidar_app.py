#!/usr/bin/env python
import	rospy
import roslib
import sys
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

def listner():
	rospy.init_node('rp',anonymous = True)
	speed = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
    position = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
	rospy.Subscriber('scan',LaserScan,callback)
	rospy.spin()

def callback(data):
	min=100;
	temp_angle=0;
	for i in range(0,359):
		lidar_dis=data.ranges[i]
		
		if lidar_dis<min:
			min=lidar_dis
			temp_angle=i		
	

	
	if temp_angle>305 or temp_angle<45:
		print("front")
	elif temp_angle>45 and temp_angle<135:
		print("letf")
	elif temp_angle>135 and temp_angle<215:
		 print("behind")
	elif temp_angle>215 and temp_angle<305:
		print("right")

	#print('')

if __name__=='__main__':
	listner()

