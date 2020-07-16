#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

def callback(data):
	for i in range (180, 185):
		lidar_dis = data.ranges[i]
		lidar_dis1 = data.ranges[180]
		print("%dth value" %i, lidar_dis)
	print("")

	if lidar_dis1 < 2.0:
		print("stop")



def distance():
    rospy.init_node('kunsan', anonymous=True)
    rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    distance()