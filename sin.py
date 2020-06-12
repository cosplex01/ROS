#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import math
def talker():
    pub = rospy.Publisher('sending', Float64, queue_size=10)
    rospy.init_node('Math', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    x = 0
    while not rospy.is_shutdown():
        f_x = math.sin(x) * 10
        pub.publish(f_x)
        x = x+0.1
        print('i am sending data')
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
