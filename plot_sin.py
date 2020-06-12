#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
# these are libreray for ploting 
import matplotlib.pyplot as plt
x = 0
def walid(data):
    global x
    plt.plot(x , data,'*')
    plt.axis("equal")
    plt.draw()
    plt.pause(0.00000000001)
    plt.ion()
    plt.show()

    x += 1
    
   
def listener():
    
    rospy.init_node('plot_node', anonymous=True)
    

    rospy.Subscriber("sending", Float64 , walid)
    rospy.spin()
    print('i am getting data')

if __name__ == '__main__':
    listener()
