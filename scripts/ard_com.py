#!/usr/bin/env python
#ROS license here

import time 
import serial


import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32



                        ###note###
# ____________________________________________________
#|ROS node for communicating with the serial port     |
#|____________________________________________________|



##########VARIABLES##########


#callback function for subscriber
def callback(data):
    rospy.loginfo("received data: " + data.data)

    #expecting a string in format s0.xxxx,s0.xxxx
    #where s is the sign (positve + or negative -)
    #and x is the numerical value)
    temp = data.data
    format_left = temp[0:7] #"{:.4f}".format(temp[0:7])
    format_right = temp[8:15] #"{:.4f}".format(temp[8:15])
    if format_left == "-0.0000":
        format_left="+0.0000"

    if format_right=="-0.0000":
        format_right="+0.0000"


    x = ':1,'+format_left+','+format_right
    rospy.loginfo("serial write data: " + x)
    ser.write(x)

#this function is for subscribing to messages
def listener():
    rospy.Subscriber('ard_com_in', String, callback)
    #sleep value
    rate = rospy.Rate(100)#100Hz
    rospy.loginfo("Node Alive")
    #rospy.loginfo("inside listener")
    #spin() keeps python from exiting
    rospy.spin()
    


if __name__=='__main__':
    #create a unique node
    rospy.init_node('ard_com', anonymous=False)

    #create a publisher object and define which topic it subscribes to
    pub = rospy.Publisher('ard_com_out', String, queue_size=500)

    #setup serial object
    ser = serial.Serial(
        port = '/dev/ttyTHS0', 
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.1
    )

    #start the subscriber
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
