#!/usr/bin/env python
#ROS license here

import time 
import serial


import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32

from twist_to_motor_rps.msg import Num

# Downsampling
global ii
ii=0
global runn #key 0: startup, 1: ready to read, 2: ready to write
runn=0
                        ###note###
# ____________________________________________________
#|ROS node for communicating with the serial port     |
#|____________________________________________________|



##########VARIABLES##########


#callback function for subscriber
def callback(data):
    global ii
    global runn
    #rospy.loginfo(data.num)

    #expecting a string in format s0.xxxx,s0.xxxx
    #where s is the sign (positve + or negative -)
    #and x is the numerical value)


#wait for a readcommand
    y=ser.readline()
    if runn==1:
        if len(y)>5:
            rospy.loginfo("speed: "+y)
            runn=2 #set we have read something go ahead to write.
    
            format_left = "{:.4f}".format(data.num[0])
            format_right = "{:.4f}".format(data.num[1])
    
            if format_left == "-0.0000":
                format_left="+0.0000"


            if data.num[0]==0:
               format_left="+"+format_left

            if data.num[0]>0:
               format_left="+"+format_left



            if format_right=="-0.0000":
                format_right="+0.0000"

            if data.num[1]==0:
                format_right="+"+format_right
    
            if data.num[1]>0:
                format_right="+"+format_right



            x = ':1,'+format_left+','+format_right
   
            #rospy.loginfo("serial write data: " + x)
        if runn==2:
            #ii=ii+1
            #if ii>10:
            rospy.loginfo("serial write data: " + x)
            ser.write(x)

            rate = rospy.Rate(3000)
            rate.sleep()
            runn=1

            #ii=0

        #rospy.loginfo("serial write data: " + x)
        #ser.write(x)
       

#this function is for subscribing to messages
def listener():
    rospy.Subscriber('wheel_vel_vector', Num, callback)
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
#on startup brake
    if runn==0:
        x = ':1,+0.0000,+0.0000'
        ser.write(x)
        runn=1

#wait for a readcommand
#    y=ser.readline()
#    if runn==1:
#        if len(y)>5:
#            rospy.loginfo("speed: "+y)
#            runn=2 #set we have read something go ahead to write.



#To ensure the wheels are not spinning at startup
    #x = ':1,+0.0000,+0.0000'
    #ser.write(x)
    
    #start the subscriber
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
