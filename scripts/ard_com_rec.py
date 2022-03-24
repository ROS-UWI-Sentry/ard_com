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

global motor_data
global runn #key 0: startup, 1: ready to read, 2: ready to write
runn=0

                        ###note###
# ____________________________________________________
#|ROS node for communicating with the serial port     |
#|____________________________________________________|



##########VARIABLES##########


#callback function for subscriber
def callback(data):
    global runn, motor_data
    #expecting a string in format s0.xxxx,s0.xxxx
    #where s is the sign (positve + or negative -)
    #and x is the numerical value)
    
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

    motor_data = ':1,'+format_left+','+format_right


    #y=ser.readline()



       

#this function is for subscribing to messages
def listener():
    rospy.Subscriber('wheel_vel_vector', Num, callback)
    #sleep value
    rate = rospy.Rate(5)#In Hz
    global runn, motor_data 
    rospy.loginfo("Arduino Communication Node Ready")
    encoder_rps_l = 0.0
    encoder_rps_r = 0.0
    encoder_rps = [0,0]
    while not rospy.is_shutdown():
        #wait for a readcommand
        if runn==1:
            if ser.in_waiting:
            #if len(y)>5:
                #rospy.loginfo(ser.in_waiting)
                #rospy.loginfo("speed: "+ser.read_until()) #'\n' by default
                serial_data_in = ser.read_until()
		print("original serial data: " + str(serial_data_in)) 
		encoder_rps_r = float(serial_data_in.split(',')[1])
		print("encoder right rps: " + str(encoder_rps_r))
		encoder_rps_l = float(serial_data_in.split(',')[0])
		print("encoder left rps: " + str(encoder_rps_l))
		encoder_rps[0]=encoder_rps_l
 		encoder_rps[1]=encoder_rps_r
                pub_encoder.publish(encoder_rps)
		print(encoder_rps)
		#print("speed: "+ser.read_until())
                #rospy.loginfo(ser.in_waiting)
                #rospy.loginfo("serial write data: " + motor_data + '\n')
                print("serial write data: " + motor_data + '\n')
                ser.write(motor_data + '\n')   
                rate.sleep()

   


if __name__=='__main__':
    #create a unique node
    rospy.init_node('ard_com', anonymous=False)

    #create a publisher object and define which topic it subscribes to
    pub = rospy.Publisher('ard_com_out', String, queue_size=500)
    pub_encoder = rospy.Publisher('encoder_rps', Num, queue_size=500)

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
        motor_data = ':1,+0.0000,+0.0000'
        ser.write(motor_data + '\n')
        runn=1
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
