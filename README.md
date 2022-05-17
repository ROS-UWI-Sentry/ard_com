# ard_com
used for serial communication between agx and arduino

This node receives strings from the browswer on the ard_com_in ROS topic and formats in so that the speed can be sent to the arduino through the serial port.

ard_com.py is a basic script
ard_com_rec.py sends speed to the Arduino and RECEIVES the current speed from the Arduino.
