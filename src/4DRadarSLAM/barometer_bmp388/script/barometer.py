#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial # dependency: pip install pyserial
import string
import math
import sys

from barometer_bmp388.msg import Barometer
from sensor_msgs.msg import Temperature
from geometry_msgs.msg import Vector3Stamped


rospy.init_node("barometer_node")
baro_pub = rospy.Publisher('/barometer/raw', Barometer, queue_size=1)

baroMsg = Barometer()

port='/dev/mega2560'
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=5)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    sys.exit(0)

seq=0


while not rospy.is_shutdown():
    
    try:
        line = ((ser.readline()).rstrip()).decode("utf-8") # Type "bytes" to "string"
        words = line.split(",")    # Fields split
        if len(words) == 3:
            #rospy.loginfo("altitude: %f, pressure: %f, temperature: %f",float(words[0]),float(words[1]),float(words[2]))
            baroMsg.altitude = float(words[0])
            baroMsg.pressure = float(words[1])
            baroMsg.temperature = float(words[2])
            baroMsg.header.stamp= rospy.Time.now()
            baroMsg.header.frame_id = 'base_link'
            baroMsg.header.seq = seq
            seq = seq + 1
            baro_pub.publish(baroMsg)
    except:
        continue
        
ser.close


'''
from tf.transformations import quaternion_from_euler

DEG2RAD = math.pi/180.0
imu_yaw_calibration = 0.0
# x points forward, y points left, z points up. Angular velocity direction right roll/head down/left is positive
rospy.loginfo("Flushing first 1 IMU entries...")
for i in range(1):
    line = ser.readline()
    print "serial from imu: " + line
rospy.loginfo("Publishing IMU data...")
'''

