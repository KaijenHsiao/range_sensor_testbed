#!/usr/bin/env python
import sys
import numpy as np
import rospy
from sensor_msgs.msg import Range
import time
import serial
import roslib
import scipy
import math
import signal
from serial import SerialException

def signal_handler(signal, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def connect_arduino():
    arduino_port = '/dev/ttyACM'
    port_num = 0
    arduino_serial = None
    while arduino_serial == None:
        port_name = arduino_port + str(port_num)
    try:
        arduino_serial = serial.Serial(port_name, baudrate=9600)
        arduino_serial.flushInput()
        rospy.loginfo("connected to arduino")
    except (KeyboardInterrupt, SystemExit):
        raise
    except Exception as e:
        port_num = port_num + 1
    if port_num > 10:
        rospy.logerr("Failed to open arduino on %s with error: %s"%(port_name, str(e)))
        time.sleep(1)
        rospy.loginfo("Trying again")
        port_num = 0
    return arduino_serial

rospy.init_node('publish_sonar_readings_node')
sonar_pub = rospy.Publisher('sonars', Range)
arduino_serial = connect_arduino()

min_ranges = {"sdm-io" : 0.01,
              "hc-sro4" : 0.04}
max_ranges = {"sdm-io" : 0.75,
              "hc-sro4" : 0.75}
angle_spreads = {"sdm-io": 15./180.*math.pi,
                 "hc-sro4": 40./180.*math.pi}

sonar_types = ["hc-sro4", "hc-sro4", "hc-sro4"]

nlines = 0
while not rospy.is_shutdown():
    try:
        l = arduino_serial.readline()
        failed_count = 0
    except (KeyboardInterrupt, SystemExit):
        raise
    except SerialException as e:
        rospy.logwarn("Serial exception: %s"%str(e))
        time.sleep(1)
        if failed_count > 10:
            rospy.logerr("Failed to read %d lines, trying to reconnect"%failed_count)
            try:
                arduino_serial = connect_arduino()
            except:
                rospy.logerr("Failed to reconect.")
                sys.exit(1)
        else:
            failed_count += 1
    except Exception as e:
        rospy.logwarn("caught exception while reading from arduino: %s"%str(e))
        time.sleep(1)

    nlines += 1

    # the first few lines we get form the board are often junk; ignore them
    if nlines < 20:
        continue

    strs = l.strip().split()
    if strs[1] == "timeout":
        continue

    try:
        sonar_id = int(strs[0])
        sonar_range = float(strs[1]) / 100.
    except Exception as e:
        rospy.logerr('Failed to parse line from arduino board: %s' % l.strip())
        rospy.logerr(e)
        continue

    sonar_type = sonar_types[sonar_id]
    print "id: {0},\ttype: {1},\trange: {2:0.2f}".format(sonar_id, sonar_type, sonar_range)

    if sonar_range < min_ranges[sonar_type]:
        print "lower than min range of", min_ranges[sonar_type] 
        continue
    if sonar_range > max_ranges[sonar_type]:
        print "higher than max range of", max_ranges[sonar_type] 
        continue

    range_msg = Range()
    range_msg.radiation_type = range_msg.ULTRASOUND
    range_msg.field_of_view = angle_spreads[sonar_type]
    range_msg.min_range = min_ranges[sonar_type]
    range_msg.max_range = max_ranges[sonar_type]
    range_msg.range = sonar_range
    
    range_msg.header.stamp = rospy.Time.now()
    range_msg.header.frame_id = "sonar" + str(sonar_id)

    sonar_pub.publish(range_msg)
     

