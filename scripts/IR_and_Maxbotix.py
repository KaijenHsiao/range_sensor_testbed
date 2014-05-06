#!/usr/bin/env python
import sys
import numpy as np
import rospy
from sensor_msgs.msg import Range
import time
import serial
import roslib
roslib.load_manifest('object_manipulator')
import object_manipulator.draw_functions as draw_functions
import scipy

# IR 1 (red; right from the sensor's pov) is a Sharp 2D120X (4-30 cm range) 
# IR 2 (magenta; left from the sensor's pov) is a Sharp 2Y0A21 (10-80 cm range)
# Sonar 1 (blue; right from the sensor's pov) is a MaxBotix HRLV-MaxSonar-EZ MB1043
# Sonar 2 (cyan; left from the sensor's pov) is a MaxBotix XL-MaxSonar-EZ MB1240

# from phidgets page on 2D120X
def ir1_value_to_range(v):
    if v < 80.2: #30 cm
        return np.inf
    return 20.76/(v-11)

# from phidgets page on 2Y0A21
def ir2_value_to_range(v):
    if v < 80.0: #80 cm
        return np.inf
    return 48./(v-20)

# HRLV: bits*5 = mm
def sonar1_value_to_range(v):
    if v == 0:
        return np.inf
    return v*5 / 1000.

# XL: 1 bit = 1 cm 
def sonar2_value_to_range(v):
    if v == 0:
        return np.inf
    return v / 100.

arduino_port = '/dev/ttyACM0'

rospy.init_node('publish_sensor_vals_node')
ir1_pub = rospy.Publisher('ir1', Range)
ir2_pub = rospy.Publisher('ir2', Range)
sonar1_pub = rospy.Publisher('sonar1', Range)
sonar2_pub = rospy.Publisher('sonar2', Range)
draw_funcs = draw_functions.DrawFunctions('range_sensors')

try:
    s = serial.Serial(arduino_port, baudrate=57600)
except Exception as e:
    rospy.logerr('Failed to open port %s' % arduino_port)
    rospy.logerr(str(e))
    sys.exit(-1)

nlines = 0
while not rospy.is_shutdown():
    l =  s.readline()
    nlines += 1

    # the first few lines we get form the board are often junk; ignore them
    if nlines < 20:
        continue

    print l

    try:
        vals = [float(tok) for tok in l.strip().split()]
    except Exception as e:
        rospy.logerr('Failed to parse line from arduino board: %s' % l.strip())
        rospy.logerr(e)
        continue

    if len(vals) != 4:
        rospy.logerr('Line from arduino board has wrong number of tokens: %s' % l.strip())
        continue

    # Sharp 2D120X (4-30 cm range)    
    ir1_range = Range()
    ir1_range.radiation_type = Range.INFRARED
    ir1_range.min_range = 0.04
    ir1_range.max_range = 0.30
    ir1_range.range = ir1_value_to_range(vals[0])

    # Sharp 2Y0A21 (10-80 cm range)    
    ir2_range = Range()
    ir2_range.radiation_type = Range.INFRARED
    ir2_range.min_range = 0.10
    ir2_range.max_range = 0.80
    ir2_range.range = ir2_value_to_range(vals[1]) 

    # HRLV-MaxSonar-EZ MB1043 reports anything below 30 cm as 30 cm
    # inaccurate readings closer than 50 cm
    sonar1_range = Range()
    sonar1_range.radiation_type = Range.ULTRASOUND
    sonar1_range.min_range = 0
    sonar1_range.max_range = 5.0
    sonar1_range.range = sonar1_value_to_range(vals[2])

    # XL-MaxSonar-EZ MB1240 reports anything below 20 cm as 20 cm
    # inaccurate readings closer than 50 cm
    sonar2_range = Range()
    sonar2_range.radiation_type = Range.ULTRASOUND
    sonar2_range.min_range = 0
    sonar2_range.max_range = 7.62  #25 ft
    sonar2_range.range = sonar2_value_to_range(vals[3])

    print "ir1: {0}\tir2: {1}\tsonar1: {2}\tsonar2: {3}".format(ir1_range.range, ir2_range.range, sonar1_range.range, sonar2_range.range)

    # draw the ranges
    if ir1_range.range < ir1_range.max_range:
        ir1_mat = scipy.matrix([[1.,0.,0.,0.05],
                                [0.,1.,0.,-0.035],
                                [0.,0.,1.,-0.065],
                                [0.,0.,0.,1.]])
        ir1_mat[0,3] += ir1_range.range
        draw_funcs.draw_rviz_box(ir1_mat, [.025, .1, .1], duration = 0.5,\
                                     frame = '/camera_link', \
                                     color = [1,0,0], id=1)

    if ir2_range.range < ir2_range.max_range:
        ir2_mat = scipy.matrix([[1.,0.,0.,0.05],
                                [0.,1.,0.,0.035],
                                [0.,0.,1.,-0.065],
                                [0.,0.,0.,1.]])
        ir2_mat[0,3] += ir2_range.range
        draw_funcs.draw_rviz_box(ir2_mat, [.025, .1, .1], duration = 0.5,\
                                     frame = '/camera_link', \
                                     color = [1,0,1], id=2)

    if sonar1_range.range < sonar1_range.max_range:
        sonar1_mat = scipy.matrix([[1.,0.,0.,0.05],
                                   [0.,1.,0.,-0.0425],
                                   [0.,0.,1.,-0.05],
                                   [0.,0.,0.,1.]])
        sonar1_mat[0,3] += sonar1_range.range
        draw_funcs.draw_rviz_box(sonar1_mat, [.025, .1, .1], duration = 0.5,\
                                     frame = '/camera_link', \
                                     color = [0,0,1], id=3)

    if sonar2_range.range < sonar2_range.max_range:
        sonar2_mat = scipy.matrix([[1.,0.,0.,0.05],
                                   [0.,1.,0.,0.0425],
                                   [0.,0.,1.,-0.05],
                                   [0.,0.,0.,1.]])
        sonar2_mat[0,3] += sonar2_range.range
        draw_funcs.draw_rviz_box(sonar2_mat, [.025, .1, .1], duration = 0.5,\
                                     frame = '/camera_link', \
                                     color = [0,1,1], id=4)

    #publish the Range messages
    ir1_pub.publish(ir1_range)
    ir2_pub.publish(ir2_range)
    sonar1_pub.publish(sonar1_range)
    sonar2_pub.publish(sonar2_range)

