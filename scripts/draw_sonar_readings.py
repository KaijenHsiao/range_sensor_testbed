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
import tf
import tf.transformations

def mat_to_pos_and_quat(mat):
    """
    converts a 4x4 scipy matrix to position and quaternion lists
    """
    quat = tf.transformations.quaternion_from_matrix(mat).tolist()
    pos = mat[0:3,3].T.tolist()[0]
    return (pos, quat)

arduino_port = '/dev/ttyACM0'

rospy.init_node('publish_sonar_readings_node')
sonar_pub = rospy.Publisher('sonars', Range)
tf_broadcaster = tf.TransformBroadcaster()

try:
    s = serial.Serial(arduino_port, baudrate=9600)
except Exception as e:
    rospy.logerr('Failed to open port %s' % arduino_port)
    rospy.logerr(str(e))
    sys.exit(-1)

min_ranges = {"sdm-io" : 0.01,
              "hc-sro4" : 0.04}
max_ranges = {"sdm-io" : 0.75,
              "hc-sro4" : 0.75}
angle_spreads = {"sdm-io": 15./180.*math.pi,
                 "hc-sro4": 40./180.*math.pi}

sonar_frame_mat = [[1.,0.,0.,0.31],
                   [0.,1.,0.,0.],
                   [0.,0.,1.,0.05],
                   [0.,0.,0.,1.]]
sonar_frame_pos, sonar_frame_quat = mat_to_pos_and_quat(scipy.matrix(sonar_frame_mat))

sonar_types = ["hc-sro4", "hc-sro4", "hc-sro4", "hc-sro4", "hc-sro4"]
sonar_mats = [[[1.,0.,0.,0.02],  #red
               [0.,1.,0.,0.0],
               [0.,0.,1.,0.04],
               [0.,0.,0.,1.]],

              [[1.,0.,0.,0.02],  #green
               [0.,1.,0.,-0.19],
               [0.,0.,1.,0.04],
               [0.,0.,0.,1.]],

              [[1.,0.,0.,0.02],  #blue
               [0.,1.,0.,0.19],
               [0.,0.,1.,0.04],
               [0.,0.,0.,1.]],

              [[0., 1., 0., -0.05], #yellow
               [-1., 0., 0., -0.25],
               [0., 0., 1., 0.04],
               [0.,0.,0.,1.]],

              [[0., -1., 0., -0.05], #cyan
               [1., 0., 0., 0.25],
               [0., 0., 1., 0.04],
               [0.,0.,0.,1.]]]

colors = [[1,0,0], [0,1,0], [0,0,1], [1,1,0], [0,1,1]]
sonar_mats = [scipy.matrix(x) for x in sonar_mats]



nlines = 0
while not rospy.is_shutdown():
    l =  s.readline()
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

    marker_trans = scipy.matrix([[0., 0., 1., sonar_range],
                               [1., 0., 0., 0.],
                               [0., 1., 0., 0.],
                               [0., 0., 0., 1.]])
    marker_mat = sonar_mats[sonar_id] * marker_trans

    range_msg = Range()
    range_msg.radiation_type = range_msg.ULTRASOUND
    range_msg.field_of_view = angle_spreads[sonar_type]
    range_msg.min_range = min_ranges[sonar_type]
    range_msg.max_range = max_ranges[sonar_type]
    range_msg.range = sonar_range
    
    range_msg.header.stamp = rospy.Time.now()
    range_msg.header.frame_id = "sonar" + str(sonar_id)

    sonar_pub.publish(range_msg)
    sonar_pos, sonar_quat = mat_to_pos_and_quat(sonar_mats[sonar_id])
    tf_broadcaster.sendTransform(sonar_frame_pos, sonar_frame_quat, rospy.Time.now(), "sonar_frame", "base_footprint" )
    tf_broadcaster.sendTransform(sonar_pos, sonar_quat, rospy.Time.now(), "sonar" + str(sonar_id), "sonar_frame" )
     

#    draw_funcs.draw_rviz_cylinder(marker_mat, sonar_range * math.tan(angle_spreads[sonar_type]/2.), 
#                                  0.025, frame = '/camera_link', 
#                                  ns = 'sonars', id = sonar_id, duration = 0.5, color = [0,1,1], 
#                                  opaque = 0.5)
