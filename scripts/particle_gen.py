#!/usr/bin/python3

import rospy
import argparse
from yaml import load
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import sys
import pickle

poses = []
file =  open('/home/ghost/Code/path.data', 'rb')
poses = pickle.load(file)
file.close()

#sys.exit()
index = 0

new_poses = []
for i in range(0, len(poses), 5):
    new_poses.append(poses[i])
poses = new_poses

rospy.init_node('map_markers', anonymous=True)
map_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    # type(object) = MarkerArray
print(len(poses))
markerArray = MarkerArray()
for i in range(len(poses)):    
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time()

    marker.ns = "visguide"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.id = i

    marker.text = "testing"
    marker.pose.position.x = poses[i][0]
    marker.pose.position.y = poses[i][1]
    marker.pose.position.z = 1
    marker.pose.orientation.w = 1

    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.scale.z = 0.2
    marker.scale.y = 0.2
    marker.scale.x = 0.2
    markerArray.markers.append(marker)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    map_publisher.publish(markerArray)
    rate.sleep()