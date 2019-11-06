#!/usr/bin/env python

import rospy
import argparse
from yaml import load
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray

#def cloud_to_2d(data):
  #  return 0 
    
    

rospy.init_node('map_markers', anonymous=True)
map_publisher = rospy.Publisher('map_doors', MarkerArray, queue_size=10)
    # type(object) = MarkerArray

markerArray = MarkerArray()    
marker = Marker()
marker.header.frame_id = "base_link"
marker.header.stamp = rospy.Time()

marker.ns = "visguide"
marker.type = Marker.SPHERE
marker.action = Marker.ADD
marker.id = 0

marker.text = "testing"
marker.pose.position.x = 1
marker.pose.position.y = 1
marker.pose.position.z = 1
marker.pose.orientation.w = 1

marker.color.r = 1.0
marker.color.g = 1.0
marker.color.b = 0.0
marker.color.a = 1.0

marker.scale.z = 1
marker.scale.y = 1
marker.scale.x = 1


markerArray.markers.append(marker)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    map_publisher.publish(markerArray)
    rate.sleep()
#rospy.subscriber('/visguide/zed_node/seg/image_rect_color', Image, cloud_to_2d)
#rospy.spin()