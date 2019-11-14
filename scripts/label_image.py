#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

bridge = CvBridge()

def callback(data):
    img = bridge.imgmsg_to_cv2(data, "8UC1")
    #print(img[img != 0].shape)
    cv2.imshow('im', img)
    cv2.waitKey(1)

rospy.init_node('View_label_im', anonymous=True)
rospy.Subscriber('/darknet_ros/label_image', Image, callback)
rospy.spin()