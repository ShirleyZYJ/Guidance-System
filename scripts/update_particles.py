#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import pickle

counter = 0
poses = []
file =  open('/home/ghost/Code/path.data', 'rb')
poses = pickle.load(file)
file.close()
new_poses = []
for i in range(0, len(poses), 5):
    new_poses.append(poses[i])
poses = new_poses.copy()

def display_particles(poses_pub):
    markerArray = MarkerArray()
    #print(poses_pub[1])
    #print('-------')
    for i in range(len(poses_pub)):    
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()

        marker.ns = "turtle_bot"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.id = i

        marker.text = "testing"
        marker.pose.position.x = poses_pub[i][0]
        marker.pose.position.y = poses_pub[i][1]
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
    map_publisher.publish(markerArray)

def callback(data):
    #print('checkpoint')
    global counter
    global poses
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    #print([x, y])
    if(counter % 5 == 0):
        old_poses = []
        #print([x, y])
        #print(poses[1])
        for i in range(len(poses)):
            old_poses.append([poses[i][0] + x, poses[i][1] + y])
        display_particles(old_poses)
    counter += 1

rospy.init_node('update_particles', anonymous=True)
rospy.Subscriber('/stereo/odom', Odometry, callback)
map_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
rospy.spin()