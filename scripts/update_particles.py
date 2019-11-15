#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import pickle
from detection_clustering import DetectionClustering
from geometry_msgs.msg import PoseArray, Pose
from yaml import load

#Initial points ------------------------------------
counter = 0
poses = []
file =  open('/home/ghost/Code/path.data', 'rb')
poses = pickle.load(file)
file.close()
new_poses = []
for i in range(0, len(poses), 5):
    new_poses.append(poses[i])
poses = new_poses.copy()
# -------------------------------------------------------------------


# Global Landmarks --------------------------------------------------
detected = {}
detection_names = rospy.get_param('/darknet_ros/yolo_model/detection_classes/names')
file = open('/home/ghost/.ros/detections_dbscan.db', 'r')
global_detection = load(file)
global_detected = {}
for name, pos in global_detection.items():
    for p in pos:
        if(name in global_detected):
            global_detected[name].append(p)
        else:
            global_detected[name] = p
#---------------------------------------------------------------------


# Particle distance form global landmarks ----------------------------
def dist_from_landmarks(cur_pos):
    part_id = {}
    for name, pos in global_detected:
        min_dist = 1000000
        for p in pos:
            dist = sqrt((p.x - cur_pos[0])^2 + (p.y - cur_pos[1]))
            min_dist = min(min_dist, dist)
        part_id[name] = min_dist
    return part_id
#----------------------------------------------------------------------


# Weight of each particle ---------------------------------------------
def dist_between_part(part1_id, part2_id):
    dist = 0
    for name, pos in global_detected:
        if(name in part1_id):
            dist = dist + (part1_id[name] - part2_id[name])^2
    dist = sqrt(dist)
    return dist
# ---------------------------------------------------------------------


# Resample particle respective to their weights -----------------------
def resample_particles(particles):
    do_stuff = 0
# ---------------------------------------------------------------------


# Particle filter main ------------------------------------------------
def compare(clusters, particles_poses):
    for i in range(len(particles_poses)):
        do_stuff = 0
# ---------------------------------------------------------------------


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
        dc = DetectionClustering(detected, min_samples=1)
        #print([x, y])
        #print(poses[1])

        
        for i in range(len(poses)):
            old_poses.append([poses[i][0] + x, poses[i][1] + y])
        
        
        display_particles(old_poses)
        counter = 0
    counter += 1

def update_key(key, val):
    global detected
    if(key in detected):
        detected[key].append(val)
    else:
        detected[key] = [val]

def collect(msg):
    global detection_names
    for i, pose in enumerate(msg.poses):
        if(pose != Pose()):
            pos = pose.position
            val = [pos.x, pos.y, pos.z]
            key = detection_names[i]
            update_key(key, val)

rospy.init_node('update_particles', anonymous=True)
map_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
rospy.Subscriber('/stereo/odom', Odometry, callback)
rospy.Subscriber('/cluster_decomposer/centroid_pose_array', PoseArray, collect)
rospy.spin()