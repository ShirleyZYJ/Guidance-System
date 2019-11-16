#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import pickle
from detection_clustering import DetectionClustering
from geometry_msgs.msg import PoseArray, Pose
from yaml import load
from pfilter import ParticleFilter, gaussian_noise, squared_error, independent_sample
import numpy as np
import math

class customPose(object):
    def __init__(self):
        self.x = 0
        self.y = 0

        
#Initial points ------------------------------------
counter = 1
poses = []
file =  open('/home/ghost/Code/path.data', 'rb')
poses = pickle.load(file)
file.close()
new_poses = []
for i in range(0, len(poses), 5):
    new_poses.append(poses[i])
#poses = new_poses.copy()
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
            global_detected[name] = [p]
#---------------------------------------------------------------------


# Particle distance form global landmarks(person) ----------------------------
def dist_from_landmarks(cur_pos, landmarks):
    part_id = []
    for p in landmarks['person']:
        #print(p)
        #print(cur_pos)
        #print('--------------')
        dist = math.sqrt(math.pow((p[0] - cur_pos[0]), 2) + math.pow((p[1] - cur_pos[1]), 2))
        part_id.append(dist)
    return part_id
#----------------------------------------------------------------------


# Weight of each particle ---------------------------------------------
def get_weight(part1_id, part2_id):
    dist = 0
    part1_id = np.asarray(part1_id)
    wt = 0
    for i in range(len(part2_id)):
        wt += np.min(np.abs(np.subtract(part1_id, part2_id[i])))
    return wt
# ---------------------------------------------------------------------


# Resample particle respective to their weights -----------------------
def resample_particles(particles, weight):
    particles = np.asarray(particles)
    # print(sum(weight))
    # print(len(weight))
    # print(len(particles))
    # print('-------------------------------------------')
    new_particles = particles[np.random.choice(particles.shape[0], particles.shape[0], p=weight)]
    return new_particles
# ---------------------------------------------------------------------


# Particle filter main ------------------------------------------------
def compare(cam_pose, clusters, particles_poses):
    global global_detected
    weight = []
    print(clusters['person'])
    print('-------------------------------')
    part2_id = dist_from_landmarks(cam_pose, clusters)
    for i in range(len(particles_poses)):
        part1_id = dist_from_landmarks(particles_poses[i], global_detected)
        part_wt = get_weight(part1_id, part2_id)
        weight.append(part_wt)
    weight /= sum(weight)
    return resample_particles(particles_poses, weight)
# ---------------------------------------------------------------------


# Visualize particles in rviz ------------------------------------------
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

        marker.scale.z = 0.1
        marker.scale.y = 0.1
        marker.scale.x = 0.1
        markerArray.markers.append(marker)
    map_publisher.publish(markerArray)
# ---------------------------------------------------------------------


# Updating particles --------------------------------------------------
def callback(data):
    #print('checkpoint')
    global counter
    global poses
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    #print([x, y])
    if(counter % 10 == 0):
        old_poses = []
        dc = DetectionClustering(detected, min_samples=2)
        if('person' in dc.clusters):
            poses = compare([x, y], dc.clusters, poses)
        for i in range(len(poses)):
            old_poses.append([poses[i][0] + x, poses[i][1] + y])
        display_particles(old_poses)
        counter = 0
    counter += 1
# ---------------------------------------------------------------------


# Collecting objects --------------------------------------------------
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
# ------------------------------------------------------------------------


rospy.init_node('update_particles', anonymous=True)
map_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
rospy.Subscriber('/stereo/odom', Odometry, callback)
rospy.Subscriber('/cluster_decomposer/centroid_pose_array', PoseArray, collect)
rospy.spin()