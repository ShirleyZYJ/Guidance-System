#!/usr/bin/python3

import numpy as np 
import turtle
import argparse
import time
import rospy
import math

from maze import Maze, Particle, WeightedDistribution, weight_gaussian_kernel
from nav_msgs.msg import Odometry
from detection_clustering import DetectionClustering
from geometry_msgs.msg import PoseArray, Pose


class Particle_Filter(object):
    def __init__(self, window_width, window_height, num_particles, sensor_limit_ratio, grid_height, grid_width, num_rows, num_cols, wall_prob, random_seed, robot_speed, kernel_sigma, particle_show_frequency):
        self.sensor_limit = sensor_limit_ratio * max(grid_height * num_rows, grid_width * num_cols)
        self.window = turtle.Screen()
        self.window.setup(width = window_width, height = window_height)
        self.world = Maze(grid_height = grid_height, grid_width = grid_width, num_rows = num_rows, num_cols = num_cols, wall_prob = wall_prob, random_seed = random_seed)
        self.particle_show_frequency = particle_show_frequency
        self.prev_x = 0
        self.prev_y = 0
        self.live_x = 0
        self.live_y = 0
        self.counter = 0
        self.tracker = 0
        self.dc = None
        self.particles = list()

        # Initilizing particles randomly -------------------------------------------------------------------------------------
        for i in range(num_particles):
            x = np.random.uniform(0, self.world.width)
            y = np.random.uniform(0, self.world.height)
            if(not(x >= 19 and x <= 241 and y >= 29 and y <= 301)):
                if(not(x <= 1 or y <= 1 or x >= 268 or y >= 328)):
                    self.particles.append(Particle(x = x, y = y, maze = self.world, sensor_limit = self.sensor_limit))

        time.sleep(1)
        self.world.show_maze()
        

        # Initializing pretrained objects -------------------------------------------------------------------------------------
        
        self.detected = {}
        self.detection_names = rospy.get_param('/darknet_ros/yolo_model/detection_classes/names')
        rospy.init_node('update_particles', anonymous=True)
        
        readings_robot = []

        # Moving particles ----------------------------------------------------------------------------------------------------

        while(1):
            #print(self.tracker)
            dx = self.live_x - self.prev_x
            dy = self.live_y - self.prev_y
            
            # Particle filtering ------------------------------------------------------------------------------------------------------------------------------------------------
            
            self.dc = DetectionClustering(self.detected.copy())
            if('person' in self.dc.clusters):
                print(self.dc.clusters['person'])
                readings_robot = self.zed_sensor_reading()
                #print('hew')
                #print(readings_robot)
                particle_weight_total = 0
                for particle in self.particles:
                    readings_particle = particle.read_sensor(maze=self.world)
                    particle.weight = weight_gaussian_kernel(x1 = readings_robot, x2 = readings_particle)
                    particle_weight_total += particle.weight
                
                if particle_weight_total == 0:
                    particle_weight_total = 1e-8

                for particle in self.particles:
                    particle.weight /= particle_weight_total
                
                distribution = WeightedDistribution(particles = self.particles)
                particles_new = list()
                
                for i in range(num_particles):

                    particle = distribution.random_select()

                    if particle is None:
                        x = np.random.uniform(0, self.world.width)
                        y = np.random.uniform(0, self.world.height)
                        while((x >= 19 and x <= 241 and y >= 29 and y <= 301) or (x <= 1 or y <= 1 or x >= 268 or y >= 328)):
                            x = np.random.uniform(0, self.world.width)
                            y = np.random.uniform(0, self.world.height)
                        particles_new.append(Particle(x = x, y = y, maze = self.world, sensor_limit = self.sensor_limit))

                    else:
                        particles_new.append(Particle(x = particle.x, y = particle.y, maze = self.world, heading = particle.heading, sensor_limit = self.sensor_limit, noisy = True))
                
                self.particles = particles_new

            # END ----------------------------------------------------------------------------------------------------------------------------------------------------------------

            for particle in self.particles:
                particle.try_move(maze=self.world, dx=dx*10, dy=dy*10)

            self.prev_x = self.live_x
            self.prev_y = self.live_y
            self.world.show_particles(particles = self.particles, show_frequency = self.particle_show_frequency)
            rospy.Subscriber('/stereo/odom', Odometry, self.callback)
            rospy.Subscriber('/cluster_decomposer/centroid_pose_array', PoseArray, self.collect)
            #self.world.show_estimated_location(particles = self.particles)
            self.world.clear_objects()
            #x=0

    def callback(self, data):
        #self.tracker += 1
        self.live_x = data.pose.pose.position.x
        self.live_y = data.pose.pose.position.y

    # Collecting objects --------------------------------------------------
    def update_key(self, key, val):
        if(key in self.detected):
            self.detected[key].append(val)
        else:
            self.detected[key] = [val]

    def collect(self, msg):
        for i, pose in enumerate(msg.poses):
            if(pose != Pose()):
                pos = pose.position
                val = [pos.x, pos.y, pos.z]
                key = self.detection_names[i]
                self.update_key(key, val)
    
    def zed_sensor_reading(self):
        readings_robot = []
        for p in self.dc.clusters['person']:
            dist = math.sqrt(math.pow((p[0] - self.live_x), 2) + math.pow((p[1] - self.live_y), 2))
            readings_robot.append(dist)
        return readings_robot
    # ------------------------------------------------------------------------

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description = 'Particle filter in maze.')

    window_width = 500
    window_height = 500
    num_particles = 3000
    sensor_limit_ratio = 0.3
    grid_height = 10
    grid_width = 10
    num_rows = 33
    num_cols = 27
    wall_prob = 0.25
    random_seed = 100
    robot_speed = 10
    kernel_sigma = 500
    particle_show_frequency = 1

    Particle_Filter(window_width = window_width, window_height = window_height, num_particles = num_particles, sensor_limit_ratio = sensor_limit_ratio, grid_height = grid_height, grid_width = grid_width, num_rows = num_rows, num_cols = num_cols, wall_prob = wall_prob, random_seed = random_seed, robot_speed = robot_speed, kernel_sigma = kernel_sigma, particle_show_frequency = particle_show_frequency)