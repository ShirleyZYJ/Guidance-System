#!/usr/bin/python3

import numpy as np 
import turtle
import bisect
import argparse
import random
import math

class Maze(object):

    def __init__(self, grid_height, grid_width, maze = None, num_rows = None, num_cols = None, wall_prob = None, random_seed = None):
        self.grid_height = grid_height
        self.grid_width = grid_width

        self.random_maze(num_rows = num_rows, num_cols = num_cols, wall_prob = wall_prob, random_seed = random_seed)

        self.height = self.num_rows * self.grid_height
        self.width = self.num_cols * self.grid_width

        self.turtle_registration()

    def turtle_registration(self):
        turtle.register_shape('tri', ((-3, -2), (0, 3), (3, -2), (0, 0)))

    def random_maze(self, num_rows, num_cols, wall_prob, random_seed = None):

        self.num_rows = 33
        self.num_cols = 27
        self.maze = np.zeros((self.num_rows, self.num_cols), dtype = np.int8)
        #print(self.num_cols)
        # Outer boundary ------------------------------------------

        # left
        for i in range(33):
            self.maze[i, 0] |= 8
        # up
        for i in range(27):
            self.maze[0, i] |= 1
        # right
        for i in range(33):
            self.maze[i, 26] |= 2
        # down
        for i in range(27):
            self.maze[32, i] |= 4

        # Inner boundary ------------------------------------------

        # left
        for i in range(3, 30):
            self.maze[i, 2] |= 2
            self.maze[i, 3] |= 8
        # up
        for i in range(3, 24):
            self.maze[2, i] |= 4
            self.maze[3, i] |= 1
        # right
        for i in range(3, 30):
            self.maze[i, 24] |= 8
            self.maze[i, 23] |= 2
        # down
        for i in range(3, 24):
            self.maze[30, i] |= 1
            self.maze[29, i] |= 4

    def permissibilities(self, cell):
        '''
        Check if the directions of a given cell are permissible.
        Return:
        (up, right, down, left)
        '''
        cell_value = self.maze[cell[0], cell[1]]
        return (cell_value & 1 == 0, cell_value & 2 == 0, cell_value & 4 == 0, cell_value & 8 == 0)

    def distance_to_walls(self, coordinates):
        '''
        Measure the distance of coordinates to nearest walls at four directions.
        Return:
        (up, right, down, left)
        '''

        x, y = coordinates

        i = int(y // self.grid_height)
        j = int(x // self.grid_width)
        d1 = y - y // self.grid_height * self.grid_height
        while self.permissibilities(cell = (i,j))[0]:
            i -= 1
            d1 += self.grid_height

        i = int(y // self.grid_height)
        j = int(x // self.grid_width)
        d2 = self.grid_width - (x - x // self.grid_width * self.grid_width)
        while self.permissibilities(cell = (i,j))[1]:
            j += 1
            d2 += self.grid_width

        i = int(y // self.grid_height)
        j = int(x // self.grid_width)
        d3 = self.grid_height - (y - y // self.grid_height * self.grid_height)
        while self.permissibilities(cell = (i,j))[2]:
            i += 1
            d3 += self.grid_height

        i = int(y // self.grid_height)
        j = int(x // self.grid_width)
        d4 = x - x // self.grid_width * self.grid_width
        while self.permissibilities(cell = (i,j))[3]:
            j -= 1
            d4 += self.grid_width

        return [d1, d2, d3, d4]

    def show_maze(self):

        turtle.setworldcoordinates(0, 0, self.width, self.height)

        wally = turtle.Turtle()
        wally.speed(0)
        wally.width(1.5)
        wally.hideturtle()
        turtle.tracer(0, 0)
        #print('heer')
        for i in range(self.num_rows):
            for j in range(self.num_cols):
                permissibilities = self.permissibilities(cell = (i,j))
                turtle.up()
                wally.setposition((j * self.grid_width, i * self.grid_height))
                # Set turtle heading orientation
                # 0 - east, 90 - north, 180 - west, 270 - south
                wally.setheading(0)
                if not permissibilities[0]:
                    wally.down()
                else:
                    wally.up()
                wally.pencolor('black')
                if(i == 3 and (j == 21 or j == 20)):
                    wally.pencolor('yellow')
                wally.forward(self.grid_width)
                
                wally.setheading(90)
                wally.up()
                if not permissibilities[1]:
                    wally.down()
                else:
                    wally.up()
                wally.pencolor('black')
                if((i == 11 or i == 12)  and j == 26):
                    wally.pencolor('yellow')

                wally.forward(self.grid_height)
                
                wally.setheading(180)
                wally.up()
                if not permissibilities[2]:
                    wally.down()
                else:
                    wally.up()
                wally.pencolor('black')
                if(i == 32 and (j == 25 or j == 26)):
                    wally.pencolor('yellow')

                if(i == 2 and (j == 21 or j == 20)):
                    wally.pencolor('yellow')

                wally.forward(self.grid_width)
                
                wally.setheading(270)
                wally.up()
                if not permissibilities[3]:
                    wally.down()
                else:
                    wally.up()
                wally.pencolor('black')
                if((i == 25 or i == 26) and j == 0):
                    wally.pencolor('yellow')

                wally.forward(self.grid_height)
                
                wally.up()

        turtle.update()


    def weight_to_color(self, weight):

        return '#%02x00%02x' % (int(weight * 255), int((1 - weight) * 255))


    def show_particles(self, particles, show_frequency = 1):

        turtle.shape('tri')

        for i, particle in enumerate(particles):
            if i % show_frequency == 0:
                turtle.setposition((particle.x, particle.y))
                turtle.setheading(90 - particle.heading)
                turtle.color(self.weight_to_color(particle.weight))
                turtle.stamp()
        
        turtle.update()

    def show_estimated_location(self, particles):
        '''
        Show average weighted mean location of the particles.
        '''

        x_accum = 0
        y_accum = 0
        heading_accum = 0
        weight_accum = 0

        num_particles = len(particles)

        for particle in particles:

            weight_accum += particle.weight
            x_accum += particle.x * particle.weight
            y_accum += particle.y * particle.weight
            heading_accum += particle.heading * particle.weight

        if weight_accum == 0:

            return False

        x_estimate = x_accum / weight_accum
        y_estimate = y_accum / weight_accum
        heading_estimate = heading_accum / weight_accum

        turtle.color('orange')
        turtle.setposition(x_estimate, y_estimate)
        turtle.setheading(90 - heading_estimate)
        turtle.shape('turtle')
        turtle.stamp()
        turtle.update()

    def clear_objects(self):
        turtle.clearstamps()


class Particle(object):

    def __init__(self, x, y, maze, heading = None, weight = 1.0, sensor_limit = None, noisy = False):

        if heading is None:
            heading = np.random.uniform(0,360)

        self.x = x
        self.y = y
        self.heading = heading
        self.weight = weight
        self.maze = maze
        self.sensor_limit = sensor_limit

        if noisy:
            std = max(self.maze.grid_height, self.maze.grid_width) * 0.2
            self.x = self.add_noise(x = self.x, std = std)
            self.y = self.add_noise(x = self.y, std = std)
            self.heading = self.add_noise(x = self.heading, std = 360 * 0.05)

        self.fix_invalid_particles()


    def fix_invalid_particles(self):

        # Fix invalid particles
        if self.x < 0:
            self.x = 0
        if self.x > self.maze.width:
            self.x = self.maze.width * 0.9999
        if self.y < 0:
            self.y = 0
        if self.y > self.maze.height:
            self.y = self.maze.height * 0.9999
        self.heading = self.heading % 360

    @property
    def state(self):

        return (self.x, self.y, self.heading)

    def add_noise(self, x, std):

        return x + np.random.normal(0, std)

    def read_sensor(self, maze):
        readings = []
        d1 = math.sqrt(math.pow((self.x - 210), 2) + math.pow((self.y - 30), 2))
        d2 = math.sqrt(math.pow((self.x - 260), 2) + math.pow((self.y - 120), 2))
        d3 = math.sqrt(math.pow((self.x - 250), 2) + math.pow((self.y - 320), 2))
        d4 = math.sqrt(math.pow((self.x - 0), 2) + math.pow((self.y - 250), 2))
        readings.append(d1)
        readings.append(d2)
        readings.append(d3)
        readings.append(d4)
        return readings

    def try_move(self, maze, dx, dy, noisy = False):

        heading = self.heading
        heading_rad = np.radians(heading)

        x = self.x + dx
        y = self.y + dy

        while((x >= 19 and x <= 241 and y >= 29 and y <= 301) or (x <= 1 or y <= 1 or x >= 268 or y >= 328)):
            x = np.random.uniform(0, maze.width)
            y = np.random.uniform(0, maze.height)
        self.x = x
        self.y = y


class Robot(Particle):

    def __init__(self, x, y, maze, heading = None, speed = 1.0, sensor_limit = None, noisy = True):

        super(Robot, self).__init__(x = x, y = y, maze = maze, heading = heading, sensor_limit = sensor_limit, noisy = noisy)
        self.step_count = 0
        self.noisy = noisy
        self.time_step = 0
        self.speed = speed

    def choose_random_direction(self):

        self.heading = np.random.uniform(0, 360)

    def add_sensor_noise(self, x, z = 0.05):

        readings = list(x)

        for i in range(len(readings)):
            std = readings[i] * z / 2
            readings[i] = readings[i] + np.random.normal(0, std)

        return readings

    def read_sensor(self, maze):

        # Robot has error in reading the sensor while particles do not.

        readings = super(Robot, self).read_sensor(maze = maze)
        if self.noisy == True:
            readings = self.add_sensor_noise(x = readings)

        return readings

    def move(self, maze):

        while True:
            self.time_step += 1
            if self.try_move(speed = self.speed, maze = maze, noisy = False):
                break
            self.choose_random_direction()


class WeightedDistribution(object):

    def __init__(self, particles):
        
        accum = 0.0
        self.particles = particles
        self.distribution = list()
        for particle in self.particles:
            accum += particle.weight
            self.distribution.append(accum)

    def random_select(self):

        try:
            return self.particles[bisect.bisect_left(self.distribution, np.random.uniform(0, 1))]
        except IndexError:
            # When all particles have weights zero
            return None

def euclidean_distance(x1, x2):

    return np.linalg.norm(np.asarray(x1) - np.asarray(x2))

def weight_gaussian_kernel(x1, x2):
    #print('haw')
    #print(x1)
    dist = 0
    x2 = np.asarray(x2)
    wt = 0
    for i in range(len(x1)):
        wt += np.min(np.abs(np.subtract(x2, x1[i])))
    return wt







