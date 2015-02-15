'''
    Created On: Feb 13, 2015
    Author: Nehchal Jindal
            Eric Huang

    Desc: Implementationo of Particle Filter
'''

import numpy as np

class Particle(object):
    ''' Particle in the particle filter '''
    def __init__(self, x, y, theta, weight):
        self.state = [x, y, theta]
        self.weight = weight    # importance weight

class Pos():
    ''' Posiition of the robot '''
    def __init__(x, y, theta):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)

class laserReading():
    ''' The measument data at particular time instant '''
    def __init__():
        # ith value is distance of object from robot at angle i degrees
        # in cms. The values are ordered in counter-clockwise direction
        self.reading = np.zeros(180)
        return None

class SensorModel():
    def __init__(self, z_map, z_hit, z_short, z_max, z_rand):
        """Beam based sensor model"""f
        self.z_T = np.array([z_hit, z_short, z_max, z_rand])
        self.initSensor(z_map)

    def initSensor(map):
        """Build forward sensor model based on map"""
        self.map = map

    def computeProbability(z_t, x_t):
        """Compute the probability of z_t at x_t"""

        # Get the robot's x, y, and theta in world coordinates.
        x_r = x_t[0]
        y_r = x_t[1]
        t_r = x_t[2]

        # Get the laser's x, y, and theta relative to odometry coordinates.
        dx_l = z_t[3] - z_t[0]
        dy_l = z_t[4] - z_t[1]
        dt_l = z_t[5] - z_t[2]

        # Compute the laser's x, y, and theta in word coordinates.
        x_w = x_r + dx_l
        y_w = y_r + dy_l
        t_w = t_r + dt_l

        # Build matrix of laser angles cosines and sines for projection.
        t_s = np.linspace(0, np.pi, 180, true) # scan theta
        cos_s = np.cos(t_w + t_s)
        sin_s = np.sin(t_w + t_s)

        # Get laser measurements.
        z = z_t[6:]

        # Project measurements into world frame.
        x_z = x_w + np.multiply(z, cos_s)
        y_z = y_w + np.multiply(z, cos_s)

        # Get grid cell indicies. The map origin is located at bottom left.
        x_z = x_z.astype('int32', copy=False)
        y_z = y_z.astype('int32', copy=False)

        # Clip coordinates to map size.
        cols = self.map.shape[1]
        rows = self.map.shape[0]
        np.clip(x_z, 0, cols)
        np.clip(y_z, 0, rows)

        # Get occupancy probabilities associated with each laser reading.
        p_hit = self.map[np.array([y_z, x_z])]

        # Get error distribution.
        p_rand = 1 / z_rand * np.ones(180)
        p_rand[z > z_max] = 0

        # Get the max threshold distribution.
        p_max = np.ones(180)
        p_max[z < z_max] = 0

        # Compute the probability of z_t given x_t
        p_z = z_hit * p_hit + z_rand * p_rand + (1 - p_hit - p_rand) * p_max

        return p_z


class ParticleFilter():
    ''' The particle filter '''
    def __init__(self):
        self.X = []  # set of particles
        return

    def initParticles(self, map):
        ''' uniforly distributes the particles in free areas '''

        for x in np.linspace(0, 7999, 100):
            for y in np.linspace(0, 7999, 100):
                # add only if it lies in free area
                i = int(x/10.0)
                j = int(y/10.0)
                if (map.grid[i][j] > 0.8):
                    for theta in np.linspace(-np.pi, np.pi, 10):
                        self.X.append(Particle(x, y, theta, 1))

        return            

    def update(self, posPrev, odom, odomPrev):
        ''' Returns updated state according to motion model '''
        return

    def step(self, X_prev, u_t, z_t):
        '''
        This steps through particle filter at time t and generate new belief state.
        @param X_prev : List of Particle instances. Particle set at time t-1 .
        @param u_t : Control action at time t
        @param z_t : measurement at time t
        returns : List of Particle instances. Particle set at time t.
        '''

        X_t = None  # new partile set
        M = X_prev.size # Number of particles

        '''
        For each particle 
            update the particle using transition model (motion model)
            specify weight of the particle
            add this particle to new set. '''
        #for m in range(M):


        '''
        Generate new set of particles from above particle using sampling by 
        replacement. Each particle is chosen by probability proportional to 
        its importance weight. '''
        return None

