'''
    Created On: Feb 13, 2015
    Author: Nehchal Jindal
            Eric Huang

    Desc: Implementationo of Particle Filter
'''

import numpy as np
from numpy.linalg import inv

import random

from map import Map

def state_to_transform_2d(x):
    ct = np.cos(x[2]);
    st = np.sin(x[2]);
    return np.array([
            [ct, -st, x[0]],
            [st, ct, x[1]],
            [0., 0., 1.]])

def transform_to_state_2d(H):
    return np.array([H[0,2], H[1,2], np.math.atan2(H[1,0], H[0,0])])


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
        """Beam based sensor model"""
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
        self.prevPos = None
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
                        w = random.random()
                        #self.X.append(Particle(x, y, theta, 1))
                        self.X.append(Particle(x, y, theta, w))

        return            

    # private function
    def _update(self, prevPos_W, prevPos_Odom, pos_Odom):
        ''' Updates state of particles according to motion model 
        @param prevPos_W : 3-tuple (x, y, theta). Previous position of robot in
                           world frame
        @param pos_Odom : 3-tuple (x, y, theta). New position of robot in odom 
                          frame
        @param prevPos_Odom : 3-tuple (x, y, theta). New position of robot in 
                              odom frame '''

        # Rnew : frame attached on robot at new position
        # Rprev : frame attached on robot at old position
        # odom : odometery frame

        # Transforms
        T_Rnew2odom = state_to_transform_2d(pos_Odom)

        T_Rprev2odom = state_to_transform_2d(prevPos_Odom)

        T_Rnew2Rprev = np.dot(inv(T_Rprev2odom), T_Rnew2odom)
        
        T_Rnew2W = np.dot(state_to_transform_2d(prevPos_W), T_Rnew2Rprev)
        
        pos_W = transform_to_state_2d(T_Rnew2W)

        # Maybe we can add some noise
        pos_W[0] = pos_W[0] + np.random.normal(0, 1)
        pos_W[1] = pos_W[1] + np.random.normal(0, 1)
        pos_W[2] = pos_W[2] + np.random.normal(0, 0.01)

        return pos_W

    # resample
    def _resample(self, X):
        '''
        Sampling with replacement based on weights and generates new 
        particle set
        @param X : particle set
        returns resampled particle set
        '''
        X_new = []
        M = len(X)  # num of particle
        weights = np.zeros(M)   # the default data type is float

        totals = []
        running_total = 0

        for i in range(M):
            running_total += X[i].weight
            totals.append(running_total)

        # sample with replacement
        '''
        for i in range(M):
            rnd = random.random() * running_total
            for ind, total in enumerate(totals):
                if rnd < total:
                    X_new.append(X[ind])
        '''
        rnds = []
        for i in range(M):
            rnd = random.random() * running_total
            rnds.append(rnd)

        ptr1 = 0
        ptr2 = 0

        while ptr1 < M and ptr2 < M:
            if rnds[ptr1] <= totals[ptr2]:
                X_new.append(X[ptr1])
                ptr1 = ptr1 + 1
            else:
                ptr2 = ptr2 + 1

        return X_new

    def step(self, X_prev, u_t, z_t):
        '''
        This steps through particle filter at time t and generate new belief state.
        @param X_prev : List of Particle instances. Particle set at time t-1 .
        @param u_t : Control action at time t
        @param z_t : measurement at time t
        returns : List of Particle instances. Particle set at time t.
        '''

        X_temp = None  # new partile set
        M = len(self.X) # Number of particles

        '''
        For each particle 
            update the particle using transition model (motion model)
            specify weight of the particle using sensor model
            add this particle to new set. '''
        #for m in range(M):



        '''
        Generate new set of particles from above particle using sampling by 
        replacement. Each particle is chosen by probability proportional to 
        its importance weight. '''
        self.X = _resample(X_temp)

        return None


def testParticleFilter():
    print 'Testing the ParticleFilter class ...'

    # Read map
    map = Map()
    map.readMap('../data/map/wean.dat')

    # basic tests
    filter = ParticleFilter()
    filter.initParticles(map)
    filter._resample(filter.X)

    # test for update function
    pos = filter._update((5.0, 6.0, 0.0), (1, 2, 0.0), (2, 2, 0.0))
    print pos # this should print something close to (6, 6, 0)

    '''
    ind = weighted_choice([0.4, 0, 0])
    if ind != 0:
        print 'FAILED weighted_choice test.'

    ind = weighted_choice([0.4, .6, 0.2])
    print 'ind = ', ind
    '''

    return

if __name__ == '__main__':
    testParticleFilter()
