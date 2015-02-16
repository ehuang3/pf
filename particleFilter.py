'''
    Created On: Feb 13, 2015
    Author: Nehchal Jindal
            Eric Huang

    Desc: Implementationo of Particle Filter
'''

import numpy as np
from numpy.linalg import inv
from visualization import *
import pylab as plt

from numpy import *

import copy

from scipy.stats import norm

#from joblib import Parallel, delayed

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

def update(i, pos_prev, pos_curr, z_t, X_update):
    x_m = self._update(self.X[m].state, pos_prev, pos_curr)
    w_m = self.sensor_model.computeProbability(z_t, x_m)
    X_update[i] = Particle(x_m[0], x_m[1], x_m[2], w_m)

def normpdf(x, mu, sigma):
    u = (x-mu)/abs(sigma)
    y = (1/(sqrt(2*pi)*abs(sigma)))*exp(-u*u/2)
    return y

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
    def __init__(self, z_map, z_hit, z_max, z_rand):
        """Beam based sensor model"""
        self.z_T = np.array([z_hit, z_max, z_rand])
        self.z_hit = z_hit
        self.z_max = z_max
        self.z_rand = z_rand
        self.initSensor(z_map)

        # Ray search 
        self.ray_sigma = 200 #20
        self.ray_pdf = np.zeros(self.ray_sigma * 4 + 10)
        for i in range(self.ray_pdf.size):
            self.ray_pdf[i] = norm.pdf(i, 0, self.ray_sigma)

        self.laserMap = np.zeros((800, 800, 16))

    def buildRayTraceLookupTable():
        angles = np.linspace(-np.pi, np.pi, 16)
        for x in range(800):
            for y in range(800):
                for angle in range(16):
                    self.rayTrace(x*10.0, y*10.0, 0)


    def initSensor(self, map):
        """Build forward sensor model based on map"""
        self.map = map

    def rayTrace(self, pos_world, angle):
        '''
        Returns the distance of nearest obstacle in the 
        given direction
        @param pos_world: [x y theta]. pos of laser in world frame.
        @param angle: angle in which direction to look in radians
        '''
        x = int(pos_world[0]/10)
        y = int(pos_world[1]/10)
        t = int(pos_world[2])   # theta

        for d in range(0, 300, 3):
            cell_x = int(x + d*cos(t + angle))
            cell_y = int(y + d*sin(t + angle))
            
            if cell_x > 799 or cell_x < 0 or cell_y > 799 or cell_y < 0:
                return d * 10.0

            if d > 300:
                return d*10.0

            # if cell is occupied
            if self.map.grid[cell_x, cell_y] < 0.2:
                return d*10.0

        return d * 10.0

    def raySearch(self, pos_world, angle, laser, sigma):
        """
        Returns the distance of the nearest obstable within bounds or the bound.
        @param pos_world: [x y theta]. pos of laser in world frame.
        @param angle: angle in which direction to look in radians

        """
        x = int(pos_world[0]/10)
        y = int(pos_world[1]/10)
        t = int(pos_world[2])   # theta

        d_max = sigma * 4

        dray = np.array([cos(t + angle), sin(t + angle)])

        for d in range(0, d_max, 10):
            for s in [-1, 1]:
                cell_x = int(x + (laser + s*d)*dray[0])
                cell_y = int(y + (laser + s*d)*dray[1])

                if cell_x > 799 or cell_x < 0:
                    return d_max
                if cell_y > 799 or cell_y < 0:
                    return d_max

                if self.map.grid[cell_x, cell_y] < 0.2:
                    return d

        return d_max

    # @profile
    def computeProbability(self, z_t, x_t):
        """Compute the probability of z_t at x_t
        @param: z_t : laser data including odometery
        @param: x_t : robot position in world frame
        """

        # Get the laser's x, y, and theta
        T_laser2odom = state_to_transform_2d(z_t[3:6])
        T_robot2odom = state_to_transform_2d(z_t[0:3])
        T_robot2world = state_to_transform_2d(x_t)
        T_laser2world = np.dot(T_robot2world, np.dot(inv(T_robot2odom), T_laser2odom))

        # Convert laser to world frame x y theta.
        l_w = transform_to_state_2d(T_laser2world)

        # Compute the laser's x, y, and theta in word coordinates.
        #x_w = l_w[0]
        #y_w = l_w[1]
        #t_w = l_w[2]

        # Build matrix of laser angles cosines and sines for projection.
        #t_s = np.linspace(-np.pi/2.0, np.pi/2.0, 180, True) # scan theta
        
        # select 8 values
        n = 8 # number of laser reading
        t_s = np.linspace(-np.pi/2.0, np.pi/2.0, n, True) # scan theta
        #cos_s = np.cos(t_w + t_s)
        #sin_s = np.sin(t_w + t_s)

        # Get laser measurements.
        #z = z_t[6:]
        ind = np.linspace(0, 179, n)
        z = [ z_t[int(i) + 6] for i in ind]
        #z = []
        #for i in ind:
        #    z.append(z_t[6 + int(i)])

        # Project measurements into world frame.
        # x_z = (x_w + np.multiply(z, cos_s))
         #y_z = (y_w + np.multiply(z, sin_s))
        #x_z = (x_w + np.multiply(z, cos_s)) / 10.0
        #y_z = (y_w + np.multiply(z, sin_s)) / 10.0

        # Get grid cell indicies. The map origin is located at bottom left.
        #x_z = x_z.astype('int32')
        #y_z = y_z.astype('int32', copy=False)

        # Clip coordinates to map size.
        cols = self.map.grid.shape[1]
        rows = self.map.grid.shape[0]
        #x_z = np.clip(x_z, 0, cols-1)
        #y_z = np.clip(y_z, 0, rows-1)

        # Get occupancy probabilities associated with each laser reading.
        #p_hit = 1 - self.map.grid[y_z, x_z]
        p_hit = []
        sigma = self.ray_sigma
        for i in range(n):
            actualReading = self.rayTrace(l_w, t_s[i])
            #p = norm.pdf(z[i], actualReading, 200)
            p = normpdf(z[i], actualReading, 500)
            #d = self.raySearch(l_w, t_s[i], z[i], sigma)
            #p = self.ray_pdf[d]
            #print 'p', p
            #print 'acutalReading', acutalReading
            #print 'measuredReading', z[i]
            p_hit.append(p)

        p_hit = np.array(p_hit)

        # Get error distribution.
        p_rand = 1 / self.z_rand * np.ones(180)
        p_rand[z > self.z_max] = 0

        # Get the max threshold distribution.
        p_max = np.ones(180)
        p_max[z < self.z_max] = 0

        # Compute the probability of z_t given x_t
        #p_z = self.z_hit * p_hit + self.z_rand * p_rand + self.z_max * p_max
        p_z = p_hit + 0.000000000000000000000000000000001 # avoid zero in the log

        weight = np.exp(np.sum(np.log(p_z)))

        #print 'weight', weight

        return weight

class ParticleFilter():
    ''' The particle filter '''
    def __init__(self):
        self.X = []  # set of particles
        self.prevPos = None
        self.sensor_model = []
        self.X_update = []
        return

    def setSensorModel(self, sensor_model):
        self.sensor_model = sensor_model

    def initParticles(self, map):
        ''' uniforly distributes the particles in free areas '''
        self.X = []
        self.X_update = []

        for x in np.linspace(0, 7999, 100):
            for y in np.linspace(0, 7999, 100):
                # add only if it lies in free area
                i = int(x/10.0)
                j = int(y/10.0)
                if (map.grid[i][j] > 0.8):
                    #
                    for theta in np.linspace(-np.pi, np.pi, 10):
                        w = random.random()
                        #self.X.append(Particle(x, y, theta, 1))
                        self.X.append(Particle(x, y, theta, w))

        self.X_update = copy.copy(self.X)

        return

    #@profile
    def run(self, map, z):
        '''
        map : instance of class map
        z : 2D array. Each element is reading at particular time instant
        '''
        # Initialize the particles uniformly.
        self.initParticles(map)

        vis = Visualization()
        vis.drawMap(map)


        # Outer loop.
        pos_prev = z[0, 0:3]
        for i in range(z.shape[0]):
            pos_curr = z[i, 0:3]
            z_t = z[i, 0:186]

            # clip the sensor readings
            z_t = np.clip(z_t, -3000, 3000)

            self.step(pos_prev, pos_curr, z_t)
            pos_prev = pos_curr
            vis.drawParticles(self.X)

            # find particle with max weight
            maxW = 0;
            maxInd = 0
            for ind, p in enumerate(self.X):
                if p.weight > maxW:
                    maxW = p.weight
                    maxInd = ind

            t_s = np.linspace(-np.pi/2.0, np.pi/2.0, 4, True) # scan theta
            ind = np.linspace(0, 179, 4)
            zTemp = [ z[i, int(each) + 6] for each in ind]

            vis.drawLaser(self.X[maxInd].state, zTemp, t_s)
            #vis.drawParticles([self.X[maxInd]])

            text = 'frame = ' + str(i) + ', t = ' + str(z[i, 186])
            vis.writeText(text)

            vis.saveImage()

            print i


    # private function
    def _update(self, prevPos_W, prevPos_Odom, pos_Odom):
        ''' Updates state of particles according to motion model 
        @param prevPos_W : 3-tuple (x, y, theta). Previous position of robot in
                           world frame
        @param prevPos_Odom : 3-tuple (x, y, theta). Previous position of robot
                           in odom frame
        @param pos_Odom : 3-tuple (x, y, theta). New position of robot in 
                              odom frame 
        returns new position (a numpy array of length 3)
        '''

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
        pos_W[0] = pos_W[0] + np.random.normal(0, 30)
        pos_W[1] = pos_W[1] + np.random.normal(0, 30)
        pos_W[2] = pos_W[2] + np.random.normal(0, 0.20)

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
        # print 'totals', totals

        
        # for i in range(M):
        #     rnd = random.random() * running_total
        #     for ind, total in enumerate(totals):
        #         if rnd < total:
        #             X_new.append(X[ind])
        #             # print ind
        #             break

        # Uniformly sample from the running total.
        i = 0
        for r in np.linspace(0, running_total, M/2, endpoint=False):
            while totals[i] <= r:
                i = i + 1
            if r < totals[i]:
                X_new.append(X[i])

        # add some particle to decrease convergence
        for i in range(M/2):
            X_new.append(X[2*i])


        '''
        for i in range(M):
            X_new.append(X[int(random.random()*M)])
        '''

        '''
        rnds = []
        for i in range(M):
            rnd = random.random() * running_total
            rnds.append(rnd)

        rnds.sort()
        ptr1 = 0
        ptr2 = 0

        print rnds

        while ptr1 < M and ptr2 < M:
            if rnds[ptr1] <= totals[ptr2]:
                print 'particle selected is ', ptr2
                X_new.append(X[ptr2])
                ptr1 = ptr1 + 1
            else:
                ptr2 = ptr2 + 1
        '''
        #print 'Num of particles = ', len(X_new)
        
        return X_new

    # @profile
    def step(self, pos_prev, pos_curr, z_t):
        '''
        This steps through particle filter at time t and generate new belief state.
        @param X_prev : List of Particle instances. Particle set at time t-1 .
        @param u_t : Control action at time t
        @param z_t : measurement at time t. (list of 180 values)
        returns : List of Particle instances. Particle set at time t.
        '''

        # if robot doesn't move, do nothing
        if abs(pos_prev[0] - pos_curr[0]) < 0.1:   # 1 mm
            return None
        if abs(pos_prev[1] - pos_curr[1]) < 0.01:  # 1 mm
            return None
        if abs(pos_prev[2] - pos_curr[2]) < 0.005:  # 0.25 deg
            return None

        X_temp = []  # new partile set
        M = len(self.X) # Number of particles

        '''
        For each particle
            update the particle using transition model (motion model)
            specify weight of the particle using sensor model
            add this particle to new set. '''
        w = np.zeros(M)
        for m in range(M):
            x_m = self._update(self.X[m].state, pos_prev, pos_curr)
            w_m = self.sensor_model.computeProbability(z_t, x_m)
            w[m] = w_m
            self.X_update[m] = Particle(x_m[0], x_m[1], x_m[2], w_m)

        wSum = sum(w)
        for m in range(M):
            self.X_update[m].weight = self.X_update[m].weight/wSum

        print 'Mum of particles', M
        #print w
        #plt.hist(w, 1000)
        # plt.hist(w, 100, normed=1, histtype='bar')
        #plt.show()

        '''
        Generate new set of particles from above particle using sampling by 
        replacement. Each particle is chosen by probability proportional to 
        its importance weight. '''
        self.X = self._resample(self.X_update)
        #self.X = self.X_update


        return None


def testParticleFilter():
    print 'Testing the ParticleFilter class ...'

    # Read map
    map = Map()
    map.readMap('../data/map/wean.dat')

    # basic tests
    filter = ParticleFilter()
    filter.initParticles(map)

    X = [Particle(0,0,0, 0.1), Particle(0,0,0, 0.1), Particle(0,0,0, 0.1)]
  
    print 'X after resampling', filter._resample(X)

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
