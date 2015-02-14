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
                if (map.grid[i][j] < 0.5):
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

