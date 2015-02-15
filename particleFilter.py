'''
    Created On: Feb 13, 2015
    Author: Nehchal Jindal
            Eric Huang

    Desc: Implementationo of Particle Filter
'''

import numpy as np
from numpy.linalg import inv

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
                        self.X.append(Particle(x, y, theta, 1))

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
            specify weight of the particle using sensor model
            add this particle to new set. '''
        #for m in range(M):


        '''
        Generate new set of particles from above particle using sampling by 
        replacement. Each particle is chosen by probability proportional to 
        its importance weight. '''
        return None


def testParticleFilter():
    print 'Testing the ParticleFilter class ...'

    # basic tests
    filter = ParticleFilter()

    # test for update function
    pos = filter._update((5.0, 6.0, 0.0), (1, 2, 0.0), (2, 2, 0.0))
    print pos # this should print something close to (6, 6, 0)

    return

if __name__ == '__main__':
    testParticleFilter()