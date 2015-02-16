#!/usr/bin/env python
import argparse
import matplotlib
import numpy as np
import scipy.stats
import pylab as plt
from map import Map

class LikelihoodField:
    def __init__(self, grid, thresh, sigma, subsample):
        self.field = None
        self.grid = grid
        self.thresh = thresh
        self.sigma = sigma
        self.subsample = subsample

    def computeField(self):
        print "Computing likelihood field..."

        grid = self.grid
        thresh = self.thresh
        sigma = self.sigma
        subsample = self.subsample

        x_n = grid.shape[1]
        y_n = grid.shape[0]

        if self.field == None:
            self.field = np.zeros([y_n, x_n])

        field = self.field

        # Compute the size of the square to apply the gaussian over.
        s_n = np.rint(sigma * 3 / 10)
        if s_n % 2 == 0:
            s_n = s_n + 1
        s_n = int(s_n)

        # Precompute the gaussian square
        G = np.zeros([s_n, s_n])
        s_m = (s_n - 1) / 2
        for s_y in range(s_n):
            for s_x in range(s_n):
                d = np.linalg.norm(np.array([s_y - s_m, s_x - s_m])) * 10 # cm
                p = scipy.stats.norm.pdf(d, 0, sigma)
                G[s_y, s_x] = p

        # print G
        # print grid
        # print sum(sum(grid < 0.01))
        # print sum(sum(grid > -0.5))

        for y in range(y_n):
            for x in range(x_n):
                if grid[y, x] > thresh or grid[y, x] < -0.5:
                    continue
                for s_y in range(s_n):
                    for s_x in range(s_n):
                        g_y = np.clip(y + s_y - s_m, 0, y_n-1)
                        g_x = np.clip(x + s_x - s_m, 0, x_n-1)
                        field[g_y, g_x] = np.max([field[g_y, g_x], G[s_y, s_x]])
                        # print G[s_y, s_x]
                        # print field[g_y, g_x]

        # print sum(sum(field > 0.1))
        # print field

        print "Finished computing likelihood field."

    def plotField(self):
        f_max = np.max(np.max(self.field))
        plt.imshow(self.field, norm=matplotlib.colors.Normalize(0, f_max))

    def saveField(self, fpath):
        np.savetxt(fpath, self.field)

    def loadField(self, fpath):
        self.field = np.loadtxt(fpath)

    def computeProbability(self, x_t, z_t):
                # Get the laser's x, y, and theta
        T_laser2odom = state_to_transform_2d(z_t[3:6])
        T_robot2odom = state_to_transform_2d(z_t[0:3])
        T_robot2world = state_to_transform_2d(x_t)
        T_laser2world = np.dot(T_robot2world, np.dot(inv(T_robot2odom), T_laser2odom))

        # Convert laser to world frame x y theta.
        l_w = transform_to_state_2d(T_laser2world)

        # Compute the laser's x, y, and theta in word coordinates.
        x_w = l_w[0]
        y_w = l_w[1]
        t_w = l_w[2]

        # The indicies we should subsample at.
        t_sub = np.linspace(0, 180, self.subsample, endpoint=False, dtype='int32')

        # Build matrix of laser angles cosines and sines for projection.
        t_s = np.linspace(-np.pi/2.0, np.pi/2.0, 180, endpoint=True, dtype='int32') # scan theta
        t_s = t_s[t_sub]        # subsample
        cos_s = np.cos(t_w + t_s)
        sin_s = np.sin(t_w + t_s)

        # Get laser measurements.
        z_laser = z_t[6:]
        z_laser = z_laser[t_sub] # subsample

        # Project measurements into world frame.
        x_z = (x_w + np.multiply(z_laser, cos_s)) / 10.0
        y_z = (y_w + np.multiply(z_laser, sin_s)) / 10.0

        # Get grid cell indicies. The map origin is located at bottom left.
        x_z = x_z.astype('int32', copy=False)
        y_z = y_z.astype('int32', copy=False)

        # Clip coordinates to map size.
        cols = self.map.grid.shape[1]
        rows = self.map.grid.shape[0]
        x_z = np.clip(x_z, 0, cols-1)
        y_z = np.clip(y_z, 0, rows-1)

        # Get occupancy probabilities associated with each laser reading.
        p_hit = self.field[y_z, x_z]

        # # Get error distribution.
        # p_rand = 1 / self.z_rand * np.ones(180)
        # p_rand[z > self.z_max] = 0

        # # Get the max threshold distribution.
        # p_max = np.ones(180)
        # p_max[z < self.z_max] = 0

        # Compute the probability of z_t given x_t
        # p_z = self.z_hit * p_hit + self.z_rand * p_rand + self.z_max * p_max
        p_z = self.z_hit * p_hit

        weight = np.exp(np.sum(np.log(p_z)))

        return weight

def plot_likelihoodfield(grid, thresh, sigma, subsample):
    L = LikelihoodField(grid, thresh, sigma, subsample)
    L.plotField()

def main():
    parser = argparse.ArgumentParser(description='Laser unit test')
    parser.add_argument('--map', type=str, default='./data/map/wean.dat', help='Ground truth occupancy map')
    parser.add_argument('--log', type=str, default='./data/log/robotdata1.log', help='Robot data log file')
    args = parser.parse_args()

    # Read in map.
    map = Map()
    map.readMap(args.map)

    # plt.imshow(map.grid, norm=matplotlib.colors.Normalize(0, 1))
    # plt.show()

    L = LikelihoodField(map.grid, 0.01, 30, 8)
    # L.computeField()
    # L.saveField("config/lfield.csv")
    L.loadField("config/lfield.csv")
    L.plotField()
    plt.show()


if __name__ == '__main__':
    main()

