#!/usr/bin/env python

import numpy as np
import pylab as pl
import argparse

from visualization import Visualization
from map import Map
from particleFilter import ParticleFilter
from particleFilter import SensorModel

def main():
    parser = argparse.ArgumentParser(description='Laser unit test')
    parser.add_argument('--map', type=str, default='./data/map/wean.dat', help='Ground truth occupancy map')
    parser.add_argument('--log', type=str, default='./data/log/robotdata1.log', help='Robot data log file')
    args = parser.parse_args()

    # Read in map.
    # map = Map()
    # map.readMap(args.map)

    # Number of data points.
    num_odometry = 0;
    num_laser = 0;

    # Laser scan data.
    z = []

    # Read in log file.
    with open(args.log) as log:
        for line in log:
            token = line.split()
            if token[0] == 'O':
                num_odometry += 1
            elif token[0] == 'L':
                num_laser += 1
                z.append(np.array(token[1:]))

    # Convert z into full numpy array.
    z = np.array(z, dtype='float64')

    print num_laser, z.shape

    # Accumulate all laser scan points.
    z_l = np.zeros(z.shape[0] * 180)
    z_i = 0;
    for i in range(z.shape[0]):
        for j in range(6, z.shape[1]-1):
            z_l[z_i] = z[i,j]
            z_i += 1

    # print z[z > 8000]
    pl.figure()
    n, bins, patches = pl.hist(z_l, 100, normed=1, histtype='bar')
    pl.setp(patches, 'facecolor', 'g', 'alpha', 0.75)
    pl.show()


if __name__ == '__main__':
    main()

