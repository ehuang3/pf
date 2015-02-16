#!/usr/bin/env python

import numpy as np
import pylab as pl
import argparse
import time

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
    map = Map()
    map.readMap(args.map)

    # Number of data points.
    num_odometry = 0;
    num_laser = 0;

    # Laser scan data.
    z = []

    # Odometry data.
    x = []

    # Time data.
    t_z = []
    t_x = []

    # Read in log file.
    with open(args.log) as log:
        for line in log:
            token = line.split()
            if token[0] == 'O':
                num_odometry += 1
                x.append(np.array(token[1:4]))
                t_x.append(token[4])
            elif token[0] == 'L':
                num_laser += 1
                z.append(np.array(token[1:187]))
                t_z.append(token[187])

    # Convert data into full numpy arrays.
    x = np.array(x, dtype='float64')
    z = np.array(z, dtype='float64')
    t_x = np.array(t_x, dtype='float64')
    t_z = np.array(t_z, dtype='float64')

    # Clip laser readings at 3000
    z = np.clip(z, -3000, 3000)

    # Plot the odometry trajectory.
    pl.plot(x[:,0], x[:,1])
    # pl.show()
    pl.show(block=False)

    print z.shape[0]

    # Generate a list of x, y points to plot.
    for i in range(z.shape[0]):
        x_o = z[i,3]
        y_o = z[i,4]
        t_o = z[i,5]

        # Build matrix of laser angles cosines and sines for projection.
        t_s = np.linspace(-np.pi/2.0, np.pi/2.0, 180, True) # scan theta
        cos_s = np.cos(t_o + t_s)
        sin_s = np.sin(t_o + t_s)

        # Get laser measurements.
        z_t = z[i, 6:]

        # Project measurements into world frame.
        x_z = (x_o + np.multiply(z_t, cos_s))
        y_z = (y_o + np.multiply(z_t, sin_s))

        pl.cla()
        pl.plot(x[:,0], x[:,1])
        pl.plot(x_z, y_z, ',r')
        pl.draw()
        time.sleep(0.01)

    pl.show()


if __name__ == '__main__':
    main()

