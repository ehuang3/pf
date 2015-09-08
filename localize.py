#!/usr/bin/env python
'''
    Created On: Feb 13, 2015
    Author: Nehchal Jindal
            Eric Huang

    Desc: localization of the robot in given map

    Lab 1

    CS8803: Statistical Techniques in Robotics
    Instructor: Byron Boots
    Spring '15
'''
import cv2
import numpy as np
import pylab as pl
import argparse

from visualization import Visualization
from map import Map
from particleFilter import ParticleFilter
from particleFilter import SensorModel
from sensor import LikelihoodField

def main():
    parser = argparse.ArgumentParser(description='Laser unit test')
    parser.add_argument('--map', type=str, default='../data/map/wean.dat', help='Ground truth occupancy map')
    parser.add_argument('--log', type=str, default='../data/log/robotdata1.log', help='Robot data log file')
    parser.add_argument('--lfield', type=str, default='./config/lfield_40.csv', help='Likelihood field')
    parser.add_argument('--save', type=str, default='./test/', help='Which folder to save to.')
    args = parser.parse_args()

    # Read in map.
    map = Map()
    map.readMap(args.map)

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

    # n = len(z)
    # z = [z[i] for i in range(0, n-1, 3)]

    # Convert z into full numpy array.
    z = np.array(z, dtype='float64')

    # Load likelihood field.
    L = LikelihoodField(0.01, 40, 8)
    L.loadField(args.lfield)

    # Run filter
    filter = ParticleFilter()
    # filter.setSensorModel(SensorModel(map, 0.7, 0.29, 0.01))
    filter.setLikelihoodField(L)
    filter.run(map, z, args.save)


if __name__ == '__main__':
    main()
