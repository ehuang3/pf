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

import numpy as np
import cv2

from visualization import Visualization
from map import Map
from particleFilter import ParticleFilter




if __name__ == '__main__':
    
    ''' Read map '''
    map = Map()
    map.readMap('../data/map/wean.dat')
    map.printMap();

    ''' Create Visualization '''
    vis = Visualization()
    vis.drawMap(map)


    filter = ParticleFilter()
    filter.initParticles(map)

    vis.drawParticles(filter.X)

    # open log file
    logFile = open('../data/log/robotdata1.log','r');

    # log file has 2218 lines
    for i in range(2000):
        line = logFile.readline()

        # read line only with laser data. Ignore others.
        if line[0] == 'L':
            z = np.zeros(180)
            words = line.split()
            for j in range(180):
                z[j] = float(words[j+1])