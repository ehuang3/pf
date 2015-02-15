'''
    Created On: Feb 14, 2015
    Author: Nehchal Jindal

    Desc: Map class

'''

import numpy as np
import cv2

class Map():
    ''' Map of the environment '''
    def __init__(self):
        '''
        -1  = don't know
        any value in [0;1] is a probability for unoccupied:
         1   = unoccupied with probability 1
         0   = occupied with probability 1
         0.5 = occupied with probability 0.5 '''
        self.grid = np.zeros((800, 800))

        self.resX = 10    # resolution
        self.resY = 10
        self.sizeX = 8000 # max length in x-direction
        self.sizeY = 8000 # max lenght in y-direction
        return

    def readMap(self, fileName):

        f = open(fileName, 'r')

        line = ''
        while line[:6] != 'global':
            line = f.readline()

        for y in (range(800)):
            line = f.readline()
            words = line.split()
            for x in range(800):
                self.grid[x][y] = float(words[x])

                # cells for which value not known, make them occupied
                if abs(self.grid[x][y] + 1) < 0.001:
                    self.grid[x][y] = 0

        return

    def printMap(self):
        ''' shows the current map '''
        #rows, cols = self.grid.shape
        #for rows in range(rows):
        #    for cols in range(cols):
        #        if
        cv2.imwrite('map.jpg', self.grid * 255)
