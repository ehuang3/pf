'''
    Created On: Feb 13, 2015
    Author: Nehchal Jindal
            Eric Huang

    Desc: Visualization of particle filter
'''

from map import Map
from graphics import *

import numpy as np

import cv2

class Particle(object):
    ''' Particle in the particle filter '''
    def __init__(self, x, y, theta, weight):
        self.state = [x, y, theta]
        self.weight = weight    # importance weight

class Visualization():


    def __init__(self, pixelsX=800, pixelsY=800):
        self.pixelsX = pixelsX  # size of window
        self.pixelsY = pixelsY  # size of window
        # self.win = GraphWin('Robot localizaton', pixelsX, pixelsY)

        self.im = None

        self.count = 0

        return

    def drawMap(self, map):
        ''' Visualize the map '''

        # rect = Rectangle(Point(0,0), Point(self.pixelsX-1, self.pixelsY-1))
        # rect.draw(self.win);

        Xmax, Ymax = np.array(map.grid).shape
        '''
        for x in range(0, Xmax):
            for y in range(0, Ymax):
                P1 = Point(x*map.resX*self.pixelsX/map.sizeX, 
                            y*map.resY*self.pixelsY/map.sizeY,)
                P2 = Point((x+1)*map.resX*self.pixelsX/map.sizeX, 
                            (y+1)*map.resY*self.pixelsY/map.sizeY,)
                rect = Rectangle(P1, P2)
                i = int(255.0*(1 - map.grid[x][y])) # intensity
                c = color_rgb(i, i, i)
                rect.setFill(c)
                rect.setOutline(c)
                rect.draw(self.win)
        '''

        self.im = np.zeros((Xmax, Ymax, 3))
        self.imX = Xmax
        self.imY = Ymax
        self.sizeX = float(Xmax*map.resX)
        self.sizeY = float(Ymax*map.resY)
        for x in range(0, Xmax):
            for y in range(0, Ymax):
                c = 255 * (map.grid[x][y])
                self.im[x][y][0] = c
                self.im[x][y][1] = c
                self.im[x][y][2] = c

        cv2.imwrite('map_temp.jpg', self.im)


        return

    def drawParticles(self, X):
        ''' Visualize the partiles '''
        im_temp = self.im.copy()
        for p in X:
            '''
            pt = Point(p.state[0]*self.pixelsX/sizeX, p.state[1]*self.pixelsY/sizeY)
            pt.setFill('red')
            pt.draw(self.win)
            '''
            i = p.state[0]*self.imX/self.sizeX
            j = p.state[1]*self.imY/self.sizeY

            i = np.clip(i, 0, self.imX-1)
            j = np.clip(j, 0, self.imY-1)

            im_temp[i][j][0] = 0
            im_temp[i][j][1] = 0
            im_temp[i][j][2] = 255

        cv2.imwrite(str(self.count) + '.jpg', im_temp)
        self.count = self.count + 1

        return
