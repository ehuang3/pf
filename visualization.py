'''
    Created On: Feb 13, 2015
    Author: Nehchal Jindal
            Eric Huang

    Desc: Visualization of particle filter
'''

from map import Map
from particleFilter import Particle
from graphics import *

import numpy as np

class Visualization():

    def __init__(self, pixelsX=800, pixelsY=800):
        self.pixelsX = pixelsX  # size of window
        self.pixelsY = pixelsY  # size of window
        self.win = GraphWin('Robot localizaton', pixelsX, pixelsY)
        return

    def drawMap(self, map):
        ''' Visualize the map '''

        rect = Rectangle(Point(0,0), Point(self.pixelsX-1, self.pixelsY-1))
        rect.draw(self.win);

        Xmax, Ymax = np.array(map.grid).shape
        for x in range(0, Xmax, 5):
            for y in range(0, Ymax, 5):
                P1 = Point(x*map.resX*self.pixelsX/map.sizeX, 
                            y*map.resY*self.pixelsY/map.sizeY,)
                P2 = Point((x+1)*map.resX*self.pixelsX/map.sizeX, 
                            (y+1)*map.resY*self.pixelsY/map.sizeY,)
                rect = Rectangle(P1, P2)
                i = int(255.0*map.grid[x][y]) # intensity
                c = color_rgb(i, i, i)
                rect.setFill(c)
                rect.setOutline(c)
                rect.draw(self.win)

        return

    def drawParticles(self, X, sizeX, sizeY):
        ''' Visualize the partiles '''
        for p in X:
            pt = Point(p.state[0]*self.pixelsX/sizeX, p.state[1]*self.pixelsY/sizeY)
            pt.setFill('red')
            pt.draw(self.win)
        return None