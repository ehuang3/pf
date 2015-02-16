'''
    Created On: Feb 13, 2015
    Author: Nehchal Jindal
            Eric Huang

    Desc: Visualization of particle filter
'''

from map import Map
from graphics import *

import numpy as np
import pylab as plt

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

        self.im = np.zeros((pixelsX, pixelsY, 3), np.uint8)
        self.im1 = self.im.copy()

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

        self.im = np.zeros((Xmax, Ymax, 3), np.uint8)
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

        cv2.imwrite('test/map_temp.jpg', self.im)

        self.im1 = self.im.copy()

        return

    def plotMap(self, map):
        x = np.linspace(0, 8000, 800)
        y = np.linspace(0, 8000, 800)
        xv, yv = np.meshgrid(x, y, sparse=False, indexing='ij')
        plt.scatter(xv, yv, c=map.grid.flatten(), s=500)

    def drawParticles(self, X):
        ''' Visualize the partiles '''
        #im_temp = self.im.copy()
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

            self.im1[i][j][0] = 0
            self.im1[i][j][1] = 0
            self.im1[i][j][2] = 255

        return

    def drawLaser(self, pos, z, angles):
        '''
        Draw the robot at pos x with the laser projections.
        @param pos : [x, y, z]
        @param z   : list of laser reading
        @pararm angles: list of angles in radians
        '''
        x = int(pos[0]/10.0)
        y = int(pos[1]/10.0)
        t = pos[2]

        # Draw green filled circle as robot
        cv2.circle(self.im1, (x,y), 3, (255, 0, 0), -1)


        # Draw lines for laser
        for i in range(len(z)):
            d = z[i]/10.0
            #print x
            #print y
            #print t
            #print angles[i]
            #print d
            #print np.cos(t + angles[i])
            #print z
            x_end = int(x + d*np.cos(t + angles[i]))
            y_end = int(y + d*np.sin(t + angles[i]))
            cv2.line(self.im1, (x, y), (x_end, y_end), (0, 255, 0))


    def writeText(self, text):
        cv2.putText(
                        self.im1, 
                        text, 
                        (20, 20),
                        cv2.FONT_HERSHEY_COMPLEX_SMALL,1,
                        (255, 255, 255))

    def saveImage(self):
        cv2.imwrite('test/' + str(self.count) + '.jpg', self.im1)
        self.count = self.count + 1
        self.im1 = self.im.copy()

def testVisualization():
    ''' basic testing for Visualization '''
    vis = Visualization()

if __name__ == '__main__':
    testVisualization()

