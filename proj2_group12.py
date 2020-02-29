import sys
import cv2
import numpy as np


def area(x1, y1, x2, y2, x3, y3):  # area of triangle from points
    return abs((x1 * (y2 - y3) + x2 * (y3 - y1)
                + x3 * (y1 - y2)) / 2.0)


class MapMake:

    def __init__(self, width_x, length_y):

        self.clearance = 0
        self.width_x = width_x
        self.length_y = length_y
        self.map = np.zeros([width_x, length_y, 2])
        self.map[:,:, 1] = np.inf

    def circle_obstacle(self, xpos, ypos, radius):  # makes a circle obstacle
        for i in range(self.width_x):
            for j in range(self.length_y):
                if np.sqrt(np.square(ypos - j) + np.square(xpos - i)) <= radius:
                    self.map[i, j, 0] = 1

    def oval_obstacle(self, xpos, ypos, radius_x, radius_y):  # makes oval obstacle
        for i in range(self.width_x):
            for j in range(self.length_y):
                first_oval_term = np.square(i - xpos) / np.square(radius_x)
                second_oval_term = np.square(j - ypos) / np.square(radius_y)
                if first_oval_term + second_oval_term <= 1:
                    self.map[i, j, 0] = 1

    def triangle_obstacle(self, point1, point2, point3):  # makes triangle obstacle
        tri_area = area(point1[0], point1[1], point2[0], point2[1], point3[0], point3[1])

        for i in range(self.width_x):
            for j in range(self.length_y):
                a1 = area(i, j, point2[0], point2[1], point3[0], point3[1])
                a2 = area(point1[0], point1[1], i, j, point3[0], point3[1])
                a3 = area(point1[0], point1[1], point2[0], point2[1], i, j)
                if np.abs(a1 + a2 + a3 - tri_area) <= .001:
                    self.map[i, j, 0] = 1


a = MapMake(300, 200)

a.circle_obstacle(225, 150, 25)  # upper right circle
a.oval_obstacle(150,100,40,20)  # center oval

a.triangle_obstacle([20,120],[25,185],[50,150])  # upper left polygon
a.triangle_obstacle([25,185],[50,150],[75,185])
a.triangle_obstacle([75,185],[100,150],[50,150])
a.triangle_obstacle([50,150],[100,150],[75,120])

a.triangle_obstacle([225,10],[225,40],[250,25])  # lower right diamond
a.triangle_obstacle([225,10],[225,40],[200,25])

point1 = [95,30]
point2 = [95+10*np.sin(np.deg2rad(30)),30+10*np.cos(np.deg2rad(30))]
point3 = [point2[0]-75*np.cos(np.deg2rad(30)),point2[1]+75*np.sin(np.deg2rad(30))]
point4 = [95-75*np.cos(np.deg2rad(30)),30+75*np.sin(np.deg2rad(30))]

a.triangle_obstacle(point3,point1,point2)  # lower left rectangle
a.triangle_obstacle(point4,point3,point1)


cv2.imshow('map', np.rot90(a.map[:, :, 0]))
cv2.waitKey()
