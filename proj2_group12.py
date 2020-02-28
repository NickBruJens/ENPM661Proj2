import sys
import cv2
import numpy as np


start_node = [10,20]  # in [x,y]
goal_node = [30,50]
radius = 0  # radius of the robot
clearance = 0  #


map = np.zeros([300,200,2])
obstacle_layer = map[:,:,0]
map[:, :, 1] = np.inf  # high cost to come
cost2comelayer = map[:, :, 1]
cv2.imshow('map',obstacle_layer)
cv2.waitKey()


