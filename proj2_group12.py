import sys
import cv2
import numpy as np
import heapq
global l


class node:
    global a
    def __init__(self, location, parent):
        self.loc = location
        self.value = a.map[location[0],location[1],3]


def area(x1, y1, x2, y2, x3, y3):  # area of triangle from points
    return abs((x1 * (y2 - y3) + x2 * (y3 - y1)
                + x3 * (y1 - y2)) / 2.0)

class MapMake:

    def __init__(self, width_x, length_y):

        self.clearance = 0
        self.width_x = width_x
        self.length_y = length_y
        self.map = np.zeros([width_x, length_y, 4]) # declaration
        self.map[:,:,0:3] = [0,255,0]               # stores the rgb values for visualisation
        self.map[:,:, 3] = np.inf                   # last element stores the cost to come

    def circle_obstacle(self, xpos, ypos, radius):  # makes a circle obstacle
        for i in range(self.width_x):
            for j in range(self.length_y):
                if np.sqrt(np.square(ypos - j) + np.square(xpos - i)) <= radius:
                    self.map[i, j, 0:3] = [0, 0, 255]

    def oval_obstacle(self, xpos, ypos, radius_x, radius_y):  # makes oval obstacle
        for i in range(self.width_x):
            for j in range(self.length_y):
                first_oval_term = np.square(i - xpos) / np.square(radius_x)
                second_oval_term = np.square(j - ypos) / np.square(radius_y)
                if first_oval_term + second_oval_term <= 1:
                    self.map[i, j, 0:3] = [0, 0, 255]

    def triangle_obstacle(self, point1, point2, point3):  # makes triangle obstacle
        tri_area = area(point1[0], point1[1], point2[0], point2[1], point3[0], point3[1])

        for i in range(self.width_x):
            for j in range(self.length_y):
                a1 = area(i, j, point2[0], point2[1], point3[0], point3[1])
                a2 = area(point1[0], point1[1], i, j, point3[0], point3[1])
                a3 = area(point1[0], point1[1], point2[0], point2[1], i, j)
                if np.abs(a1 + a2 + a3 - tri_area) <= .001:
                    self.map[i, j, 0:3] = [0, 0, 255]

def define_map():
    global a
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

def visualize_map():   # a function to visualize the initial map generated with just obstacles
    resized = cv2.resize(np.rot90(a.map[:, :, 0:3]), (900,600))
    cv2.imshow('map', resized)
    cv2.waitKey()    

def is_obstacle(): # A function to check if the a given point is inside the obstacle space.
    pass

def visualize_path(): # A function to visualise the entire search path and the optimal path finally.
    pass

def is_goal(curr_node): # A function to check if current state is the goal point
    pass

def find_path(curr_node): # A function to find the path uptil the root by tracking each node's parent
    pass

def find_children(curr_node): # A function to find a node's possible children and update cost in the map for each child
    return children
    pass

def add_image_frame(curr_node): # A function to add the newly explored state to a frame. This would also update the color based on the cost to come
    pass

def solver(curr_node): # A function to be recursively called to find the djikstra solution
    global l
    if (is_goal(curr_node)):
        find_path(curr_node) # find the path right uptil the start node by tracking the node's parent
        return 1
    add_image_frame(curr_node) 
    children = find_children(curr_node) # a function to find possible children and update cost
    l.append(children)                  # adding possible children to the list
    heapq.heapify(l)                    # converting to a list 
    solver(heapq.heappop(l))            # recursive call to solver where we pass the element with the least cost 
    return 0        

if __name__=="__main__":
    define_map()
    # visualize_map()
    start_pt = (input("Enter start point: "))
    start_pt = [int(start_pt.split()[0]), int(start_pt.split()[1])]
    global end_pt
    end_pt = (input("Enter end point: "))
    end_pt = [int(end_pt.split()[0]), int(end_pt.split()[1])]

    if(is_obstacle(start_pt) or is_obstacle(end_pt)): # check if either of start or end node an obstacle 
        print("Enter valid points... ")

    # create start node belonging to class node
    start_node = node(start_pt,None)
    global l
    l = [(start_node.value, start_node.loc)]

    # define a priority queue and add first element
    heapq.heapify(l)

    # solve using djikstra
    flag = solver(heapq.heappop(l))

    # if found, visualise the path 
    if flag == 1:
        visualize_path()
    # else print path not found    
    else:
        print("Solution not found... ")



################# List of functions/class/methods to be implemented #######################
# Function is_obstacle
# Class node
# Function solver
# Function visualise_path()
# is_goal
# find_path
# find_children
# add_image_frame
    
    
