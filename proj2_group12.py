import sys
import cv2
import numpy as np
import heapq
from itertools import count
global l, final_path, img, vidWriter, node_cnt
sys.setrecursionlimit(10**6)

class node:
    
    def __init__(self, location, parent):
        global a, node_cnt
        self.loc = location
        self.value = a.map[location[0]][location[1]][1]
        self.parent = parent
        self.counter = node_cnt
        node_cnt += 1


def area(x1, y1, x2, y2, x3, y3):  # area of triangle from points
    return abs((x1 * (y2 - y3) + x2 * (y3 - y1)
                + x3 * (y1 - y2)) / 2.0)

def sign(point1,point2,point3):
    return (point1[0]-point3[0])*(point2[1]-point3[1])-(point2[0]-point3[0])*(point1[1]-point3[1])

def max_and_min(point_list):
    point_list = np.array(point_list)
    max_x = max(point_list[:,0])
    min_x = min(point_list[:,0])
    max_y = max(point_list[:,1])
    min_y = min(point_list[:,1])
    return (min_x,min_y),(max_x,max_y)

# self.map is shape [NxMx2]
# self.map[:,:,0:1] = [type of cell, cost to come]
# cell type 0 = free space
# cell type 1 = obstacle
# cell type 2 = clearance space
# cell type 3 = goal
# cell type 4 = start position if needed

class MapMake:

    def __init__(self, width_x, length_y):
        self.width_x = width_x
        self.length_y = length_y
        self.map = np.zeros([width_x, length_y, 4]) # declaration
        self.map[:,:, 1] = np.inf                   # last element stores the cost to come

    def circle_obstacle(self, xpos, ypos, radius):  # makes a circle obstacle
        for i in np.arange(xpos-radius,xpos+radius,1):
            for j in np.arange(ypos-radius,ypos+radius,1):
                if np.sqrt(np.square(ypos - j) + np.square(xpos - i)) <= radius:
                    self.map[i, j, 0] = 1
                    img[i,j,0:3] = [0,0,255]

    def oval_obstacle(self, xpos, ypos, radius_x, radius_y):  # makes oval obstacle
        for i in np.arange(xpos-radius_x,xpos+radius_x,1):
            for j in np.arange(ypos-radius_y,ypos+radius_y,1):
                first_oval_term = np.square(i - xpos) / np.square(radius_x)
                second_oval_term = np.square(j - ypos) / np.square(radius_y)
                if first_oval_term + second_oval_term <= 1:
                    self.map[i, j, 0] = 1
                    img[i,j,0:3] = [0,0,255]


    def triangle_obstacle(self,three_points):  # makes triangle obstacle
        v1 = three_points[0]
        v2 = three_points[1]
        v3 = three_points[2]
        min_point,max_point = max_and_min(three_points)
        for i in np.arange(min_point[0],max_point[0],1):
            for j in range(int(min_point[1]),int(max_point[1]),1):
                pt = [i,j]
                d1 = sign(pt, v1, v2)
                d2 = sign(pt, v2, v3)
                d3 = sign(pt, v3, v1)
                neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
                pos = (d1 > 0) or (d2 > 0) or (d3 > 0)

                if not (neg and pos):
                    a.map[int(i)][int(j)][0] = 1
                    img[int(i), int(j), 0:3] = [0, 0, 255]

    def clearance(self, clearance_distance):
        obstacles = np.where(self.map[:, :, 0] == 1)
        obstacles = np.array(obstacles)
        obstacles = obstacles.T
        obstacle_edges = []
        obstacle_list = obstacles.tolist()

        for i in range(len(obstacles)):
            up = [obstacles[i][0], obstacles[i][1] + 1]
            down = [obstacles[i][0], obstacles[i][1] - 1]
            left = [obstacles[i][0] - 1, obstacles[i][1]]
            right = [obstacles[i][0] + 1, obstacles[i][1]]

            surrounded = (up in obstacle_list) and \
                         (down in obstacle_list) and \
                         (left in obstacle_list) and \
                         (right in obstacle_list)

            if not surrounded:
                obstacle_edges.append(obstacles[i])  # only compares pixel indices to edge of obstacle

        for i in range(len(obstacle_edges)):
            xmin,xmax = obstacle_edges[i][0]-clearance_distance, obstacle_edges[i][0]+clearance_distance
            ymin,ymax = obstacle_edges[i][1]-clearance_distance, obstacle_edges[i][1]+clearance_distance
            for j in np.arange(xmin,xmax,1):
                for k in np.arange(ymin,ymax,1):
                    dist = np.sqrt(np.square(int(j)-obstacle_edges[i][0])+np.square(int(k)-obstacle_edges[i][1]))
                    if dist <= clearance_distance and self.map[int(j),int(k),0] != 1:
                        self.map[int(j),int(k),0] = 2
                        img[int(j),int(k),0:3] = [0,0,200]

def define_map():
    global a
    a = MapMake(300, 200)

    a.circle_obstacle(225, 150, 25)  # upper right circle
    a.oval_obstacle(150,100,40,20)  # center oval


    a.triangle_obstacle(([20,120],[25,185],[50,150]))
    a.triangle_obstacle(([50,150],[25,185],[75,185]))
    a.triangle_obstacle(([50,150],[75,185],[100,150]))
    a.triangle_obstacle(([50,150],[100,150],[75,120]))



    a.triangle_obstacle(([25,185],[50,150],[75,185]))
    a.triangle_obstacle(([75,185],[100,150],[50,150]))
    a.triangle_obstacle(([50,150],[100,150],[75,120]))

    a.triangle_obstacle(([225,10],[225,40],[250,25]))  # lower right diamond
    a.triangle_obstacle(([225,10],[225,40],[200,25]))

    point1 = [95,30]
    point2 = [95+10*np.sin(np.deg2rad(30)),30+10*np.cos(np.deg2rad(30))]
    point3 = [point2[0]-75*np.cos(np.deg2rad(30)),point2[1]+75*np.sin(np.deg2rad(30))]
    point4 = [95-75*np.cos(np.deg2rad(30)),30+75*np.sin(np.deg2rad(30))]

    a.triangle_obstacle((point3,point1,point2))  # lower left rectangle
    a.triangle_obstacle((point4,point3,point1))
    a.clearance(2)


def visualize_map():   # a function to visualize the initial map generated with just obstacles
    global img
    #free_space = [0,255,0] # what the values where before
    #obstacle = [0,0,255]
    #clearance = [0,30,100]
    # resized = cv2.resize(np.rot90(a.map[:, :, 0:3]), (900,600))
    cv2.imshow('map',cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE))
    cv2.waitKey()

def point_in_obstacle(point): # checks if the point is in obstacle or clearance space:
    if a.map[point[0],point[1],0] == 1 or a.map[point[0],point[1],0] == 2:
        return True

def allowable_moves(point): # makes sure child states are new, not on obstacles, and are on the map
    up,down,right,left = (point[0],point[1]+1),\
                         (point[0],point[1]-1),\
                         (point[0]+1,point[1]),\
                         (point[0]-1,point[1])
    nw,ne,sw,se = (point[0]-1,point[1]+1),\
                  (point[0]+1,point[1]+1),\
                  (point[0]-1,point[1]-1),\
                  (point[0]+1,point[1]-1)
    square_moves = list((up,down,right,left))
    for move in square_moves:
        if point_in_obstacle(move):
            square_moves.remove(move)  # this is in an obstacle
        elif move[0] >= a.map.shape[0] or move[0] < 0: # went off map x
            square_moves.remove(move)
        elif move[1] >= a.map.shape[1] or move[1] < 0:  # went off map y
            square_moves.remove(move)

    dia_moves = list((nw,ne,sw,se))
    for move in dia_moves:
        if point_in_obstacle(move):
            dia_moves.remove(move)  # this is in an obstacle
        elif move[0] >= a.map.shape[0] or move[0] < 0:  # went off map x
            dia_moves.remove(move)
        elif move[1] >= a.map.shape[1] or move[1] < 0:  # went off map y
            dia_moves.remove(move)

    return square_moves,dia_moves





    # return moves # returns new points that

def is_goal(curr_node): # A function to check if current state is the goal point
    if curr_node.loc[0] == end_pt[0] and curr_node.loc[1] == end_pt[1]:
        return True

def find_path(curr_node): # A function to find the path uptil the root by tracking each node's parent
    #vishnuu
    global final_path
    while(curr_node!=None):
        final_path.insert(0, curr_node)
        curr_node = curr_node.parent
    vidWriter.release()     
    return

def find_children(curr_node): # A function to find a node's possible children and update cost in the map for each child

    sqr_child_loc = allowable_moves(curr_node.loc)[0]
    sqr_child_cost = curr_node.value + 1
    sqr_children_list = []
    for state_loc in sqr_child_loc:
        if a.map[state_loc[0]][state_loc[1]][1] > sqr_child_cost:
            a.map[state_loc[0]][state_loc[1]][1] = sqr_child_cost
            sqr_child_node = node(state_loc,curr_node)
            print (state_loc)
            sqr_children_list.append((sqr_child_node.value, sqr_child_node.counter, sqr_child_node))

    dia_child_loc = allowable_moves(curr_node.loc)[1]
    dia_child_cost = curr_node.value + np.sqrt(2)
    dia_children_list = []
    for state_loc in dia_child_loc:
        if a.map[state_loc[0]][state_loc[1]][1] > dia_child_cost:
            a.map[state_loc[0]][state_loc[1]][1] = dia_child_cost
            dia_child_node = node(state_loc, curr_node)
            print (state_loc)
            dia_children_list.append((dia_child_node.value, dia_child_node.counter, dia_child_node))

    childern_list = sqr_children_list + dia_children_list
    return childern_list

    #return children_list     # [(child1.val, child1), (child2.val, child2), (child3.val, child3)]
    

def add_image_frame(curr_node): # A function to add the newly explored state to a frame. This would also update the color based on the cost to come
    #vishnuu
    global img, vidWriter
    if curr_node.value != np.inf :
        # print(curr_node.loc)
        img[curr_node.loc,0:3] = [255,0,0]
    vidWriter.write(cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE))
    return
    

def solver(curr_node):  # A function to be recursively called to find the djikstra solution
    global l
    # print (1)
    if (is_goal(curr_node)):
        find_path(curr_node) # find the path right uptil the start node by tracking the node's parent
        print("here")
        return 
    add_image_frame(curr_node)
    children_list = find_children(curr_node) # a function to find possible children and update cost
    l = l + children_list                  # adding possible children to the list
    # print (l)
    heapq.heapify(l)                    # converting to a list
    solver(heapq.heappop(l)[2])            # recursive call to solver where we pass the element with the least cost 
    return 1        

# ''' finding boundries to compare pixles to 
# f = max_and_min(list(([1,2],[4,100],[3,10],[0,0])))
# print(f)
# '''


if __name__=="__main__":
    global start_pt
    global end_pt
    global vidWriter
    global path
    global node_cnt
    global final_path
    node_cnt = 0
    final_path = []
    path = "/home/vishnuu/UMD/ENPM661/Project2/ENPM661Proj2/"
    vidWriter = cv2.VideoWriter(path + "Djikstra.mp4", cv2.VideoWriter_fourcc(*'mp4v'), 24, (300,200))
    img = np.zeros([300,200,3], dtype=np.uint8)
    img[:,:,0:3] = [0,255,0]
    define_map()
    visualize_map()
    valid_points = False
    while  valid_points == False:
        #start_pt = (input("Enter start point in form # #: "))
        #start_pt = [int(start_pt.split()[0]), int(start_pt.split()[1])]
        start_pt = [180,20]
        img[start_pt,0:3] = [0,0,0]

        #end_pt = (input("Enter end point in form # #: "))
        #end_pt = [int(end_pt.split()[0]), int(end_pt.split()[1])]
        end_pt = [190,50]
        img[end_pt,0:3] = [0,0,255]
        if(point_in_obstacle(start_pt) or point_in_obstacle(end_pt)): # check if either the start or end node an obstacle
            print("Enter valid points... ")
            continue
        else:
            valid_points = True

    a.map[start_pt, 1] = 0

    # create start node belonging to class node
    start_node = node(start_pt,None)
    start_node.value = 0
    global l
    l = [(start_node.value, start_node.counter, start_node)]

    # define a priority queue and add first element
    heapq.heapify(l)

    # solve using djikstra
    flag = solver(heapq.heappop(l)[2])
    print(flag)
    # if found, visualise the path 
    if flag == 1:
        print("Path found. Please watch the video generated.")
    # else print path not found    
    else:
        print("Solution not found... ")



################# List of functions/class/methods to be implemented #######################
# Class node
# Function solver
# Function visualise_path()
# is_goal
# find_path
# find_children
# add_image_frame
    

