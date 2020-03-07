import sys
import cv2
import numpy as np
import heapq
import time
global l, final_path, img, vidWriter, node_cnt
sys.setrecursionlimit(10**9)
#sys.settrace(exception)


class node:
    
    def __init__(self, location, parent):
        global a, node_cnt
        self.loc = location
        self.value = a.map[location[0]][location[1]][1]
        self.parent = parent
        self.counter = node_cnt
        node_cnt += 1


class MapMake:

    def __init__(self, width_x, length_y):
        self.width_x = width_x
        self.length_y = length_y
        self.map = np.zeros([width_x, length_y, 2])
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

    def triangle_obstacle(self,three_points): # makes triangle obstacle
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

        self.map[self.width_x-clearance_distance:self.width_x,:,0] = 2  # makes edge clearance in map
        self.map[0:clearance_distance,:,0] = 2
        self.map[:, 0:clearance_distance, 0] = 2
        self.map[:, self.length_y-clearance_distance:self.length_y, 0] = 2

        img[0:clearance_distance, :, 0:3] = [0, 0, 200]                 # makes edge clearance in image
        img[self.width_x-clearance_distance:self.width_x, :, 0:3] = [0, 0, 200]
        img[:, 0:clearance_distance, 0:3] = [0, 0, 200]
        img[:, self.length_y-clearance_distance:self.length_y, 0:3] = [0, 0, 200]

        obstacles = np.where(self.map[:, :, 0] == 1)
        obstacles = np.array(obstacles)
        obstacles = obstacles.T  # get all points that are in obstacles

        circle_list = []     # makes all the points that falls within a circle and uses it to find clearance
        for i in np.arange(-clearance_distance,clearance_distance,1):
            for j in np.arange(-clearance_distance,clearance_distance,1):
                dist = np.sqrt(np.square(i) + np.square(j))
                if  dist <= clearance_distance:
                    circle_list.append([i,j])

        for obstacle_point in obstacles:
            bound_list = obstacle_point+circle_list
            for bound in bound_list:
                if a.map[bound[0], bound[1],0] == 0:
                    a.map[bound[0], bound[1],0] = 2
                    img[bound[0], bound[1], 0:3] = [0, 0, 200]


def sign(point1,point2,point3):
    return (point1[0]-point3[0])*(point2[1]-point3[1])-(point2[0]-point3[0])*(point1[1]-point3[1])


def max_and_min(point_list):
    point_list = np.array(point_list)
    max_x = max(point_list[:,0])
    min_x = min(point_list[:,0])
    max_y = max(point_list[:,1])
    min_y = min(point_list[:,1])
    return (min_x,min_y),(max_x,max_y)


def define_map(clear_r):  # makes the map according to the assignment
    global a
    a = MapMake(300, 200)
    a.circle_obstacle(225, 150, 25)
    a.oval_obstacle(150,100,40,20)

    a.triangle_obstacle(([20,120],[25,185],[50,150]))
    a.triangle_obstacle(([50,150],[25,185],[75,185]))
    a.triangle_obstacle(([50,150],[75,185],[100,150]))
    a.triangle_obstacle(([50,150],[100,150],[75,120]))

    a.triangle_obstacle(([25,185],[50,150],[75,185]))
    a.triangle_obstacle(([75,185],[100,150],[50,150]))
    a.triangle_obstacle(([50,150],[100,150],[75,120]))

    a.triangle_obstacle(([225,10],[225,40],[250,25]))
    a.triangle_obstacle(([225,10],[225,40],[200,25]))

    point1 = [95,30]
    point2 = [95+10*np.sin(np.deg2rad(30)),30+10*np.cos(np.deg2rad(30))]
    point3 = [point2[0]-75*np.cos(np.deg2rad(30)),point2[1]+75*np.sin(np.deg2rad(30))]
    point4 = [95-75*np.cos(np.deg2rad(30)),30+75*np.sin(np.deg2rad(30))]
    a.triangle_obstacle((point3,point1,point2))
    a.triangle_obstacle((point4,point3,point1))

    if clear_r != 0: # puts in clearance and radius if present
        a.clearance(clear_r)
    return    



def visualize_map():   # a function to visualize the initial map generated with just obstacles
    global img
    cv2.imshow('map',cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE))
    cv2.waitKey()


def point_in_obstacle(point):  # checks is a point is in an obstacle
    if a.map[point[0]][point[1]][0] != 0:
        return True
    else:
        return False


def allowable_moves(point):  # makes child states that are on the map and not on obstacles

    up,down,right,left = (point[0],point[1]+1),\
                         (point[0],point[1]-1),\
                         (point[0]+1,point[1]),\
                         (point[0]-1,point[1])            # possible cardinal moves

    nw,ne,sw,se = (point[0]-1,point[1]+1),\
                  (point[0]+1,point[1]+1),\
                  (point[0]-1,point[1]-1),\
                  (point[0]+1,point[1]-1)                 # possible diagonal moves

    test_square_moves = list((up,down,right,left))        # cardinal moves
    allowable_square_moves = []
    for move in test_square_moves:
        if not move in visitedNode:
            if a.map.shape[0] > move[0] >= 0:            # if on map X
                if a.map.shape[1] > move[1] >= 0:        # if on map y
                    if a.map[move[0],move[1],0] == 0:    # if not in obstacle

                        allowable_square_moves.append(move)

    test_dia_moves = list((nw,ne,sw,se))                  # diagonal moves
    allowable_dia_moves = []
    for move in test_dia_moves:
        if not move in visitedNode:
            if a.map.shape[0] > move[0] >= 0:            # if on the map x
                if a.map.shape[1] > move[1] >= 0:        # if on the map y
                    if a.map[move[0],move[1],0] == 0:    # if not in obstacle

                        allowable_dia_moves.append(move)

    return allowable_square_moves, allowable_dia_moves


def is_goal(curr_node):              # checks if the current node is also the goal
    if curr_node.loc[0] == end_pt[0] and curr_node.loc[1] == end_pt[1]:
        return True


def find_path(curr_node): # A function to find the path until the root by tracking each node's parent

    global final_path
    while(curr_node!=None):
        final_path.insert(0, curr_node)
        curr_node = curr_node.parent
    for i in final_path:
        img[i.loc[0], i.loc[1], 0:3] = [255,0,0]
        for j in range(3):
            vidWriter.write(cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE))
    vidWriter.release()         
    return


def find_children(curr_node):
    test_node = curr_node

    sqr_child_loc = allowable_moves(curr_node.loc)[0]  # gets allowable cardinal moves
    sqr_child_cost = test_node.value + 1               # square move cost
    sqr_children_list = []
    for state_loc in sqr_child_loc:
        if a.map[state_loc[0]][state_loc[1]][1] > sqr_child_cost:  # if the child cost is less from the current node
            a.map[state_loc[0]][state_loc[1]][1] = sqr_child_cost  # update map node to lesser cost
            sqr_child_node = node(state_loc,curr_node)              # create new child node
            sqr_children_list.append((sqr_child_node.value, sqr_child_node.counter, sqr_child_node))

    dia_child_loc = allowable_moves(curr_node.loc)[1]  # gets allowable diagonal moves
    dia_child_cost = test_node.value + np.sqrt(2)      # diagonal moves cost
    dia_children_list = []
    for state_loc in dia_child_loc:
        if a.map[state_loc[0]][state_loc[1]][1] > dia_child_cost:  # if the child cost is less from the current node
            a.map[state_loc[0]][state_loc[1]][1] = dia_child_cost  # update map node to lesser cost
            dia_child_node = node(state_loc, curr_node)            # create new child node
            dia_children_list.append((dia_child_node.value, dia_child_node.counter, dia_child_node))

    children_list = sqr_children_list + dia_children_list
    return children_list  # list of all children with a lesser cost for the current node


    

def add_image_frame(curr_node): # A function to add the newly explored state to a frame. This would also update the color based on the cost to come
    global img, vidWriter
    img[curr_node.loc[0], curr_node.loc[1],0:3] = [0,255,np.min([50 + curr_node.value*2, 255]) ]
    vidWriter.write(cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE))
    return
    

def solver(curr_node):  # A function to be recursively called to find the djikstra solution
    while(1):
        visitedNode.update({curr_node: "s"})
        global l
        if (is_goal(curr_node)):
            find_path(curr_node) # find the path to the start node by tracking the node's parent
            # print("here")
            break
        add_image_frame(curr_node)
        children_list = find_children(curr_node) # a function to find possible children and update cost
        l = l + children_list                  # adding possible children to the list
        heapq.heapify(l)                    # converting to a list
        curr_node = heapq.heappop(l)[2]            # recursive call to solver where we pass the element with the least cost 
    return 1        


if __name__=="__main__":
    global start_pt
    global end_pt
    global vidWriter
    global path
    global node_cnt
    global final_path
    global visitedNode
    node_cnt = 0
    final_path = []
    visitedNode = {}
    vidWriter = cv2.VideoWriter("Djikstra.mp4", cv2.VideoWriter_fourcc(*'mp4v'), 288, (300,200))
    img = np.zeros([300,200,3], dtype=np.uint8)
    img[:,:,0:3] = [0,255,0]
    bot_r = int(input("Enter robot radius: "))
    clear_r = int(input("Enter the clearance: "))
    total_clear = bot_r+clear_r
    define_map_start = time.time()
    define_map(total_clear)
    t1 = time.time()-define_map_start
    print("Time to define map: " + str(t1))
    solve_problem_start = time.time()
    #visualize_map()


    valid_points = False
    while  valid_points == False:
        start_pt = (input("Enter start point in form # #: "))
        start_pt = [int(start_pt.split()[0]), int(start_pt.split()[1])]
        img[start_pt[0]][start_pt[1]][0:3] = [0,0,0]

        end_pt = (input("Enter end point in form # #: "))
        end_pt = [int(end_pt.split()[0]), int(end_pt.split()[1])]

        img[end_pt[0]][end_pt[1]][0:3] = [0,0,255]
        if(point_in_obstacle(start_pt) or point_in_obstacle(end_pt)): # check if either the start or end node an obstacle
            print("Enter valid points... ")
            continue
        else:
            valid_points = True

    a.map[start_pt[0], start_pt[1], 1] = 0

    # create start node belonging to class node
    start_node = node(start_pt,None)
    start_node.value = 0
    global l
    l = [(start_node.value, start_node.counter, start_node)]

    # define a priority queue and add first element
    heapq.heapify(l)

    print("Running..")
    # solve using djikstra
    flag = solver(heapq.heappop(l)[2])
    # print(flag)
    # if found, visualise the path 
    if flag == 1:
        print("Path found. Please watch the video generated.")
        print("Time to define map and solve problem: " + str(time.time() - solve_problem_start + t1))
    # else print path not found    
    else:
        print("Solution not found... ")


