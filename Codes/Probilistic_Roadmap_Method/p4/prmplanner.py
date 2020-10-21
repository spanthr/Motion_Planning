# prmplanner.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
#
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)

from graph import RoadmapVertex, RoadmapEdge, Roadmap
from utils import *
import numpy as np
import random
import math
from math import sqrt



disk_robot = True #(change this to False for the advanced extension)


obstacles = None # the obstacles
robot_radius = None # the radius of the robot
robot_width = None # the width of the OBB robot (advanced extension)
robot_height = None # the height of the OBB robot (advanced extension)




# ----------------------------------------
# modify the code below
# ----------------------------------------

# Construction phase: Build the roadmap
# You should incrementally sample configurations according to a strategy and add them to the roadmap,
# select the neighbors of each sample according to a distance function and strategy, and
# attempt to connect the sample to its neighbors using a local planner, leading to corresponding edges
# See graph.py to get familiar with the Roadmap class

def build_roadmap(q_range, robot_dim, scene_obstacles):

    global obstacles, robot_width, robot_height, robot_radius

    obstacles = scene_obstacles # setting the global obstacle variable

    x_limit = q_range[0] # the range of x-positions for the robot
    y_limit = q_range[1] # the range of y-positions for the robot

    theta_limit = q_range[2] # the range of orientations for the robot (advanced extension)

    robot_width, robot_height = robot_dim[0], robot_dim[1] # the dimensions of the robot, represented as an oriented bounding box

    robot_radius = max(robot_width, robot_height)/2.
    graph = Roadmap()
    n = 1100      # Number of samples

    min_xy = [x_limit[0] + 2, y_limit[0] + 2]   #Limit
    max_xy = [x_limit[1] - 2, y_limit[1] - 2]
######### Sampling method : Random sampling########################
    # array = np.random.uniform(low=min_xy, high=max_xy, size=(n, 2))

############## Sampling method: Unifrom sampling with gaussian noise addition#############
    x = np.linspace(x_limit[0] + 3, x_limit[1] - 3, 35)         # vertices in one line will be 35
    y = np.linspace(y_limit[0] + 3, y_limit[1] - 3, 35)         #

    x_n = np.random.uniform(-0.8, 0.8, 100)                    # -0.8 and 0.8 will be min max limit
    y_n = np.random.uniform(-0.8, 0.8, 100)
    array = []
    for i in range(len(x)):
        for j in range(len(y)):
            point = (x[i] + np.random.choice(x_n), y[j] + np.random.choice(y_n))
            array.append(point)

#### Sampling method: Kernnel based sampling ############333
#     array = np.random.uniform(low=min_xy, high=max_xy, size=(1, 2))
#     initial_array = np.random.uniform(low=min_xy, high=max_xy, size=(1, 2))
#     kernelx=2
#     kernely=2
#
#     l=1.75
#     l=1.75
#     for i in np.arange(x_limit[0] +2.5, x_limit[1]-l,3):
#         endx=i+kernelx
#         lengthx=[i,endx]
#         for j in np.arange(y_limit[0]+l,y_limit[1]-l,3):
#             endy=j+kernely
#             lengthy=[j,endy]
#
#             """ This is the second way to do the problem"""
#             mu, sigma = 0, 1 # mean and standard deviation
#             s = (np.random.normal(mu, 0.9, 1))
#             s1 = (np.random.normal(mu, 0.9, 1))
# #            print(s)
#             arr=[(int)(i+s),(int)(j+s1)]
#
# #            arr=[i,j]
#             """ ______________________________________________________"""
#
#             """ This is the third way to do the problem  Uncomment to use"""
# #            arr = (np.random.uniform(lengthx, lengthy, size=(10, 2)))
#             """ ______________________________________________________"""
#             initial_array= np.vstack([initial_array, arr])
#
#             endy=0
#         endx=0
# #        array=initial_array
#
#
#
#     cond12=0
#     for i in range(len(initial_array)):
#
#         for j in range(len(initial_array)):
#
#             if initial_array[i][0] != initial_array[j][0] and initial_array[i][1] != initial_array[j][1]:
#                 dista = distance([initial_array[i][0],initial_array[i][1]], [initial_array[j][0],initial_array[j][1]])
#                 if(dista>0.3):
#                     pass
#                 else:
# #                    print(dista)
#                     cond12=1                       # Do not add in this case
#
#         if (cond12!=1):
#             x1=[initial_array[i][0],initial_array[i][1]]
#             array= np.vstack([array, x1])
# #            print(dista)
#         cond12=0
#
    coll_list = []

    for point in range(len(array)):        #Loop for checking whether the vertices are pn the obstacle or not and elimination of such vertices
        obstacle_check = 0
        for i in range(len(obstacles)):
            obstacle_i = obstacles[i].points
            xmin = obstacle_i[0][0] - robot_radius
            xmax = obstacle_i[2][0] + robot_radius
            ymin = obstacle_i[2][1] - robot_radius
            ymax = obstacle_i[0][1] + robot_radius

            if xmin<array[point][0]<xmax and ymin<array[point][1]<ymax:
                obstacle_check = 1

        if obstacle_check != 1:
            collision_free=graph.addVertex(array[point])   #Adding vertex in roadmap
            coll_list.append(collision_free)
            # print(coll_list[a].q)


    list = []                                           #For adding edges to the roadmap according to distance and to eliminate loopy path
    visited = [False for i in range(len(coll_list))]     #Initially visited will be false as no vertices are visted
    for pts in coll_list:
        list.append(pts)
        visited[pts.id] =True                              #After visiting the vertices make the visited of that point becomes True.

        while list:
            new_parent = list.pop(0)
            for child in coll_list:
                if visited[child.id] == False and child.id != pts.id:           #If its not visited and it is not the same point find distance.
                    dis = distance(new_parent.q, child.q)

                    if dis <5:                                                   #Condition for adding edges
                        list.append(child)
                        visited[child.id] = True
                        # check_point = np.linspace(new_parent.q,child.q,10)
                        # check = 0
                        # check_list = []
                        # for point in (check_point):
                        #     for p in range(len(obstacles)):
                        #         obstacle_i = obstacles[p].points
                        #         xmin = obstacle_i[0][0] - robot_radius
                        #         xmax = obstacle_i[2][0] + robot_radius
                        #         ymin = obstacle_i[2][1] - robot_radius
                        #         ymax = obstacle_i[0][1] + robot_radius
                        #
                        #         if xmin < point[0] < xmax and ymin < point[1] < ymax:
                        #             check = 1
                        #
                        # if check != 1:

                        graph.addEdge(new_parent, child, dis,path = [])

    # the roadmap


    # uncomment this to export the roadmap to a file
    graph.saveRoadmap("prm_roadmap.txt")
    return graph

# ----------------------------------------
# modify the code below
# ----------------------------------------

# Query phase: Connect start and goal to roadmap and find a path using A*
# (see utils for Value, PriorityQueue, OrderedSet classes that you can use as in project 3)
# The returned path should be a list of configurations, including the local paths along roadmap edges
# Make sure that start and goal configurations are collision-free. Otherwise return None

def find_path(q_start, q_goal, graph):
    path = []
    x1 = q_start[0]
    y1 = q_start[1]
    x2 = q_goal[0]
    y2 = q_goal[1]

    start_n = graph.addVertex([x1, y1])     #Adding start and end goal
    end_n = graph.addVertex([x2, y2])

    all = Roadmap.getVertices(graph)
    number = Roadmap.getNrVertices(graph)
    parent = ["" for i in range(len(all))]
    visited = [False] * len(all)

    distances = []
    distanceg = []

    for i in all:
        if i.id != start_n.id:
            distance_s = distance(i.q, start_n.q)    #For finding the neighbour for the start point
            distances.append(distance_s)

    min_index_s = np.argmin(distances)
    min_s = distances[min_index_s]

    for i in all:
        if i.id != end_n.id:
            distance_g = distance(i.q, end_n.q)
            distanceg.append(distance_g)

    min_index_g = np.argmin(distanceg)         #Index of nearest neighbour
    min_g = distanceg[min_index_g]             #Distance of nearest neighbour

    near_s = all[min_index_s]
    graph.addEdge(start_n, near_s, min_s)     #Connecting start point to the nearest neighbour
    near_g = all[min_index_g]
    graph.addEdge(end_n, near_g, min_g)       #Connecting end point to the nearest neighbour


    # Use the OrderedSet for your closed list
    closed_set = OrderedSet()

    # Use the PriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)

    g = 0
    h = distance(start_n.q, end_n.q)
    f = g + h

    open_set.put(start_n, Value(f=f, g=g))

    while len(open_set) != 0:

        node, value = open_set.pop()
        visited[node.id] = True
        closed_set.add(node)
        # g = value.g
        p_cost = value.g

        if node == end_n:
            last = end_n
            for i in range(len(closed_set)):
                if last != start_n:
                    prev_parent = parent[last.id]
                    path.append(prev_parent.q)
                    last = prev_parent
            break

        else:
            children = RoadmapVertex.getEdges(node)
            for child in children:
                if not visited[child.id]:
                    c_vert = all[child.id]
                    g = sqrt((node.q[0] - c_vert.q[0]) ** 2 + (node.q[1] - c_vert.q[1]) ** 2) + p_cost
                    h = sqrt((end_n.q[0] - c_vert.q[0]) ** 2 + (end_n.q[1] - c_vert.q[1]) ** 2)
                    f = g + h

                    if c_vert not in open_set or c_vert not in closed_set:

                        open_set.put(c_vert, Value(f=f, g=g))
                    elif open_set.has[c_vert]:                      #For backtracking of the nodes to plot the path
                        c_val = open_set.get(c_vert)
                        if c_val.g > g:
                            open_set.remove(c_vert)
                            open_set.put(c_vert, Value(f=f, g=g))
                    parent[child.id] = node

    path = path[::-1]             #We have to reverse the path due to backtracking

    return path


# ----------------------------------------
# below are some functions that you may want to populate/modify and use above
# ----------------------------------------

def nearest_neighbors(graph, q, max_dist=10):
    """
        Returns all the nearest roadmap vertices for a given configuration q that lie within max_dist units
        You may also want to return the corresponding distances
    """

    return None, None


def k_nearest_neighbors(graph, q, K=10):
    """
        Returns the K-nearest roadmap vertices for a given configuration q.
        You may also want to return the corresponding distances
    """

    return None

def distance (q1, q2):
    """
        Returns the distance between two configurations.
        You may want to look at the getRobotPlacement function in utils.py that returns the OBB for a given configuration
    """
    dist = sqrt((q1[0] - q2[0])** 2 + ((q1[1] - q2[1])** 2))

    return dist

def collision(q):
    """
        Determines whether the robot placed at configuration q will collide with the list of AABB obstacles.
    """


    return False


def interpolate (q1, q2, stepsize):
    """
        Returns an interpolated local path between two given configurations.
        It can be used to determine whether an edge between vertices is collision-free.
    """

    return None


if __name__ == "__main__":
    from scene import Scene
    import tkinter as tk

    win = tk.Tk()
    Scene('prm1.csv', disk_robot, (build_roadmap, find_path), win)
    win.mainloop()
