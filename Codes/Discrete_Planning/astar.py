# astar.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
#
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#Name of the students : Shaurya Panthri and Prajval Vaskar
import collections
from collections import deque
import numpy as np
direction=(0,1,2,3)
# Compute the optimal path from start to goal.
# The car is moving on a 2D grid and
# its orientation can be chosen from four different directions:
forward = [[-1,  0], # 0: go north
           [ 0, -1], # 1: go west
           [ 1,  0], # 2: go south
           [ 0,  1]] # 3: go east

# The car can perform 3 actions: -1: right turn and then move forward, 0: move forward, 1: left turn and then move forward
action = [-1, 0, 1]   #Action : -1 for right , 0 for forward, 1 for left turn
action_name = ['R', 'F', 'L']
# cost = [1, 1, 1] # corresponding cost values
cost = [1, 1, 10]
# cost = [10, 1, 1]
# GRID:
#     0 = navigable space
#     1 = unnavigable space
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = (4, 3, 0) # (grid row, grid col, orientation)

goal = (2, 0, 1) # (grid row, grid col, orientation)


heuristic = [[2, 3, 4, 5, 6, 7], # Manhattan distance
        [1, 2, 3, 4, 5, 6],
        [0, 1, 2, 3, 4, 5],
        [1, 2, 3, 4, 5, 6],
        [2, 3, 4, 5, 6, 7]]

from utils import (Value, OrderedSet, PriorityQueue)

"""
Two data structures are provided for your open and closed lists:

 1. OrderedSet is an ordered collection of unique elements.
 2. PriorityQueue is a key-value container whose `pop()` method always pops out
    the element whose value has the highest priority.

 Common operations of OrderedSet, and PriorityQueue
   len(s): number of elements in the container s
   x in s: test x for membership in s
   x not in s: text x for non-membership in s
   s.clear(): clear s
   s.remove(x): remove the element x from the set s;
                nothing will be done if x is not in s

 Unique operations of OrderedSet:
   s.add(x): add the element x into the set s
   s.pop(): return and remove the LAST added element in s;

 Example:
   s = Set()
   s.add((0,1,2))    # add a triplet into the set
   s.remove((0,1,2)) # remove the element (0,1,2) from the set
   x = s.pop()

 Unique operations of PriorityQueue:
   PriorityQueue(order="min", f=lambda v: v): build up a priority queue
       using the function f to compute the priority based on the value
       of an element
   s.put(x, v): add the element x with value v into the queue
                update the value of x if x is already in the queue
   s.get(x): get the value of the element x
            raise KeyError if x is not in s
   s.pop(): return and remove the element with highest priority in s;
            raise IndexError if s is empty
            if order is "min", the element with minimum f(v) will be popped;
            if order is "max", the element with maximum f(v) will be popped.
 Example:
   s = PriorityQueue(order="min", f=lambda v: v.f)
   s.put((1,1,1), Value(f=2,g=1))
   s.put((2,2,2), Value(f=5,g=2))
   x, v = s.pop()  # the element with minimum value of v.f will be popped
"""

# ----------------------------------------
# modify the code below
# ----------------------------------------
def compute_path(grid,start,goal,cost,heuristic):

    # Use the OrderedSet for your closed list
    closed_set = OrderedSet()

    # Use thePriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)


    # Keep track of the parent of each node. Since the car can take 4 distinct orientations,
    # for each orientation we can store a 2D array indicating the grid cells.
    # E.g. parent[0][2][3] will denote the parent when the car is at (2,3) facing up
    parent = [[[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]]

    # The path of the car
    path =[['-' for row in range(len(grid[0]))] for col in range(len(grid))]

    x = start[0]
    y = start[1]
    theta = start[2]
    h = heuristic[x][y]
    g = 0
    f = g+h
    children = []
    # print(start)

    open_set.put(start, Value(f=f,g=g))

    while open_set:
        node,new_cost = open_set.pop()
        print('The current node is:', node)
        print('==============================')
        if node == goal:
            return path, closed_set
        closed_set.add(node)

        row_grid=len(grid)-1   #4   Boundry of the grids
        column_grid=(len(grid[0]))-1 #5
        x1=node[0]    #Parrent x co-ordinate
        y1=node[1]    # Parrent y co-ordinate
        theta1=node[2]  # Parrent orientation
        f1=new_cost.f   #Parent total cost
        h=heuristic[x1][y1] #Heuristic of parrent
        g1=new_cost.g   #Path cost of parrent
        neighbor=[]
        f12 =[]
        h12=[]
        g12=[]
        for i in range(len(action)):
            temp_dir=collections.deque(direction)   # creating a deque tuple            # 0 N   :::: 1 west     2 south :::: east 3
            index=theta1                                        # orientation of the Parent
            child_index=action[i]                   # possible index of the child
            n=-1*child_index
            temp_dir.rotate(n)
            child_theta=temp_dir[theta1]        # using the index of the parent to find out the current orientation of the child

            """ calculating the distances of children"""

            x_pos=node[0]+forward[child_theta][0]               # this gives the position of x,y of the child
            y_pos=node[1]+forward[child_theta][1]
            pos=(x_pos,y_pos,child_theta)
            g1 = []
            if(pos[0]>row_grid or pos[1]>(column_grid) or pos[1]<0 or pos[0]<0):
                continue
            elif(grid[pos[0]][pos[1]] == 1):
                continue
            else:
                neighbor.append(pos)
                h1=heuristic[x_pos][y_pos]       # heuristic of neighbor that is dist from the goal
                g1=new_cost.g+cost[i]            # path function
                print(new_cost.g)
                print("---------------------")
                print(cost[i])
                f1=g1+h1                           # total cost
                f12=np.append(f12,f1)
                g12=np.append(g12,g1)
                h12=np.append(h12,h1)



            gmin=min(g12)
            posi=np.argmin(g12)
            c=0


            for neigh in neighbor:                                      #Condition for child which is closest to the goal
                if neigh not in open_set or neigh not in closed_set:
                    open_set.put(neigh,Value(f=f12[c],g=g12[c]))
                    # print(open_set.get(neigh).g)
                    print(neigh)
                    print(open_set.get(neigh).g)


                elif neigh in open_set and gmin< open_set.get(neigh).g:
                    open_set.put(neigh,Value(f=f12[c],g=g12[c]))
                    # print("ggggggggg")
                c=c+1
            c=0




    return path, closed_set



    # your code: implement A*

    # Initially you may want to ignore theta, that is, plan in 2D.
    # To do so, set actions=forward, cost = [1, 1, 1, 1], and action_name = ['U', 'L', 'R', 'D']
    # Similarly, set parent=[[' ' for row in range(len(grid[0]))] for col in range(len(grid))]




if __name__ == "__main__":
    path,closed=compute_path(grid, init, goal, cost, heuristic)

    for i in range(len(path)):
        print(path[i])

    print("\nExpanded Nodes")
    for node in closed:
        print(node)

"""
To test the correctness of your A* implementation, when using cost = [1, 1, 10] your code should return

['-', '-', '-', 'R', 'F', 'R']
['-', '-', '-', 'F', '-', 'F']
['*', 'F', 'F', 'F', 'F', 'R']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-']

In this case, the elements in your closed set (i.e. the expanded nodes) are:
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 4, 3)
(1, 3, 0)
(2, 5, 3)
(0, 3, 0)
(0, 4, 3)
(0, 5, 3)
(1, 5, 2)
(2, 5, 2)
(2, 4, 1)
(2, 3, 1)
(2, 2, 1)
(2, 1, 1)
(2, 0, 1)

"""
