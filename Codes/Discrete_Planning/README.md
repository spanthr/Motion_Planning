# Project 3
## Project Name : Discrete Planning
## Name of the students : Shaurya Panthri and Prajval Vaskar 
In this project, we used A star algorithm to plan the discrete motion of a simple car. For simplicity,
state space is 3 dimensional and is given by an x position, y position and an orinetation(theta)
and speed is assumed as uniform and fixed.

There are three assumption in the given code.
1. Car is moving on 2D grid and orientation can assume four distinct direction: 
 0 : North
 1 : West
 2 : South
 3 : East

2. Three possible that car can take:
  Action = [Right , Forward , Left]

3. Heuristic for the Astar search is the number of steps to the 2D goal cell. 

For three action we used array action = [-1 0 1]
 
1.We used collections container database for the orientation of the car. deque is list like coontainer
that we used to rotate and index the child according to its parrents orientation. 
2.First we find parrent from adding the initial point into open set and popping it out as a variable node. Then for that node
we found children using action array. 
3.There are three action left right and forward and there will 
possible three children of a given node considering no u turn is allowed in 3D structure. Then we created ceque tuple of 
direction(North, west, south, east).
4.For the indexing of child we referrend the orientation(theta) of the parrent. Using action array we found possioble index
of child. As a result we found possible children and its orientation.


We used pseudocode as a reference that was given in the lectures slides.
Priority queue data structure is very helpful to select the child with low f value among all children.
So, the open_set is made from priority queue data sturcture which helps to finalise the child of each parrent.

Three cost function is used to test the code.[1,1,1],[1,1,10],[10,1,1]
It is giving the path as per expectation of astar algorith.
Currently the cost is set as [1,1,10]. As a result, astar generates longer path as from (4,3,0) to (2,0,1)
there is left turn and its cost is 10 so that results in higher g and f values. so it is taking straight path and taking respective
right turns to reach the goal
 
  
For Dijkstra, same logic is used for orinetation of the parrent and child. Dijkstra's algorithm is an algorithm that is used to 
solve the shortest distance problem. Initially all the distances apart from start is set to infinity, after that we check whether 
the goal and node points are same or not(to check whether the goal is reached). If it not equal then we add that node into open_set. 
The condition when open_set is not empty is required to pop the node with less distance value and the same node is added to the closed_set. 
Neighbour of v is checked which are not in closed_set and its distance is calculated and check the distance with distance of v and updated 
the distance accordingly.The loop continues till the node reached the goal and shortest path is achieved. 
