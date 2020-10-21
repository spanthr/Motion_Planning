# Project 1
## Project Name : Sampling Based Local Navigation 
## Name of the students : Prajval Vaskar & Shaurya Panthri
In this project, we used sampling-based velocity approach for local navigation.

## Paramater for each simulation scenarios

1. 3_agents scenario:
	alpha = 1  
	beta =  1
	gamma = 2
	Number of samples = 300
	Sensing Radius = 8
2. 8_agents scenario:
	alpha = 1  
	beta =  1
	gamma = 2
	Number of samples = 200
	Sensing Radius = 8
3. Multiagents scenario:
	alpha = 1  
	beta =  1
	gamma = 2
	Number of samples = 200
	Sensing Radius = 8

 	1.The scaling factor alpha beta gamma are given in the assignment. So, we used the same parameters.

 	2.Number of sampling is choosen by simulatng the scenarios under different sampling number
 and it is finalised based on computing speed and motion of the agent whether it is smooth or not and also time required to cross the agents.
 For the multicrossing agents scenario, we used sampling number as 200 as given in the assignment.
 
	3.We tried to reduce the number of agents in the multicrossing agents scenario in order to reduce the crossing time and smooth motion.  


	4.When we increased the value of aplha the reaction of the agents to avoid obstacles was less and therefore with alpha=10 and more they collided.

	5.On increasing the value of scaling parameter beta the agents moved in the y-direction. The collision was avoided succesfully but the path
traced by the agents was curved and the motion was not smooth. At very high values the motions were intermittent.

	6.On increasing the value of scaling factor gamma the agents tend to go slow as it increased the safety factor. At higher values of gamma the agents
get confused and interrupt their respective path, decreasing the smoothness of the motion.


### 3 agents
![ ](https://github.com/Praj390/CPSC8810_motion_planning/blob/master/Sampling%20based%20Local%20Navigation/3_agent.gif)

### 8 agents

![ ](https://github.com/Praj390/CPSC8810_motion_planning/blob/master/Sampling%20based%20Local%20Navigation/8_agent.gif)

### crossing agents

![ ](https://github.com/Praj390/CPSC8810_motion_planning/blob/master/Sampling%20based%20Local%20Navigation/crossing_agent.gif)


	




