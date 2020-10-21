# Project 2
## Project Name : Local Navigation with TTC Forces
## Name of the students : Prajval Vaskar 
In this project, we used predictive TTC forces approach for local navigation.


For all condition we have to limit forces to 10 in x and y direction so the coodntion is applied.
In all condition sensing radius is 10

## Paramater for each simulation scenarios
For isotropic time-to-collision there are two condition 

1. Error, e = 0
This will react as normal TTC forces because the error is zero.(Vanilla TTC forces)
Parameters for all the scenerios like 3_agents, 8_agents, crossing_agents
Sensing Radius = 10
Time horizon as given in assignment = 4s 

2. Error, e = 0.2
This is isotropic time to collision in which error between true and sensed relative velocities smaller than or equal to e i.e 0.2
Parameters for all the scenerios like 3_agents, 8_agents, crossing_agents
Sensing Radius = 10
Time horizon as given in assignment 



Extra Credits
1. Implement powerlaw model in which the avoidance force is different than TTC approach. 
Parameters used for powerlaw model 
These parameters are based on simulation results,
tau0 is given as 3s,
m is generally taken as 2
1. tau0 = 3
2. m = 2
3. k = 2 This is tuning parameter, i got better simulation result with 2 so i finalized k = 2

Powerlaw model applied with error e=0 and e= 0.2 and tau0=3 in both condition
 	

2.Modify sensor error
Here in this case, we have used the value of n which will be sampled randomly from disk centered at 0 having radius v which 0.1
and two simulations were done (e=0.2)
I have used powerlaw model with sensing error and TTC approach with sensing error 


3.Implement adversarial uncertainty model for both TTC and powerlaw.
Here, for ttc collision and repulsive forces relative velocity will be changed from Vab to V_hatab - e(Xab/mod(Xab))
Simulation was done using error 0.2 and v(radius) 0.1.

Total there are 7 folders according to different scenarios
1 Simulation for TTC aproach for Error 0
2 Simulation for TTC aproach for Error 0.2
3 Simulation using powerlaw model for Error 0	
4 Simulation using powerlaw model for Error 0.2
5 Simulation using model sensing error with Error 0.2 and radius 0.1 (powerlaw)
6 Simulation using model sensing error with Error 0.1 and radius 0.1 (TTC)
7 Simulation using adversarial uncertainty model using powerlaw approach with Error 0.2 and radius 0.1
8 Simulation using adversarial uncertainty using TTC approach using error 0.2 and radius 0.1


# 3 Agents
 ![](https://github.com/Praj390/CPSC8810_motion_planning/blob/master/TTC%20Forced%20based%20Navigation/3_agent.gif)

# 8 agents

![](https://github.com/Praj390/CPSC8810_motion_planning/blob/master/TTC%20Forced%20based%20Navigation/8_agent.gif)

# crossing agents

![](https://github.com/Praj390/CPSC8810_motion_planning/blob/master/TTC%20Forced%20based%20Navigation/crossing_agent.gif)
