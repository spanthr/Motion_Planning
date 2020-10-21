# agent.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
#
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#

## Name of the student :Prajval Vaskar & Shaurya Panthri
# Importing packages.
import matplotlib.pyplot as plt
import numpy as np
from math import sqrt

class Agent(object):



    def __init__(self, csvParameters, dhor = 8, goalRadiusSq=1):
        """
            Takes an input line from the csv file,
            and initializes the agent
        """
        self.id = int(csvParameters[0]) # the id of the agent
        self.gid = int(csvParameters[1]) # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])]) # the position of the agent
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])]) # the goal of the agent
        self.prefspeed = float(csvParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(sqrt(self.gvel.dot(self.gvel )))*self.prefspeed
        self.maxspeed = float(csvParameters[7]) # the maximum sped of the agent
        self.radius = float(csvParameters[8]) # the radius of the agent
        self.goalRadiusSq =goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.dhor = dhor # the sensing radius
        self.vnew = np.zeros(2) # the new velocity of the agent




    def computeNewVelocity(self, neighbors=[]):
        """
            Your code to compute the new velocity of the agent.
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors.
            The code should set the vnew of the agent to be one of the sampled admissible one. Please do not directly set here the agent's velocity.
        """
        # Scaling values
        alpha = 1
        beta = 1
        gamma = 2
        #Assigning empty arrays
        cost_function = []
        neighbour_agents = []
        ####### Uniform sampling N velocities from the admissible velocity space ##########################################

        n = 200        #Different number of Candidate Velocities.
        #n = 200       #Uncomment this portion of code for crosiing agents.
        r = np.sqrt(np.random.uniform(0,4,n))
        theta = np.pi*np.random.uniform(0,2,n)
        vel_x = r*np.cos(theta)
        vel_y = r*np.sin(theta)
        sample_vel = np.array([vel_x,vel_y])   #Array of sampling velocities
        sample_vel = sample_vel.T
        for vcandidate in sample_vel:
            ttc = []
            tc_min = []
            for agent in neighbors:
                if agent != self:
                    distance = sqrt(((agent.pos[0]-self.pos[0])**2) + (agent.pos[1] - self.pos[1])**2)  #Calculating distance between neighbouring agent
                    if distance <= self.dhor:                                    # Condition for neighbour agents to act when it comes in contact with circle of sensing radius
                        neighbour_agents.append(agent)
                                                                                # Compute time for collision between the agents
                        rad = agent.radius + self.radius                        # Combined radius
                        w = agent.pos - self.pos                                # Relative displacement between the agents
                        c = w.dot(w) - rad*rad
                        rel_vel = agent.vel - vcandidate                        # Relative velocity between the agents
                        a = rel_vel.dot(rel_vel)                                # Quadtratic equation for calculating time for collision
                        b = w.dot(rel_vel)
                        d = b*b - a *c
                        if d >= 0 and b<0:
                            tau = c/(-b + sqrt(d))     #Smallest root
                            if tau < 0:
                                tau = 0                 #Agents are colliding
                        else :
                            tau = float('inf')
                    else:
                        pass
                        tau = float('inf')
                    ttc.append(tau)

            tc = min (ttc)



            cost_fun_initial = (alpha*(sqrt((vcandidate - self.gvel).dot((vcandidate - self.gvel))))) + (beta* (sqrt((vcandidate - self.vel). dot((vcandidate - self.vel)))))      #Cost  function as given in the assignment
                                                                                #Using scaling constant with their default values
            if tc != 0:                                                         # when agents are coliding then tc will be zero,
                    cost_3 = gamma / tc                                         # Adding the third term to evaluate the collision cost term
                    cost_function.append(cost_fun_initial+cost_3)               #Appending the cost terms into cost function array
            else:
                cost_function.append(float('inf'))                              # When agents collide, cost term will give 'inf' values
        min_cost_index = np.argmin (cost_function)
        vnewest = np.array(sample_vel[min_cost_index])                          #New velocity to avoid collision in an array [velocity in x, velocity in y]


        if not self.atGoal:
            self.vnew[:] = vnewest[:]   # here I just set the new velocity to be the goal velocity


    def update(self, dt):
        """
            Code to update the velocity and position of the agent
            as well as determine the new goal velocity
        """
        if not self.atGoal:
            self.vel[:] = self.vnew[:]
            self.pos += self.vel*dt   #update the position

            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq:
                self.atGoal = True  # Goal has been reached
            else:
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed
