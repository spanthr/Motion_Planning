# agent.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
#
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#

# Project 2 by Prajval Vaskar
import numpy as np
from math import sqrt
import random

class Agent(object):


    def __init__(self, csvParameters, ksi=0.5, dhor = 10, timehor=4, goalRadiusSq=1, maxF = 10):
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
        self.goalRadiusSq = goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.ksi = ksi # the relaxation time used to compute the goal force
        self.dhor = 10 # the sensing radius
        self.timehor = timehor # the time horizon for computing avoidance forces
        self.F = np.zeros(2) # the total force acting on the agent
        self.maxF = maxF # the maximum force that can be applied to the agent
        #self.e = 0                                      # For ttc approach error e is 0 and for uncertaintly aware approach error is 0.2
        self.e = 0.2


    def computeForces(self, neighbors=[]):
        """
            Your code to compute the forces acting on the agent.
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors
        """

        Favoid = []
        tau0 = 3
        m = 2
        k = 2

        for agent in neighbors:

            Force = (self.gvel - self.vel)/self.ksi

            if agent != self:
                distance = sqrt(((agent.pos[0]-self.pos[0])**2) + (agent.pos[1] - self.pos[1])**2)  #Calculating distance between neighbouring agent


                if distance <= self.dhor:                                    # Condition for neighbour agents to act when it comes in contact with circle of sensing radius

                    sample = random.randint(-10,10)
                    sample_n = sample/100
                                                                            # Compute time for collision between the agents
                    rad = agent.radius + self.radius                        # Combined radius
                    w = self.pos - agent.pos                                # Relative displacement between the agents
                    c = w.dot(w) - rad*rad

                # For TTC approach use following rel_vel
                    rel_vel = self.vel - agent.vel                             # Relative velocity between the agents

                # For model sensing error we have to use following rel_vel
                    #rel_vel = self.vel - agent.vel + sample_n

                #For adversarial uncertainty model for the TTC forces and/or the PowerLaw use the following relative velocity
                    #rel_vel = ((self.vel - agent.vel) + sample_n) - self.e*((w)/sqrt(np.dot((w),(w)))) #Uncomment this for adversarial uncertainty

                # For choosing the powerlaw or ttc you can select avoidance force Favoid accordingly from below formulaes

                    a = rel_vel.dot(rel_vel) - (self.e)**2                               # Isotropic TTC collision condtion.
                    b = w.dot(rel_vel) - (self.e*rad)                                    # For ttc calculation the error will be 0 and for the isotropic collision the error e is 0.2
                    d = b*b - a *c

                    if d >= 0 and b<0:
                        tau = c/(-b + sqrt(d))     #Smallest root
                        n = (w + rel_vel*tau)
                        D = sqrt((n).dot(n))
                        N = n/D

                    #Avoidance force for ttc forced based approach
                        F = ((max(self.timehor-tau,0))*N/tau)

                    #Avoidance force for powerlaw appraoch
                        #F = ((k * np.exp((-tau)/tau0))/tau**(m+1))*(m+(tau/tau0))*((w + rel_vel*tau)/sqrt(d))

                        Favoid.append(F)
                        if tau < 0:                 #Agents are colliding
                            tau = 0

                    else :
                        tau = float('inf')
                else:
                    pass
                    tau = float('inf')


        Force += sum(Favoid)

        if not self.atGoal:
            self.F = Force


    def update(self, dt):
        """
            Code to update the velocity and position of the agents.
            as well as determine the new goal velocity
        """

        if not self.atGoal:
            if self.F[0] > self.maxF:              # For max force condition
                self.F[0] = self.maxF
            if self.F[1] > self.maxF:
                self.F[1] = self.maxF
            if self.F[0] < -self.maxF:
                self.F[0] = -self.maxF
            if self.F[1] < -self.maxF:
                self.F[1] = -self.maxF


            #Update velocity and position
            self.vel += self.F*dt
            self.pos += self.vel*dt          #update the position

            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq:
                self.atGoal = True  # goal has been reached
            else:
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed
