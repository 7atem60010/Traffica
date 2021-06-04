import sys, os

sys.path.append(os.path.join(os.environ.get("SUMO_HOME"), 'tools'))
import numpy as np
import traci
import math
import random
from random import randrange


class env():
    def __init__(self):  # vehicle agent is an instant from Auto-vehicle class
        # self.ActionsList = ["ChangeLR", "ChangeLF", "fast", "slow", "stop", "DoNothing"]
        # self.ActionsDict = {"ChangeLR": 0, "ChangeLF": 1, "fast": 2, "slow": 3, "stop": 4, "DoNothing": 5}
        self.TopLeft = (499.50, 499.50)
        self.BotRight = (520.50, 520.50)
        self.intersection_Xlen = self.BotRight[0] - self.TopLeft[0]
        self.intersection_Ylen = self.BotRight[1] - self.TopLeft[1]
        self.cells_per_side = 24
        self.cell_len = self.intersection_Ylen / self.cells_per_side
        self.dT = .5  # sec
        self.V_range = list(range(5))
        self.A_range = [-1, 0, 1]

        self.ActionsList = ["acc", "dec", "keep_going"]
        self.ActionsDict = {"acc": 0, "dec": 1, "keep_going": 2}

        self.intersectionAgentList = []  # List of Cars in the intersection
        self.overLap = []
        self.states = {}

    def updateIntersectionAgents(self, existing_agents):
        self.intersectionAgentList = []
        for car in existing_agents:
            try:
                if car.inIntersection():
                    self.intersectionAgentList.append(car)
                    car.UpdateStatus()
            except:
                pass

    def updateStates(self):
        self.states = {}
        for car in self.intersectionAgentList:
            cont_cells, desired_cells = car.cont_cells, car.desired_cells
            velocity = car.currentspeed
            queue = car.queuelen
            self.states[car.ID] = (cont_cells, velocity, desired_cells, queue)

    def is_overlap(self, agent_asking):
        car_id = agent_asking.ID
        joint_agents = []
        R1 = [agent_asking.desired_cells[0][0], agent_asking.desired_cells[1][0], agent_asking.desired_cells[0][1],
              agent_asking.desired_cells[1][1]]
        for agent in self.intersectionAgentList:
            if agent.ID != car_id:
                R2 = [agent.desired_cells[0][0], agent.desired_cells[1][0], agent.desired_cells[0][1],
                      agent.desired_cells[1][1]]
                if self.isRectangleOverlap(R1, R2):
                    joint_agents.append(agent)
        return joint_agents

    def isRectangleOverlap(self, R1, R2):
        if (R1[0] >= R2[2]) or (R1[2] <= R2[0]) or (R1[3] <= R2[1]) or (R1[1] >= R2[3]):
            return False
        else:
            return True

    def doOverlap(self, l1, r1, l2, r2):  # geeks 4 geeks

        # To check if either rectangle is actually a line
        # For example  :  l1 ={-1,0}  r1={1,1}  l2={0,-1}  r2={0,1}

        if (l1[0] == r1[0] or l1[1] == r2[1] or l2[0] == r2[0] or l2[1] == r2[1]):  # geeks 4 geeks
            # the line cannot have positive overlap
            return False

        # If one rectangle is on left side of other
        if (l1[0] >= r2[0] or l2[0] >= r1[0]):
            return False

        # If one rectangle is above other~
        if (l1[1] <= r2[1] or l2[1] <= r1[1]):
            return False

        return True

    def get_desired_cells(self):
        pass

    def get_global_reward(self):
        print("Global Reward")

    def get_agent_individual_reward(self, vehicle):
        reward = -1 * (vehicle.get_time_step_distance()/vehicle.speedDict[vehicle.currentspeed] -
                       vehicle.get_time_step_distance() / vehicle.maxspeed)
        if np.isnan(reward):
            reward = 0
        elif np.isinf(reward):
            reward = -2000
        return reward

    def get_agent_coordinated_reward(self, cardIds):
        print("Coordinated agents reward")



