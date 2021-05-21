import sys , os
sys.path.append(os.path.join(os.environ.get("SUMO_HOME"), 'tools'))

import traci
import math
import random
from random import randrange

class env():
    def __init__(self):  # vehicle agent is an instant from Auto-vehicle class
        # self.ActionsList = ["ChangeLR", "ChangeLF", "fast", "slow", "stop", "DoNothing"]
        # self.ActionsDict = {"ChangeLR": 0, "ChangeLF": 1, "fast": 2, "slow": 3, "stop": 4, "DoNothing": 5}
        self.TopLeft = (499.50,499.50)
        self.BotRight = (520.50,520.50)
        self.intersection_Xlen = self.BotRight[0] - self.TopLeft[0]
        self.intersection_Ylen = self.BotRight[1] - self.TopLeft[1]
        self.cells_per_side = 24
        self.cell_len = self.intersection_Ylen/self.cells_per_side
        self.dT = .5 #sec
        self.V_range = list(range(5))
        self.A_range = [-1,0,1]

        self.ActionsList = ["acc", "dec", "keep_going"]
        self.ActionsDict = {"acc": 0, "dec": 1 , "keep_going" : 2}

        self.intersectionAgentList = [] # List of Cars in the intersection
        self.overLap = []
        self.states = {}

    def updateIntersectionAgents(self, existing_agents):
        self.intersectionAgentList = []
        for agent in existing_agents:
            try:
                if agent.car.inIntersection():
                    self.intersectionAgentList.append(agent)
                    agent.car.UpdateStatus()
            except:
                pass




    # def is_overlap(self, agent):
    #     agent => 5
    #     if self.overLap[agent.id] :
    #         return false
    #     else:
    #         1,2
    #         return []

    # def is_intersect(self):
    #     pass #TODO:

    def updateStates(self):
        self.states = {}
        for agent in self.intersectionAgentList:
            cont_cells, desired_cells = agent.car.cont_cells, agent.car.desired_cells
            velocity = agent.car.currentspeed
            queue = agent.car.queuelen
            self.states[agent.car.ID] = (cont_cells, velocity, desired_cells, queue)
            #print((cont_cells,velocity,desired_cells,queue))
        #if len(self.states): print(self.states)

    def get_overlapping_cars(self):
        for i,car1 in enumerate(self.intersectionAgentList):
            for j in range(i+1,len(self.intersectionAgentList)):
                car2 = self.intersectionAgentList[j]
                dc1 = self.states[car1][1]
                dc2 = self.states[car2][1]
                if self.doOverlap(dc1[0],dc1[1],dc2[0],dc2[1]):
                    self.append(car1,car2)

    def is_overlap(self, agent_asking):
        car_id = agent_asking.car.ID
        joint_agents = []
        l1 = (agent_asking.car.desired_cells[0][0], agent_asking.car.desired_cells[1][0])
        r1 = (agent_asking.car.desired_cells[0][1], agent_asking.car.desired_cells[1][1])
        for agent in self.intersectionAgentList:
            if agent.car.ID != car_id:
                l2 = (agent.car.desired_cells[0][0], agent.car.desired_cells[1][0])
                r2 = (agent.car.desired_cells[0][1], agent.car.desired_cells[1][1])
                if self.doOverlap(l1, r1, l2, r2):
                    joint_agents.append(agent.car.ID)
        return joint_agents

    def doOverlap(self,l1, r1, l2, r2): #geeks 4 geeks
        
        # To check if either rectangle is actually a line
        # For example  :  l1 ={-1,0}  r1={1,1}  l2={0,-1}  r2={0,1}
        
        if (l1[0] == r1[0] or l1[1] == r2[1] or l2[0] == r2[0] or l2[1] == r2[1]): #geeks 4 geeks
            # the line cannot have positive overlap
            return False
        
        # If one rectangle is on left side of other
        if(l1[0] >= r2[0] or l2[0] >= r1[0]):
            return False
    
        # If one rectangle is above other~
        if(l1[1] <= r2[1] or l2[1] <= r1[1]):
            return False
    
        return True



    def get_desired_cells(self):
        pass

    def Reward(self, agent):
        try:
            agent.car.UpdateStatus()
            agent.car.reward = 0
            if agent.car.isStop() == False:
                agent.car.reward += 1

            agent.car.reward -= agent.car.wait_time()

            agent.car.reward += (agent.car.spd / agent.car.maxspeed) * 10

            nonlane_vehicle_num = traci.edge.getLastStepVehicleNumber(
                agent.car.edge) - traci.lane.getLastStepVehicleNumber(agent.car.lane)
            if traci.lane.getLastStepVehicleNumber(agent.car.lane) == max(
                    traci.lane.getLastStepVehicleNumber(agent.car.lane), nonlane_vehicle_num):
                agent.car.reward += traci.lane.getLastStepVehicleNumber(agent.car.lane) - nonlane_vehicle_num
            else:
                agent.car.reward += traci.lane.getLastStepVehicleNumber(agent.car.lane) - nonlane_vehicle_num
        except:
            pass
        # print("Reward :", self.agent.reward)



