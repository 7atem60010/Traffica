import sys , os
sys.path.append(os.path.join(os.environ.get("SUMO_HOME"), 'tools'))

import traci
import math
import random
from random import randrange

class env():
    def __init__(self):  # vehicle agent is an instant from Auto-vehicle class
        self.ActionsList = ["ChangeLR", "ChangeLF", "fast", "slow", "stop", "DoNothing"]
        self.ActionsDict = {"ChangeLR": 0, "ChangeLF": 1, "fast": 2, "slow": 3, "stop": 4, "DoNothing": 5}
        self.TopLeft = (499.50,499.50)
        self.BotRight = (520.50,520.50)
        self.intersection_Xlen = self.BotRight[0] - self.TopLeft[0]
        self.intersection_Ylen = self.BotRight[1] - self.TopLeft[1]
        self.cells_per_side = 24
        self.cell_len = self.intersection_Ylen/self.cells_per_side
        self.dT = .5 #sec
        self.V_range = list(range(5))
        self.A_range = [-1,0,1]
        # self.agent = vehicleAgent
        # self.ActionsList = ["acc", "dec"]
        # self.ActionsDict = {"acc": 0, "dec": 1}

    def point_to_cell(self,point):
        x,y = point[0],point[1]
        x,y = x-self.TopLeft[0],y-self.TopLeft[1]
        x_i,y_i = x//(self.cell_len),y//(self.cell_len)
        return x_i,y_i


    def get_current_cells(self,agent):
        angle = math.radians(agent.car.angle)
        pos = agent.car.position #position at center of front dumper
        l,w = agent.car.L, self.car.W

        front_left = (pos[0] + w*.5*math.cos(angle+90), pos[1] + w*.5*math.sin(angle+90))
        front_right = (pos[0] + w*.5*math.cos(angle-90), pos[1] + w*.5*math.sin(angle-90))
        pos_back = (pos[0] - l*math.cos(angle), pos[1] - l*math.sin(angle))
        back_left = (pos_back[0] + w*.5*math.cos(angle+90), pos_back[1] + w*.5*math.sin(angle+90))
        back_right = (pos_back[0] + w*.5*math.cos(angle-90), pos_back[1] + w*.5*math.sin(angle-90))

        cell_FL = self.point_to_cell(front_left)
        cell_FR = self.point_to_cell(front_right)
        cell_BL = self.point_to_cell(back_left)
        cell_BR = self.point_to_cell(back_right)
        cells = [cell_BL,cell_BR,cell_FR,cell_FL]

        xmin,xmax = min([cell[0] for cell in cells]),max([cell[0] for cell in cells])
        ymin,ymax = min([cell[1] for cell in cells]),max([cell[1] for cell in cells]) 
        container_cells = [(xmin,ymin),(xmax,ymax)]

        v = agent.car.currentspeed 
        a = agent.car.accel #TODO:
        if v == self.V_range[-1] :
            dc = max(math.floor(v**2/2/a),math.floor(v*self.dT))
        elif v < self.V_range[-1]:
            dc = max(math.floor(v**2/2/a),math.floor(v*self.dT+.5*a*self.dT))
        else:
            raise("Wrong value of V detected at cat",agent.car.ID)
        dc_x = math.ceil(dc * math.cos(angle))
        dc_y = math.ceil(dc * math.sin(angle))
        xmin += dc_x
        xmax += dc_x
        ymin += dc_y
        ymax += dc_y
        desired_cells = [(xmin,ymin),(xmax,ymax)]
        return container_cells,desired_cells


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

    def Car_current_state(self, agent):
        '''
        Traffic state on the lane
        # of cares in the lane
        my speed
        avg speed of the cares in my lane
        '''
        agent.car.UpdateStatus()
        state = ()
        try:
            state = (traci.lane.getLastStepVehicleNumber(agent.car.lane), agent.car.spd, traci.lane.getLastStepMeanSpeed(agent.car.lane))
 #           print(state)
        except:
            pass
        return state

    def getFeasibleActions(self, agent):

        left = 1
        right = -1
        changeLeftPossible = traci.vehicle.couldChangeLane(agent.car.ID, left, state=None)
        changeRightPossible = traci.vehicle.couldChangeLane(agent.car.ID, right, state=None)
        accelerate_possible = True
        try:
            _, leaderDist = traci.vehicle.getLeader(agent.car.ID , 10)
            accelerate_possible = min(agent.car.maxspeed, agent.car.spd + agent.car.maxacc / 2) <= leaderDist
        except:
            pass
        proposedActions = self.ActionsList.copy()
        if changeLeftPossible == False:
            proposedActions.remove("ChangeLF")
        if changeRightPossible == False:
            proposedActions.remove("ChangeLR")
        if accelerate_possible == False:
            proposedActions.remove("fast")
#        print(proposedActions)
        return proposedActions
