import sys , os
sys.path.append(os.path.join(os.environ.get("SUMO_HOME"), 'tools'))

import traci
import math
from random import randrange

class AutoVehicle:
    '''
    This is a class for the main agent in our system - the autonomous vehicle.
    The class mainly used for getting the state of the vehicle and applying actions.
    '''

    def __init__(self, ID):
        ################# Parameters ###################
        self.maxspeed = 4 # We use cell/step convention
        self.dT = 0.5 # This is the step value
        self.accl = 1   # We use cell/step^2 convention
        self.currentspeed = 0  # We use cell/step convention
        self.TopLeft = (499.50,499.50)
        self.BotRight = (520.50,520.50)
        self.intersection_Xlen = self.BotRight[0] - self.TopLeft[0]
        self.intersection_Ylen = self.BotRight[1] - self.TopLeft[1]
        self.cells_per_side = 24
        self.cell_len = self.intersection_Ylen/self.cells_per_side
        self.dT = .5 #sec
        self.V_range = list(range(5))
        self.A_range = [-1,0,1]
        ################## Dicts  ######################
        self.speedDict = {0:0, 1: 1.75 , 2: 2.5 , 3: 5.25 , 4: 7} # Mapping from cell/step to m/s
        self.accSumo = 3.5 # m/s^2

        ################# Intersection dimensions ################
        self.TopLeft = (499.50, 499.50)
        self.BotRight = (520.50, 520.50)
        ################################################

        self.ID = ID
        traci.vehicle.setMaxSpeed(self.ID ,self.maxspeed )
        traci.vehicle.setAccel(self.ID , self.accl)
        self.L, self.W = traci.vehicle.getLength(self.ID), traci.vehicle.getWidth(self.ID)
        self.DoI = 0

    ###################################################################
    def inIntersection(self):
        self.pos = traci.vehicle.getPosition(self.ID) #(x,y)
        print(self.pos)
        if self.pos[0] > self.TopLeft[0] and  self.pos[0] < self.BotRight[0] and self.pos[1] > self.TopLeft[1] and  self.pos[1] < self.BotRight[1]:
            return True
        return False

    ############################# Getter ###############################

    def getCells(self,lane_len,side_cells,intersection_width): #retracted
        x,y = traci.vehicle.getPosition(self.ID)
        x,y = x-lane_len,y-lane_len
        x_i,y_i = x//(intersection_width/side_cells),y//(intersection_width/side_cells)
        x_i,y_i = int(x_i),int(y_i)
        if intersection_width>=x>=0 and intersection_width>=y>=0:
            print(x_i,y_i)
        return x_i,y_i

    def m_sec_2_cell_step(self,v):
        m_step = v*self.dT 
        cell_step = m_step / self.cell_len
        if cell_step > self.V_range[-1]:
            print("Warning, over max speed.")
            cell_step = self.V_range[-1]
        else:
            cell_step = round(cell_step)
        return cell_step

    def getPose(self):
        lane_pose = traci.vehicle.getLanePosition(self.ID)
        print(lane_pose)
        position=traci.vehicle.getPosition(self.ID)
        print(position)
        pos = traci.vehicle.getLateralLanePosition(self.ID)
        print(pos)

        #self.pose = lane_pose + self.route * 500

    def getBoxintersection(self):
        lane_pose = traci.vehicle.getLanePosition(self.ID)
        edge_id  = traci.vehicle.getRoadID(self.ID)
        position = traci.vehicle.getPosition(self.ID)
        print(traci.lane.getWidth(traci.vehicle.getLaneID(self.ID)))
        #print(position)
        #and edge_id in [":0_9" , ":0_1" , ":0_5" ,":0_13" , ":0_8" , ":0_0" , ":0_12" , ":0_4"]
        if lane_pose >0 and lane_pose<20 and position[0]>500 and edge_id in [":0_9" , ":0_1" , ":0_5" ,":0_13" , ":0_8" , ":0_0" , ":0_12" , ":0_4"]:
            print(lane_pose )
            print("I am in intersection" )
            print("I am in position" , round(lane_pose/5 ),"in edge" , edge_id)


    def UpdateStatus(self):
        self.spd = traci.vehicle.getSpeed(self.ID)
        self.accel = traci.vehicle.getAcceleration(self.ID)
        self.lane = traci.vehicle.getLaneID(self.ID)
        self.edge = traci.vehicle.getRoadID(self.ID)
        self.angle = traci.vehicle.getAngle(self.ID)
        self.pos = traci.vehicle.getPosition(self.ID) #(x,y)
        self.currentspeed = self.m_sec_2_cell_step(self.spd)
        traci.vehicle.setSpeed(self.ID, self.speedDict[self.currentspeed])

   ########################################## Actions Functions ##########################################
    def acc(self):
        if self.currentspeed < self.maxspeed:
            traci.vehicle.slowdown(self.ID,self.speedDict[self.currentspeed+1] , self.timestep )
            self.currentspeed += 1
            self.accl = 1

    def dec(self):
        if self.currentspeed > 0 :
            traci.vehicle.slowdown(self.ID,self.speedDict[self.currentspeed-1] , self.timestep )
            self.currentspeed -= 1
            self.accl = -1

    def keepgoing(self):
        self.accl = 0



    ############ end  #################

