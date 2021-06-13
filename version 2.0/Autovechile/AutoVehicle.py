import sys , os
sys.path.append(os.path.join(os.environ.get("SUMO_HOME"), 'tools'))
import numpy as np
import traci
import math
from collections import deque
from random import randrange

class AutoVehicle:
    '''
    This is a class for the main agent in our system - the autonomous vehicle.
    The class mainly used for getting the state of the vehicle and applying actions.
    '''

    def __init__(self, ID):
        ################# Parameters ###################
        self.dT = 0.5 # This is the step value
        self.accl = 1   # We use cell/step^2 convention
        self.currentspeed = 0  # We use cell/step convention
        self._pos = (0, 0)
        self._step = -1
        self.TopLeft = (143.6,143.6 )
        self.BotRight = (156.40, 156.40)
        self.intersection_Xlen = self.BotRight[0] - self.TopLeft[0]
        self.intersection_Ylen = self.BotRight[1] - self.TopLeft[1]
        self.cells_per_side = 24
        self.cell_len = self.intersection_Ylen/self.cells_per_side
        self.dT = .5 #sec
        self.V_range = list(range(5))
        self.A_range = [-1,0,1]
        self.queuelen = 0
        ################## Dicts  ######################
        self.speedDict = {0:0, 1: 1.75 , 2: 3.5 , 3: 5.25 , 4: 7} # Mapping from cell/step to m/s
        self.maxspeed = 25.2 # We use m/s convention
        self.accSumo = 3.5 # m/s^2
        # self.set
        ################# Intersection dimensions ################
        ################################################

        self.isPrevIndividual = True
        self._hot_time = -1
        self.ID = ID
        traci.vehicle.setMaxSpeed(self.ID ,self.maxspeed )
        traci.vehicle.setAccel(self.ID , self.accl)
        self.L, self.W = traci.vehicle.getLength(self.ID), traci.vehicle.getWidth(self.ID)
        self.DoI = 0
        self.max_len_old_keep = 70
        self.old_states = deque([], maxlen=self.max_len_old_keep)


        ###################################################################
    def inRealIntersection(self):
        TopLeft = (143.6 + self.extenstion,143.6 + self.extenstion)
        BotRight = (156.40- self.extenstion, 156.40- self.extenstion)
        self.pos = traci.vehicle.getPosition(self.ID) #(x,y)
        self.inter = False
        if self.pos[0] > self.TopLeft[0] and  self.pos[0] < self.BotRight[0] and self.pos[1] > self.TopLeft[1] and  self.pos[1] < self.BotRight[1]:
            self.inter = True
        return self.inter

    def inIntersection(self):
        self.pos = traci.vehicle.getPosition(self.ID) #(x,y)
        self.inter = False
        if self.pos[0] > self.TopLeft[0] and  self.pos[0] < self.BotRight[0] and self.pos[1] > self.TopLeft[1] and  self.pos[1] < self.BotRight[1]:
            self.inter = True
        if not self.inter:
            self.lane = traci.vehicle.getLaneID(self.ID)
            self.queuelen = traci.lane.getLastStepVehicleNumber(self.lane)
        return self.inter

    ############################# Getter ###############################

    def add_current_state(self, current_state):
        self.old_states.append(f"{current_state}")

    def is_potential_dead_lock(self):
        return len(set(self.old_states)) == 1 and len(list(self.old_states)) > 69

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
            #print("Warning, over max speed.")
            cell_step = self.V_range[-1]
        elif cell_step < self.V_range[0]:
            # print("Warning, negative speed")
            cell_step = self.V_range[0]
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
        # self.old_pos = self.pos
        self.pos = traci.vehicle.getPosition(self.ID) #(x,y)
        self.currentspeed = self.m_sec_2_cell_step(self.spd)
        self.get_current_cells()
        #bad hot fix, separate pos variable
        # traci.vehicle.setSpeed(self.ID, self.speedDict[self.currentspeed])

    def hot_update_pos(self):
        if self._hot_time == traci.simulation.getTime():
            return
        self._step = traci.simulation.getTime()
        self._pos_old = self._pos
        self._pos = traci.vehicle.getPosition(self.ID)

    def get_time_step_distance(self):


        distance = np.sqrt((self._pos[0] - self._pos_old[0])**2+(self._pos[1] - self._pos_old[1])**2)
        return distance

    def point_to_cell(self,point):
        x,y = point[0],point[1]
        x,y = x-self.TopLeft[0],y-self.TopLeft[1]
        x_i,y_i = x//(self.cell_len),y//(self.cell_len)
        return x_i,y_i

    def get_current_cells(self):
        angle_d = (-self.angle + 90)% 360
        angle = math.radians(angle_d)
        #print(angle_d)
        pos = self.pos #position at center of front dumper
        l, w = self.L, self.W

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
        _24 = self.cells_per_side
        xmin,xmax=min([cell[0] for cell in cells]), max([cell[0] for cell in cells])
        ymin,ymax = min([cell[1] for cell in cells]), max([cell[1] for cell in cells])
        # container_cells = [(xmin,ymin),(xmax,ymax)]

        xmin_,xmax_ = max(0, min([cell[0] for cell in cells])), min(_24, max([cell[0] for cell in cells]))
        ymin_,ymax_ = max(0, min([cell[1] for cell in cells])), min(_24 ,max([cell[1] for cell in cells]) )
        container_cells = [(xmin_,xmax_),(ymin_,ymax_)]

        v = self.currentspeed
        # a = agent.car.accel #TODO:
        a = 1
        if v == self.V_range[-1] :
            dc = max(math.ceil(v**2/2/a),math.ceil(v*self.dT))
        elif v < self.V_range[-1]:
            dc = max(math.ceil(v**2/2/a),math.ceil(v*self.dT+.5*a*self.dT))
        else:
            raise("Wrong value of V detected at cat",self.ID)
        if self.speedDict[v] <= 0:
            dc = 15
        # dc_x = math.ceil(dc * math.cos(angle))  if dc * math.cos(angle) >0  else math.floor(dc * math.cos(angle))
        # dc_y = math.ceil(dc * math.sin(angle))  if dc * math.sin(angle) >0  else math.floor(dc * math.sin(angle))
        #print(angle)
        dc_x = round(dc * math.cos(angle))
        dc_y = round(dc * math.sin(angle))
        #print(dc,dc_x,dc_y)
        xmin += dc_x
        xmax += dc_x
        ymin += dc_y
        ymax += dc_y
        xmin_,xmax_ =max(0, xmin), min(_24, xmax)
        ymin_,ymax_ = max(0, ymin), min(_24 ,ymax)
        # desired_cells = [(xmin,ymin),(xmax,ymax)]
        desired_cells = [(xmin_, xmax_), (ymin_, ymax_)]
        self.cont_cells = container_cells
        self.desired_cells = desired_cells


   ########################################## Actions Functions ##########################################
    def acc(self):
        if self.currentspeed < 4:
            traci.vehicle.slowDown(self.ID,self.speedDict[self.currentspeed+1] , self.dT/10 )
            self.currentspeed += 1
            # traci.vehicle.setSpeed(self.ID,self.speedDict[self.currentspeed])
            self.accl = 1

    def dec(self):
        if self.currentspeed > 0 :
            traci.vehicle.slowDown(self.ID,self.speedDict[self.currentspeed-1] , self.dT/10 )
            self.currentspeed -= 1
            # traci.vehicle.setSpeed(self.ID,self.speedDict[self.currentspeed])
            self.accl = -1


    def keepgoing(self):
        traci.vehicle.slowDown(self.ID, self.speedDict[self.currentspeed], self.dT / 10)
        self.accl = 0


    def get_time(self):
        if self.speedDict[self.currentspeed] != 0:
            time = self.get_time_step_distance() / self.speedDict[self.currentspeed]
        else:
            time = 1
        return time



    ############ end  #################
