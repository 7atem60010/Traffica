import sys , os
sys.path.append(os.path.join(os.environ.get("SUMO_HOME"), 'tools'))

import traci
from random import randrange

class AutoVehicle:
    '''
    This is a class for the main agent in our system - the autonomous vehicle.
    The class mainly used for getting the state of the vehicle and applying actions.
    '''

    def __init__(self, ID):
        self.ID = ID
        self.maxspeed = traci.vehicle.getMaxSpeed(self.ID)
        self.maxacc = traci.vehicle.getAccel(self.ID)
        self.maxdec = traci.vehicle.getDecel(self.ID)
        self.reward = 0


    ############################# Getter ###############################

    def getCells(self,lane_len,side_cells,intersection_width):
        x,y = traci.vehicle.getPosition(self.ID)
        x,y = x-lane_len,y-lane_len
        x_i,y_i = x//(intersection_width/side_cells),y//(intersection_width/side_cells)
        x_i,y_i = int(x_i),int(y_i)
        if intersection_width>=x>=0 and intersection_width>=y>=0:
            print(x_i,y_i)
        return x_i,y_i


    def getPose(self):
        lane_pose = traci.vehicle.getLanePosition(self.ID)
        print(lane_pose)
        position=traci.vehicle.getPosition(self.ID)
        #print(position)
        pos = traci.vehicle.getLateralLanePosition(self.ID)
        #print(pos)

        #self.pose = lane_pose + self.route * 500

    def getBoxintersection(self):
        lane_pose = traci.vehicle.getLanePosition(self.ID)
        edge_id  = traci.vehicle.getRoadID(self.ID)
        position = traci.vehicle.getPosition(self.ID)
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


   ########################################## Actions Functions ##########################################

    # WHat is SimTime? / Get SimTime!
    def changeLane (self , input = 'right'  , simTime = 10 ):
        #lane = traci.vehicle.getLaneIndex(self.ID)
        if input == 'right':
            traci.vehicle.changeLane(self.ID, 1, simTime)
        if input == 'left':
            traci.vehicle.changeLane(self.ID, 0, simTime)


    # At some point this function should take the acceleration to set speed by using it.
    # Changing the speed that way is not realistic  !
    def changeSpeed(self ,  input = "slow"  ):
        """
        In this function we change the speed of the car by the following procedure :
         fast : newspeed = 1.3*oldspeed
         slow : newspeed = 0.7*oldspeed
         stop : newspeed = 0
        """
        self.UpdateStatus()
        if input == "fast":
            traci.vehicle.setSpeed(self.ID , max(1.3 * self.spd ,60))
        if input == "slow":
            traci.vehicle.setSpeed(self.ID , max(0.5 * self.spd ,30))
        if input == "stop":
            traci.vehicle.setSpeed(self.ID , 0)

    def isStop(self):
        return traci.vehicle.isStopped(self.ID)

    def wait_time(self):
        return traci.vehicle.getWaitingTime(self.ID)
    def next_TL(self):
        return traci.vehicle.getNextTLS(self.ID)

    ############ end  #################

