
import traci
from random import randrange

class AutoVehicle():
    '''
    This is a class for the main agent in our system - the autonomous vehicle.
    The class mainly used for getting the state of the vehicle and applying actions.
    '''

    def __init__(self, ID):
        self.ID = ID

    def getSpd(self):
        self.spd = traci.vehicle.getSpeed(self.ID)

    def getRoute(self):
        try:
            self.route = int(traci.vehicle.getRoadID(self.ID)[1])
        except:
            pass
            #print("Warning,current route status: "+traci.vehicle.getRoadID(self.ID) )
    def getPose(self):
        lane_pose = traci.vehicle.getLanePosition(self.ID)
        self.pose = lane_pose + self.route*1000

    def getAcc(self):
        self.accel = traci.vehicle.getAcceleration(self.ID)
    def getL(self):
        self.lane = traci.vehicle.getLaneIndex(self.ID)
    def chL(self,L):
        traci.vehicle.changeLane(self.ID,L, SimTime)
    def acc(self,spd,t):
        traci.vehicle.slowDown(self.ID,spd,t)
    def revertSpd(self):
        traci.vehicle.setSpeed(self.ID,-1)

   ########################################## Actions Functions ##########################################

    # WHat is SimTime? / Get SimTime!
    def changeLane (self , input = 'right'  , simTime = 10):
        #lane = traci.vehicle.getLaneIndex(self.ID)
        if input == 'right':
            traci.vehicle.changeLane(self.ID, 1, simTime)
        if input == 'left':
            traci.vehicle.changeLane(self.ID, 0, simTime)


    # At some point this function should take the acceleration to set speed by using it.
    # Changing the speed that way is not realistic  !
    def changeSpeed(self , speed = 10 ):
        traci.vehicle.setSpeed(self.ID , speed)

    def wait_time(self):
        return traci.vehicle.getWaitingTime(self.ID)

    ############ end  #################


class env():
    def __init__(self ,vehicleAgent): # vehicle agent is an instant from Auto-vehicle class
        self.ActionsList = ["ChangeLR" , "ChangeLF" ,"ChangeS","DoNothing"]
        self.ActionsDict = {"ChangeLR":0,"ChangeS":1,"DoNothing":2}
        self.agent = vehicleAgent

    def takeAction(self , step):

        if self.Action=="ChangeLR":
            self.agent.changeLane("right"  , step)
            print("Hey , env action to right lane")

        if self.Action=="ChangeLF":
            self.agent.changeLane("left"  , step)
            print("Hey , env action to left lane")

        elif self.Action=="ChangeS":
            self.agent.changeSpeed()
        elif self.Action=="DoNothing":
            pass

    def pickAction(self): # Random policy to take an action
        self.Action = self.ActionsList[randrange(len(self.ActionsList))]

    def evaluate(self):

        waiting_time =  self.agent.wait_time()
        print(waiting_time)
        if waiting_time == 0:
            self.reward = 10


