import traci
from random import randrange


class AutoVechile():
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
    def changeLane (self , input = 'right'):
        #lane = traci.vehicle.getLaneIndex(self.ID)
        if input == 'right':
            traci.vehicle.changeLane(self.ID, 1, SimTime)
        if input == 'left':
            traci.vehicle.changeLane(self.ID, 0, SimTime)

    # At some point this function should take the acceleration to set speed by using it.
    # Changing the speed that way is not realistic  !
    def changeSpeed(self , speed = 10 ):
        traci.vehicle.setSpeed(self.ID , speed)

    ############ end  #################


class env():
    def __init__(self ,vehicleAgent): # vehicle agent is an instant from Auto-vehicle class
        self.ActionsList = ["ChangeLR" ,"ChangeS","DoNothing"]
        self.ActionsDict = {"ChangeLR":0,"ChangeS":1,"DoNothing":2}
        self.agent = vehicleAgent

    def takeAction(self):

        if self.Action=="ChangeLR":
            self.agent.changeLane()
        elif self.Action=="ChangeS":
            self.agent.changeSpeed()
        elif self.Action=="DoNothing":
            pass

    def pickAction(self): # Random policy to take an action
        self.Action = self.ActionsList[randrange(len(self.ActionsList))]
