
import traci
from random import randrange

class AutoVehicle():
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

    def getPose(self):
        lane_pose = traci.vehicle.getLanePosition(self.ID)
        self.pose = lane_pose + self.route * 500


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

#######################################################################################################################3

class env():
    def __init__(self ,vehicleAgent): # vehicle agent is an instant from Auto-vehicle class
        self.ActionsList = ["ChangeLR" , "ChangeLF" ,"fast" , "slow" , "stop" ,"DoNothing"]
        self.ActionsDict = {"ChangeLR":0,"ChangeLF":1 , "fast" : 2 , "slow":3 , "stop":4 ,"DoNothing":5}
        self.agent = vehicleAgent

    def TakeAction(self , step):

        if self.Action=="ChangeLR":
            self.agent.changeLane("right"  , step)
            #print("Hey , env action to right lane")

        if self.Action=="ChangeLF":
            self.agent.changeLane("left"  , step)
            #print("Hey , env action to left lane")

        if self.Action=="fast":
            self.agent.changeSpeed(self.Action)
            #print("Hey , I am faster")

        if self.Action=="slow":
            self.agent.changeSpeed(self.Action)
            #print("Hey , I am slower")

        if self.Action=="stop":
            self.agent.changeSpeed(self.Action)
            #print("Hey , I stopped")

        elif self.Action=="DoNothing":
            pass

    def PickAction(self): # Random policy to take an action
        self.Action = self.ActionsList[randrange(len(self.ActionsList))]

    def Reward(self):

        self.agent.UpdateStatus()
        self.agent.reward = 0
        if self.agent.isStop() == False:
            self.agent.reward +=1

        self.agent.reward -= self.agent.wait_time()

        self.agent.reward += (self.agent.spd/self.agent.maxspeed) * 10

        nonlane_vehicle_num = traci.edge.getLastStepVehicleNumber(self.agent.edge)-traci.lane.getLastStepVehicleNumber(self.agent.lane)
        if traci.lane.getLastStepVehicleNumber(self.agent.lane) == max(traci.lane.getLastStepVehicleNumber(self.agent.lane) , nonlane_vehicle_num):
            self.agent.reward += traci.lane.getLastStepVehicleNumber(self.agent.lane) - nonlane_vehicle_num
        else :
            self.agent.reward += traci.lane.getLastStepVehicleNumber(self.agent.lane) - nonlane_vehicle_num

<<<<<<< HEAD
        #print("Reward :", self.agent.reward)

    def Current_state(self):
        '''
        Traffic state on the lane
        # of cares in the lane
        my speed
        avg speed of the cares in my lane
        '''
        state =( self.agent.next_TL() , self.agent.spd , traci.lane.getLastStepMeanSpeed(self.agent.lane) )
        print(state)
=======
        print("Reward :", self.agent.reward)

    def getFeasibleActions(self):
        
        left = 1
        right = -1
        changeLeftPossible = traci.vehicle.couldChangeLane(agent.ID, left, state=None)
        changeRightPossible = traci.vehicle.couldChangeLane(agent.ID, right, state=None)
        _,leaderDist = trace.getLeader(self.agent.ID)
        accelerate_possible = min(self.agent.maxspeed, self.agent.spd + agent.maxacc/2) <= leaderDist
        proposedActions = self.actionList.copy()
        if changeLeftPossible == False:
            proposedActions.remove("changeLF")
        if changeRightPossible == False:
            proposedActions.remove("changeLR")
        if accelerate_possible == False:
            proposedActions.remove("fast")
        return proposedActions
>>>>>>> 4b2e0f04656233a1d5487993a8c9766c85cdfbf1
