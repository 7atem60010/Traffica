import traci
from random import randrange

class SingleAgent():

    def __init__(self, env , vehicleAgent):
        #self.name = name
        self.env = env
        self.ActionsList = self.env.ActionsList
        self.car = vehicleAgent


    def PickAction(self): # Random policy to take an action
        self.Action = self.ActionsList[randrange(len(self.ActionsList))]

    def TakeAction(self , step):

        if self.Action=="ChangeLR":
            self.car.changeLane("right"  , step)
            #print("Hey , env action to right lane")

        if self.Action=="ChangeLF":
            self.car.changeLane("left"  , step)
            #print("Hey , env action to left lane")

        if self.Action=="fast":
            self.car.changeSpeed(self.Action)
            #print("Hey , I am faster")

        if self.Action=="slow":
            self.car.changeSpeed(self.Action)
            #print("Hey , I am slower")

        if self.Action=="stop":
            self.car.changeSpeed(self.Action)
            #print("Hey , I stopped")

        elif self.Action=="DoNothing":
            pass

