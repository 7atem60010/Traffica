import traci
import random
from random import randrange
import numpy as np

class SingleAgent():

    def __init__(self, env , vehicleAgent , algorithm = 'qlearning' ):
        self.algorithm = algorithm
        self.env = env
        self.ActionsDict = self.env.ActionsDict
        self.ActionsList = self.env.ActionsList
        self.car = vehicleAgent

        if algorithm == 'qlearning':
            self.qtable = np.zeros((6 , 10 , 10 , 10))
            """
            param : # of actions
            param : max # of cars in the lane = 500/5 as 5 is the car length
            param : # of speeds avaliable 10 : 100/10
            param : avg speed in the lane 
            
            """
            self.exp_exp_tradeoff = random.uniform(0, 1)
            self.epsilon = 1.0

            self.gamma = 0.618
            self.learning_rate = 0.7
            self.max_epsilon = 1.0
            self.min_epsilon = 0.01
            self.decay_rate = 0.01
            ## Reference :

    # Random policy to take an action
    #self.Action = self.ActionsList[randrange(len(self.ActionsList))]

    def PickAction(self ,possible_actions, current_state):
        self.Action = self.ActionsList[randrange(len(self.ActionsList))]
        print(current_state)
        print(possible_actions)




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

