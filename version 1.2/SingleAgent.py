import sys , os
sys.path.append(os.path.join(os.environ.get("SUMO_HOME"), 'tools'))
from collections import defaultdict, deque
import traci
import random
from random import randrange
import numpy as np
import pickle

class SingleAgent():

    def __init__(self, env , algorithm = 'qlearning' ):
        self.algorithm = algorithm
        self.env = env
        self.ActionsDict = self.env.ActionsDict
        self.ActionsList = self.env.ActionsList
        self.num_episodes = 100

        if algorithm == 'qlearning':
            self.exp_exp_tradeoff = random.uniform(0, 1)
            self.epsilon = 1.0
            self.gamma = 0.618
            self.alpha = 0.7
            self.max_epsilon = 1.0
            self.min_epsilon = 0.01
            self.decay_rate = 0.01
            self.nA = len(self.env.ActionsList)
            self.nA_joint = len(self.env.ActionsList) ** len(self.env.ActionsList)

            def dict_inner(self, size):
                return np.zeros(size)

            try:
                f = open("./output/Q_i.pickle", "rb")
                self.Q_i = pickle.load(f)
            except:
                self.Q_i = defaultdict(self.indvidual_action)

            try:
                with open("./output/Q_I.pickle", "rb") as f:
                    self.Q_I = pickle.load(f)
            except:
                self.Q_I = defaultdict(self.joint_action)


    def indvidual_action(self):
        return np.zeros(self.nA)

    def joint_action(self):
        return np.zeros(self.nA_joint)

    def train(self):
        if self.algorithm == 'qlearning':
            return self.q_learning()

    def q_learning(self):
        # initialize empty dictionary of arrays

        def update_individual(state, reward, chosen_action, next_state):
            best_action = np.argmax(self.Q_i[f"{next_state}"])
            self.Q_i[f"{state}"][chosen_action] = self.Q_i[f"{state}"][chosen_action] + self.alpha * (reward + self.gamma * self.Q_i[f"{next_state}"][best_action] - self.Q_i[f"{state}"][chosen_action])

        def update_coordinated(state, reward, chosen_action, next_state):
            best_action = np.argmax(self.Q_I[f"{next_state}"])
            self.Q_I[f"{state}"][chosen_action] = self.Q_I[f"{state}"][chosen_action] + self.alpha * ( reward + self.gamma * self.Q_I[f"{next_state}"][best_action] - self.Q_I[f"{state}"][chosen_action])

        def update_from_coordinated_to_individual():
            print("Update from coordinated to individual")

        def update_from_individual_to_coordinated():
            print("Update from individual to coordinated")

        self.epsilon = 1 / self.epsilon


        for car in self.env.intersectionAgentList:
            current_state = self.env.states[car.ID]
            in_joint_state_with = self.env.is_overlap(car)
            if(len(in_joint_state_with) > 0):
                print("In joint state")
            else:
                each_element_prob = self.epsilon / self.nA
                prob = [each_element_prob] * self.nA
                prob[np.argmax(self.Q_i[f"{current_state}"])] += 1 - self.epsilon
                action = np.random.choice(np.arange(self.nA), p=prob)
                if action == 0:
                    car.acc()
                elif action == 1:
                    car.dec()
                elif action == 2:
                    car.keepgoing()
                car.UpdateStatus()
                next_state = self.env.states[car.ID]
                reward = self.env.get_agent_individual_reward(car)
                update_individual(current_state, reward, action, next_state)


        return self.Q_i, self.Q_I



