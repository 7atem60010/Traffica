import sys , os
sys.path.append(os.path.join(os.environ.get("SUMO_HOME"), 'tools'))
from collections import defaultdict, deque
import traci
import random
from random import randrange
import numpy as np

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


    def train(self):
        if self.algorithm == 'qlearning':
            return self.q_learning()


    def q_learning(self):
        # initialize empty dictionary of arrays
        nA = len(self.env.ActionsList)
        nA_joint = len(self.env.ActionsList)**len(self.env.ActionsList)
        Q_i = defaultdict(lambda: np.zeros(nA))
        Q_I = defaultdict(lambda: np.zeros(nA_joint))

        def update_individual(state, reward, chosen_action, next_state):
            print("update individual")
            best_action = np.argmax(Q_i[f"{next_state}"])
            Q_i[f"{state}"][chosen_action] = Q_i[f"{state}"][chosen_action] + self.alpha * (reward + self.gamma * Q_i[f"{next_state}"][best_action] - Q_i[f"{state}"][chosen_action])

        def update_coordinated(state, reward, chosen_action, next_state):
            print("update coordinated")
            best_action = np.argmax(Q_I[f"{next_state}"])
            Q_I[f"{state}"][chosen_action] = Q_I[f"{state}"][chosen_action] + self.alpha * ( reward + self.gamma * Q_I[f"{next_state}"][best_action] - Q_I[f"{state}"][chosen_action])

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
                print("In individual state")
                each_element_prob = self.epsilon / nA
                prob = [each_element_prob] * nA
                prob[np.argmax(Q_i[f"{current_state}"])] += 1 - self.epsilon
                action = np.random.choice(np.arange(nA), p=prob)
                print(f"for car: {car.ID} it took action: {action}")
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


        return Q_i, Q_I