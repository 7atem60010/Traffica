import sys , os
sys.path.append(os.path.join(os.environ.get("SUMO_HOME"), 'tools'))
from collections import defaultdict, deque
import traci
import random
import AutoVehicle
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
            self.epsilon = 0.9
            self.gamma = 0.9
            self.alpha = 0.05
            self.min_epsilon = 0.01
            self.nA = len(self.env.ActionsList)
            self.nA_joint = len(self.env.ActionsList) ** len(self.env.ActionsList)

            try:
                f = open("./output/Q_i.pickle", "rb")
                self.Q_i = pickle.load(f)
            except:
                self.Q_i = defaultdict(self.indvidual_action)

            try:
                with open("./output/Q_I_coordinated.pickle", "rb") as f:
                    self.Q_I = pickle.load(f)
            except:
                self.Q_I = defaultdict(self.indvidual_action)


    def indvidual_action(self):
        return np.zeros(self.nA)

    def joint_action(self):
        return np.zeros(self.nA_joint)

    def train(self, episode , existing_agents ,my_env ):
        if self.algorithm == 'qlearning':
            return self.q_learning(episode ,existing_agents, my_env)

    def q_learning(self, episode , existing_agents , my_env):
        # initialize empty dictionary of arrays

        def update_env():
            traci.simulationStep()
            arv = traci.simulation.getArrivedIDList()
            dep = traci.simulation.getDepartedIDList()

            # ADD newly added cars as an agents
            for car_id in dep:
                existing_agents.append(AutoVehicle.AutoVehicle(car_id))

            # Remove the arrived-to-destination cars
            for agent in existing_agents:
                if agent.ID in arv:
                    existing_agents.remove(agent)
            
            for agent in existing_agents:
                try:
                    _ = traci.vehicle.getPosition(agent.ID)
                except:
                    existing_agents.remove(agent)
            # Cars in
            my_env.updateIntersectionAgents(existing_agents)
            my_env.updateStates()

            for agent in my_env.intersectionAgentList:
                joint = my_env.is_overlap(agent)

        def update_individual(state, reward, chosen_action, next_state):
            best_action = np.argmax(self.Q_i[f"{next_state}"])
            self.Q_i[f"{state}"][chosen_action] = self.Q_i[f"{state}"][chosen_action] + self.alpha *\
                                                  (reward + self.gamma * self.Q_i[f"{next_state}"][best_action] -
                                                   self.Q_i[f"{state}"][chosen_action])

        def update_coordinated(state, reward, chosen_action, next_state):
            best_action = np.argmax(self.Q_I[f"{next_state}"])
            self.Q_I[f"{state}"][chosen_action] = self.Q_I[f"{state}"][chosen_action] +\
                                                  self.alpha * (
                                                          reward + self.gamma * self.Q_I[f"{next_state}"][best_action] -
                                                          self.Q_I[f"{state}"][chosen_action])

        def update_from_coordinated_to_individual(state, reward, chosen_action, next_state_1, next_state_2):
            best_action_1 = np.argmax(self.Q_i[f"{next_state_1}"])
            best_action_2 = np.argmax(self.Q_i.get(f"{next_state_2}", np.random.choice([0, 1, 2])))
            self.Q_I[f"{state}"][chosen_action] = (1 - self.alpha) * self.Q_I[f"{state}"][chosen_action] + self.alpha *\
                                                  ( reward + self.gamma * (self.Q_i[f"{next_state_1}"][best_action_1] \
                                                                           + self.Q_i[f"{next_state_2}"][best_action_2]))


        def update_from_individual_to_coordinated(state, reward, chosen_action, next_state):
            best_action = np.argmax(self.Q_I[f"{next_state}"])
            # print(state)
            self.Q_i[f"{state}"][chosen_action] = (1 - self.alpha) * self.Q_i[f"{state}"][chosen_action] + self.alpha *\
                                                  (reward + self.gamma * 0.5 * (self.Q_I[f"{next_state}"][best_action]))

        self.epsilon = self.epsilon / episode
        if self.epsilon < self.min_epsilon:
            self.epsilon = self.min_epsilon

        car_state_action_reward_nextState =[]

        state_with_reward_dict = {}

        for car in self.env.intersectionAgentList:
            current_state = self.env.states[car.ID]
            in_joint_state_with = self.env.is_overlap(car)

            each_element_prob = self.epsilon / self.nA
            prob = [each_element_prob] * self.nA
            other_car_state = None

            if len(in_joint_state_with) > 0:

                other_car_state = self.env.states[in_joint_state_with[0].ID]
                current_state = (current_state, other_car_state)
                prob[np.argmax(self.Q_I[f"{current_state}"])] += 1 - self.epsilon
                action = np.random.choice(np.arange(self.nA), p=prob)
                car.isPrevIndividual = False

            else:

                prob[np.argmax(self.Q_i[f"{current_state}"])] += 1 - self.epsilon
                action = np.random.choice(np.arange(self.nA), p=prob)
                car.isPrevIndividual = True


            if action == 0:
                car.acc()
            elif action == 1:
                car.dec()
            elif action == 2:
                car.keepgoing()

            state_with_reward_dict[car.ID] = (car, current_state, action, other_car_state)


        update_env()

        step_reward = 0
        step_waiting_time = 0

        for car in self.env.intersectionAgentList:
            try:
                (car, current_state, action, other_car_state) = state_with_reward_dict[car.ID]
                next_state = self.env.states[car.ID]
                reward = self.env.get_agent_individual_reward(car)
                step_reward += reward
                step_waiting_time += 0.5
                car_state_action_reward_nextState.append((car, current_state, action, reward, next_state, other_car_state))
            except:
                t = 1

        locked_count = 0
        is_dead_lock = False
        dead_lock_states = []
        dead_lock_reward  = -2000
        for car, current_state, action, reward, next_state, other_car_state in car_state_action_reward_nextState:
            car.add_current_state(current_state)
            if car.is_potential_dead_lock():
                dead_lock_states.append(current_state)
                locked_count += 1
            if locked_count > 3 :
                is_dead_lock = True


        for car, current_state, action, reward, next_state, other_car_state in car_state_action_reward_nextState:
            if(len(self.env.is_overlap(car)) == 0 and car.isPrevIndividual):
                # print("In individual and still in individual")
                if is_dead_lock and current_state in dead_lock_states:
                    update_individual(current_state, dead_lock_reward, action, next_state)
                else:
                    update_individual(current_state, reward, action, next_state)


            elif(len(self.env.is_overlap(car)) > 0 and car.isPrevIndividual):
                # print("In coordinated and switched to individual")
                car_to_coordinate_with = self.env.is_overlap(car)[0]
                car_to_coordinate_with_state = self.env.states[car_to_coordinate_with.ID]
                coordinated_state = (next_state, car_to_coordinate_with_state)
                if is_dead_lock and current_state in dead_lock_states:
                    update_from_individual_to_coordinated(current_state, dead_lock_reward, action, coordinated_state)
                else:
                    update_from_individual_to_coordinated(current_state,reward, action, coordinated_state)

            elif(len(self.env.is_overlap(car)) == 0 and not car.isPrevIndividual):
                # print("In Individual and switched to coordinated")
                next_state_1 = next_state
                next_state_2 = other_car_state
                if is_dead_lock and current_state in dead_lock_states:
                    update_from_coordinated_to_individual(current_state, dead_lock_reward, action, next_state_1, next_state_2)
                else:
                    update_from_coordinated_to_individual(current_state, reward, action, next_state_1, next_state_2)


            elif(len(self.env.is_overlap(car)) > 0 and not car.isPrevIndividual):
                # print("In coordinated and still in coordinated")
                car_to_coordinate_with = self.env.is_overlap(car)[0]
                car_to_coordinate_with_state = self.env.states[car_to_coordinate_with.ID]
                coordinated_state = (next_state, car_to_coordinate_with_state)
                if is_dead_lock and current_state in dead_lock_states:
                    update_coordinated(current_state, dead_lock_reward, action, coordinated_state)
                else:
                    update_coordinated(current_state,reward, action, coordinated_state)




        return self.Q_i, self.Q_I, step_reward, step_waiting_time, is_dead_lock



