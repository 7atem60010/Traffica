#!/usr/bin/env python


from __future__ import absolute_import
from __future__ import print_function
import pandas as pd
import os
import sys
import optparse
import random
import pickle
import numpy as np
from collections import defaultdict
# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from Autovechile import SingleAgent
from Autovechile import env
from sumolib import checkBinary  # noqa
import traci  # noqa

def run(episode):
    """execute the TraCI control loop"""
    step = 0
    existing_agents = []
    my_env = env.env()
    trainer = SingleAgent.SingleAgent(my_env)
    episode_reward_sum = 0
    waiting_time_sum = 0
    vehNr = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        step += 1
        vehNr += traci.simulation.getDepartedNumber()
        time = traci.simulation.getTime()


        Q_i, Q_I, step_reward, step_waiting_time, is_dead_lock = trainer.train(episode ,  existing_agents ,my_env)
        episode_reward_sum += step_reward
        waiting_time_sum += step_waiting_time

        if is_dead_lock:
            break

    try:
        with open("./output/episode_reward_dict.pickle", "rb") as reward_dic_reader:
            episode_reward_dict = pickle.load(reward_dic_reader)
    except:
        episode_reward_dict = defaultdict(int)

    try:
        with open("./output/average_waiting_time_dict.pickle", "rb") as waiting_time_reader:
            average_waiting_time_dict = pickle.load(waiting_time_reader)
    except:
        average_waiting_time_dict = defaultdict(int)

    episode_reward_dict[f"{episode}"] = episode_reward_sum / vehNr
    average_waiting_time_dict[f"{episode}"] = waiting_time_sum / vehNr

    with open("./output/episode_reward_dict.pickle", "wb") as reward_dic_writer:
        pickle.dump(episode_reward_dict, reward_dic_writer)

    with open("./output/average_waiting_time_dict.pickle", "wb") as waiting_time_writer:
        pickle.dump(average_waiting_time_dict, waiting_time_writer)

    with open("./output/Q_i.pickle", "wb") as f:
        pickle.dump(Q_i, f)

    with open("./output/Q_I_coordinated.pickle", "wb") as f2:
        pickle.dump(Q_I, f2)

    df = pd.DataFrame([[episode, average_waiting_time] for episode, average_waiting_time in episode_reward_dict.items()],
                      columns=['Episode', 'Average Waiting Time'])
    print(df.to_csv("./output/episode_waiting_time.csv"))

    df = pd.DataFrame([[episode, reward] for episode, reward in average_waiting_time_dict.items()],
                      columns=['Episode', 'Reward'])
    print(df.to_csv("./output/episode_reward.csv"))

    df = pd.DataFrame([ [state, actions_rewards] for state, actions_rewards in Q_i.items()],  columns=['state', 'Actions Rewards'])
    print(df.to_csv("./output/qtable.csv"))

    df = pd.DataFrame([[state, actions_rewards] for state, actions_rewards in Q_I.items()],
                      columns=['state', 'Actions Rewards'])
    print(df.to_csv("./output/qtableCoordinated.csv"))


    traci.close()
    sys.stdout.flush()



def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    optParser.add_option("--start", action="store",
                          default=0, help="skip to specific time")
    optParser.add_option("--step", action="store",help="set the step length to simulate at")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    print(sumoBinary)
    # first, generate the route file for this simulation
    #generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    if options.step is None:
        options.step = .1
    print((options.step))
    # for-loop
    # for i in range(100000000000,100000000001):
    for i in range(1):
        traci.start([sumoBinary, "-n", "2way-single-intersection/single-intersection.net.xml",
                                 "-r" , "2way-single-intersection/single-intersection-vhvh.rou.xml",
                                 "--tripinfo-output", "tripinfo.xml",
                                 "--collision.mingap-factor", "0",
                                 "--collision.action","none",
                                 "--step-length", ".5",
                                 "--time-to-teleport", "-1",
                                 "--time-to-teleport.highways", "-1"]
                                 # "--time-to-teleport.disconnected", "-1"]
        )
        # save the csv
        print("Simulation Step: ", i)
        run(i+1)