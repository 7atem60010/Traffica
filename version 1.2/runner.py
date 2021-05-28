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

import AutoVehicle
import SingleAgent
import env
from sumolib import checkBinary  # noqa
import traci  # noqa

def generate_routefile(num_vehicles):
    random.seed(420)  # make tests reproducible
    dT = .5
    N = 3600  # number of seconds
    S = N/dT
    with open("data/cross.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="car" accel="3.5" decel="3.5" sigma="0.5" width="1.8" length="4.5" minGap="1" maxSpeed="25.2" guiShape="passenger"/>

        <route id="right" edges="51o 1i 2o 52i" />
        <route id="right_down" edges="51o 1i 3o 53i" />
        <route id="right_up" edges="51o 1i 4o 54i" />
        <route id="left" edges="52o 2i 1o 51i" />
        <route id="left_down" edges="52o 2i 3o 53i" />
        <route id="left_up" edges="52o 2i 4o 54i" />

        <route id="up" edges="53o 3i 4o 54i" />
,        <route id="up_right" edges="53o 3i 2o 52i" />
        <route id="up_left" edges="53o 3i 1o 51i" />
        <route id="down_right" edges="54o 4i 2o 52i" />
        <route id="down_left" edges="54o 4i 1o 51i" />
        <route id="down" edges="54o 4i 3o 53i" />
        """, file=routes)
        vehNr = 0
        # demand per second from different directions
<<<<<<< HEAD
        p = (num_vehicles/S)/100
=======
        p = (num_vehicles/S)
>>>>>>> 6c83954629751486508b5cedbfcd367c1882638e
        # TODO: difference between time step and simulation step
        myroutes = ['right','right_up','right_down','left','left_up','left_down',"up", "up_right" , "up_left" , "down_right" , "down_left" , "down"]
        myroutes = ['right','left',"up", "down"]
        Ps = [p,p/2,p/2,p/2]
        for i in np.arange(1, N , dT):
            for j,route in enumerate(myroutes):
                p = Ps[j]
                r = random.uniform(0, 1)
                if r < p:
                    print(f'        <vehicle id="{route}_{vehNr}" type="car"  route="{route}" depart="{i}" />' , file=routes)
                    vehNr += 1
        print("</routes>", file=routes)
        return vehNr

def run(episode):
    """execute the TraCI control loop"""
    step = 0
    vehNr = generate_routefile(3000)
    existing_agents = []
    my_env = env.env()
    trainer = SingleAgent.SingleAgent(my_env)
    episode_reward_sum = 0

    while traci.simulation.getMinExpectedNumber() > 0:
        step += 1
        time = traci.simulation.getTime()
<<<<<<< HEAD
=======
        # print(time)
>>>>>>> 6c83954629751486508b5cedbfcd367c1882638e

        Q_i, Q_I, step_reward = trainer.train(episode ,  existing_agents ,my_env)
        episode_reward_sum += step_reward

    try:
        with open("./output/episode_reward_dict.pickle", "rb") as reward_dic_reader:
            episode_reward_dict = pickle.load(reward_dic_reader)
    except:
        episode_reward_dict = defaultdict(int)

    episode_reward_dict[f"{episode}"] = episode_reward_sum / vehNr

    with open("./output/episode_reward_dict.pickle", "wb") as reward_dic_writer:
        pickle.dump(episode_reward_dict, reward_dic_writer)

    with open("./output/Q_i.pickle", "wb") as f:
        pickle.dump(Q_i, f)

    with open("./output/Q_I_coordinated.pickle", "wb") as f2:
        pickle.dump(Q_I, f2)

    df = pd.DataFrame([[episode, reward] for episode, reward in episode_reward_dict.items()],
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
    for i in range(10):
        traci.start([sumoBinary, "-c", "data/cross.sumocfg",
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
