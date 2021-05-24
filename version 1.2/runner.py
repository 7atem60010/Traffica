#!/usr/bin/env python


from __future__ import absolute_import
from __future__ import print_function
import pandas as pd
import os
import sys
import optparse
import random
import pickle
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

def generate_routefile():
    random.seed(420)  # make tests reproducible
    N = 360  # number of time steps

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
        p = 1/100
        #TODO: difference between time step and simulation step
        myroutes = ['right','right_up','right_down','left','left_up','left_down',"up", "up_right" , "up_left" , "down_right" , "down_left" , "down"]
        for i in range(N):
            for route in myroutes:
                r = random.uniform(0,1)
                if r < p:
                    print(f'        <vehicle id="{route}_{vehNr}" type="car" route="{route}" depart="{i}" />' , file=routes)
                    vehNr += 1
        print("</routes>", file=routes)

def run(episode):
    """execute the TraCI control loop"""
    step = 0
    generate_routefile()
    existing_agents = []
    my_env = env.env()
    trainer = SingleAgent.SingleAgent(my_env)

    while traci.simulation.getMinExpectedNumber() > 0:
        step += 1
        time = traci.simulation.getTime()
        # traci.simulationStep()
        # arv = traci.simulation.getArrivedIDList()
        # dep = traci.simulation.getDepartedIDList()
        #
        # # ADD newly added cars as an agents
        # for car_id in dep:
        #     existing_agents.append(AutoVehicle.AutoVehicle(car_id))
        #
        # # Remove the arrived-to-destination cars
        # for agent in existing_agents:
        #     if agent.ID in arv:
        #         existing_agents.remove(agent)
        #
        # # Cars in
        # my_env.updateIntersectionAgents(existing_agents)
        # my_env.updateStates()
        #
        # for agent in my_env.intersectionAgentList:
        #     joint = my_env.is_overlap(agent)
            # if len(joint) > 0:
            #     print(time, joint)

        Q_i, Q_I = trainer.train(episode ,  existing_agents ,my_env)
        # print("Train Done")
        # print(Q_i)
        # print(f"In Episode {i}: Q table is ")

    with open("./output/Q_i.pickle", "wb") as f:
        pickle.dump(Q_i, f)

    with open("./output/Q_I.pickle", "wb") as f:
        pickle.dump(Q_I, f)

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
    for i in range(1):
        traci.start([sumoBinary, "-c", "data/cross.sumocfg",
                                 "--tripinfo-output", "tripinfo.xml",
                                 "--collision.action","none",
                                 "--step-length", ".5"]
                                 # "--begin", "400"]
        )
        # save the csv
        print("Simulation Step: ", i)
        run(i+1)
