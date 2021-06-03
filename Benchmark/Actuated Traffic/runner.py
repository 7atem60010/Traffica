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

from Autovechile import AutoVehicle
from Autovechile import env
from sumolib import checkBinary  # noqa
import traci  # noqa

def run_benchmark():
    step = 0
    existing_agents = []
    my_env = env.env()
    total_intersection_waiting_time = 0
    vehNr = 0

    while traci.simulation.getMinExpectedNumber() > 0:

        traci.simulationStep()
        arv = traci.simulation.getArrivedIDList()
        dep = traci.simulation.getDepartedIDList()
        vehNr += traci.simulation.getDepartedNumber()

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
        total_intersection_waiting_time += traci.simulation.getDeltaT() * len(my_env.intersectionAgentList)

    Avg_waiting_time = total_intersection_waiting_time/vehNr
    print(Avg_waiting_time)




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
        run_benchmark()