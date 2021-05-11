#!/usr/bin/env python


from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random

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

def run():
    """execute the TraCI control loop"""
    step = 0
    car = AutoVehicle.AutoVehicle("agent")
    car2 = AutoVehicle.AutoVehicle("D1")
    car3 = AutoVehicle.AutoVehicle("D2")
    car4 = AutoVehicle.AutoVehicle("LH")

    my_env = env.env()
    agent = SingleAgent.SingleAgent( my_env , car)
    agent1 = SingleAgent.SingleAgent( my_env , car2)


    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step +=1
        # if step%7 == 0:
        #     print(step)
        #     agent.changeLane("left" , step)
        #     print("Hey , i changed my lane to left ")
        # if step % 33 == 0:
        #     print(step)
        #     agent.changeLane("right"  , step)
        #     print("Hey , i changed my lane to right")

        # myenv = AutoVehicle.env(agent)
        # myenv.PickAction()
        # myenv.TakeAction( step )
        # myenv.Current_state()
        #try:
        current_state =  my_env.Current_state(agent)
        possible_actions = my_env.getFeasibleActions(agent)
        agent.PickAction(current_state , possible_actions)
        agent.TakeAction(step)
        my_env.Reward(agent)
        # agent1.PickAction()
        # agent1.TakeAction(step)
        # my_env.Current_state(agent1)
        # my_env.Reward(agent1)
        #except:
         #   pass
        car2.getPose()
        #car.getBoxintersection()
        #car2.getBoxintersection()
        #car3.getBoxintersection()
        #car.getBoxintersection()

#[:0_9 , :0_1 , :0_5 ,:0_13]




    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
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

    # first, generate the route file for this simulation
    #generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "data/cross.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
    run()
