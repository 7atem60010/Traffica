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

    # # car generation
    # id_list = traci.vehicle.getIDList()
    # vehicleList =[]
    # vechile_dict ={}
    # print(len(id_list))
    # for id in id_list:
    #     print(id)
    #     car = AutoVehicle.AutoVehicle(id)
    #     vehicleList.append(car)
    #     vechile_dict[id] = car
    # car2 = AutoVehicle.AutoVehicle("LH")
    # car3 = AutoVehicle.AutoVehicle("D2")
    # car4 = AutoVehicle.AutoVehicle("LH")
    car = AutoVehicle.AutoVehicle("agent")
    #vehicleList = [car , car2 , car3 , car4]

    my_env = env.env()
    agent = SingleAgent.SingleAgent(my_env, car)
    # agent2 = SingleAgent.SingleAgent( my_env , car2)
    car_dict = {"agent":car }
    my_env.vehicleList = [SingleAgent.SingleAgent(my_env, car) for car in list(car_dict.values())]

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        print(traci.simulation.getTime())
        #
        # id_list = traci.vehicle.getIDList()
        # vehicleList = [] # in car generation
        # vechile_dict = {}  # in car generation
        # print(len(id_list))
        id_list = traci.vehicle.getIDList()
        car.UpdateStatus()
        for id in id_list:
            car = car_dict[id]
            print(traci.vehicle.getLaneID(car.ID))
            car.UpdateStatus()
            # print(car.pos)
            agent = SingleAgent.SingleAgent(my_env, car)
            if car.inIntersection():
                curr_cells, desired_cells = my_env.get_current_cells(agent)
                agent.PickAction()
                agent.TakeAction()
                print(car.queue)

                # print(curr_cells, desired_cells, car.currentspeed, traci.vehicle.getSpeed(car.ID))
                # print(car.currentspeed, car)

            # print(car.inIntersection())
        #  x .1 x'
        #  x .5 x'
        step += 1
        # my_env.vehicleList = []
        #id_list = traci.vehicle.getIDList()

        # Intersection Testing (Passed)
        # for vehicle in vehicleList:
        #     try:
        #         if vehicle.inIntersection():
        #             my_env.vehicleList.append(vehicle)
        #     except:
        #         pass


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
        #current_state =  my_env.Car_current_state(agent)
        #possible_actions = my_env.getFeasibleActions(agent)
        #agent.PickAction(current_state , possible_actions)
        #agent.TakeAction(step)
        #my_env.Reward(agent)

        # my_env.
        # agent1.PickAction()
        # agent1.TakeAction(step)
        # my_env.Current_state(agent1)
        # my_env.Reward(agent1)
        #except:
         #   pass
        #car2.getPose()
        # car.getBoxintersection()
        #car2.getBoxintersection()
        #car3.getBoxintersection()
        #car.getBoxintersection()
        # car.printPose()
        # car.printPose(500,4,20)

#[:0_9 , :0_1 , :0_5 ,:0_13]




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
    traci.start([sumoBinary, "-c", "data/cross.sumocfg",
                             "--tripinfo-output", "tripinfo.xml",
                             "--step-length", ".5"]
                             # "--begin", "400"]
    )
    run()
